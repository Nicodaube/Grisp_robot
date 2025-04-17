-module(main_loop).

-export([robot_init/2, modify_frequency/1]).

-define(RAD_TO_DEG, 180.0/math:pi()). % Conversion factor from radians to degrees

-define(INCH_TO_CM, 2.54). % Conversion factor from inch to cm

-define(LOG_DURATION, 15000).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Robot Initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

robot_init(Hera_pid, _Role) ->

    % Set maximum process priority for real-time critical tasks
    process_flag(priority, max),

    % Initial timestamp [ms]
    T0 = erlang:system_time() / 1.0e6,

    % Create ETS table for shared variables
    % - 'variables' table (public, named) to store real-time global state
    ets:new(variables, [set, public, named_table]),
    ets:insert(variables, {"Freq_Goal", 300.0}),  % Initial loop frequency goal [Hz]

    % Calibration of gyroscope offsets
    io:format("[Robot] Calibrating... Do not move the pmod_nav!~n"),
    grisp_led:color(1, {1, 0, 0}),  % LEDs red during calibration
    grisp_led:color(2, {1, 0, 0}),
    [_Gx0, Gy0, _Gz0] = helper_module:calibrate(),
    io:format("[Robot] Done calibrating~n"),

    % Initialize Kalman filter state
    % - X0: State vector [angle, angular velocity, distance to obstacle]
    % - P0: State covariance matrix
    X0 = mat:matrix([[0], [0], [0]]),
    P0 = mat:matrix([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]),

    % Open I2C bus for communication with external devices (ESP32, etc.)
    io:format("[Robot] Opening I2C bus...~n"),
    I2Cbus = grisp_i2c:open(i2c1),

    % Initialize PID controllers
    % - Pid_Speed: PI controller for advance speed
    % - Pid_Stability: PD controller for tilt stability
    % - Pid_Obstacle_Avoidance: PI controller for slowing down when detecting obstacles
    Pid_Speed = spawn(pid_controller, pid_init, [-0.12, -0.07, 0.0, -1, 60.0, 0.0]),
    Pid_Stability = spawn(pid_controller, pid_init, [17.0, 0.0, 4.0, -1, -1, 0.0]),
    Pid_Obstacle_Avoidance = spawn(pid_controller, pid_init, [-0.06, -0.02, 0.0, -1, 60.0, 0.0]),

    io:format("[Robot] Pid of the speed controller: ~p~n", [Pid_Speed]),
    io:format("[Robot] Pid of the stability controller: ~p~n", [Pid_Stability]),
    io:format("[Robot] Pid of the obstacle avoidance controller: ~p~n", [Pid_Obstacle_Avoidance]),
    io:format("[Robot] Starting movement of the robot.~n"),

    % Launch main control loop with initial state
    robot_main(
        T0, Hera_pid,
        {rest, false},            % Robot state (rest) and "robot up" flag (false)
        {T0, X0, P0},              % Initial time, Kalman state, Kalman covariance
        I2Cbus,                   % I2C communication bus
        {0, T0, []},               % Logging: counter, end time, empty log list
        {Gy0, 0.0, 0.0},           % Gyroscope offset, complementary filter initial angle/rate
        {Pid_Speed, Pid_Stability, Pid_Obstacle_Avoidance}, % PID controllers
        {0.0, 0.0},                % Initial advance and turning velocities [cm/s, deg/s]
        {0, 0, 200.0, T0}          % Loop frequency tracking {N, freq, mean_freq, time_end}
    ).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Main Robot Control Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Main control loop for the robot.
%% This function handles sensor data acquisition, control logic, and communication with the ESP32.
%% @param Start_Time - The initial timestamp for the robot.
%% @param Hera_pid - The process ID of the Hera process for logging.
%% @param {Robot_State, Robot_Up} - The current state of the robot and its upright status.
%% @param {T0, X0, P0} - The current time, Kalman state, and Kalman covariance.
%% @param I2Cbus - The I2C bus for communication with external devices.
%% @param {Logging, Log_End, Log_List} - Logging status, end time, and log list.
%% @param {Gy0, Angle_Complem, Angle_Rate} - Gyroscope offset and complementary filter parameters.
%% @param {Pid_Speed, Pid_Stability, Pid_Obstacle_Avoidance} - PID controllers for speed, stability, and obstacle avoidance.
%% @param {Adv_V_Ref, Turn_V_Ref} - Reference velocities for advance and turning.
%% @param {N, Freq, Mean_Freq, T_End} - Loop frequency tracking variables.
%% @return - The updated state of the robot after processing the loop.
robot_main(Start_Time, Hera_pid, {Robot_State, Robot_Up}, {T0, X0, P0},
    I2Cbus, {Logging, Log_End, Log_List},
    {Gy0, Angle_Complem, Angle_Rate},
    {Pid_Speed, Pid_Stability, Pid_Obstacle_Avoidance},
    {Adv_V_Ref, Turn_V_Ref},
    {N, Freq, Mean_Freq, T_End}) ->

    % Compute elapsed time since last iteration
    T1 = erlang:system_time() / 1.0e6,
    Dt = (T1 - T0) / 1000.0,

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Sensor Reading
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    [Gy, Ax, Az] = pmod_nav:read(acc, [out_y_g, out_x_xl, out_z_xl], #{g_unit => dps}),
    Distance_Sonar_inch = pmod_maxsonar:get(),
    Distance_Sonar_cm = helper_module:round(Distance_Sonar_inch * ?INCH_TO_CM, 4),

    Filtered_Distance = distance_filtering(Distance_Sonar_cm, 600.0),

    % Read data from ESP32 (wheel speeds, control flags)
    [<<SL1, SL2, SR1, SR2, CtrlByte>>] = grisp_i2c:transfer(I2Cbus, [{read, 16#40, 1, 5}]),
    [Speed_L, Speed_R] = hera_com:decode_half_float([<<SL1, SL2>>, <<SR1, SR2>>]), % Decode the half-float values for left and right wheel speeds
    Speed = (Speed_L + Speed_R) / 2, % Compute the average speed of the robot
    [Arm_Ready, Switch, Test, Get_Up, Forward, Backward, Left, Right] = hera_com:get_bits(CtrlByte),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Command Computation
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    Adv_V_Goal = helper_module:speed_ref(Forward, Backward),
    Turn_V_Goal = helper_module:turn_ref(Left, Right),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Angle Estimation
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Angle from accelerometer only (for basic check)
    Angle_Accelerometer = math:atan(Az / (-Ax)) * ?RAD_TO_DEG,

    % Kalman Filter fusion: accelerometer + gyroscope + sonar distance
    {X1, P1} = helper_module:kalman_angle(Dt, Ax, Az, Gy, Gy0, X0, P0, Filtered_Distance),
    [Th_Kalman, _W_Kalman, D_Kalman] = mat:to_array(X1), % Extract the angle (Th_Kalman) and distance from the state matrix.
    Angle_Kalman = Th_Kalman * ?RAD_TO_DEG,

    io:format("[Robot] Sonar: ~p cm | Kalman Distance Estimate: ~p cm~n", [Filtered_Distance, D_Kalman]),

    % Complementary filter fusion: gyroscope + accelerometer (fast)
    K = 1.25 / (1.25 + (1.0 / Mean_Freq)), % Compute the weighting factor.
    {Angle_Complem_New, Angle_Rate_New} = helper_module:complem_angle({Dt, Ax, Az, Gy, Gy0, K, Angle_Complem, Angle_Rate}),

    % Select angle estimator
    Angle = helper_module:select_angle(Switch, Angle_Kalman, Angle_Complem),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Stability Control
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Compute the acceleration and turning velocity references using the stability engine.
    {Acc, Adv_V_Ref_New, Turn_V_Ref_New} =
        stability_engine:controller(
            {Dt, Angle, Speed},
            {Pid_Speed, Pid_Stability, Pid_Obstacle_Avoidance},
            {Adv_V_Goal, Adv_V_Ref},
            {Turn_V_Goal, Turn_V_Ref},
            {D_Kalman, exponential}
        ),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Robot State Management
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Determine the state of the robot (whether it is upright or not)
    Robot_Up_New =
        case {Robot_Up, abs(Angle)} of
            % If the robot is upright and the absolute angle exceeds 20 degrees, it is considered to have fallen.
            {true, A} when A > 20 -> false;
            % If the robot is not upright and the absolute angle is less than 18 degrees, it is considered to have recovered.
            {false, A} when A < 18 -> true;
            _ -> Robot_Up
        end,


    % Determine the forward/backward direction of the robot based on the angle.
    F_B = case Angle > 0.0 of true -> 1; false -> 0 end,

    Next_Robot_State = helper_module:robot_state_transition(Robot_State, Get_Up, Robot_Up, Arm_Ready, abs(Angle)),

    {Power, Freeze, Extend, Robot_Up_Bit} = helper_module:robot_output_state(Next_Robot_State),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Communication to ESP32
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    Output_Byte = helper_module:get_byte([Power, Freeze, Extend, Robot_Up_Bit, F_B, 0, 0, 0]),
    [HF1, HF2] = hera_com:encode_half_float([Acc, Turn_V_Ref_New]),

    grisp_i2c:transfer(I2Cbus, [{write, 16#40, 1, [HF1, HF2, <<Output_Byte>>]}]),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Logging & Testing Features
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    {N_New, Freq_New, Mean_Freq_New} = helper_module:frequency_computation(Dt, N, Freq, Mean_Freq),

    % Determine the end time for logging based on the test mode.
    Log_End_New = if Test -> erlang:system_time()/1.0e6 + ?LOG_DURATION; true -> Log_End end,
    Logging_New = (erlang:system_time()/1.0e6) < Log_End_New,

    % Flickering LEDs when logging
    helper_module:flicker_led(Logging_New, N),

    % Start/Stop log messages to Hera
    helper_module:manage_logging_transition(Hera_pid, Logging, Logging_New),

    % Send values to ESP32 if requested, otherwise accumulate values in the log list
    Log_List_New = helper_module:update_log_list(Logging_New, Log_List, 
        [T1-Start_Time, 1/Dt, Gy, Acc, CtrlByte, -Angle_Accelerometer, -Angle_Kalman, -Angle_Complem, Adv_V_Ref, Switch, Adv_V_Ref_New, Turn_V_Ref_New, Speed]),
    
    % Handle potential incoming messages (get_all_data, freq, acc)
    helper_module:handle_incoming_messages({
        T1-Start_Time, 1/Dt, Gy, Acc, CtrlByte,
        -Angle_Accelerometer, -Angle_Kalman, -Angle_Complem,
        Adv_V_Ref, Switch, Adv_V_Ref_New, Turn_V_Ref_New, Speed
    }),
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Loop Timing Management
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    helper_module:enforce_loop_frequency(T_End),

    T_End_New = erlang:system_time() / 1.0e6,

    % Recursive call (robot continues operation)
    robot_main(
        Start_Time, Hera_pid,
        {Next_Robot_State, Robot_Up_New},
        {T1, X1, P1},
        I2Cbus,
        {Logging_New, Log_End_New, Log_List_New},
        {Gy0, Angle_Complem_New, Angle_Rate_New},
        {Pid_Speed, Pid_Stability, Pid_Obstacle_Avoidance},
        {Adv_V_Ref_New, Turn_V_Ref_New},
        {N_New, Freq_New, Mean_Freq_New, T_End_New}
    ).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Global variables related functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Updates the desired frequency goal in the ETS table.
%% This function modifies the value associated with the key `"Freq_Goal"`
%% in the `variables` ETS table to the provided frequency value.
modify_frequency(Freq) ->
    ets:insert(variables, {"Freq_Goal", Freq}),
    ok.

%% @doc Filters the distance measurement to avoid large fluctuations.
%% This function checks the current distance against the previous distance
%% @param Rounded_distance The current distance measurement (rounded).
%% @param Max_Change The maximum allowable change in distance.
%% @return The filtered distance measurement.
distance_filtering(Rounded_distance, Max_Change) ->
    % Retrieve the previous distance measurement from the ETS table, or default to the current distance
    case ets:lookup(variables, "Prev_Distance") of
        [] ->
            % If no previous distance is stored, use the current distance
            Prev_Distance = Rounded_distance;
        [{_, Prev_Distance_Stored}] ->
            Prev_Distance = Prev_Distance_Stored
    end,

    % Compute the absolute change in distance
    Distance_Change = abs(Rounded_distance - Prev_Distance),

    % Check if the change exceeds the maximum allowable threshold
    Filtered_Distance =
        if
            Distance_Change > Max_Change ->
                % If the change is too large, use the previous distance
                Prev_Distance;
            true ->
                % Otherwise, use the current distance
                Rounded_distance
        end,

    % Store the filtered distance as the new previous distance in the ETS table
    ets:insert(variables, {"Prev_Distance", Filtered_Distance}),
    Filtered_Distance.

