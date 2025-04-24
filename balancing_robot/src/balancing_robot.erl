-module(balancing_robot).

-behavior(application).

-define(RAD_TO_DEG, 180.0/math:pi()).
-define(LOG_DURATION, 15000).
-define(FREQ_GOAL, 300.0).

-export([start/2, stop/1]).
-export([robot_init/0]).
    
%============================================================================================================================================
%========================================================= BASIC GRISP FUNC =================================================================
%============================================================================================================================================

% Application start callback
start(_Type, _Args) ->
    {ok, Supervisor} = balancing_robot_sup:start_link(), % Start the supervisor
    [grisp_led:flash(L, yellow, 500) || L <- [1, 2]],

    config(),
    {ok, Hera_pid} = hera:start_measure(hera_interface, []), % Start measurement using hera interface
    persistent_term:put(hera_pid, Hera_pid),
    {ok, Supervisor}. % Return ok and the supervisor

% Application stop callback
stop(_State) -> ok.

%============================================================================================================================================
%========================================================= CONFIG TIME FUNC =================================================================
%============================================================================================================================================

config() ->
    %who_am_i(),

    pmod_nav:config(acc, #{odr_g => {hz,238}}), % Configure PMOD Nav accelerometer with output data rate of 238 Hz
    numerl:init(), % Initialize numerl library
    _ = grisp:add_device(spi2, pmod_nav), % Add PMOD Nav to Grisp Card on port SPI2

    I2Cbus = grisp_i2c:open(i2c1),
    persistent_term:put(i2c, I2Cbus),

    hera_sub:subscribe(self()),
    
    ok.

who_am_i() ->
    persistent_term:put(sensor_name, robot),
    await_connection().

await_connection() ->
    % Waiting for HERA to notify succesful connection
    % @param Id : Sensor's Id set by the jumpers (Integer)
    io:format("[ROBOT] WiFi setup starting...~n"),
    receive        
        {hera_notify, "connected"} -> % Received when hera_com managed to connect to the network
            io:format("[ROBOT] WiFi setup done~n~n"),
            [grisp_led:flash(L, white, 1000) || L <- [1, 2]],
            discover_server()
    after 18000 ->
        io:format("[ROBOT] WiFi setup failed:~n~n"),
        [grisp_led:flash(L, red, 750) || L <- [1, 2]],
        await_connection()
    end.

discover_server() ->
    % Waits forever until the server sends a Ping
    % @param Id : Sensor's Id set by the jumpers (Integer)
    io:format("[ROBOT] Waiting for ping from server~n"),
    receive
        {hera_notify, ["ping", Name, SIp, Port]} -> % Received upon server ping reception
            {ok, Ip} = inet:parse_address(SIp),
            IntPort = list_to_integer(Port),
            hera_com:add_device(list_to_atom(Name), Ip, IntPort),
            ack_loop()
    after 9000 ->
        io:format("[ROBOT] no ping from server~n"),
        discover_server()
    end.

ack_loop() ->
    % Tries to pair with the server by a Hello -> Ack
    % @param Id : Sensor's Id set by the jumpers (Integer)
    send_udp_message(server, "Hello from robot", "UTF8"),
    receive
        {hera_notify, ["Ack", _]} -> % Ensures the discovery of the sensor by the server
            io:format("[ROBOT] Received ACK from server~n"),
            [grisp_led:flash(L, green, 1000) || L <- [1, 2]],
            ok
    after 5000 ->
        ack_loop()
    end.

send_udp_message(Name, Message, Type) ->
    % Sends message
    % @param Name : name of the device to send to (atom)
    % @param Message : message to be sent (String/Tuple)
    % @param Type : type of message, can be UTF8 or Binary (String)
    hera_com:send_unicast(Name, Message, Type).
    

%============================================================================================================================================
%========================================================= LOOP =============================================================================
%============================================================================================================================================

robot_init() ->
    process_flag(priority, max), %Sets process priority to max
    Init_time = erlang:system_time()/1.0e6,

    {ok, _, Gy0, _} = calibrate(),

    % Initialize Kalman filter matrices
    X0 = mat:matrix([[0], [0]]), % Initial state matrix
    P0 = mat:matrix([[0.1, 0], [0, 0.1]]), % Initial covariance matrix

    % Initialize PID controllers for speed and stability
    Pid_Speed = spawn(pid_controller, pid_init, [-0.12, -0.07, 0.0, -1, 60.0, 0.0]),
    Pid_Stability = spawn(pid_controller, pid_init, [17.0, 0.0, 4.0, -1, -1, 0.0]),

    io:format("[Robot] Pid of the speed controller: ~p.~n", [Pid_Speed]),
    io:format("[Robot] Pid of the stability controller: ~p.~n", [Pid_Stability]),
    io:format("[Robot] Starting movement of the robot.~n"),

    % State definition
    Robot_state = #{
    init_time => Init_time,
    robot_state => {rest, false},
    kalman_state => {Init_time, X0, P0},
    logging_state => {0, Init_time, []},
    gyroscope_comp_filter => {Gy0, 0.0, 0.0},
    pid_controllers =>{Pid_Speed, Pid_Stability},
    movement_controls => {0.0, 0.0},
    frequency_var => {0, 0, 200.0, Init_time}
    },

    robot_main(Robot_State).

robot_main(Start_Time, {Robot_State, Robot_Up}, {T0, X0, P0}, {Logging, Log_End, Log_List}, {Gy0, Angle_Complem, Angle_Rate}, {Pid_Speed, Pid_Stability}, {Adv_V_Ref, Turn_V_Ref}, {N, Freq, Mean_Freq, T_End}) ->

    Hera_pid = persistent_term:get(hera_pid),

    Dt = get_delta_time(State),

    %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Angle Computations %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%

    % Compute the angle based directly on the accelerometer sensor data.
    % The angle is calculated using the arctangent of the ratio of the Z-axis
    % acceleration to the negative X-axis acceleration, converted from radians to degrees.
    Angle_Accelerometer = math:atan(Az / (-Ax)) * ?RAD_TO_DEG,

    % Perform Kalman filter computation to estimate the angle.
    % The Kalman filter uses the current sensor data (accelerometer and gyroscope)
    % and the previous state (X0, P0) to compute the new state (X1, P1).
    {X1, P1} = helper_module:kalman_angle(Dt, Ax, Az, Gy, Gy0, X0, P0),
    [Th_Kalman, _W_Kalman] = mat:to_array(X1), % Extract the angle (Th_Kalman) from the state matrix.
    Angle_Kalman = Th_Kalman * ?RAD_TO_DEG,   % Convert the angle from radians to degrees.

    % Compute the complementary filter angle.
    % The complementary filter combines gyroscope and accelerometer data to estimate the angle.
    % The filter uses a weighting factor (K) based on the mean frequency of the loop.
    K = 1.25 / (1.25 + (1.0 / Mean_Freq)), % Compute the weighting factor.
    {Angle_Complem_New, Angle_Rate_New} = helper_module:complem_angle({Dt, Ax, Az, Gy, Gy0, K, Angle_Complem, Angle_Rate}),

    % Select the angle to use for further computations.
    % The selection is based on the Switch flag:
    % - If Switch is true, use the Kalman filter angle.
    % - Otherwise, use the complementary filter angle.
    Angle = helper_module:select_angle(Switch, Angle_Kalman, Angle_Complem),


    %%%%%%%%%%%%%%%%%%
    %%% Controller %%%
    %%%%%%%%%%%%%%%%%%

    % Call the stability engine controller to compute motor acceleration and velocity references.
    % Inputs:
    % - Measures: Time delta (Dt), current angle (Angle), and current speed (Speed).
    % - Process IDs of the two PID controllers: Pid_Speed and Pid_Stability.
    % - Advance and turning velocity goals: Adv_V_Goal and Turn_V_Goal.
    % - Loopback velocity references: Adv_V_Ref and Turn_V_Ref.
    % Outputs:
    % - Acc: Acceleration of the motors, which will be sent to the ESP32.
    % - Adv_V_Ref_New: Updated advance velocity reference for the next loop iteration.
    % - Turn_V_Ref_New: Updated turning velocity reference for the next loop iteration, also sent to the ESP32.
    {Acc, Adv_V_Ref_New, Turn_V_Ref_New} = stability_engine:controller(
        {Dt, Angle, Speed}, 
        {Pid_Speed, Pid_Stability}, 
        {Adv_V_Goal, Adv_V_Ref}, 
        {Turn_V_Goal, Turn_V_Ref}
    ),

    %%%%%%%%%%%%%%%%%%%%%%%
    %%% Output to ESP32 %%%
    %%%%%%%%%%%%%%%%%%%%%%%
    
    % Determine the state of the robot (whether it is upright or not)
    % - If the robot is currently upright (Robot_Up is true) and the absolute angle exceeds 20 degrees,
    %   it is considered to have fallen, so Robot_Up_New is set to false.
    % - If the robot is not upright (Robot_Up is false) and the absolute angle is less than 18 degrees,
    %   it is considered to have recovered, so Robot_Up_New is set to true.
    % - Otherwise, the state remains unchanged.
    if 
        Robot_Up and (abs(Angle) > 20) ->
            Robot_Up_New = false;
        not Robot_Up and (abs(Angle) < 18) -> 
            Robot_Up_New = true;
        true ->
            Robot_Up_New = Robot_Up
    end,

    % Determine the forward/backward direction of the robot based on the angle.
    % If the angle is positive, the robot is moving forward, so F_B is set to 1.
    % Otherwise, it is moving backward or stationary, so F_B is set to 0.
    if
        Angle > 0.0 ->
            F_B = 1;
        true ->
            F_B = 0
    end,

    % State change logic for the robot's operation
    case Robot_State of
        % If the robot is in the "rest" state:
        rest -> 
            if
                % Transition to "raising" state if the Get_Up flag is true
                Get_Up -> Next_Robot_State = raising;
                % Otherwise, remain in the "rest" state
                true -> Next_Robot_State = rest
            end;
        % If the robot is in the "raising" state:
        raising -> 
            if
                % Transition to "stand_up" state if the robot is upright
                Robot_Up -> Next_Robot_State = stand_up;
                % Transition to "soft_fall" state if the Get_Up flag is false
                not Get_Up -> Next_Robot_State = soft_fall;
                % Otherwise, remain in the "raising" state
                true -> Next_Robot_State = raising
            end;
        % If the robot is in the "stand_up" state:
        stand_up -> 
            if
                % Transition to "wait_for_extend" state if the Get_Up flag is false
                not Get_Up -> Next_Robot_State = wait_for_extend;
                % Transition to "rest" state if the robot is no longer upright
                not Robot_Up -> Next_Robot_State = rest;
                % Otherwise, remain in the "stand_up" state
                true -> Next_Robot_State = stand_up
            end;
        % If the robot is in the "wait_for_extend" state:
        wait_for_extend -> 
            % Transition to "prepare_arms" state
            Next_Robot_State = prepare_arms;
        % If the robot is in the "prepare_arms" state:
        prepare_arms -> 
            if
                % Transition to "free_fall" state if the arms are ready
                Arm_Ready -> Next_Robot_State = free_fall;
                % Transition to "stand_up" state if the Get_Up flag is true
                Get_Up -> Next_Robot_State = stand_up;
                % Transition to "rest" state if the robot is no longer upright
                not Robot_Up -> Next_Robot_State = rest;
                % Otherwise, remain in the "prepare_arms" state
                true -> Next_Robot_State = prepare_arms
            end;
        % If the robot is in the "free_fall" state:
        free_fall -> 
            if
                % Transition to "wait_for_retract" state if the angle exceeds 10 degrees
                abs(Angle) > 10 -> Next_Robot_State = wait_for_retract;
                % Otherwise, remain in the "free_fall" state
                true -> Next_Robot_State = free_fall
            end;
        % If the robot is in the "wait_for_retract" state:
        wait_for_retract -> 
            % Transition to "soft_fall" state
            Next_Robot_State = soft_fall;
        % If the robot is in the "soft_fall" state:
        soft_fall -> 
            if
                % Transition to "rest" state if the arms are ready
                Arm_Ready -> Next_Robot_State = rest;
                % Transition to "raising" state if the Get_Up flag is true
                Get_Up -> Next_Robot_State = raising;
                % Otherwise, remain in the "soft_fall" state
                true -> Next_Robot_State = soft_fall
            end
    end,

    % Determine the output state of the robot based on its current state
    {Power, Freeze, Extend, Robot_Up_Bit} = 
        case Next_Robot_State of
            rest -> 
                {0, 0, 0, 0}; % No power, no freeze, no extension, robot is not upright
            raising -> 
                {1, 0, 1, 0}; % Power is on, no freeze, extension is active, robot is not upright
            stand_up -> 
                {1, 0, 0, 1}; % Power is on, no freeze, no extension, robot is upright
            wait_for_extend -> 
                {1, 0, 1, 1}; % Power is on, no freeze, extension is active, robot is upright
            prepare_arms -> 
                {1, 0, 1, 1}; % Power is on, no freeze, extension is active, robot is upright
            free_fall -> 
                {1, 1, 1, 1}; % Power is on, freeze is active, extension is active, robot is upright
            wait_for_retract -> 
                {1, 0, 0, 0}; % Power is on, no freeze, no extension, robot is not upright
            soft_fall -> 
                {1, 0, 0, 0}  % Power is on, no freeze, no extension, robot is not upright
        end,

    % Send output to ESP32
    % Construct the output byte to send to the ESP32.
    % The byte is created by combining the following bits:
    % - Power: Indicates whether the robot's power is on.
    % - Freeze: Indicates whether the robot's motors are frozen.
    % - Extend: Indicates whether the robot's arms are extended.
    % - Robot_Up_Bit: Indicates whether the robot is upright.
    % - F_B: Indicates the forward/backward direction of the robot.
    % The remaining bits are set to 0.
    Output_Byte = helper_module:get_byte([Power, Freeze, Extend, Robot_Up_Bit, F_B, 0, 0, 0]),

    % Encode the acceleration and turning velocity reference as half-float values.
    % These values are sent to the ESP32 for motor control.
    [HF1, HF2] = hera_com:encode_half_float([Acc, Turn_V_Ref_New]),

    % Send the encoded data and the output byte to the ESP32 via the I2C bus.
    % The data is written to the ESP32's address (16#40) with the specified format.
    grisp_i2c:transfer(I2Cbus, [{write, 16#40, 1, [HF1, HF2, <<Output_Byte>>]}]),

    %%%%%%%%%%%%%%%%%%%%%%%%
    %%% Testing Features %%%
    %%%%%%%%%%%%%%%%%%%%%%%%

    % Compute the frequency of the loop and update the frequency-related variables.
    % This helps monitor the loop's execution speed and adjust it if necessary.
    {N_New, Freq_New, Mean_Freq_New} = helper_module:frequency_computation(Dt, N, Freq, Mean_Freq),

    % Determine the end time for logging based on the test mode.
    % If the robot is in test mode (`Test` is true), set the logging duration to a predefined value.
    % Otherwise, retain the previous logging end time.
    if 
        Test ->
            Log_End_New = erlang:system_time()/1.0e6 + ?LOG_DURATION;
        true -> 
            Log_End_New = Log_End
    end,

    % Check if logging is still active by comparing the current time with the logging end time.
    % If the current time is less than the logging end time, logging remains active.
    Logging_New = erlang:system_time()/1.0e6 < Log_End_New,

    % Control the LED flickering behavior while logging.
    % If logging is active, the LEDs flicker in a pattern based on the loop count (`N`).
    % - For `N rem 9 < 4`, the LEDs are set to yellow.
    % - Otherwise, the LEDs are turned off.
    if 
        Logging_New -> 
            if 
                N rem 9 < 4 -> 
                    grisp_led:color(1, {1, 1, 0}), % Set LED 1 to yellow
                    grisp_led:color(2, {1, 1, 0}); % Set LED 2 to yellow
                true->
                    grisp_led:color(1, {0, 0, 0}), % Turn off LED 1
                    grisp_led:color(2, {0, 0, 0})  % Turn off LED 2
            end;
        true ->
            ok
    end,

    % Check for the start or end of a logging sequence
    % - If logging was previously inactive (`not Logging`) and is now active (`Logging_New`),
    %   send a `start_log` message to the Hera process to initiate logging.
    % - If logging was previously active (`Logging`) and is now inactive (`not Logging_New`),
    %   set the LEDs to yellow and send a `stop_log` message to the Hera process to stop logging.
    % - Otherwise, do nothing.
    if
        not Logging and Logging_New ->
            Hera_pid ! {self(), start_log}; % Notify Hera process to start logging
        Logging and not Logging_New ->
            grisp_led:color(1, {1, 1, 0}), % Set LED 1 to yellow
            grisp_led:color(2, {1, 1, 0}), % Set LED 2 to yellow
            Hera_pid ! {self(), stop_log}; % Notify Hera process to stop logging
        true ->
            ok % No change in logging state
    end,

    % Send values to ESP32 if requested, otherwise accumulate values in the log list
    receive
        % If a message with the tag `log_values` is received:
        % - Send the current log list to the requesting process (`From`).
        % - Reset the log list to an empty list for the next logging cycle.
        {From, log_values} ->  
            From ! {self(), log, Log_List},
            Log_List_New = []
    after 0 ->
        % If no message is received, check if logging is active.
        if
            Logging_New ->
                % If logging is active, prepend the current data to the log list.
                % The data includes:
                % - Time since the robot started (`T1-Start_Time`).
                % - Loop frequency (`1/Dt`).
                % - Gyroscope reading (`Gy`).
                % - Accelerometer reading (`Acc`).
                % - Control byte (`CtrlByte`).
                % - Angles from different computations (accelerometer, Kalman filter, complementary filter).
                % - Advance velocity reference (`Adv_V_Ref`).
                % - Switch state (`Switch`).
                % - Updated velocity references (`Adv_V_Ref_New`, `Turn_V_Ref_New`).
                % - Current speed (`Speed`).
                Log_List_New = [[T1-Start_Time, 1/Dt, Gy, Acc, CtrlByte, -Angle_Accelerometer, -Angle_Kalman, -Angle_Complem, Adv_V_Ref, Switch, Adv_V_Ref_New, Turn_V_Ref_New, Speed] | Log_List];
            true ->
                % If logging is not active, keep the log list unchanged.
                Log_List_New = Log_List
        end
    end,

    % Communication with Hera (more messages can be implemented by the user)
    receive
        % Handle a request to get all data
        {From1, get_all_data} -> 
            From1 ! {self(), data, [T1-Start_Time, 1/Dt, Gy, Acc, CtrlByte, -Angle_Accelerometer, -Angle_Kalman, -Angle_Complem, Adv_V_Ref, Switch, Adv_V_Ref_New, Turn_V_Ref_New, Speed]};

        % Handle a request to get the loop frequency
        {From1, freq} -> 
            From1 ! {self(), 1/Dt};

        % Handle a request to get the accelerometer reading
        {From2, acc} -> 
            From2 ! {self(), Acc};

        % Handle unrecognized messages
        {_, Msg} -> 
            io:format("[Robot] Message [~p] not recognized. ~nPossible queries are: [get_all_data, freq, acc]. ~nMore queries can be added.", [Msg])
    after 0 ->
        % If no message is received, do nothing and continue.
        ok
    end,


    % Imposed maximum frequency
    % Get the current system time in milliseconds
    T2 = erlang:system_time()/1.0e6,

    % Calculate the desired delay between loop iterations based on the frequency goal
    Delay_Goal = 1.0 / ?FREQ_GOAL * 1000.0,

    % Check if the elapsed time since the last loop iteration is less than the desired delay
    if
        T2 - T_End < Delay_Goal ->
            % If the elapsed time is less, wait for the remaining time to meet the desired delay
            helper_module:wait(Delay_Goal - (T2 - T1));
        true ->
            % Otherwise, proceed without waiting
            ok
    end,

    % Update the end time for the current loop iteration
    T_End_New = erlang:system_time()/1.0e6,

    %Loop back with updated state
    robot_main(Start_Time, {Next_Robot_State, Robot_Up_New}, {T1, X1, P1}, {Logging_New, Log_End_New, Log_List_New}, {Gy0, Angle_Complem_New, Angle_Rate_New}, {Pid_Speed, Pid_Stability}, {Adv_V_Ref_New, Turn_V_Ref_New}, {N_New, Freq_New, Mean_Freq_New, T_End_New}).

%============================================================================================================================================
%========================================================= HELPER FUNCTION ==================================================================
%============================================================================================================================================

get_delta_time(State) ->
    {T0, _, _} = maps:get(kalman_state, State),
    T1 = erlang:system_time()/1.0e6,
    Dt = (T1 - T0)/1000.0,
    Dt.

    
i2C_data() ->
    I2Cbus = persistent_term:get(i2c),
    % Receive data from the ESP32 via I2C and decode it
    [<<SL1,SL2,SR1,SR2,CtrlByte>>] = grisp_i2c:transfer(I2Cbus, [{read, 16#40, 1, 5}]),
    
    % Decode the half-float values for left and right wheel speeds
    [Speed_L, Speed_R] = hera_com:decode_half_float([<<SL1, SL2>>, <<SR1, SR2>>]),
    
    % Compute the average speed of the robot based on the left and right wheel speeds
    Speed = (Speed_L + Speed_R) / 2,

    [Arm_Ready, Switch, Test, Get_Up, Forward, Backward, Left, Right] = hera_com:get_bits(CtrlByte), % Get General command from LilyGo

    Adv_V_Goal = helper_module:speed_ref(Forward, Backward), % Get movement goal

    Turn_V_Goal = helper_module:turn_ref(Left, Right). % Get rotation goal