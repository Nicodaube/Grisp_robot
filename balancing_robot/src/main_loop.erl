-module(main_loop).

-export([robot_init/1, modify_frequency/1]).

-define(RAD_TO_DEG, 180.0/math:pi()). % Conversion factor from radians to degrees

-define(INCH_TO_CM, 2.54). % Conversion factor from inch to cm

%Duration of a logging sequence
-define(LOG_DURATION, 15000).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Robot 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

robot_init(Hera_pid) ->

    % Set process priority to maximum for time-critical operations. (adds CPU performance)
    process_flag(priority, max),

    % Starting timestamp for the robot in ms
    T0 = erlang:system_time()/1.0e6,

    % Create a table for global variables and set the default frequency goal
    %% The code initializes an ETS (Erlang Term Storage) table named `variables`.
    %% 
    %% - The table is created with the following options:
    %%   - `set`: Specifies that the table is a set, meaning each key is unique.
    %%   - `public`: Makes the table accessible to all processes.
    %%   - `named_table`: Allows the table to be referenced by its name (`variables`) instead of a table identifier.
    %%
    %% - After creation, an entry is inserted into the table:
    %%   - Key: `"Freq_Goal"` (a string).
    %%   - Value: `300.0` (a floating-point number).
    %%
    %% The resulting table structure is as follows:
    %%   - Table Name: `variables`
    %%   - Key-Value Pairs:
    %%     - `"Freq_Goal"` => `300.0`
    ets:new(variables, [set, public, named_table]),
    ets:insert(variables, {"Freq_Goal", 300.0}),

    % Calibration process for the gyroscope
    io:format("[Robot] Calibrating... Do not move the pmod_nav!~n"),
    grisp_led:color(1, {1, 0, 0}), % Set LED 1 to red during calibration
    grisp_led:color(2, {1, 0, 0}), % Set LED 2 to red during calibration
    [_Gx0, Gy0, _Gz0] = helper_module:calibrate(), % Perform calibration
    io:format("[Robot] Done calibrating~n"),

    % Initialize Kalman filter matrices
    X0 = mat:matrix([[0], [0]]), % Initial state matrix
    P0 = mat:matrix([[0.1, 0], [0, 0.1]]), % Initial covariance matrix

    % Open the I2C bus for communication
    I2Cbus = grisp_i2c:open(i2c1),

    % Initialize PID controllers for speed and stability
    Pid_Speed = spawn(pid_controller, pid_init, [-0.12, -0.07, 0.0, -1, 60.0, 0.0]),
    Pid_Stability = spawn(pid_controller, pid_init, [17.0, 0.0, 4.0, -1, -1, 0.0]),

    % ============================ MODIFIED CODE ============================

    % TODO: Nouvelle boucle qui controle la vitesse du robot pour ralentir devant un obstacle

    % =======================================================================

    io:format("[Robot] Pid of the speed controller: ~p.~n", [Pid_Speed]),
    io:format("[Robot] Pid of the stability controller: ~p.~n", [Pid_Stability]),
    io:format("[Robot] Starting movement of the robot.~n"),

    % Call the main loop of the robot with initial parameters
    robot_main(
        T0, Hera_pid, 
        {rest, false}, % Initial robot state and "robot up" status
        {T0, X0, P0}, % Initial time, state, and covariance
        I2Cbus, % I2C bus handle
        {0, T0, []}, % Logging state, end time, and log list
        {Gy0, 0.0, 0.0}, % Gyroscope offset and complementary filter state
        {Pid_Speed, Pid_Stability}, % PID controllers
        {0.0, 0.0}, % Initial advance and turning velocity references
        {0, 0, 200.0, T0} % Frequency computation parameters
    ).

%% @doc Main loop function for the robot's operation.
%% @spec robot_main(
%%           Start_Time :: integer(),
%%           Hera_pid :: pid(),
%%           Robot_State_Up :: {atom(), boolean()},
%%           Sensor_Data :: {float(), float(), float()},
%%           I2Cbus :: term(),
%%           Logging_Info :: {boolean(), term(), list()},
%%           Gyroscope_Data :: {float(), float(), float()},
%%           PID_Controllers :: {term(), term()},
%%           Velocity_Refs :: {float(), float()},
%%           Loop_Info :: {integer(), float(), float(), float()}
%%       ) -> term()
%% @param Start_Time The initial timestamp when the robot started, in milliseconds.
%% @param Hera_pid The process identifier (PID) of the Hera process.
%% @param Robot_State_Up A tuple containing the robot's state and whether it is upright.
%% @param Sensor_Data A tuple with the initial sensor readings: time, position, and orientation.
%% @param I2Cbus The I2C bus reference used for communication with sensors.
%% @param Logging_Info A tuple containing logging configuration: whether logging is enabled, 
%%                     the logging end condition, and the list of logged data.
%% @param Gyroscope_Data A tuple with gyroscope data: initial gyroscope reading, complementary angle, 
%%                       and angular rate.
%% @param PID_Controllers A tuple containing the PID controller configurations for speed and stability.
%% @param Velocity_Refs A tuple with the advanced velocity reference and turning velocity reference.
%% @param Loop_Info A tuple containing loop-related information: loop count, frequency, mean frequency, 
%%                  and the end time of the loop.
%% @return The function does not explicitly return a value but performs the main operations of the robot's loop.
robot_main(Start_Time, Hera_pid, {Robot_State, Robot_Up}, {T0, X0, P0}, I2Cbus, {Logging, Log_End, Log_List}, {Gy0, Angle_Complem, Angle_Rate}, {Pid_Speed, Pid_Stability}, {Adv_V_Ref, Turn_V_Ref}, {N, Freq, Mean_Freq, T_End}) ->

    %Delta time of loop
    T1 = erlang:system_time()/1.0e6, %[ms]
    Dt = (T1 - T0)/1000.0,           %[s]

    %%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Input from Sensor %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%

    %% Reads data from the accelerometer and gyroscope.
    %% The gyroscope data (Gy) represents the angular velocity, typically measured in degrees per second (°/s).
    %% The accelerometer data includes:
    %% - Ax: Acceleration along the X-axis, typically measured in meters per second squared (m/s²).
    %% - Az: Acceleration along the Z-axis, typically measured in meters per second squared (m/s²).
    %Read data from the accelerometer and gyroscope
    [Gy, Ax, Az] = pmod_nav:read(acc, [out_y_g, out_x_xl, out_z_xl], #{g_unit => dps}),
    
    % ============================ MODIFIED CODE ============================
    % Mesures doivent etre faites en alternance pour eviter les interferences entre plusieurs capteurs
    % Ajouter une calibration sur les sonars pour eviter les fausses mesures

    % Get the distance from the sonar sensor
    Distance_Sonar = pmod_maxsonar:get(), % return distance in inch to the closest object detected 
    Distance_Sonar = Distance_Sonar * ?INCH_TO_CM, % convert distance from inch to cm

    % TODO: Filter noise for huge peaks 
    

    % Log the measured distance from the sonar sensor
    io:format("[SONAR SENSOR] Measured distance: ~p ~n cm", [Distance_Sonar]),

    % =======================================================================

    %%%%%%%%%%%%%%%%%%%%%%%%
    %%% Input from ESP32 %%%
    %%%%%%%%%%%%%%%%%%%%%%%%

    % Receive data from the ESP32 via I2C and decode it
    [<<SL1,SL2,SR1,SR2,CtrlByte>>] = grisp_i2c:transfer(I2Cbus, [{read, 16#40, 1, 5}]),
    
    % Decode the half-float values for left and right wheel speeds
    [Speed_L, Speed_R] = hera_com:decode_half_float([<<SL1, SL2>>, <<SR1, SR2>>]),
    
    % Compute the average speed of the robot based on the left and right wheel speeds
    Speed = (Speed_L + Speed_R) / 2,

    % Retrieve control flags from the ESP32
    % Flags include:
    % - Arm_Ready: Indicates if the robot's arms are ready
    % - Switch: A general-purpose switch flag
    % - Test: Indicates if the robot is in test mode
    % - Get_Up: Command to make the robot stand up
    % - Forward, Backward: Flags for forward and backward movement
    % - Left, Right: Flags for turning left and right
    [Arm_Ready, Switch, Test, Get_Up, Forward, Backward, Left, Right] = hera_com:get_bits(CtrlByte),

    %%%%%%%%%%%%%%%%%%%%%
    %%% Command Logic %%%
    %%%%%%%%%%%%%%%%%%%%%

    % Set the advance speed goal based on the Forward and Backward flags.
    % If the Forward flag is true, the robot moves forward at maximum speed.
    % If the Backward flag is true, the robot moves backward at maximum speed.
    % Otherwise, the advance speed goal is set to 0 (no movement).
    Adv_V_Goal = helper_module:speed_ref(Forward, Backward),

    % Set the turning speed goal based on the Left and Right flags.
    % If the Left flag is true, the robot turns left at maximum speed.
    % If the Right flag is true, the robot turns right at maximum speed.
    % Otherwise, the turning speed goal is set to 0 (no turning).
    Turn_V_Goal = helper_module:turn_ref(Left, Right),

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

    % ============================ MODIFIED CODE ============================
    
    % TODO: Add the sonar data to the filter to adapt the speed based on the environment
   
    % =======================================================================


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

    % Retrieve the desired frequency goal from the ETS table
    [{_, Freq_Goal}] = ets:lookup(variables, "Freq_Goal"),

    % Calculate the desired delay between loop iterations based on the frequency goal
    Delay_Goal = 1.0 / Freq_Goal * 1000.0,

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
    robot_main(Start_Time, Hera_pid, {Next_Robot_State, Robot_Up_New}, {T1, X1, P1}, I2Cbus, {Logging_New, Log_End_New, Log_List_New}, {Gy0, Angle_Complem_New, Angle_Rate_New}, {Pid_Speed, Pid_Stability}, {Adv_V_Ref_New, Turn_V_Ref_New}, {N_New, Freq_New, Mean_Freq_New, T_End_New}).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Global variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Updates the desired frequency goal in the ETS table.
%% This function modifies the value associated with the key `"Freq_Goal"`
%% in the `variables` ETS table to the provided frequency value.
modify_frequency(Freq) ->
    ets:insert(variables, {"Freq_Goal", Freq}),
    ok.