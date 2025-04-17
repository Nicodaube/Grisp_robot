-module(helper_module).

-export([
    round/2, calibrate/0, kalman_angle/8, complem_angle/1, select_angle/3,
    speed_ref/2, turn_ref/2, frequency_computation/4, wait/1, get_byte/1,
    enforce_loop_frequency/1, flicker_led/2, manage_logging_transition/3,
    update_log_list/3, handle_incoming_messages/1, robot_state_transition/5,
    robot_output_state/1, get_grisp_id/0
]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Macros
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

-define(DEG_TO_RAD, math:pi() / 180).
-define(RAD_TO_DEG, 180.0 / math:pi()).

-define(ADV_V_MAX, 30.0).     % Maximum forward/backward speed in cm/s
-define(TURN_V_MAX, 80.0).    % Maximum turning speed in deg/s
-define(COEF_FILTER, 0.667).  % Complementary filter weighting
-define(FREQ_WINDOW, 100).    % Window size for frequency smoothing
-define(MS_TO_S, 1.0e-3).     % Milliseconds to seconds conversion

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Math helpers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Rounds a floating-point number to a specified number of decimal places.
%% @param Number - The number to round.
%% @param Precision - The number of decimal places to round to.
%% @return - The rounded number.
round(Number, Precision) ->
    Power = math:pow(10, Precision),
    round(Number * Power) / Power.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Sensor Calibration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Calibrates the gyroscope by averaging N samples of raw sensor data.
%% @return - A tuple containing the average gyroscope readings for the x, y, and z axes.
calibrate() ->
    N = 500,
    Data = [list_to_tuple(pmod_nav:read(acc, [out_x_g, out_y_g, out_z_g])) || _ <- lists:seq(1, N)],
    {X, Y, Z} = lists:unzip3(Data),
    [lists:sum(X) / N, lists:sum(Y) / N, lists:sum(Z) / N].

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Angle estimation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Extended Kalman Filter for fusing accelerometer, gyroscope, and sonar measurements.
%% - Predicts the next state based on the gyroscope data and updates it with the accelerometer and sonar data.
%% @param {Dt, Ax, Az, Gy, Gy0, X0, P0, D_sonar} - A tuple containing the time delta (Dt), accelerometer readings (Ax, Az),
%%   gyroscope readings (Gy, Gy0), initial state (X0), initial covariance (P0), and sonar distance (D_sonar).
%% @return - The updated state and covariance matrix after applying the Kalman filter.
kalman_angle(Dt, Ax, Az, Gy, Gy0, X0, P0, D_sonar) ->
    % Measurement noise covariance matrix (confidence in sensors)
    R = mat:matrix([[3.0, 0.0, 0.0],
                    [0.0, 3.0e-6, 0.0],
                    [0.0, 0.0, 25.0]]),

    % Process noise covariance matrix (confidence in model prediction)
    Q = mat:matrix([[3.0e-5, 0.0, 0.0],
                    [0.0, 10.0, 0.0],
                    [0.0, 0.0, 100.0]]),

    % Predict next state
    F = fun(X) ->
            [Th, W, D] = mat:to_array(X),
            mat:matrix([[Th + Dt * W], [W], [D]])
        end,

    % Jacobian of the prediction function
    Jf = fun(_) ->
            mat:matrix([[1, Dt, 0],
                        [0, 1, 0],
                        [0, 0, 1]])
         end,

    % Measurement function: maps internal state to expected measurements
    H = fun(X) ->
            [Th, W, D] = mat:to_array(X),
            mat:matrix([[Th], [W], [D]])
        end,

    % Jacobian of the measurement function
    Jh = fun(_) ->
            mat:matrix([[1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]])
         end,

    % Build measurement vector
    Z = mat:matrix([
        [math:atan(Az / (-Ax))],
        [(Gy - Gy0) * ?DEG_TO_RAD],
        [D_sonar]
    ]),

    kalman:ekf({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z).

%% @doc Complementary filter combining gyroscope integration and accelerometer absolute angle.
%% - High-frequency response from gyroscope (short-term stability).
%% - Low-frequency correction from accelerometer (long-term drift correction).
%% @param {Dt, Ax, Az, Gy, Gy0, K, Angle_Complem, Angle_Rate} - A tuple containing the time delta (Dt), accelerometer readings (Ax, Az),
%%   gyroscope readings (Gy, Gy0), filter coefficient (K), current angle (Angle_Complem), and angular rate (Angle_Rate).
%% @return - The updated angle and angular rate after applying the complementary filter.
complem_angle({Dt, Ax, Az, Gy, Gy0, K, Angle_Complem, Angle_Rate}) ->
    % Low-pass filter on gyroscope angular rate
    Angle_Rate_New = (Gy - Gy0) * ?COEF_FILTER + Angle_Rate * (1 - ?COEF_FILTER),
    Delta_Gyr = Angle_Rate_New * Dt,
    Angle_Acc = math:atan(Az / (-Ax)) * ?RAD_TO_DEG,
    % Weighted combination
    Angle_Complem_New = (Angle_Complem + Delta_Gyr) * K + Angle_Acc * (1 - K),
    {Angle_Complem_New, Angle_Rate_New}.

%% @doc Selects the angle estimation method.
%% @param Switch - Boolean flag to select the angle estimation method.
%% @param Angle_Kalman - The angle estimated by the Kalman filter.
%% @param Angle_Complem - The angle estimated by the complementary filter.
%% @return - The selected angle based on the `Switch` parameter.
select_angle(true, Angle_Kalman, _Angle_Complem) -> Angle_Kalman;
select_angle(false, _Angle_Kalman, Angle_Complem) -> Angle_Complem.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Motion reference helpers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Computes the target advance velocity based on forward/backward control flags.
%% @param Forward - Boolean flag for forward motion.
%% @param Backward - Boolean flag for backward motion.
%% @return - The target advance velocity based on the control flags.
speed_ref(true, _) -> ?ADV_V_MAX;
speed_ref(_, true) -> -?ADV_V_MAX;
speed_ref(_, _)    -> 0.0.

%% @doc Computes the target turning velocity based on left/right control flags.
%% @param Left - Boolean flag for left turn.
%% @param Right - Boolean flag for right turn.
%% @return - The target turning velocity based on the control flags.
turn_ref(true, false) -> -?TURN_V_MAX;
turn_ref(false, true) -> ?TURN_V_MAX;
turn_ref(_, _)        -> 0.0.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Timing helpers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Computes running mean loop frequency based on elapsed time between iterations.
%% - Resets every ?FREQ_WINDOW iterations to avoid accumulating floating point errors.
%% @param Dt - Elapsed time since last iteration.
%% @param N - Number of iterations since last reset.
%% @param Freq - Current frequency estimate.
%% @param Mean_Freq - Running mean frequency.
%% @return - A tuple containing the updated number of iterations, frequency estimate, and running mean frequency.
frequency_computation(Dt, N, Freq, Mean_Freq) ->
    if
        N == ?FREQ_WINDOW ->
            {0, 0, Freq};  % Reset window
        true ->
            {N + 1, ((Freq * N) + (1 / Dt)) / (N + 1), Mean_Freq}
    end.

%% @doc Active wait for T milliseconds.
%% Busy-waits by continuously checking the system clock.
%% @param T_ms - Time in milliseconds to wait.
%% @return - Returns `ok` after the specified time has elapsed.
wait(T_ms) ->
    Tend = erlang:system_time() * ?MS_TO_S + T_ms,
    wait_help(erlang:system_time() * ?MS_TO_S, Tend).

%% @doc Helper function for active wait.
%% Continuously checks the system clock until the specified time has elapsed.
%% @param Tnow - Current time in seconds.
%% @param Tend - Target time in seconds.
%% @return - Returns `ok` if the current time is greater than or equal to the target time.
wait_help(Tnow, Tend) when Tnow >= Tend -> ok;
wait_help(_, Tend) ->
    wait_help(erlang:system_time() * ?MS_TO_S, Tend).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Bit manipulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Converts a list of 8 bits (MSB first) into a single byte.
%% @param [A, B, C, D, E, F, G, H] - A list of 8 bits.
%% @return - The byte value represented by the list of bits.
get_byte([A, B, C, D, E, F, G, H]) ->
    A*128 + B*64 + C*32 + D*16 + E*8 + F*4 + G*2 + H.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Loop frequency management
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Ensures the robot loop runs at the desired frequency by inserting active delays.
%% @param T_End - The end time of the previous loop iteration.
%% @return - Returns `ok` after enforcing the loop frequency.
enforce_loop_frequency(T_End) ->
    T_Now = erlang:system_time() / 1.0e6,

    case ets:lookup(variables, "Freq_Goal") of
        [{_, Freq_Goal}] ->
            Delay_Goal_ms = 1000.0 / Freq_Goal, % Target loop time in ms
            Elapsed_ms = T_Now - T_End,

            if
                Elapsed_ms < Delay_Goal_ms ->
                    helper_module:wait(Delay_Goal_ms - Elapsed_ms);
                true ->
                    ok
            end;
        [] ->
            % If Freq_Goal not found, do nothing
            ok
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Robot testing features
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Manages LED flickering pattern during active logging.
%% @param Logging - Boolean flag indicating whether logging is active.
%% @param N - The current loop iteration count.
%% @return - The updated LED state based on the logging status and loop count.
flicker_led(true, N) ->
    if
        N rem 9 < 4 ->
            grisp_led:color(1, {1, 1, 0}),
            grisp_led:color(2, {1, 1, 0});
        true ->
            grisp_led:color(1, {0, 0, 0}),
            grisp_led:color(2, {0, 0, 0})
    end;

flicker_led(false, _) ->
    ok.

%% @doc Handles the start and stop of logging based on state transitions.
%% @param Hera_pid - The process ID of the Hera process for logging.
%% @param Logging - The current logging state (true or false).
%% @param Logging_New - The new logging state after transition.
%% @return - The updated logging state based on the transition.
manage_logging_transition(Hera_pid, false, true) ->
    % Transition from not logging to logging
    Hera_pid ! {self(), start_log};

manage_logging_transition(Hera_pid, true, false) ->
    % Transition from logging to not logging
    grisp_led:color(1, {1, 1, 0}),
    grisp_led:color(2, {1, 1, 0}),
    Hera_pid ! {self(), stop_log};

manage_logging_transition(_, _, _) ->
    % No change
    ok.

%% @doc Updates the robot log list with a new data entry if logging is active.
%% @param Logging_New - The new logging state (true or false).
%% @param Log_List - The current log list.
%% @param New_Data - The new data entry to be added to the log list.
%% @return - The updated log list based on the logging state.
update_log_list(Logging_New, Log_List, New_Data) ->
    receive
        {From, log_values} ->  
            From ! {self(), log, Log_List}
    after 0 ->
        if
            Logging_New ->
                [New_Data | Log_List];
            true ->
                Log_List
        end
    end.

%% @doc Handles asynchronous incoming messages to the robot (Hera queries, debug commands, etc.)
%% @param Data_List - The list of data to be sent in response to incoming messages.
%% @return - Returns `ok` after processing the incoming messages.
handle_incoming_messages(Data_List) ->
    receive
        {From, get_all_data} ->
            From ! {self(), data, Data_List};

        {From, freq} ->
            [_, One_on_Dt, _, _, _, _, _, _, _, _, _, _, _] = Data_List,
            From ! {self(), One_on_Dt}; 

        {From, acc} ->
            [_, _, _, Acc, _, _, _, _, _, _, _, _, _] = Data_List,
            From ! {self(), Acc};

        {_, Msg} ->
            io:format("[Robot] Unknown message received: ~p~nSupported: [get_all_data, freq, acc]~n", [Msg])
    after 0 ->
        ok
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Robot State Management
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Handles state transitions for the robot based on sensors and commands.
%% @param Robot_State The current state of the robot.
%% @param Get_Up Flag indicating whether the robot should get up.
%% @param Robot_Up Flag indicating whether the robot is upright.
%% @param Arm_Ready Flag indicating whether the robot's arms are ready.
%% @param Abs_Angle The absolute angle of the robot.
%% @return The next state of the robot based on the current state and conditions.
robot_state_transition(Robot_State, Get_Up, Robot_Up, Arm_Ready, Abs_Angle) ->
    case Robot_State of
        rest ->
            if Get_Up -> raising; true -> rest end;
        
        raising ->
            if Robot_Up -> stand_up;
               not Get_Up -> soft_fall;
               true -> raising
            end;
        
        stand_up ->
            if not Get_Up -> wait_for_extend;
               not Robot_Up -> rest;
               true -> stand_up
            end;
        
        wait_for_extend ->
            prepare_arms;
        
        prepare_arms ->
            if Arm_Ready -> free_fall;
               Get_Up -> stand_up;
               not Robot_Up -> rest;
               true -> prepare_arms
            end;
        
        free_fall ->
            if Abs_Angle > 10 -> wait_for_retract;
               true -> free_fall
            end;
        
        wait_for_retract ->
            soft_fall;
        
        soft_fall ->
            if Arm_Ready -> rest;
               Get_Up -> raising;
               true -> soft_fall
            end
    end.

%% @doc Determines output command flags based on robot high-level state.
%% @param State The current state of the robot.
%% @return A tuple containing the output flags: {Power, Freeze, Extend, Robot_Up_Bit}.
robot_output_state(State) ->
    % Determine the output state of the robot based on its current state
    case State of
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
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Grisps identification
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Reads the jumper GPIO pins to identify the GRiSP board.
%% @return - A tuple containing the GRiSP ID based on the jumper settings.
get_grisp_id() ->
    %% Jumper GPIO pins
    JMP1 = grisp_gpio:open(jumper_1, #{mode => input}),
    JMP2 = grisp_gpio:open(jumper_2, #{mode => input}),
    JMP3 = grisp_gpio:open(jumper_3, #{mode => input}),
    JMP4 = grisp_gpio:open(jumper_4, #{mode => input}),
    JMP5 = grisp_gpio:open(jumper_5, #{mode => input}),

    %% Read the jumper GPIO pins
    V1 = grisp_gpio:get(JMP1),
    V2 = grisp_gpio:get(JMP2),
    V3 = grisp_gpio:get(JMP3),
    V4 = grisp_gpio:get(JMP4),
    V5 = grisp_gpio:get(JMP5),

    % Convert the jumper values to a single byte
    SUM = (V1) + (V2 bsl 1) + (V3 bsl 2) + (V4 bsl 3) + (V5 bsl 4),
    {ok, SUM}.
