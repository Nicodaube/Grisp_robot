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
    io:format("[ROBOT] Starting ...~n"),
    {ok, Supervisor} = balancing_robot_sup:start_link(), % Start the supervisor
    [grisp_led:flash(L, yellow, 500) || L <- [1, 2]],

    config(),
    {ok, Supervisor}. % Return ok and the supervisor

% Application stop callback
stop(_State) -> ok.

%============================================================================================================================================
%========================================================= CONFIG TIME FUNC =================================================================
%============================================================================================================================================

config() ->
    %who_am_i(),
    
    numerl:init(), % Initialize numerl library
    _ = grisp:add_device(spi2, pmod_nav), % Add PMOD Nav to Grisp Card on port SPI2
    pmod_nav:config(acc, #{odr_g => {hz,238}}), % Configure PMOD Nav accelerometer with output data rate of 238 Hz

    I2Cbus = grisp_i2c:open(i2c1),
    persistent_term:put(i2c, I2Cbus),

    hera_subscribe:subscribe(self()),
    {ok, Gyro_Pid} = hera:start_measure(gyroscope_measure, []),
    timer:sleep(5000),    
    {ok, Kal_Pid} = hera:start_measure(kalman_stability, []),

    persistent_term:put(hera_gyro, Gyro_Pid),
    persistent_term:put(hera_kal, Kal_Pid),
    timer:sleep(1000),
    robot_init(),
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

    % Initialize PID controllers for speed and stability
    Pid_Speed = spawn(pid_controller, pid_init, [-0.12, -0.07, 0.0, -1, 60.0, 0.0]),
    persistent_term:put(pid_speed, Pid_Speed),
    Pid_Stability = spawn(pid_controller, pid_init, [17.0, 0.0, 4.0, -1, -1, 0.0]),
    persistent_term:put(pid_stability, Pid_Stability),

    io:format("[ROBOT] Pid of the speed controller: ~p.~n", [Pid_Speed]),
    io:format("[ROBOT] Pid of the stability controller: ~p.~n", [Pid_Stability]),
    io:format("[ROBOT] Starting movement of the robot.~n"),    
    T0 = erlang:system_time()/1.0e6,   

    % State definition
    State = #{
    robot_state => {rest, false},
    logging_state => {0, T0, []},
    movement_controls => {0.0, 0.0}
    },

    robot_main(State).

robot_main(State) ->
    {Adv_V_Ref, Turn_V_Ref} = maps:get(movement_controls, State),
    {Robot_state, Robot_Up} = maps:get(robot_state, State),
    {Logging, Log_End, Log_List} = maps:get(logging_state, State),

    {Speed, Arm_Ready, Switch, Test, Get_Up, Adv_V_Goal, Turn_V_Goal} = i2C_data(),

    % Select between kalman and comp filter
    [{_, _, _, [Tk, Xk, _]}] = hera_data:get(k_stability_state),
    [{_, _, _, [_, Angle_Complem, _]}] = hera_data:get(comp_filter_state),
    [Th_Kalman, _W_Kalman] = mat:to_array(Xk),
    Angle_Kalman = Th_Kalman * ?RAD_TO_DEG,
    Angle = helper_module:select_angle(Switch, Angle_Kalman, Angle_Complem),  

    Pid_Speed = persistent_term:get(pid_speed),
    Pid_Stability = persistent_term:get(pid_stability),
    [{_, _, _, [Dt]}] = hera_data:get(delta_time),

    {Acc, Adv_V_Ref_New, Turn_V_Ref_New} = stability_engine:controller(
        {Dt, Angle, Speed}, 
        {Pid_Speed, Pid_Stability}, 
        {Adv_V_Goal, Adv_V_Ref}, 
        {Turn_V_Goal, Turn_V_Ref}
    ),

    Robot_is_Up = is_robot_standing(Angle, Robot_Up),
    
    New_robot_state = get_robot_state({Robot_state, Robot_Up, Get_Up, Arm_Ready, Angle}),

    Move_direction = get_movement_direction(Angle),    
    Output_state = get_output_state(New_robot_state, Move_direction),
    Output_Byte = get_byte(Output_state),


    [HF1, HF2] = hera_com:encode_half_float([Acc, Turn_V_Ref_New]),
    I2Cbus = persistent_term:get(i2c),
    grisp_i2c:transfer(I2Cbus, [{write, 16#40, 1, [HF1, HF2, <<Output_Byte>>]}]),

    {N, T_End} = update_frequency(Dt),    

    % Determine the end time for logging based on the test mode.
    Log_End_New = if Test -> erlang:system_time()/1.0e6 + ?LOG_DURATION; true -> Log_End end,
    Logging_New = (erlang:system_time()/1.0e6) < Log_End_New,

    helper_module:flicker_led(Logging_New, N),

    manage_logging_transition(Logging, Logging_New),    

    stabilize_frequency(T_End, Tk),

    New_State = State#{
        robot_state => {New_robot_state, Robot_is_Up},
        logging_state => {Logging_New, Log_End_New, Log_List},
        movement_controls => {Adv_V_Ref_New, Turn_V_Ref_New}},
   
    robot_main(New_State).

%============================================================================================================================================
%========================================================= HELPER FUNCTION ==================================================================
%============================================================================================================================================
    
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

    Turn_V_Goal = helper_module:turn_ref(Left, Right), % Get rotation goal
    {Speed, Arm_Ready, Switch, Test, Get_Up, Adv_V_Goal, Turn_V_Goal}.

is_robot_standing(Angle, Robot_Up) ->
    if 
        Robot_Up and (abs(Angle) > 20) ->
            false;
        not Robot_Up and (abs(Angle) < 18) -> 
            true;
        true ->
            Robot_Up
    end.

get_movement_direction(Angle) ->
    case Angle > 0.0 of
        true ->
            1;
        false ->
            0
    end.

get_robot_state(Robot_State) -> % {Robot_state, Robot_Up, Get_Up, Arm_ready, Angle}
    case Robot_State of
        {rest, _, true, _, _} -> raising;
        {rest, _, _, _, _} -> rest;
        {raising, true, _, _, _} -> stand_up;
        {raising, _, false, _, _} -> soft_fall;
        {raising, _, _, _, _} -> raising;
        {stand_up, _, false, _, _} -> wait_for_extend;
        {stand_up, false, _, _, _} -> rest;
        {stand_up, _, _, _, _} -> stand_up;
        {wait_for_extend, _, _, _, _} -> prepare_arms;
        {prepare_arms, _, _, true, _} -> free_fall;
        {prepare_arms, _, true, _, _} -> stand_up;
        {prepare_arms, false, _, _, _} -> rest;
        {prepare_arms, _, _, _, _} -> prepare_arms;
        {free_fall, _, _, _, Angle} ->
            case abs(Angle) >10 of
                true -> wait_for_retract;
                _ ->free_fall
            end;
        {wait_for_retract, _, _, _, _} -> soft_fall;
        {soft_fall, _, _, true, _} -> rest;
        {soft_fall, _, true, _, _} -> raising;
        {soft_fall, _, _, _, _} -> soft_fall
    end.

get_output_state(State, Move_direction) ->
    case State of
        rest -> 
            [0, 0, 0, 0, Move_direction, 0, 0, 0];
        raising -> 
            [1, 0, 1, 0, Move_direction, 0, 0, 0];
        stand_up -> 
            [1, 0, 0, 1, Move_direction, 0, 0, 0];
        wait_for_extend -> 
            [1, 0, 1, 1, Move_direction, 0, 0, 0];
        prepare_arms -> 
            [1, 0, 1, 1, Move_direction, 0, 0, 0];
        free_fall -> 
            [1, 1, 1, 1, Move_direction, 0, 0, 0];
        wait_for_retract -> 
            [1, 0, 0, 0, Move_direction, 0, 0, 0];
        soft_fall -> 
            [1, 0, 0, 0, Move_direction, 0, 0, 0]
    end.

get_byte(List) ->
    [A, B, C, D, E, F, G, H] = List,
    A*128 + B*64 + C*32 + D*16 + E*8 + F*4 + G*2 + H. 

update_frequency(Dt)->    
    T_End_New = erlang:system_time()/1.0e6,
    [{ _, _, Seq, [N, Freq, Mean_Freq, T_End]}] = hera_data:get(frequency),
    {N_New, Freq_New, Mean_Freq_New} = helper_module:frequency_computation(Dt, N, Freq, Mean_Freq),
    hera_data:store(frequency, node(), Seq+1, [N_New, Freq_New, Mean_Freq_New, T_End_New]),
    {N, T_End}.

manage_logging_transition(false, true) ->
    % Transition from not logging to logging
    Hera_pid = persistent_term:get(hera_pid),
    Hera_pid ! {self(), start_log};

manage_logging_transition(true, false) ->
    % Transition from logging to not logging
    [grisp_led:color(L, yellow) || L <-[1,2]],
    Hera_pid = persistent_term:get(hera_pid),
    Hera_pid ! {self(), stop_log};

manage_logging_transition(_, _) ->
    % No change
    ok.

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

stabilize_frequency(T_End, Tk) ->
    T2 = erlang:system_time()/1.0e6,
    Delay_Goal = 1.0 / ?FREQ_GOAL * 1000.0,

    if
        T2 - T_End < Delay_Goal ->
            helper_module:wait(Delay_Goal - (T2 - Tk));
        true ->
            ok
    end.