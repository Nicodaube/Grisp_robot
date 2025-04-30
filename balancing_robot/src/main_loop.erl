-module(main_loop).

-export([robot_init/0]).

-define(RAD_TO_DEG, 180.0/math:pi()).
-define(DEG_TO_RAD, math:pi()/180.0).

%Advance constant
-define(ADV_V_MAX, 30.0).

%Turning constant
-define(TURN_V_MAX, 80.0).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Robot 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

robot_init() ->

    process_flag(priority, max),

    %Starting timestamp
    T0 = erlang:system_time()/1.0e6,

    persistent_term:put(freq_goal, 300.0),

    %Calibration
    calibrate(),    

    %Kalman matrices
    X0 = mat:matrix([[0], [0]]),
    P0 = mat:matrix([[0.1, 0], [0, 0.1]]),

    %I2C bus
    I2Cbus = grisp_i2c:open(i2c1),
    persistent_term:put(i2c, I2Cbus),

    %PIDs initialisation
    Pid_Speed = spawn(pid_controller, pid_init, [-0.12, -0.07, 0.0, -1, 60.0, 0.0]),
    Pid_Stability = spawn(pid_controller, pid_init, [17.0, 0.0, 4.0, -1, -1, 0.0]),

    init_kalman_constant(),
	io:format("[ROBOT] Robot ready.~n"),

    %Call main loop
    robot_main(T0, {rest, false}, {T0, X0, P0}, {Pid_Speed, Pid_Stability}, {0.0, 0.0}, {0, 0, 200.0, T0}).

robot_main(Start_Time, {Robot_State, Robot_Up}, {T0, X0, P0}, {Pid_Speed, Pid_Stability}, {Adv_V_Ref, Turn_V_Ref}, {N, Freq, Mean_Freq, T_End}) ->

    %Delta time of loop
    T1 = erlang:system_time()/1.0e6, %[ms]
	Dt = (T1- T0)/1000.0,            %[s]

    [Gy,Ax,Az] = pmod_nav:read(acc, [out_y_g, out_x_xl, out_z_xl], #{g_unit => dps}), % Read Pmod Nav

    %Receive I2C and conversion
    I2Cbus = persistent_term:get(i2c),
    [<<SL1,SL2,SR1,SR2,CtrlByte>>] = grisp_i2c:transfer(I2Cbus, [{read, 16#40, 1, 5}]),
	[Speed_L,Speed_R] = hera_com:decode_half_float([<<SL1, SL2>>, <<SR1, SR2>>]),
    Speed = (Speed_L + Speed_R)/2,

    %Retrieve flags from ESP32
    [Arm_Ready, _, _, Get_Up, Forward, Backward, Left, Right] = hera_com:get_bits(CtrlByte),

    %Set advance speed from flags
    Adv_V_Goal = speed_ref(Forward, Backward),

    %Set turning speed from flags
    Turn_V_Goal = turn_ref(Left, Right),

    %Kalman filter computation
    [Angle, {X1, P1}] = kalman_angle(Dt, Ax, Az, Gy, X0, P0),

    {Acc, Adv_V_Ref_New, Turn_V_Ref_New} = stability_engine:controller({Dt, Angle, Speed}, {Pid_Speed, Pid_Stability}, {Adv_V_Goal, Adv_V_Ref}, {Turn_V_Goal, Turn_V_Ref}),
    
    %State of the robot
    Robot_Up_New = is_robot_up(Angle, Robot_Up),
    Next_Robot_State = get_robot_state({Robot_State, Robot_Up, Get_Up, Arm_Ready, Angle}),
    Output_Byte = get_output_state(Next_Robot_State, Angle),    

    %Send output to ESP32    
    [HF1, HF2] = hera_com:encode_half_float([Acc, Turn_V_Ref_New]),
    grisp_i2c:transfer(I2Cbus, [{write, 16#40, 1, [HF1, HF2, <<Output_Byte>>]}]),

    %Frequency computation
    {N_New, Freq_New, Mean_Freq_New} = frequency_computation(Dt, N, Freq, Mean_Freq),

    %Imposed maximum frequency
    T2 = erlang:system_time()/1.0e6,
    Freq_Goal = persistent_term:get(freq_goal),
    Delay_Goal = 1.0/Freq_Goal * 1000.0,
    if
        T2-T_End < Delay_Goal ->
            wait(Delay_Goal-(T2-T1));
        true ->
            ok
    end,
    T_End_New = erlang:system_time()/1.0e6,

    robot_main(Start_Time, {Next_Robot_State, Robot_Up_New}, {T1, X1, P1}, {Pid_Speed, Pid_Stability}, {Adv_V_Ref_New, Turn_V_Ref_New}, {N_New, Freq_New, Mean_Freq_New, T_End_New}).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate() ->
    io:format("[ROBOT] Calibrating... Do not move the pmod_nav!~n"),
    N = 500,
    Y_List = [pmod_nav:read(acc, [out_y_g]) || _ <- lists:seq(1, N)],
    Gy0 = lists:sum([Y || [Y] <- Y_List]) / N,
    io:format("[ROBOT] Done calibrating~n"),
    [grisp_led:flash(L, green, 500) || L <- [1, 2]],
    persistent_term:put(gy0, Gy0).    

init_kalman_constant() ->
    R = mat:matrix([[3.0, 0.0], [0, 3.0e-6]]),
    Q = mat:matrix([[3.0e-5, 0.0], [0.0, 10.0]]),
    Jh = fun (_) -> mat:matrix([  	[1, 0],
								    [0, 1] ])
		 end,
    persistent_term:put(kalman_constant, {R, Q, Jh}).

kalman_angle(Dt, Ax, Az, Gy, X0, P0) ->
    Gy0 = persistent_term:get(gy0),
    {R, Q, Jh} = persistent_term:get(kalman_constant),
    
    F = fun (X) -> [Th, W] = mat:to_array(X),
				mat:matrix([ 	[Th+Dt*W],
								[W      ] ])
		end,
    Jf = fun (_) -> mat:matrix([  	[1, Dt],
								    [0, 1 ] ])
		 end,
    H = fun (X) -> [Th, W] = mat:to_array(X),
				mat:matrix([ 	[Th],
								[W ] ])
		end,
    
    Z = mat:matrix([[math:atan(Az / (-Ax))], [(Gy-Gy0)*?DEG_TO_RAD]]),
    {X1, P1} = kalman:ekf({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z),

    [Th_Kalman, _W_Kalman] = mat:to_array(X1),
    Angle = Th_Kalman * ?RAD_TO_DEG,
    [Angle, {X1, P1}].

speed_ref(Forward, Backward) ->
    if
        Forward ->
            Adv_V_Goal = ?ADV_V_MAX;
        Backward ->
            Adv_V_Goal = - ?ADV_V_MAX;
        true ->
            Adv_V_Goal = 0.0
    end,
    Adv_V_Goal.

turn_ref(Left, Right) ->
    if
        Right ->
            Turn_V_Goal = ?TURN_V_MAX;
        Left ->
            Turn_V_Goal = - ?TURN_V_MAX;
        true ->
            Turn_V_Goal = 0.0
    end,
    Turn_V_Goal.


frequency_computation(Dt, N, Freq, Mean_Freq) ->
    if 
        N == 100 ->
            N_New = 0,
            Freq_New = 0,
            Mean_Freq_New = Freq;
        true ->
            N_New = N+1,
            Freq_New = ((Freq*N)+(1/Dt))/(N+1),
            Mean_Freq_New = Mean_Freq
    end,
    {N_New, Freq_New, Mean_Freq_New}.

wait(T) -> 
	Tnow = erlang:system_time()/1.0e6,
	wait_help(Tnow,Tnow+T).
wait_help(Tnow, Tend) when Tnow >= Tend -> ok;
wait_help(_, Tend) -> 
    Tnow = erlang:system_time()/1.0e6,
    wait_help(Tnow,Tend).

%Transforms a list of 8 bits into a byte
get_byte(List) ->
    [A, B, C, D, E, F, G, H] = List,
    A*128 + B*64 + C*32 + D*16 + E*8 + F*4 + G*2 + H. 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Global variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

get_output_state(State, Angle) ->
    Move_direction = get_movement_direction(Angle),    
    % Output bits = [Power, Freeze, Extend, Robot_Up_Bit, Move_direction, 0, 0, 0]
    case State of 
        rest -> 
            get_byte([0, 0, 0, 0, Move_direction, 0, 0, 0]);
        raising -> 
            get_byte([1, 0, 1, 0, Move_direction, 0, 0, 0]);
        stand_up -> 
            get_byte([1, 0, 0, 1, Move_direction, 0, 0, 0]);
        wait_for_extend -> 
            get_byte([1, 0, 1, 1, Move_direction, 0, 0, 0]);
        prepare_arms -> 
            get_byte([1, 0, 1, 1, Move_direction, 0, 0, 0]);
        free_fall -> 
            get_byte([1, 1, 1, 1, Move_direction, 0, 0, 0]);
        wait_for_retract -> 
            get_byte([1, 0, 0, 0, Move_direction, 0, 0, 0]);
        soft_fall -> 
            get_byte([1, 0, 0, 0, Move_direction, 0, 0, 0])
    end.

is_robot_up(Angle, Robot_Up) ->
    if 
        Robot_Up and (abs(Angle) > 20) ->
            false;
        not Robot_Up and (abs(Angle) < 18) -> 
            true;
        true ->
            Robot_Up
    end.

get_movement_direction(Angle) ->
    if
        Angle > 0.0 ->
            1;
        true ->
            0
    end.