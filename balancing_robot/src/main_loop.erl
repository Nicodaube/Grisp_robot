-module(main_loop).

-export([robot_init/1, modify_frequency/1]).

-define(RAD_TO_DEG, 180.0/math:pi()).
-define(DEG_TO_RAD, math:pi()/180.0).

%Advance constant
-define(ADV_V_MAX, 30.0).

%Turning constant
-define(TURN_V_MAX, 80.0).

%Angle at which robot is considered "down"
-define(MAX_ANGLE, 25.0).

%Coefficient for the complementary filter
-define(COEF_FILTER, 0.667).

%Duration of a logging sequence
-define(LOG_DURATION, 15000).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Robot 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

robot_init(Hera_pid) ->

    process_flag(priority, max),

    %Starting timestamp
    T0 = erlang:system_time()/1.0e6,

    %Table for global variables
    ets:new(variables, [set, public, named_table]),
    ets:insert(variables, {"Freq_Goal", 300.0}),

    %Calibration
    io:format("[ROBOT] Calibrating... Do not move the pmod_nav!~n"),
    [grisp_led:flash(L, green, 500) || L <- [1, 2]],
    [_, Gy0, _] = calibrate(),
    io:format("[ROBOT] Done calibrating~n"),

    %Kalman matrices
    X0 = mat:matrix([[0], [0]]),
    P0 = mat:matrix([[0.1, 0], [0, 0.1]]),

    %I2C bus
    I2Cbus = grisp_i2c:open(i2c1),
    persistent_term:put(i2c, I2Cbus),

    %PIDs initialisation
    Pid_Speed = spawn(pid_controller, pid_init, [-0.12, -0.07, 0.0, -1, 60.0, 0.0]),
    Pid_Stability = spawn(pid_controller, pid_init, [17.0, 0.0, 4.0, -1, -1, 0.0]),
    io:format("[ROBOT] Pid of the speed controller: ~p.~n", [Pid_Speed]),
    io:format("[ROBOT] Pid of the stability controller: ~p.~n", [Pid_Stability]),
	io:format("[ROBOT] Starting movement of the robot.~n"),

    %Call main loop
    robot_main(T0, {rest, false}, {T0, X0, P0}, {Gy0, 0.0, 0.0}, {Pid_Speed, Pid_Stability}, {0.0, 0.0}, {0, 0, 200.0, T0}).

robot_main(Start_Time, {Robot_State, Robot_Up}, {T0, X0, P0}, {Gy0, Angle_Complem, Angle_Rate}, {Pid_Speed, Pid_Stability}, {Adv_V_Ref, Turn_V_Ref}, {N, Freq, Mean_Freq, T_End}) ->

    %Delta time of loop
    T1 = erlang:system_time()/1.0e6, %[ms]
	Dt = (T1- T0)/1000.0,            %[s]

    %%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Input from Sensor %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%

    %Read data
    [Gy,Ax,Az] = pmod_nav:read(acc, [out_y_g, out_x_xl, out_z_xl], #{g_unit => dps}),

    %%%%%%%%%%%%%%%%%%%%%%%%
    %%% Input from ESP32 %%%
    %%%%%%%%%%%%%%%%%%%%%%%%

    %Receive I2C and conversion
    I2Cbus = persistent_term:get(i2c),
    [<<SL1,SL2,SR1,SR2,CtrlByte>>] = grisp_i2c:transfer(I2Cbus, [{read, 16#40, 1, 5}]),
	[Speed_L,Speed_R] = hera_com:decode_half_float([<<SL1, SL2>>, <<SR1, SR2>>]),
    Speed = (Speed_L + Speed_R)/2,

    %Retrieve flags from ESP32
    [Arm_Ready, Switch, Test, Get_Up, Forward, Backward, Left, Right] = hera_com:get_bits(CtrlByte),

    %%%%%%%%%%%%%%%%%%%%%
    %%% Command Logic %%%
    %%%%%%%%%%%%%%%%%%%%%

    %Set advance speed from flags
    Adv_V_Goal = speed_ref(Forward, Backward),

    %Set turning speed from flags
    Turn_V_Goal = turn_ref(Left, Right),

    %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Angle Computations %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%

    %Angle based directly on the sensors
    Angle_Accelerometer = math:atan(Az / (-Ax))*?RAD_TO_DEG,

    %Kalman filter computation
    {X1, P1} = kalman_angle(Dt, Ax, Az, Gy, Gy0, X0, P0),
	[Th_Kalman, _W_Kalman] = mat:to_array(X1),
    Angle_Kalman = Th_Kalman*?RAD_TO_DEG,

    %Complementary angle computation
    K = 1.25/(1.25+(1.0/Mean_Freq)),
    {Angle_Complem_New, Angle_Rate_New} = complem_angle({Dt, Ax, Az, Gy, Gy0, K, Angle_Complem, Angle_Rate}),

    %Select angle between kalman or complementary
    Angle = select_angle(Switch, Angle_Kalman, Angle_Complem),


    %%%%%%%%%%%%%%%%%%
    %%% Controller %%%
    %%%%%%%%%%%%%%%%%%

    %Takes as input:
    % -Measures
    % -Process IDs of the two PID controllers
    % -Advance and turning velocity to reach and loopback velocity of trapezoidal profile
    {Acc, Adv_V_Ref_New, Turn_V_Ref_New} = stability_engine:controller({Dt, Angle, Speed}, {Pid_Speed, Pid_Stability}, {Adv_V_Goal, Adv_V_Ref}, {Turn_V_Goal, Turn_V_Ref}),
    %Gives as output:
    % -Acceleration of the motors -> sent to ESP32 
    % -Loopback V_Ref for advance
    % -Loopback V_Ref for turning -> sent to ESP32 

    %%%%%%%%%%%%%%%%%%%%%%%
    %%% Output to ESP32 %%%
    %%%%%%%%%%%%%%%%%%%%%%%
    
    %State of the robot
    if 
        Robot_Up and (abs(Angle) > 20) ->
            Robot_Up_New = false;
        not Robot_Up and (abs(Angle) < 18) -> 
            Robot_Up_New = true;
        true ->
            Robot_Up_New = Robot_Up
    end,
    if
        Angle > 0.0 ->
            F_B = 1;
        true ->
            F_B = 0
    end,

    Next_Robot_State = get_robot_state({Robot_State, Robot_Up, Get_Up, Arm_Ready, Angle}),
    Output_Bits = get_output_state(Next_Robot_State, F_B),

    %Send output to ESP32
    Output_Byte = get_byte(Output_Bits),
    [HF1, HF2] = hera_com:encode_half_float([Acc, Turn_V_Ref_New]),
    grisp_i2c:transfer(I2Cbus, [{write, 16#40, 1, [HF1, HF2, <<Output_Byte>>]}]),

    %%%%%%%%%%%%%%%%%%%%%%%%
    %%% Testing Features %%%
    %%%%%%%%%%%%%%%%%%%%%%%%

    %Frequency computation
    {N_New, Freq_New, Mean_Freq_New} = frequency_computation(Dt, N, Freq, Mean_Freq),


    %Imposed maximum frequency
    T2 = erlang:system_time()/1.0e6,
    [{_,Freq_Goal}] = ets:lookup(variables, "Freq_Goal"),
    Delay_Goal = 1.0/Freq_Goal * 1000.0,
    if
        T2-T_End < Delay_Goal ->
            wait(Delay_Goal-(T2-T1));
        true ->
            ok
    end,
    T_End_New = erlang:system_time()/1.0e6,

    %Loop back with updated state
    robot_main(Start_Time, {Next_Robot_State, Robot_Up_New}, {T1, X1, P1}, {Gy0, Angle_Complem_New, Angle_Rate_New}, {Pid_Speed, Pid_Stability}, {Adv_V_Ref_New, Turn_V_Ref_New}, {N_New, Freq_New, Mean_Freq_New, T_End_New}).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate() ->
    N = 500,
    Data = [list_to_tuple(pmod_nav:read(acc, [out_x_g, out_y_g, out_z_g])) || _ <- lists:seq(1,N)],
    {X, Y, Z} = lists:unzip3(Data),
    [lists:sum(X)/N, lists:sum(Y)/N, lists:sum(Z)/N]. %[Gx0, Gy0, Gz0]

kalman_angle(Dt, Ax, Az, Gy, Gy0, X0, P0) ->
    R = mat:matrix([[3.0, 0.0], [0, 3.0e-6]]),
    Q = mat:matrix([[3.0e-5, 0.0], [0.0, 10.0]]),
    F = fun (X) -> [Th, W] = mat:to_array(X),
				mat:matrix([ 	[Th+Dt*W],
								[W      ] ])
		end,
    Jf = fun (X) -> [_Th, _W] = mat:to_array(X),
				mat:matrix([  	[1, Dt],
								[0, 1 ] ])
		 end,
    H = fun (X) -> [Th, W] = mat:to_array(X),
				mat:matrix([ 	[Th],
								[W ] ])
		end,
    Jh = fun (X) -> [_Th, _W] = mat:to_array(X),
				mat:matrix([  	[1, 0],
								[0, 1] ])
		 end,
    Z = mat:matrix([[math:atan(Az / (-Ax))], [(Gy-Gy0)*?DEG_TO_RAD]]),
    kalman:ekf({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z).

complem_angle({Dt, Ax, Az, Gy, Gy0, K, Angle_Complem, Angle_Rate}) ->

    Angle_Rate_New = (Gy - Gy0) * ?COEF_FILTER + Angle_Rate * (1 - ?COEF_FILTER),

    %Angle increment computed from gyroscope
    Delta_Gyr = Angle_Rate_New * Dt,

    %Absolute angle computed form accelerometer
    Angle_Acc = math:atan(Az / (-Ax)) * 180 / math:pi(),

    %Complementary filter combining gyroscope and accelerometer
    Angle_Complem_New = (Angle_Complem + Delta_Gyr) * K + Angle_Acc * (1 - K), 
    
    {Angle_Complem_New, Angle_Rate_New}.

select_angle(Switch, Angle_Kalman, Angle_Complem) ->
    if
        Switch -> 
            Angle = Angle_Kalman;
        true ->
            Angle = Angle_Complem
    end,
    Angle.

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

modify_frequency(Freq) ->
    ets:insert(variables, {"Freq_Goal", Freq}),
    ok.

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
    % Output bits = [Power, Freeze, Extend, Robot_Up_Bit, Move_direction, 0, 0, 0]
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