%%-----------------------------------------------------------------------------%%
%% Module: kalman_measure_updated.erl
%% Extended EKF with state [x, y, θ, v, b_ω]
%%-----------------------------------------------------------------------------%%
-module(kalman_measure_update).
-behavior(hera_measure).

-export([init/1, measure/1]).

-define(RAD_TO_DEG, 180.0/math:pi()).
-define(BETA, 0.07).

%%-----------------------------------------------------------------------------%%
%% Noise parameters (discrete-time)
%% Q_continuous * Δt -> Q_discrete
-define(Q_P,   0.1).    % process noise cont for position [m²/s]
-define(Q_TH,  0.005).  % process noise cont for angle [rad²/s]
-define(Q_V,   0.2).    % process noise cont for velocity [ (m/s)²/s ]
-define(Q_BW,  0.0001). % process noise cont for gyro bias [rad²/s³]

-define(R_S,   0.0025). % measurement noise pos [m²]
-define(R_TH,  0.0020). % measurement noise angle [rad²]
-define(R_V,   0.02).   % measurement noise velocity [ (m/s)² ]
-define(R_BW,  0.00001).% measurement noise bias [rad²]
%%-----------------------------------------------------------------------------%%
init(_Args) ->
    timer:sleep(2000),
    io:format("~n[KALMAN_MEASURE] Starting measurements~n"),
    persistent_term:put(ahrs_quat, [1,0,0,0]),
    calibrate(),
    calibrate_speed(),
    
    %% Initial state: [x, y, θ, v, b_ω]
    X0 = mat:matrix([[0],[0],[0],[0]]),
    P0 = mat:diag([10,10, 10, 10]),

    State = #{
      t0    => erlang:system_time()/1.0e6,
      x_pos => X0,
      p_pos => P0,
      seq   => 1
    },
    {ok, State, #{name => kalman_measure, iter => infinity, timeout => 100}}.

%%-----------------------------------------------------------------------------%%
%% measure/1: prediction + conditional update
measure(State) ->
    #{t0 := T0, x_pos := Xprev, p_pos := Pprev, seq := Seq} = State,
    case hera_data:get(robot_pos, robot) of 
        [{_, _, _, [_OldX,_OldY, _OldAngle, OldRoom]}] ->
        %% Compute Δt
        T1 = erlang:system_time()/1.0e6,
        Dt = (T1 - T0)/1000.0,

        %% IMU and speed inputs
        {V_mm, _} = i2c_read(),
        V_mes = V_mm/100,
        {_, _, Gyro_mtx, Acc_mtx, _} = get_val_nav(Dt),
        [Omega, _, _] = mat:to_array(Gyro_mtx),
        [Ax, Ay, Az]    = mat:to_array(Acc_mtx),

        %% Build discrete Q = Q_cont * Dt
        Qd = mat:diag([
        ?Q_P*Dt,
        ?Q_P*Dt,
        ?Q_TH*Dt,
        ?Q_V*Dt
        ]),

        %% Define F and Jf for extended CTRV + bias model
        F = fun(X) ->
            [Xc, Yc, Thc, Vc ] = mat:to_array(X),
            Omega_corr = Omega,
            Acc_long = -Az,%Ax*math:cos(Thc) + Ay*math:sin(Thc),
            Vnext = Vc + Acc_long*Dt,
            Xn = Xc + Vc*math:cos(Thc)*Dt + 0.5*Acc_long*Dt*Dt,
            Yn = Yc + Vc*math:sin(Thc)*Dt + 0.5*Acc_long*Dt*Dt,
            Thn = Thc + Omega_corr*Dt,
            
            mat:matrix([[Xn],[Yn],[Thn],[Vnext]])
        end,

        Jf = fun(X) ->
            [_, _, Thc, Vc, _Bwc] = mat:to_array(X),
            _Acc_long = Ax*math:cos(Thc) + Ay*math:sin(Thc),
            % dAcc_long/dθ = -Ax*sin(Thc) + Ay*cos(Thc)
            DAdTh = 0,%-Ax*math:sin(Thc) + Ay*math:cos(Thc),
            % Jacobian 5x5
            mat:matrix([
            [1, 0, -Vc*math:sin(Thc)*Dt + 0.5*DAdTh*Dt*Dt, math:cos(Thc)*Dt],
            [0, 1,  Vc*math:cos(Thc)*Dt + 0.5*DAdTh*Dt*Dt, math:sin(Thc)*Dt],
            [0, 0, 1,                               0],
            [0, 0, DAdTh*Dt,                        1],
            [0, 0, 0,                               0]
            ])
        end,

        %% Prediction step
        {Xpred, Ppred} = ekf_predict({Xprev, Pprev}, {F, Jf}, Qd),

        %% Measurement update (5x1) if sonar OK
        case get_new_robot_pos(OldRoom) of
        no_intersection ->
            Xnew = Xpred,
            Pnew = Ppred;
        {Xmes, Ymes} ->
            [_,_,Th_pred, _V_pred] = mat:to_array(Xpred),
            Theta_mes = Th_pred,  % or use Roll from quaternion if preferred
            Z = mat:matrix([[Xmes],[Ymes],[Theta_mes],[V_mes]]),
            R5 = mat:diag([
            ?R_S, ?R_S, ?R_TH, ?R_V
            ]),
            H5 = fun(X) -> X end,
            Jh5 = fun(_) -> mat:eye(4) end,
            {Xnew, Pnew} = kalman:ekf({Xpred,Ppred}, {F,Jf}, {H5,Jh5}, Qd, R5, Z)
        end,

        %% Store and send
        [Xf, Yf, Thf, _, _] = mat:to_array(Xnew),
        ThetaDeg = Thf * ?RAD_TO_DEG,
        hera_data:store(robot_pos, robot, Seq, [Xf, Yf, ThetaDeg, OldRoom]),
        send_robot_pos([Xf, Yf, ThetaDeg, OldRoom]),

        %% New state
        NewState = #{t0 => T1, x_pos => Xnew, p_pos => Pnew, seq => Seq+1},
        {ok, [Xf,Yf,ThetaDeg,OldRoom], robot_pos, robot, NewState}
    end.            
%============================================================================================================================================
%======================================================= CALIBRATION FUNC ===================================================================
%============================================================================================================================================

calibrate() ->
    N=500,
    Gyro_data = [ pmod_nav:read(acc, [out_x_g, out_y_g, out_z_g])
                || _ <- lists:seq(1, N) ],

    G_x_List = [ X || [X,_,_] <- Gyro_data ],
    G_y_List = [ Y || [_,Y,_] <- Gyro_data ],
    G_z_List = [ Z || [_,_,Z] <- Gyro_data ],

    AngVel_data = [pmod_nav:read(mag, [out_x_m, out_y_m, out_z_m]) || _ <- lists:seq(1, N)],

    M_x_List = [ X || [X,_,_] <- AngVel_data ],
    M_y_List = [ Y || [_,Y,_] <- AngVel_data ],
    M_z_List = [ Z || [_,_,Z] <- AngVel_data ],

    Acc_data = [ pmod_nav:read(acc, [out_x_xl, out_y_xl, out_z_xl]) || _ <- lists:seq(1, N) ],

    Accc_x_List = [ X || [X,_,_] <- Acc_data ],
    Accc_y_List = [ Y || [_,Y,_] <- Acc_data ],
    Accc_z_List = [ Z || [_,_,Z] <- Acc_data ],

    [Gx0_pos, Gy0_pos, Gz0_pos] = [lists:sum(List) / N || List <- [G_x_List, G_y_List, G_z_List]],
    [Mx0, My0, Mz0] = [lists:sum(List) / N || List <- [M_x_List, M_y_List, M_z_List]],
    [Accx0, Accy0, Accz0] = [lists:sum(List) / N || List <- [Accc_x_List, Accc_y_List, Accc_z_List]],
    io:format("[KALMAN_MEASURE] Done calibrating~n"),

    persistent_term:put(gyro_init, {Gx0_pos, Gy0_pos, Gz0_pos}),
    persistent_term:put(mag_init, {Mx0, My0, Mz0}),
    persistent_term:put(acc_init, {Accx0, Accy0, Accz0}).

calibrate_speed() ->
    I2Cbus = persistent_term:get(i2c),
    N = 10,
    RawSpeeds = [grisp_i2c:transfer(I2Cbus, [{read, 16#40, 1, 5}]) || _ <- lists:seq(1, N)],
    Decoded = [hera_com:decode_half_float([<<SL1, SL2>>, <<SR1, SR2>>]) ||
                  [<<SL1, SL2, SR1, SR2, _>>] <- RawSpeeds],
    SpeedsL = [L || [L, _] <- Decoded],
    SpeedsR = [R || [_, R] <- Decoded],
    OffsetL = lists:sum(SpeedsL) / N,
    OffsetR = lists:sum(SpeedsR) / N,
    persistent_term:put(i2c_offset, {OffsetL, OffsetR}).


%============================================================================================================================================
%======================================================= HELPER FUNC ========================================================================
%============================================================================================================================================       

send_robot_pos(Pos) ->
    Pos_string = string:join([lists:flatten(io_lib:format("~p", [Val])) || Val <- Pos], ","),
    Msg = "Robot_pos," ++ Pos_string,
    hera_com:send_unicast(server, Msg, "UTF8").
            
get_new_robot_pos(Room) ->
    [Sensor1, Sensor2] = get_room_sensors(Room),
    %io:format("[KALMAN_MEASURE] The two sensors in the current room are : ~p and ~p ~n",[Sensor1, Sensor2]),
    {X1, Y1, _} = get_sensor_pos(Sensor1),
    {X2, Y2, _} = get_sensor_pos(Sensor2),
    [{_, _, _, [Dist1]}] = hera_data:get(distance, Sensor1), % distance on the ground
    [{_, _, _, [Dist2]}] = hera_data:get(distance, Sensor2),
    {TLx, TLy, BRx, BRy} = get_room_info(Room),
    get_pos({X1, Y1}, {X2, Y2}, {Dist1} , {Dist2},{TLx, TLy, BRx, BRy}).
    
get_room_sensors(Room) ->
    Devices = persistent_term:get(devices),
    lists:foldl(
        fun({Name, _, _}, Acc) ->
            case Name of
                _ ->
                    case hera_data:get(room, Name) of
                        [{_, _, _, [ORoom]}] when Room =:= ORoom ->
                                [Name | Acc];
                        _ ->
                            Acc
                    end
            end
        end,
        [],
        Devices
    ).

get_sensor_pos(SensorName) ->
    case hera_data:get(pos, SensorName) of
        [{_, _, _, [X, Y, _, A]}] ->
            {X, Y, A};
        _ ->
            io:format("[KALMAN_MEASURE] Can't get the pos of sensor : ~p~n", [SensorName])
    end.

get_room_info(OldRoom) ->
    case hera_data:get(room_info,OldRoom) of
        [{_, _, _, [TLx, TLy, BRx, BRy]}] ->
            {TLx, TLy, BRx, BRy};
        _ ->
            io:format("[KALMAN_MEASURE] Can't get the pos of room n°~p~n", [OldRoom])
    end.

get_pos({X1,Y1}, {X2,Y2}, {Dist1}, {Dist2},{TLx, TLy, BRx, BRy}) ->
    Dx = (X2 - X1) * 100,
    Dy = (Y2 - Y1) * 100,
    D = math:sqrt(Dx*Dx + Dy * Dy),
    case (D > Dist1 + Dist2) orelse (D < abs(Dist1 - Dist2)) orelse (D == 0 andalso Dist1 == Dist2) of
        true ->
            no_intersection;
        false ->
            A = (Dist1 * Dist1 - Dist2 * Dist2 + D*D) / (2*D),
            H = math:sqrt(Dist1*Dist1 - A*A),

            Px = (X1*100) + A * (Dx/D),
            Py = (Y1*100) + A * (Dy/D),

            Rx = -Dy * (H/D),
            Ry =  Dx * (H/D),

            Xout1 = Px + Rx,
            Yout1 = Py + Ry,
            Xout2 = Px - Rx,
            Yout2 = Py - Ry,
            check_good_point(Xout1, Yout1, Xout2, Yout2, TLx, TLy, BRx, BRy)
    end.

check_good_point(Xout1, Yout1, Xout2, Yout2, TLx, TLy, BRx, BRy) ->

    MaxRoomX = lists:max([TLx, BRx])*100,
    MaxRoomy = lists:max([TLy, BRy])*100,
    MinRoomX = lists:min([TLx, BRx])*100,
    MinRoomy = lists:min([TLy, BRy])*100,
    case {MinRoomX =< Xout1 andalso Xout1 =< MaxRoomX, MinRoomy =< Yout1 andalso Yout1 =< MaxRoomy} of
        {true, true} ->
            {Xout1/100, Yout1/100};
        _ ->
            case {MinRoomX =< Xout2 andalso Xout2 =< MaxRoomX, MinRoomy =< Yout2 andalso Yout2 =< MaxRoomy} of
                {true, true} ->
                    {Xout2/100, Yout2/100};
                _ ->
                    no_intersection
            end
    end.


get_val_nav(Dt) ->
    [Ax, Ay, Az] = pmod_nav:read(acc, [out_x_xl, out_y_xl, out_z_xl]),
    [Gx, Gy, Gz] = pmod_nav:read(acc, [out_x_g, out_y_g, out_z_g]),
    [Mx, My, Mz] = pmod_nav:read(mag, [out_x_m, out_y_m, out_z_m]),

    {Ax0, Ay0, Az0} = persistent_term:get(acc_init),
    Acc = scale([Ax - Ax0, Ay - Ay0, Az - Az0], 9.81),
    
    
    {GBx, GBy, GBz} = persistent_term:get(gyro_init),
    Gyro = scale([Gx-GBx,Gy-GBy,Gz-GBz],math:pi()/180),

    {MBx,MBy,MBz} = persistent_term:get(mag_init),
    Mag = mat:matrix([[Mx-MBx,My-MBy,Mz-MBz]]),


    %R0 = ahrs([Ax,Ay,-Az], [-(Mx-MBx),My-MBy,-(Mz-MBz)]),
    Quat0 = persistent_term:get(ahrs_quat),
    Quat1 = update(Gyro, Acc, mat:to_array(Mag),Dt, Quat0),
    Quat1_norm = normalize(Quat1),
    R0 = quat_to_matrix(Quat1_norm),
    
    persistent_term:put(ahrs_quat, Quat1_norm),
    AccRot = mat:'*'(mat:matrix([Acc]), mat:tr(R0)),  % rotation dans le repère monde
    RotAcc = mat:'-'(AccRot, mat:matrix([[0, 0, 9.81]])),  % compensation gravité
    
    R0t = mat:tr(R0),


    {mat:matrix([Acc]), RotAcc, mat:matrix([Gyro]), Mag,R0t}. 

scale(List, Factor) ->
    [X*Factor || X <- List].


i2c_read() ->
    %Receive I2C and conversion
    I2Cbus = persistent_term:get(i2c),
    [<<SL1,SL2,SR1,SR2,CtrlByte>>] = grisp_i2c:transfer(I2Cbus, [{read, 16#40, 1, 5}]),
    {OffsetL,OffsetR} = persistent_term:get(i2c_offset),
    [Speed_L,Speed_R] = hera_com:decode_half_float([<<SL1, SL2>>, <<SR1, SR2>>]),
    Speed2 = ((Speed_L - OffsetL) + (Speed_R - OffsetR))/2,
    Speed = case erlang:abs(Speed2) < 0.1 of
        true -> 0.0;
        false -> Speed2
    end,
{Speed, CtrlByte}.



update([Gx, Gy, Gz], [Ax, Ay, Az], [Mx, My, Mz], Dt, [Q0, Q1, Q2, Q3]) ->
    %% Normalize sensor inputs
    Acc = normalize([Ax, Ay, Az]),
    Mag = normalize([Mx, My, Mz]),
    [Axn, Ayn, Azn] = Acc,
    [_Mxn, _Myn, _Mzn] = Mag,

    %% Reference direction of Earth's magnetic field (based on current quaternion)
    R = mat:tr(quat_to_matrix([Q0, Q1, Q2, Q3])),
    H = mat:'*'(mat:matrix([Mag]), R),
    [Hx, Hy, _Hz] = mat:to_array(H),

    %% Error vector (acc + mag fusion)
    F1 = 2*(Q1*Q3 - Q0*Q2) - Axn,
    F2 = 2*(Q0*Q1 + Q2*Q3) - Ayn,
    F3 = 2*(0.5 - Q1*Q1 - Q2*Q2) - Azn,
    F4 = Hx - 1.0,
    F5 = Hy,

    %% Gradient descent step
    Grad0 = -F1*Q2 + F2*Q1,
    Grad1 = F1*Q3 + F2*Q0 - 4*Q1*F3,
    Grad2 = -F1*Q0 + F2*Q3 - 4*Q2*F3,
    Grad3 = F1*Q1 + F2*Q2,

    GradNorm = normalize([Grad0, Grad1, Grad2, Grad3]),
    [G0, G1, G2, G3] = GradNorm,

    %% Quaternion rate of change
    QDot0 = 0.5 * (-Q1*Gx - Q2*Gy - Q3*Gz) - ?BETA * G0,
    QDot1 = 0.5 * ( Q0*Gx + Q2*Gz - Q3*Gy) - ?BETA * G1,
    QDot2 = 0.5 * ( Q0*Gy - Q1*Gz + Q3*Gx) - ?BETA * G2,
    QDot3 = 0.5 * ( Q0*Gz + Q1*Gy - Q2*Gx) - ?BETA * G3,

    %% Integrate to get new quaternion
    Q0n = Q0 + QDot0 * Dt,
    Q1n = Q1 + QDot1 * Dt,
    Q2n = Q2 + QDot2 * Dt,
    Q3n = Q3 + QDot3 * Dt,

    normalize([Q0n, Q1n, Q2n, Q3n]).


%% Converts a unit quaternion [w,x,y,z] into a rotation matrix R0 (3x3)
quat_to_matrix([W, X, Y, Z]) ->
    Wx = W*X, Wy = W*Y, Wz = W*Z,
    Xx = X*X, Xy = X*Y, Xz = X*Z,
    Yy = Y*Y, Yz = Y*Z, Zz = Z*Z,

    mat:matrix([
        [1 - 2*(Yy + Zz),     2*(Xy - Wz),     2*(Xz + Wy)],
            [    2*(Xy + Wz), 1 - 2*(Xx + Zz),     2*(Yz - Wx)],
            [    2*(Xz - Wy),     2*(Yz + Wx), 1 - 2*(Xx + Yy)]
    ]).

%% Normalize a vector
normalize(Vec) ->
    Norm = math:sqrt(lists:sum([X*X || X <- Vec])),
    case Norm of
        0 -> Vec;
        _ -> [X / Norm || X <- Vec]
    end.

    
quat_to_euler([W, X, Y, Z]) ->
    % Roll (rotation autour de X)
    Sinr_cosp = 2 * (W * X + Y * Z),
    Cosr_cosp = 1 - 2 * (X * X + Y * Y),
    Roll = math:atan2(Sinr_cosp, Cosr_cosp),

    % Pitch (rotation autour de Y)
    Sinp = 2 * (W * Y - Z * X),
    Pitch = case erlang:abs(Sinp) >= 1 of
        true -> math:pi() / 2 * math:sign(Sinp); % clamp to +-90°
        false -> math:asin(Sinp)
    end,

    % Yaw (rotation autour de Z)
    Siny_cosp = 2 * (W * Z + X * Y),
    Cosy_cosp = 1 - 2 * (Y * Y + Z * Z),
    Yaw = math:atan2(Siny_cosp, Cosy_cosp),

    [Roll, Pitch, Yaw].


ekf_predict({X0, P0}, {F, Jf}, Q) ->
    Xp = F(X0),
    Jfx = Jf(X0),
    Pp = mat:eval([Jfx, '*', P0, '*´', Jfx, '+', Q]),
    {Xp,Pp}.