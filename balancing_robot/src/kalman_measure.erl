-module(kalman_measure).

-behavior(hera_measure).

-export([init/1, measure/1]).

%%%% incertitude dynamique %%%%
% plus la valeur est grande, moins tu fais confiance à ton modèle %

%Kalman orientation
% Il faut absolument que je recheck ça
-define(VAR_Q, 0.0002).   % variance sur θ essayé X10
-define(VAR_R, 0.15).  % variance gyroscope (≈ std² 0.06208) essayer X10
%%% incertitude dynamique (Q) %%%
% plus c’est petit, plus tu fais confiance à ton modèle %


-define(VAR_P, 0.0002).   % variance sur X et Y

%%% fiabilité capteur (R) %%%
% plus c’est petit, plus tu fais confiance en la mesure %
-define(VAR_S, 0.0075).  

-define(RAD_TO_DEG, 180.0/math:pi()).
-define(DEG_TO_RAD, math:pi()/180.0).


init(_Args) ->
    timer:sleep(1000),
    io:format("~n[KALMAN_MEASURE] Starting measurements~n"),
    calibrate2(),

    calibrate_speed(),
    State = #{
        t0 => erlang:system_time()/1.0e6,
        x_pos => mat:matrix([[0],[0]]),
        p_pos => mat:diag([1,1]),
        x_or => mat:matrix([[1],[0],[0],[0]]),
        p_or => mat:diag([1,1,1,1]),
        seq => 2,
        seqS1 => 0,
        seqS2 => 0
    },

    {ok, State, #{
        name => kalman_measure,
        iter => infinity,
        timeout => 100
    }}.

measure(State) ->
    #{ 
        t0 := T0,
        x_pos := Xpos,
        p_pos := Ppos,
        x_or := Xor,
        p_or := Por,
        seq  := Seq,
        seqS1 := SeqS1,
        seqS2 := SeqS2
        
    } = State,

    case hera_data:get(robot_pos, robot) of 
        [{_, _, _, [_OldX,_OldY, _OldAngle, OldRoom]}] ->
            

            
            {V_mes_mm,_} = i2c_read(),
            V_mes = V_mes_mm / 100,
            

            T1 = erlang:system_time()/1.0e6,


            %%%%%% Kalman Orientation %%%%%%

            {Xor1,Por1} = kalman_orientation(Xor, Por, T1, T0),
            Offset = 12 * ?DEG_TO_RAD,
            Theta_mes = - quat_to_yaw(normalize_quat(mat:to_array(Xor1))) - Offset,
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


            Q  = mat:diag([?VAR_P, ?VAR_P]),
            
            % Fonction de transition f(x)
            Dt = (T1 - T0) / 1000.0,

            F = fun(X) ->
                [Yc, Zc] = mat:to_array(X),

                Yp = Yc - V_mes *  math:cos(Theta_mes)* Dt,
                %io:format("ThetaC : ~p et cos(thetaC) : ~p ~n ",[Thetac,math:cos(Thetac)]),
                Zp = Zc - V_mes * math:sin(Theta_mes) * Dt,
                mat:matrix([[Yp], [Zp]])
            end,
            
            % Jacobienne de f
            Jf = fun(_X) ->
                mat:matrix([
                [1, 0],
                [0, 1]
                ])
            end,

            {Xpred,Ppred} = hera_kalman:extended_predict({Xpos, Ppos}, {F, Jf}, Q),
            
            case get_new_robot_pos(OldRoom) of
                {no_intersection, { _, _}} ->
                    [Xf, Yf] = mat:to_array(Xpred),
                    ThetaDegrees = Theta_mes * ?RAD_TO_DEG,
                    NewState = #{ 
                        t0   => T1,
                        x_pos => Xpred,
                        p_pos => Ppred,
                        x_or => Xor1,
                        p_or => Por1,
                        seq  => Seq +1,
                        seqS1 => SeqS1,
                        seqS2 => SeqS2
                    };
                


                {{X_mes, Y_mes},{Seqsensor1,Seqsensor2}} -> 
                    case {SeqS1 < Seqsensor1 andalso SeqS2 < Seqsensor2} of
                        {true} ->
                            
                            Z = mat:matrix([[X_mes],[Y_mes]]),
                            io:format("Valeur reçu sonar, ~p, ~p ~n", [X_mes,Y_mes]),

                            R  = mat:diag([?VAR_S, ?VAR_S]),

                          
                            % Fonction de mesure h(x) = x
                            H = fun(X) ->
                                [Xc, Yc] = mat:to_array(X),

                                mat:matrix([[Xc],[Yc]])
                            end,
                            Jh = fun(_X) ->
                                mat:matrix([
                                [1,0],
                                [0,1]
                                ])
                            end,
                            {Xnew, Pnew} = hera_kalman:extended_update({Xpred, Ppred}, {H, Jh}, R, Z),
                            
                            [Xf, Yf] = mat:to_array(Xnew),
                            ThetaDegrees =  Theta_mes * ?RAD_TO_DEG,

                            NewState = #{ 
                                t0   => T1,
                                x_pos => Xnew,
                                p_pos => Pnew,
                                x_or => Xor1,
                                p_or => Por1,
                                seq  => Seq +1,
                                seqS1 =>Seqsensor1,
                                seqS2 =>Seqsensor2

                            };
                        {false} ->
                            
                            [Xf, Yf] = mat:to_array(Xpred),
                            ThetaDegrees = Theta_mes * ?RAD_TO_DEG,
                            %io:format("False : ~p ~n", [ThetaDegrees]),

                            NewState = #{ 
                                t0   => T1,
                                x_pos => Xpred,
                                p_pos => Ppred,
                                x_or => Xor1,
                                p_or => Por1,
                                seq  => Seq +1,
                                seqS1 =>SeqS1,
                                seqS2 =>SeqS2

                            }
                    end
     
            end,
            
            io:format("pos : ~p, ~p : ~p, v : ~p ~n", [Xf,Yf,T1,V_mes]),
            io:format("angle :~p , ~p~n ", [ThetaDegrees,T1]),

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%   Store and send new data  %%%%%%%%%%% 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Room = determine_robot_room(Xf, Yf, OldRoom),
            
        if
            Room =/= OldRoom ->
                io:format(  " ~n JAI CHANNNGGGEEE DE ROOOMMMMM  ~n");
            true ->
                io:format("pas change de room")
        end,

            hera_data:store(robot_pos, robot, Seq, [Xf, Yf,  ThetaDegrees, Room]),
            send_robot_pos(Seq, [Xf, Yf,  ThetaDegrees, Room]),
            {no_share, NewState}; % If no propag graph, delete prev line and replace no_share by a ok clause (see hera_measure module)
        [] ->
            {undefined, State}
    
    end.              
%============================================================================================================================================
%======================================================= CALIBRATION FUNC ===================================================================
%============================================================================================================================================

calibrate2() ->
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

send_robot_pos(Seq, [Xf,Yf, ThetaDegrees, Room]) ->
    Msg = "Robot_pos," ++ integer_to_list(Seq) ++","++ float_to_list(Xf) ++ "," ++ float_to_list(Yf) ++ "," ++ float_to_list(ThetaDegrees) ++ "," ++ integer_to_list(Room),
    Adjacent_sensors = get_adjacent_sensors(Room), % To impose the propagation graph, not mandatory
    [hera_com:send_unicast(Device, Msg, "UTF8") || Device <- [server | Adjacent_sensors]].

get_new_robot_pos(Room) ->
    [Sensor1, Sensor2] = get_room_sensors(Room),

    io:format("[KALMAN_MEASURE] The two sensors in the current room are : ~p and ~p ~n",[Sensor1, Sensor2]),
    {X1, Y1, _} = get_sensor_pos(Sensor1),
    {X2, Y2, _} = get_sensor_pos(Sensor2),
    [{_, Seqsensor1,_,[Dist1]}] = hera_data:get(distance, Sensor1), % distance on the ground
    [{_, Seqsensor2,_,[Dist2]}] = hera_data:get(distance, Sensor2),
    {TLx, TLy, BRx, BRy} = get_room_info(Room),
    {get_pos({X1, Y1}, {X2, Y2}, {Dist1} , {Dist2},{TLx, TLy, BRx, BRy}),{Seqsensor1,Seqsensor2}}.

get_room_sensors(Room) ->
    % Returns the two sensors in the current room
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

get_adjacent_sensors(Room) ->
    % Returns all the sensors in the current room or in the adjacent rooms.
    Devices = persistent_term:get(devices),
    lists:foldl(
        fun({Name, _, _}, Acc) ->
            case Name of
                _ ->
                    case hera_data:get(room, Name) of
                        [{_, _, _, [ORoom]}] when ((Room =:= ORoom) orelse (Room+1 =:= ORoom) orelse (Room-1 =:=ORoom)) ->
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


determine_robot_room(X, Y, OldRoom) ->
    determine_robot_room(X, Y, OldRoom, 0).
determine_robot_room(X, Y, OldRoom, RoomNum) ->
    case hera_data:get(room_info, RoomNum) of
        [{_, _, _, [TLx, TLy, BRx, BRy]}] ->
            if 
                (X > TLx andalso X < BRx) andalso (Y > TLy andalso Y < BRy) ->
                    
                    RoomNum;
                true ->
                    determine_robot_room(X, Y, OldRoom, RoomNum+1)
            end;
        [] ->
            io:format("[KALMAN_MEASURE] Error: Not in a known room~n"),
            OldRoom
    end.

scale(List, Factor) ->
    [X*Factor || X <- List].

get_val_nav_2(R) ->

    [Ax, Ay, Az] = pmod_nav:read(acc, [out_x_xl, out_y_xl, out_z_xl]),
    [Gx, Gy, Gz] = pmod_nav:read(acc, [out_x_g, out_y_g, out_z_g]),
    [Mx, My, Mz] = pmod_nav:read(mag, [out_x_m, out_y_m, out_z_m]),
    {Ax0, Ay0, Az0} = persistent_term:get(acc_init),
    %Acc = scale([Ax - Ax0, Ay - Ay0, Az - Az0], 9.81),
    Acc = scale([ (Az - Az0), (Ay - Ay0), -(Ax - Ax0)], 9.81),

    
    {GBx, GBy, GBz} = persistent_term:get(gyro_init),
    %Gyro = scale([Gx-GBx,Gy-GBy,Gz-GBz],math:pi()/180),
    Gyro = scale([(Gz-GBz),(Gy-GBy),-(Gx-GBx)],math:pi()/180),
 
    {MBx,MBy,MBz} = persistent_term:get(mag_init),
    %Mag = mat:matrix([[Mx-MBx,My-MBy,Mz-MBz]]),
    Mag = mat:matrix([[(Mz-MBz),(My-MBy),-(Mx-MBx)]]),


    AccRot = mat:'*'(mat:matrix([Acc]), mat:tr(R)),  % rotation dans le repère monde
    RotAcc = mat:'-'(AccRot, mat:matrix([[9.81, 0, 0]])),  % compensation gravité

    %R0 = ahrs([Ax,Ay,Az], [(Mx-MBx),My-MBy,(Mz-MBz)]),
    R0 = ahrs([Az,Ay,-Ax], [(Mz-MBz),(My-MBy),-(Mx-MBx)]),
    mat:tr(R0),

    {mat:matrix([Acc]), RotAcc, mat:matrix([Gyro]), Mag,R0}. 

i2c_read() ->
    %Receive I2C and conversion
    I2Cbus = persistent_term:get(i2c),
    [<<SL1,SL2,SR1,SR2,CtrlByte>>] = grisp_i2c:transfer(I2Cbus, [{read, 16#40, 1, 5}]),
    {OffsetL,OffsetR} = persistent_term:get(i2c_offset),
    [Speed_L,Speed_R] = hera_com:decode_half_float([<<SL1, SL2>>, <<SR1, SR2>>]),
    Speed2 = ((Speed_L - OffsetL) + (Speed_R - OffsetR))/2,
    Speed = case erlang:abs(Speed2)/100 < 0.08 of
        true -> 0.0;
        false -> Speed2
    end,
    if
    (Speed_L < 0 andalso Speed_R > 0) orelse (Speed_L > 0 andalso Speed_R < 0) ->
        io:format("Signes opposés~n"),

        {0.0, CtrlByte};
    true ->
        io:format("Autre cas~n"),
        {Speed, CtrlByte}

    end.

quat_to_yaw([[Q0], [Q1], [Q2], [Q3]]) ->
    math:atan2(2*(Q0*Q3 + Q1*Q2), 1 - 2*(Q2*Q2 + Q3*Q3)).
normalize_quat([Q0, Q1, Q2, Q3]) ->
    Norm = math:sqrt(Q0*Q0 + Q1*Q1 + Q2*Q2 + Q3*Q3),
    [[Q0 / Norm], [Q1 / Norm], [Q2 / Norm], [Q3 / Norm]].

kalman_orientation(Xor,Por,T1,T0) ->
    Dtor = (T1-T0)/1000.0,
    Rorien = q2dcm(mat:to_array(Xor)),
    {Acc, _Acclin, Gyro, Mag, R0} = get_val_nav_2(Rorien),
    [Acx,Acy,Acz] = mat:to_array(Acc),
    [Mx,My,Mz] = mat:to_array(Mag),
    R1 = ahrs([Acx,Acy,Acz], [Mx,My,Mz]),
    Quat = dcm2quat(mat:'*'(R1,R0)),
    [Wxx,Wyy,Wzz] = mat:to_array(Gyro),
    [Wx,Wy,Wz] = [Wxx,Wyy,Wzz],
    Omega = mat:matrix([
        [0,Wx,Wy,Wz],
        [-Wx,0,-Wz,Wy],
        [-Wy,Wz,0,-Wx],
        [-Wz,-Wy,Wx,0]
    ]),

    For = mat:'+'(mat:eye(4), mat:'*'(0.5 * Dtor, Omega)),
    Qor = mat:diag([?VAR_Q,?VAR_Q,?VAR_Q,?VAR_Q]),
    Hor = mat:eye(4),
    Zor = mat:tr(Quat),
    Ror = mat:diag([?VAR_R,?VAR_R,?VAR_R,?VAR_R]),

    {Xor0, Por0} = hera_kalman:predict({Xor,Por}, For, Qor),
    {Xor1, Por1} = hera_kalman:update({Xor0, Por0}, Hor, Ror, Zor),
    {Xor1,Por1}.

q2dcm([Q0, Q1, Q2, Q3]) -> 
    R00 = 2 * (Q0 * Q0 + Q1 * Q1) - 1,
    R01 = 2 * (Q1 * Q2 - Q0 * Q3),
    R02 = 2 * (Q1 * Q3 + Q0 * Q2),
     
    R10 = 2 * (Q1 * Q2 + Q0 * Q3),
    R11 = 2 * (Q0 * Q0 + Q2 * Q2) - 1,
    R12 = 2 * (Q2 * Q3 - Q0 * Q1),
     
    R20 = 2 * (Q1 * Q3 - Q0 * Q2),
    R21 = 2 * (Q2 * Q3 + Q0 * Q1),
    R22 = 2 * (Q0 * Q0 + Q3 * Q3) - 1,

    mat:matrix([
    [R00, R01, R02],
    [R10, R11, R12],
    [R20, R21, R22]
    ]).

dcm2quat(R) ->
    [R11,R12,R13,R21,R22,R23,R31,R32,R33] = mat:to_array(R),
    Q12 = 0.25*(1+R11+R22+R33),
    Q1 = math:sqrt(Q12),
    V = [
        4*Q12,
        R32-R23,
        R13-R31,
        R21-R12
    ],
    mat:matrix([scale(V, (0.25/Q1))]).

ahrs(Acc, Mag) ->
    Down = unit([-A || A <- Acc]),
    East = unit(cross_product(Down, unit(Mag))),
    North = unit(cross_product(East, Down)),
    mat:tr(mat:matrix([North, East, Down])).
    
unit(Vec) ->
    Norm = math:sqrt(lists:sum([X*X || X <- Vec])),
    case Norm of
        0 -> Vec;
        _ -> [X / Norm || X <- Vec]
    end.

cross_product([U1,U2,U3], [V1,V2,V3]) -> 
    [U2*V3-U3*V2, U3*V1-U1*V3, U1*V2-U2*V1].