-module(kalman_measure_V2).

-behavior(hera_measure).

-export([init/1, measure/1]).

-define(VAR_Q, 0.001).
-define(VAR_R, 0.01).

-define(VAR_S, 0.01). % (0.2/2)^2
-define(VAR_P, 0.0025). % (0.1/2)^2
-define(VAR_AL, 0.1).
-define(VAR_AZ, 0.1).

init(_Args) ->
    timer:sleep(2000),
    io:format("~n[KALMAN_MEASURE] Starting measurements~n"),
    T0 = hera:timestamp(),
    Xpos = mat:zeros(9, 1),
    Ppos = mat:eye(9),
    Xor = [[1],[0],[0],[0]],
    Por = mat:diag([10,10,10,10]),
    State = {T0, Xpos, Ppos, Xor, Por, R0},
    {ok, State,#{seq => 2}, #{
        name => kalman_measure,
        iter => infinity,
        timeout => 300
    }}.


measure(State) ->
    case hera_data:get(robot_pos, robot) of 
        [{_, _, _, [OldX,OldY, OldAngle, OldRoom]}] ->
            Seq = maps:get(seq, State, 1),
            NewState = State#{seq => Seq +1},
            T1 = hera:timestamp(),
        
            % take the sonar value
            [Sensor1, Sensor2] = get_room_sensors(Room),
            {X1, Y1, A1} = get_sensor_pos(Sensor1),
            {X2, Y2, A2} = get_sensor_pos(Sensor2),
            [{_, _, _, [Dist1]}] = hera_data:get(distance, Sensor1), 
            [{_, _, _, [Dist2]}] = hera_data:get(distance, Sensor2),
            {{Xout, Yout, Zout}} = get_pos_2({X1, Y1}, {X2, Y2}, {Dist1} , {Dist2}), % faut changer dans le code avant pour avoir qu'une donnée
            Sonars = {0,0,0} % A voir comment l'initialiser
            % take PmodNav value
            
            R = q2dcm(Xor),
            {Acc,Acclin,Gyro,Mag} = get_nav(R),
            Dtpos = (T1 - T0)/1000,

            %Faut verif cette partie
            
            Fpos = [
                [1,Dtpos,(Dtpos*Dtpos)/2,0,0,0,0,0,0], % X 
                [0,1,Dtpos,0,0,0,0,0,0], % V_X
                [0,0,1,0,0,0,0,0,0], % Acc_X
                [0,0,0,1,Dtpos,(Dtpos*Dtpos)/2,0,0,0], % Y
                [0,0,0,0,1,Dtpos,0,0,0], % V_Y
                [0,0,0,0,0,1,0,0,0], % Acc_Y
                [0,0,0,0,0,0,1,Dtpos,(Dtpos*Dtpos)/2], % Z
                [0,0,0,0,0,0,0,1,Dtpos], % V_Z
                [0,0,0,0,0,0,0,0,1] % Acc_Z
            ],
            Qpos = mat:diag([?VAR_P,?VAR_P,?VAR_AL, ?VAR_P,?VAR_P,?VAR_AL, ?VAR_P,?VAR_P,?VAR_AL]),
            Hpos =  [[1,0,0,0,0,0,0,0,0] || _ <- Xout] ++ 
                    [[0,0,0,1,0,0,0,0,0] || _ <- Yout] ++
                    [[0,0,0,0,0,0,1,0,0] || _ <- Zout] ++ 
                    [[0,0,1,0,0,0,0,0,0] || _ <- AccLin] ++
                    [[0,0,0,0,0,1,0,0,0] || _ <- AccLin] ++
                    [[0,0,0,0,0,0,0,0,1] || _ <- AccLin],
            Zpos =  [[X] || X <- Xout] ++
                    [[Y] || Y <- Yout] ++
                    [[Z] || Z <- Zout] ++
                    [[Ax] || [Ax,_,_] <- AccLin] ++
                    [[Ay] || [_,Ay,_] <- AccLin] ++
                    [[Az] || [_,_,Az] <- AccLin],
            Rpos = mat:diag(
                    [?VAR_S || _ <- Sonars] ++
                    [?VAR_AZ || _ <- AccLin] ++
                    [?VAR_AZ || _ <- AccLin] ++
                    [?VAR_AZ || _ <- AccLin]
            ),

            {Xpos0, Ppos0} = kalman:kf_predict({Xpos,Ppos}, Fpos, Qpos),
            {Xpos1,Ppos1} =  kalman:kf_update({Xpos0, Ppos0}, Hpos, Rpos, Zpos),


            [_,_,[Accx],_,_,[Accy],_,_,[Accz]] = Xpos1,
            AccLin2 = [[Accx,Accy,Accz]],
            AccRot2 = mat:'*'(AccLin2, R),
            [Acc1,Acc2,Acc3] = Acc, 
            [[AccRotx,AccRoty,AccRotz]] = AccRot2, 
            R1 = ahrs([Acc1-AccRotx,Acc2-AccRoty,Acc3-AccRotz], Mag),

            Quat = dcm2quat(mat:'*'(R1, R0)),
            Dtor = (T1-T0)/1000, 
            [Wx,Wy,Wz] = Gyro,

            Omega = [
                [0,Wx,Wy,Wz],
                [-Wx,0,-Wz,Wy],
                [-Wy,Wz,0,-Wx],
                [-Wz,-Wy,Wx,0]
            ],

            For = mat:'+'(mat:eye(4), mat:'*'(0.5*Dtor, Omega)),
            Qor = mat:diag([?VAR_Q,?VAR_Q,?VAR_Q,?VAR_Q]),
            Hor = mat:eye(4),
            Zor = mat:tr([Quat]),
            Ror = mat:diag([?VAR_R,?VAR_R,?VAR_R,?VAR_R]),

            {Xor0, Por0} = kalman:kf_predict({Xor,Por}, For, Qor),
            {Xor1, Por1} = case qdot(Zor, Xor0) > 0 of
                true ->
                    kalman:kf_update({Xor0, Por0}, Hor, Ror, Zor);
                false ->
                    kalman:kf_update({mat:'*'(-1,Xor0), Por0}, Hor, Ror, Zor)
                end,

                Valor = unit([X || [X] <- Xor1]),
                Xor1Norm = [[X] || X <- Valor],

                Valpos = lists:append(Xpos1),

                {ok, Valpos ++ Valor, {T1, Xpos1, Ppos1, Xor1Norm, Por1, R0}}

    end.    




%============================================================================================================================================
%======================================================= HELPER FUNC ========================================================================
%============================================================================================================================================       


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
            {TLx, TLy, BRx, BRy}
        _ ->
            io:format("[KALMAN_MEASURE] Can't get the pos of room : ~p~n", [SensorName])
    end.

get_pos_2({X1,Y1}, {X2,Y2}, {Dist1}, {Dist2},{TLx, TLy, BRx, BRy}) ->
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
            
            {{Xout/100, Yout/100,0}} = check_good_point(Xout1, Yout1, Xout2, Yout2, TlX, TlY, BRx, BRy)
    end.

check_good_point(Xout1, Yout1, Xout2, Yout2, TlX_str, TlY_str, BRx_str, BRy_str) ->
    
    {TlX, _} = string:to_integer(TlX_str),
    {TlY, _} = string:to_integer(TlY_str),
    {BRx, _}  = string:to_integer(BRx_str),
    {BRy, _}  = string:to_integer(BRy_str),
    MaxRoomX = math:max(TLx,BRx), %jsp si faut diviser par 100 ?
    MaxRoomy = math:max(TLy,BRy), %jsp si faut diviser par 100 ?
    case {Xout1 < MaxRoomX, Yout1 < MaxRoomy} of
        {true, true} ->
            {ok, Xout1, Yout1};
        _ ->
            case {Xout2 < MaxRoomX, Yout2 < MaxRoomy} of
                {true, true} ->
                    {ok, Xout2, Yout2};
                _ ->
                    none
            end
    end.

get_nav(R) ->
    [{Ax, Ay, Az}] = pmod_nav:read(acc, [out_x_xl, out_y_xl, out_z_xl]), % Get angular velocity x,y,z
    [{Gx, Gy, Gz}] = pmod_nav:read(acc, [out_x_g, out_y_g, out_z_g]), % Get angular velocity x,y,z
    [{Mx,My,Mz}] = pmod_nav:read(mag, [out_x_m, out_y_m, out_z_m]),
    % jsp si on doit utiliser calibrate pour avoir valeur plus juste à vérifier

    Acc = [Ax,Ay,-Az],
    Gyro = [Gx-GBx,Gy-GBy,-(Gz-GBz)],
    [-(Mx-MBx),My-MBy,-(Mz-MBz)],
    Accrot = mat:'*'([Acc], mat:tr(R)),
    RotAcc = mat:'-'(Accrot, [[0,0,-9.81]]),

    {scale(Acc,9.81),RotAcc,scale(Gyro,math:pi()/180),Mag}.





ahrs(Acc, Mag) ->
    Down = unit([-A || A <- Acc]),
    East = unit(cross_product(Down, unit(Mag))),
    North = unit(cross_product(East, Down)),
    mat:tr([North, East, Down]).

cross_product([U1,U2,U3], [V1,V2,V3]) -> 
    [U2*V3-U3*V2, U3*V1-U1*V3, U1*V2-U2*V1].


unit(V) ->
    Norm = math:sqrt(lists:sum([X*X || X <- V])),
    [X/Norm || X <- V].


dcm2quat(R) ->
    [[R11,R12,R13],
     [R21,R22,R23],
     [R31,R32,R33]
    ] = R,
    Q12 = 0.25*(1+R11+R22+R33),
    Q1 = math:sqrt(Q12),
    V = [
        4*Q12,
        R32-R23,
        R13-R31,
        R21-R12
    ],
    scale(V, (0.25/Q1)).


qdot([[Q11], [Q12], [Q13], [Q14]], [[Q21], [Q22], [Q23], [Q24]]) ->
    Q11*Q21 + Q12*Q22 + Q13*Q23 + Q14*Q24.


scale(List, Factor) ->
    [X*Factor || X <- List].


q2dcm([[Q0], [Q1], [Q2], [Q3]]) -> 
    R00 = 2 * (Q0 * Q0 + Q1 * Q1) - 1,
    R01 = 2 * (Q1 * Q2 - Q0 * Q3),
    R02 = 2 * (Q1 * Q3 + Q0 * Q2),
     
    R10 = 2 * (Q1 * Q2 + Q0 * Q3),
    R11 = 2 * (Q0 * Q0 + Q2 * Q2) - 1,
    R12 = 2 * (Q2 * Q3 - Q0 * Q1),
     
    R20 = 2 * (Q1 * Q3 - Q0 * Q2),
    R21 = 2 * (Q2 * Q3 + Q0 * Q1),
    R22 = 2 * (Q0 * Q0 + Q3 * Q3) - 1,

    [[R00, R01, R02],
     [R10, R11, R12],
     [R20, R21, R22]].
