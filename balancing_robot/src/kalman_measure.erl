-module(kalman_measure).

-behavior(hera_measure).

-export([init/1, measure/1]).

-define(VAR_Q, 0.001).
-define(VAR_R, 0.01).

-define(VAR_S, 0.01). % (0.2/2)^2
-define(VAR_V, 0.05).
-define(VAR_P, 0.0025). % (0.1/2)^2
-define(VAR_AL, 0.1).
-define(VAR_AZ, 0.1).

init(_Args) ->
    timer:sleep(2000),
    io:format("~n[KALMAN_MEASURE] Starting measurements~n"),
    calibrate(),

    State = #{
        t0 => hera:timestamp(),
        x_pos => mat:zeros(6, 1),
        p_pos => mat:eye(6),
        x_or => [[1],[0],[0],[0]],
        p_or => mat:diag([10,10,10,10]),
        yaw => mat:eye(1),
        seq => 2
    },

    {ok, State, #{
        name => kalman_measure,
        iter => infinity,
        timeout => 300
    }}.

measure(State) ->
    #{ 
        t0   := T0,
        x_pos := Xpos,
        p_pos := Ppos,
        x_or := Xor,
        p_or := _Por,
        yaw := Yaw,
        seq  := Seq
        
    } = State,

    case hera_data:get(robot_pos, robot) of 
        [{_, _, _, [_OldX,_OldY, OldAngle, OldRoom]}] ->
            T1 = hera:timestamp(),
            % Faire un process nav
            R = q2dcm(Xor),
            {Acc,Acclin,Gyro,Mag,R0} = get_val_nav(R),
            
            case get_new_robot_pos(OldRoom) of
                no_intersection ->
                    correction = 1, %%sert à changer les variables de la matrice Q au plus on l'augmente au plus il est dit qu'on ne fait pas confiance à la matrice Q.
                    Dt = (T1 - T0)/1000,

                    
                    [{Xor1,Por1}] = kalman_orientation(Acc, Acclin, Gyro, Mag, R0, Xnew), %A check si je peux faire ça comme ça 
                    
                    
                    
                    Xarray2 = mat:to_array(Xor1),
                    
                    % Pour visualiser il faut utiliser le yaw =>
                    Yaw = quat_to_yaw(normalize_quat(Xor1)), 

                    % Prendre la vitesse des roues du robot 
                    {Speed,_} = main_loop:i2c_read(),
                    Vx = Speed * math:cos(Yaw),
                    Vy = Speed * math:sin(Yaw),


                    F = mat:matrix([
                        [1, Dt, Dt*Dt/2, 0, 0, 0],
                        [0, 1, Dt,       0, 0, 0],
                        [0, 0, 1,        0, 0, 0],
                        [0, 0, 0,        1, Dt, Dt*Dt/2],
                        [0, 0, 0,        0, 1, Dt],
                        [0, 0, 0,        0, 0, 1]
                    ]),

                    Q = mat:diag([?VAR_P*correction, ?VAR_P*correction, ?VAR_AL*correction, ?VAR_P*correction, ?VAR_P*correction, ?VAR_AL*correction]),
                    {Xpred, Ppred} = kalman:kf_predict({Xpos, Ppos}, F, Q),
                    
                    
                    
                    [_, _, [Axlin], _, _, [Aylin]] = Xpred,
                    AccLin2 = [[Axlin, Aylin, 0.0]],

                    [{Xor1,Por1}] = kalman_orientation(Acc,AccLin2,Gyro,Mag,R0),

                    Xarray2 = mat:to_array(Xor1),
                    
                    % Pour visualiser il faut utiliser le yaw =>
                    Yaw = quat_to_yaw(normalize_quat(Xor1)), 

                    % Prendre la vitesse des roues du robot 
                    {Speed,_} = main_loop:i2c_read(),
                    Vx = Speed * math:cos(Yaw),
                    Vy = Speed * math:sin(Yaw),

                    Z = mat:matrix([[Vx], [Vy], [AxLin], [AyLin]]),
                    R = mat:diag([?VAR_V,?VAR_V,?VAR_AZ,?VAR_AZ]),

                    
                    {Xnew, Pnew} = kalman:kf_update({Xpred, Ppred}, H, R, Z),
                    
                    Xarray1 = mat:to_array(Xnew),
                    Xarray = Xarray1 ++ Xarray2,

                    NewState = #{ 
                        t0   => T1,
                        x_pos => Xnew,
                        p_pos => Pnew,
                        x_or => Xor1,
                        p_or => Por1,
                        yaw => Yaw,
                        seq  => Seq +1
                    },
                    
                    hera_data:store(robot_pos, robot, Seq, [lists:nth(1, Xarray), lists:nth(4, Xarray), OldAngle, OldRoom]),
                    send_robot_pos([lists:nth(1, Xarray), lists:nth(4, Xarray), OldAngle, OldRoom]),

                    {ok, [lists:nth(1, Xarray), lists:nth(4, Xarray), OldAngle, OldRoom], robot_pos, robot, NewState};
                    
                {Xout, Yout} ->
                    Dt = (T1 - T0)/1000,


                    % kalman position
                    F = mat:matrix([
                        [1, Dt, Dt*Dt/2, 0, 0, 0],
                        [0, 1, Dt,       0, 0, 0],
                        [0, 0, 1,        0, 0, 0],
                        [0, 0, 0,        1, Dt, Dt*Dt/2],
                        [0, 0, 0,        0, 1, Dt],
                        [0, 0, 0,        0, 0, 1]
                    ]),

                    Q = mat:diag([?VAR_P, ?VAR_P, ?VAR_AL, ?VAR_P, ?VAR_P, ?VAR_AL]),
                    H = mat:matrix([
                        [1,0,0,0,0,0], % mesure X
                        [0,0,0,1,0,0],  % mesure Y
                        [0,1,0,0,0,0], % Vitesse X
                        [0,0,0,0,1,0], % Vitesse Y
                        [0,0,1,0,0,0], %Acc x
                        [0,0,0,0,0,1]  %ACC y   
                    
                    ]),
                
                    {Xpred, Ppred} = kalman:kf_predict({Xpos, Ppos}, F, Q),

                    [_, _, [Axlin], _, _, [Aylin]] = Xpred,
                    AccLin2 = [[Axlin, Aylin, 0.0]],

                    [{Xor1,Por1}] = kalman_orientation(Acc,AccLin2,Gyro,Mag,R0),

                    Xarray2 = mat:to_array(Xor1),
                    
                    % Pour visualiser il faut utiliser le yaw =>
                    Yaw = quat_to_yaw(normalize_quat(Xor1)), 

                    % Prendre la vitesse des roues du robot 
                    {Speed,_} = main_loop:i2c_read(),
                    Vx = Speed * math:cos(Yaw),
                    Vy = Speed * math:sin(Yaw),

                    Z = mat:matrix([[Xout],[Yout],[Vx],[Vy],[AxLin],[AyLin]]),
                    R = mat:diag([?VAR_S, ?VAR_S,?VAR_V,?VAR_V,?VAR_AZ,?VAR_AZ]),

                    
                    {Xnew, Pnew} = kalman:kf_update({Xpred, Ppred}, H, R, Z),
                    
                    Xarray1 = mat:to_array(Xnew),
                    Xarray = Xarray1 ++ Xarray2,

                    NewState = #{ 
                        t0   => T1,
                        x_pos => Xnew,
                        p_pos => Pnew,
                        x_or => Xor1,
                        p_or => Por1,
                        yaw => Yaw,
                        seq  => Seq +1
                    },
                    
                    hera_data:store(robot_pos, robot, Seq, [lists:nth(1, Xarray), lists:nth(4, Xarray), OldAngle, OldRoom]),
                    send_robot_pos([lists:nth(1, Xarray), lists:nth(4, Xarray), OldAngle, OldRoom]),

                    {ok, [lists:nth(1, Xarray), lists:nth(4, Xarray), OldAngle, OldRoom], robot_pos, robot, NewState}
            end
    end.               
%============================================================================================================================================
%======================================================= CALIBRATION FUNC ========================================================================
%============================================================================================================================================

calibrate() ->
    N = 500,
    {G_x_List, G_y_List, G_z_List} = [pmod_nav:read(acc, [out_x_g, out_y_g, out_z_g]) || _ <- lists:seq(1, N)],
    {M_x_List_ref, M_y_List_ref, M_z_List_ref} = [pmod_nav:read(acc, [out_x_m, out_y_m, out_z_m]) || _ <- lists:seq(1, N)],

    {Gx0_pos, Gy0_pos, Gz0_pos} = [lists:sum([Y || [Y] <- List]) / N || List <- [G_x_List, G_y_List, G_z_List]],
    {M_x_List, M_y_List, M_z_List} = [lists:sum([Y || [Y] <- List]) / N || List <- [M_x_List_ref, M_y_List_ref, M_z_List_ref]],

    io:format("[KALMAN_MEASURE] Done calibrating~n"),
    persistent_term:put(gyro_init, {Gx0_pos, Gy0_pos, Gz0_pos}),
    persistent_term:put(mag_init, {M_x_List, M_y_List, M_z_List}).    

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


ahrs(Acc, Mag) ->
    Down = unit([-A || A <- Acc]),
    East = unit(cross_product(Down, unit(Mag))),
    North = unit(cross_product(East, Down)),
    mat:tr([North, East, Down]).

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

scale(List, Factor) ->
    [X*Factor || X <- List].


get_val_nav(R) ->


    [Ax, Ay, Az] = pmod_nav:read(acc, [out_x_xl, out_y_xl, out_z_xl]),
    [Gx, Gy, Gz] = pmod_nav:read(acc, [out_x_g, out_y_g, out_z_g]),
    [Mx, My, Mz] = pmod_nav:read(acc, [out_x_m, out_y_m, out_z_m]),

    Acc = scale([Ax,Ay,-Az],9.81),

    [GBx, GBy, GBz] = persistent_term:get(gyro_init),
    Gyro = scale([Gx-GBx,Gy-GBy,-(Gz-GBz)],math:pi()/180),

    [MBx,MBy,MBz] = persistent_term:get(gyro_init),
    Mag = [-(Mx-MBx),My-MBy,-(Mz-MBz)],

    AccRot = mat:'*'([Acc], mat:tr(R)),  % rotation dans le repère monde
    RotAcc = mat:'-'(AccRot, [[0, 0, -9.81]]),  % compensation gravité

    R0 = ahrs([Ax,Ay,-Az], [-(Mx-MBx),My-MBy,-(Mz-MBz)]),
    mat:tr(R0),
    {Acc, RotAcc, Gyro, Mag,R0}. 


quat_to_yaw([[Q0], [Q1], [Q2], [Q3]]) ->
    math:atan2(2*(Q0*Q3 + Q1*Q2), 1 - 2*(Q2*Q2 + Q3*Q3)).

normalize_quat([[Q0], [Q1], [Q2], [Q3]]) ->
    Norm = math:sqrt(Q0*Q0 + Q1*Q1 + Q2*Q2 + Q3*Q3),
    [[Q0 / Norm], [Q1 / Norm], [Q2 / Norm], [Q3 / Norm]].

qdot([[Q11], [Q12], [Q13], [Q14]], [[Q21], [Q22], [Q23], [Q24]]) ->
    Q11*Q21 + Q12*Q22 + Q13*Q23 + Q14*Q24.

cross_product([U1,U2,U3], [V1,V2,V3]) -> 
    [U2*V3-U3*V2, U3*V1-U1*V3, U1*V2-U2*V1].


kalman_orientation(Acc, AccLin2, Gyro, Mag, R0) ->
    
    AccRot2 = mat:'*'(AccLin2, Orientation),
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
    {Xor1,Por1}.