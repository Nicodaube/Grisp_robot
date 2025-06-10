-module(kalman_measure_V2).

-behavior(hera_measure).

-export([init/1, measure/1]).


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
            T0 = hera:timestamp(),
        
            % take the sonar value
            [Sensor1, Sensor2] = get_room_sensors(Room),
            {X1, Y1, A1} = get_sensor_pos(Sensor1),
            {X2, Y2, A2} = get_sensor_pos(Sensor2),
            [{_, _, _, [Dist1]}] = hera_data:get(distance, Sensor1), 
            [{_, _, _, [Dist2]}] = hera_data:get(distance, Sensor2),
            {{Xout1, Yout1}, {Xout2, Yout2}} = get_pos_2({X1, Y1}, {X2, Y2}, {Dist1} , {Dist2}), % faut changer dans le code avant pour avoir qu'une donnée

            % take PmodNav value
            
            R = q2dcm(Xor),

            {Acc,Acclin,Gyro,Mag} = get_nav(R)

            




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

get_pos_2({X1,Y1}, {X2,Y2}, {Dist1}, {Dist2}) ->
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

            {{Xout1/100, Yout1/100}, {Xout2/100, Yout2/100}}
    end.

get_nav(R) ->
    [{Ax, Ay, Az}] = pmod_nav:read(acc, [out_x_xl, out_y_xl, out_z_xl]), % Get angular velocity x,y,z
    [{Gx, Gy, Gz}] = pmod_nav:read(acc, [out_x_g, out_y_g, out_z_g]), % Get angular velocity x,y,z
    [{Mx,My,Mz}] = pmod_nav:read(mag, [out_x_m, out_y_m, out_z_m]),
    %% jsp si on doit utiliser calibrate pour avoir valeur plus juste à vérifier


    Acc = [Ax,Ay,-Az],
    Gyro = [Gx-GBx,Gy-GBy,-(Gz-GBz)],
    [-(Mx-MBx),My-MBy,-(Mz-MBz)],
    Accrot = mat:'*'([Acc], mat:tr(R)),
    RotAcc = mat:'-'(Accrot, [[0,0,-9.81]]),

    {scale(Acc,9.81),RotAcc,scale(Gyro,math:pi()/180),Mag}.


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
