-module(kalman_measure).

-behavior(hera_measure).

-export([init/1, measure/1]).


-define(VAR_S, 0.01). % Plus haut, plus il y a du bruit dans la mesure
-define(VAR_P, 0.0025). % Plus il est grand, moins on a confiance en kalman
-define(VAR_AL, 0.05). % Plus il est grand, plus l'accélération du robot peut etre grande

init(_Args) ->
    timer:sleep(2000),
    io:format("~n[KALMAN_MEASURE] Starting measurements~n"),

    State = #{
        t0 => hera:timestamp(),
        x_pos => mat:zeros(6, 1),
        p_pos => mat:eye(6),
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
        seq  := Seq
    } = State,

    case hera_data:get(robot_pos, robot) of 
        [{_, _, _, [_OldX,_OldY, OldAngle, OldRoom]}] ->
            T1 = hera:timestamp(),
            case get_new_robot_pos(OldRoom) of
                no_intersection ->
                    Correction = 1,
                    Dt = (T1 - T0)/1000,
                    F = mat:matrix([
                        [1, Dt, Dt*Dt/2, 0, 0, 0],
                        [0, 1, Dt,       0, 0, 0],
                        [0, 0, 1,        0, 0, 0],
                        [0, 0, 0,        1, Dt, Dt*Dt/2],
                        [0, 0, 0,        0, 1, Dt],
                        [0, 0, 0,        0, 0, 1]
                    ]),

                    Q = mat:diag([?VAR_P*Correction, ?VAR_P*Correction, ?VAR_AL*Correction, ?VAR_P*Correction, ?VAR_P*Correction, ?VAR_AL*Correction]),
                    {Xpred, Ppred} = kalman:kf_predict({Xpos, Ppos}, F, Q),
                    Xarray = mat:to_array(Xpred),
                    hera_data:store(robot_pos, robot, Seq, [lists:nth(1, Xarray), lists:nth(4, Xarray), OldAngle, OldRoom]),
                    send_robot_pos([lists:nth(1, Xarray), lists:nth(4, Xarray), OldAngle, OldRoom]),
                    
                    NewState = #{ 
                        t0   => T1,
                        x_pos => Xpred,
                        p_pos => Ppred,
                        seq  => Seq +1
                    },
                    {ok, [lists:nth(1, Xarray), lists:nth(4, Xarray), OldAngle, OldRoom], robot_pos, robot, NewState};
                {Xout, Yout} ->
                    Dt = (T1 - T0)/1000,
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
                        [0,0,0,1,0,0]  % mesure Y
                    ]),
                    Z = mat:matrix([[Xout],[Yout]]),
                    R = mat:diag([?VAR_S, ?VAR_S]),
                    send_robot_sonar_pos([Xout, Yout, OldAngle, OldRoom]),

                    {Xpred, Ppred} = kalman:kf_predict({Xpos, Ppos}, F, Q),
                    {Xnew, Pnew} = kalman:kf_update({Xpred, Ppred}, H, R, Z),
                    Xarray = mat:to_array(Xnew),

                    NewState = #{ 
                        t0   => T1,
                        x_pos => Xnew,
                        p_pos => Pnew,
                        seq  => Seq +1
                    },
                    
                    hera_data:store(robot_pos, robot, Seq, [lists:nth(1, Xarray), lists:nth(4, Xarray), OldAngle, OldRoom]),
                    send_robot_pos([lists:nth(1, Xarray), lists:nth(4, Xarray), OldAngle, OldRoom]),
                    

                    {ok, [lists:nth(1, Xarray), lists:nth(4, Xarray), OldAngle, OldRoom], robot_pos, robot, NewState}
            end
    end.               

%============================================================================================================================================
%======================================================= HELPER FUNC ========================================================================
%============================================================================================================================================       

send_robot_pos(Pos) ->
    Pos_string = string:join([lists:flatten(io_lib:format("~p", [Val])) || Val <- Pos], ","),
    Msg = "Robot_pos," ++ Pos_string,
    hera_com:send_unicast(server, Msg, "UTF8").

send_robot_sonar_pos(Pos) ->
    Pos_string = string:join([lists:flatten(io_lib:format("~p", [Val])) || Val <- Pos], ","),
    Msg = "Robot_sonar_pos," ++ Pos_string,
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
