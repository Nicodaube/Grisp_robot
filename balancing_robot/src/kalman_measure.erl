-module(kalman_measure).

-behavior(hera_measure).

-export([init/1, measure/1]).

%============================================================================================================================================
%======================================================= HERA MEASURE BEHAVIOUR ==============================================================
%============================================================================================================================================

init(_Args) ->
    timer:sleep(2000),
    io:format("~n[KALMAN_MEASURE] Starting measurements~n"),
    {ok, #{seq => 2}, #{
        name => kalman_measure,
        iter => infinity,
        timeout => 300
    }}.
    
measure(State) ->
    case hera_data:get(robot_pos, robot) of
        [{_, _, _, [_, _, OldAngle, OldRoom]}] ->
            Seq = maps:get(seq, State, 1),
            NewState = State#{seq => Seq +1},
            {{Xout1, Yout1}, {Xout2, Yout2}} = get_new_robot_pos(OldRoom),
            io:format("[KALMAN_MEASURE] Two position possibilities : ~n~p ~n ~p~n", [{Xout1, Yout1}, {Xout2, Yout2}]),
            %io:format("[KALMAN_MEASURE] New robot pos : (~p,~p) at ~p in room number ~p~n",[X, Y, OldAngle, OldRoom]),
            send_robot_pos([Xout1, Yout1, OldAngle, OldRoom]),
            {ok, [Xout1, Yout1, OldAngle, OldRoom], robot_pos, robot, NewState};
        _ ->
            io:format("[KALMAN_MEASURE] Robot position not initialised~n"),
            {stop, no_robot_pos}
    end.

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
    %[{_, _, _, [Angle1]}] = hera_data:get(angle, Sensor1),
    [{_, _, _, [Dist2]}] = hera_data:get(distance, Sensor2),
    get_pos_2({X1, Y1}, {X2, Y2}, {Dist1} , {Dist2}).
    %Y = abs(Dist1 * math:sin(Angle1)),% Position relative 
    %X = abs(Dist1 * math:cos(Angle1)),% position relative 
    %get_pos({X1, Y1, A1}, {X2, Y2}, {X, Y}).
    
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





get_pos({X1, Y1, A1}, {X2, Y2}, {X, Y}) ->
    if 
        Y1 > Y2 ->
            NewY = Y1 - Y;
        Y1 < Y2 ->
            NewY = Y1 + Y;
        Y1 == Y2 ->
            if 
                A1 == 45 ->
                    NewY = Y1 + Y;
                A1 == 135 ->
                    NewY = Y1 + Y;
                A1 == 225 ->
                    NewY = Y1 - Y;
                A1 == 315 ->
                    NewY = Y1 - Y
            end
    end,

    if 
        X1 > X2 ->
            NewX = X1 - X;
        X1 < X2 ->
            NewX = X1 + X;
        X1 == X ->
            if
                A1 == 45 ->
                    NewX = X1 + X;
                A1 == 135 ->
                    NewX = X1 - X;
                A1 == 225 ->
                    NewX = X1 - X;
                A1 == 315 ->
                    NewX = X1 + X
            end
    end,
    {abs(NewX)/100, abs(NewY)/100}.