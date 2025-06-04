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
    Seq = maps:get(seq, State, 1),
    NewState = State#{seq => Seq +1},
    {Xpred, Ypred, AnglePred, RoomPred} = kalman_predict(),
    {Xupd, Yupd, AngleUpd, RoomUpd} = kalman_update(Xpred, Ypred, AnglePred, RoomPred),

    io:format("[KALMAN_MEASURE] New robot pos : (~p,~p) at ~p in room number ~p~n",[Xupd, Yupd, AngleUpd, RoomUpd]),
    send_robot_pos([Xupd, Yupd, AngleUpd, RoomUpd]),
    {ok, [Xupd, Yupd, AngleUpd, RoomUpd], robot_pos, robot, NewState}.

    
%============================================================================================================================================
%======================================================= KALMAN FUNC ========================================================================
%============================================================================================================================================ 

kalman_predict() ->
    case hera_data:get(robot_pos, robot) of
        [{_, _, _, [OldX, OldY, OldAngle, OldRoom]}] ->
            {OldX, OldY, OldAngle, OldRoom};           
        _ ->
            io:format("[KALMAN_MEASURE] Robot position not initialised~n"),
            {stop, no_robot_pos}
    end.

kalman_update(_Xpred, _Ypred, AnglePred, RoomPred) ->
    {X, Y} = get_new_robot_pos(RoomPred),
    {X, Y, AnglePred, RoomPred}.

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
    {X1, Y1, A1} = get_sensor_pos(Sensor1),
    {X2, Y2, _A2} = get_sensor_pos(Sensor2),
    [{_, _, _, [Dist1]}] = hera_data:get(distance, Sensor1),
    [{_, _, _, [Angle1]}] = hera_data:get(angle, Sensor1),
    [{_, _, _, [_Dist2]}] = hera_data:get(distance, Sensor2),
    Y = abs(Dist1 * math:sin(Angle1)),
    X = abs(Dist1 * math:cos(Angle1)),
    get_pos({X1, Y1, A1}, {X2, Y2}, {X, Y}).
    
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