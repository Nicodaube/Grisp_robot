-module(robot_angle).

-export([start_link/1]).

start_link(SensID) ->
    io:format("[ROBOT_ANGLE] Spawning~n"),
    spawn(fun () -> loop(SensID, 1) end),
    ok.

loop(SensID, Seq) ->
    timer:sleep(500),
    SensName = persistent_term:get(sensor_name),
    case hera_data:get(room, SensName) of
      [{_, _, _, [Room]}] ->
        {ok, [X, Y, Dist]} = get_data(SensName),

        case find_sensors_room(Room, SensName) of 
            [H|_] ->
                {ok, [Ox, Oy, Odist]} = get_data(H),                
                {ok, Angle} = compute_angle(X, Y, Ox, Oy, Dist, Odist),
                hera_data:store(angle, SensName, Seq, [Angle]),
                io:format("[ROBOT_ANGLE] Robot at angle : ~p~n",[Angle]),
                loop(SensID, Seq + 1);
            Other ->
                io:format("[ROBOT_ANGLE] rooms = ~p~n",[Other]),
                loop(SensID, Seq)
        end;
        
      Msg ->
        io:format("[ROBOT_ANGLE] Error in getting sensor pos ~p~n",[Msg]),
        loop(SensID, Seq)
    end.

find_sensors_room(Room, SensName) ->
    Devices = persistent_term:get(devices),
    lists:foldl(
        fun({Name, _Ip, _Port}, Acc) ->
            case Name of
                SensName ->
                    Acc;
                _ ->
                    case hera_data:get(room, Name) of
                        [{_Node, _Seq, _Ts, [ORoom]}] when Room =:= ORoom ->
                                io:format("[ROBOT_ANGLE] Sens : ~p is in the same room as this sensor~n", [Name]),
                                [Name | Acc];
                        _ ->
                            Acc
                    end
            end
        end,
        [],
        Devices
    ).
    
get_data(Name) ->
    case hera_data:get(pos, Name) of 
    [{_, _, _, [X, Y]}] ->
        case  hera_data:get(distance, Name) of 
            [{_, _, _, [Dist]}] ->
                {ok, [X, Y, Dist]};
            Msg ->
                io:format("[ROBOT_ANGLE] Error when fetching distance of ~p, ~p~n",[Name, Msg]),
                {error, distance}
        end;
    Msg ->
        io:format("[ROBOT_ANGLE] Error when fetching pos of ~p, ~p~n", [Name, Msg]),
        {error, pos}
    end.

compute_angle(X, Y, Ox, Oy, Dist, Odist) ->
    DistSens = get_distance_sensors(X, Y, Ox, Oy),
    Numerator = math:pow(Dist, 2) + math:pow(DistSens, 2) - math:pow(Odist, 2),
    Denominator = 2 * Dist * DistSens,
    case Denominator of
        0 ->
            {error, division_by_zero};
        _ ->
            CosAlpha = Numerator / Denominator,
            ClampedCosAlpha = max(-1.0, min(1.0, CosAlpha)),
            AlphaRadians = math:acos(ClampedCosAlpha),
            AlphaDegrees = AlphaRadians * 180 / math:pi(),
            {ok, AlphaDegrees}
    end.

get_distance_sensors(X, Y, Ox, Oy) ->
    Result = math:sqrt(math:pow(Ox - X, 2) + math:pow(Oy - Y, 2)),
    Result.