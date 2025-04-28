-module(target_angle).

-behavior(hera_measure).

-export([init/1, measure/1]).

init(_Args) ->
    io:format("[TARGET_ANGLE] Spawning~n"),
    case setup() of
        {ok, _} ->
            {ok, #{seq => 1}, #{
                name => angle_measure,
                iter => infinity,
                timeout => 300
            }};
        stop ->
            io:format("[TARGET_ANGLE] No other sensor, not spawning"),
            {stop, no_other_sensor}
    end.    

setup() ->
    timer:sleep(1500),
    SensName = persistent_term:get(sensor_name),
    case hera_data:get(room, SensName) of
      [{_, _, _, [Room]}] ->        
        case find_sensors_room(Room) of 
            [H|_] -> % Multiple sensors in a room
                io:format("[TARGET_ANGLE] Other sensor is : ~p~n", [H]),
                persistent_term:put(osensor, H),
                {ok, Room};
            _ -> % No other sensor
                io:format("[TARGET_ANGLE] No other sensor in the room~n"),
                stop
        end;
        
      Msg ->
        io:format("[TARGET_ANGLE] Error in getting sensor pos ~p~n",[Msg]),
        timer:sleep(500),
        setup()
    end.
measure(State) ->
    Seq = maps:get(seq, State, 1),
    SensorName = persistent_term:get(sensor_name),
    Osensor = persistent_term:get(osensor),
    {ok, [X, Y, Dist]} = get_data(SensorName),
    {ok, [Ox, Oy, Odist]} = get_data(Osensor),                
    {ok, Angle} = compute_angle(X, Y, Ox, Oy, Dist, Odist),
    %io:format("[TARGET_ANGLE] Robot at angle : ~p~n",[Angle])
    hera_data:store(angle, SensorName, Seq, [Angle]),
    NewState = State#{seq => Seq + 1},
    {ok, [Angle], angle, SensorName, NewState}.    

find_sensors_room(Room) ->
    Devices = persistent_term:get(devices),
    lists:foldl(
        fun({Name, _, _}, Acc) ->
            case Name of
                _ ->
                    case hera_data:get(room, Name) of
                        [{_, _, _, [ORoom]}] when Room =:= ORoom ->
                                %io:format("[TARGET_ANGLE] Sens : ~p is in the same room as this sensor~n", [Name]),
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
    [{_, _, _, [X, Y, _, A]}] ->
        case  hera_data:get(distance, Name) of 
            [{_, _, _, [Dist]}] ->
                {ok, [X, Y, Dist]};
            Msg ->
                io:format("[TARGET_ANGLE] Error when fetching distance of ~p, ~p~n",[Name, Msg]),
                {error, distance}
        end;
    Msg ->
        io:format("[TARGET_ANGLE] Error when fetching pos of ~p, ~p~n", [Name, Msg]),
        {error, pos}
    end.

%Il faut intégrer l'angle des sensor dans le calcul (à check)
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