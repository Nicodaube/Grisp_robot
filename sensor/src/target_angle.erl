-module(target_angle).

-behavior(hera_measure).

-export([init/1, measure/1]).

%============================================================================================================================================
%======================================================= HERA MEASURE BEHAVIOUR =============================================================
%============================================================================================================================================

init(_Args) ->
    timer:sleep(1500),
    io:format("[TARGET_ANGLE] Spawning~n"),
    Init_seq = get_initial_seq(),
    {ok, #{seq => Init_seq}, #{
        name => angle_measure,
        iter => infinity,
        timeout => 300
    }}.

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

%============================================================================================================================================
%=============================================================== ANGLE COMPUTATION FUNC =====================================================
%============================================================================================================================================

get_data(Name) ->
    % Gets the distance measurement of the set sensors
    % @param Name : Name of the sensor (atom)
    case hera_data:get(pos, Name) of 
    [{_, _, _, [X, Y, _, _]}] ->
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

compute_angle(X, Y, Ox, Oy, Dist, Odist) ->
    % Compute the angle of the robot based on the two sensors measurement and the law of cosines
    % @param X : current sensor x axis pos (Float)
    % @param Y : current sensor y axis pos (Float)
    % @param Ox : other sensor x axis pos (Float)
    % @param Oy : other sensor y axis pos (Float)
    % @param Dist : distance measurement of the current sensor (Float)
    % @param Odist : distance measurement of the other sensor (Float)
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
            %io:format("[TARGET_ANGLE] Computed angle : ~p~n", [AlphaDegrees]),
            {ok, AlphaDegrees}
    end.

get_distance_sensors(X, Y, Ox, Oy) ->
    % Computes the distance between the two sensors
    % @param X : current sensor x axis pos (Float)
    % @param Y : current sensor y axis pos (Float)
    % @param Ox : other sensor x axis pos (Float)
    % @param Oy : other sensor y axis pos (Float)
    Result = math:sqrt(math:pow(Ox - X, 2) + math:pow(Oy - Y, 2)) *100,
    %io:format("[TARGET_ANGLE] other sensor is at : ~p~n", [Result]),
    Result.

%============================================================================================================================================
%=============================================================== HELPING  FUNC ==============================================================
%============================================================================================================================================

get_initial_seq() ->
    SensorName = persistent_term:get(sensor_name),
    case hera_data:get(angle, SensorName) of
        [{_, _, Seq, [_]}] -> % Case where the sonar sensor crashed and was reloaded (need to find the latest seq number)
            io:format("[TARGET_ANGLE] Recovering from crash"),
            Seq+1;
        [] ->  % Classical sensor bootstrap
            1
    end.