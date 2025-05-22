-module(sonar_sensor).

-behavior(hera_measure).

-define(ROBOT_HEIGHT, 0). % RESETED TO 0, try
-export([init/1, measure/1]).

%============================================================================================================================================
%======================================================= HERA MEASURE BEHAVIOUR =============================================================
%============================================================================================================================================

init(_Args) ->
    io:format("~n[SONAR_SENSOR] Starting measurements~n"),
    Init_seq = get_sensor_role(),
    {ok, #{seq => Init_seq}, #{
        name => sonar_sensor,
        iter => infinity,
        timeout => 300
    }}.
    
measure(State) ->    
    SensorName = persistent_term:get(sensor_name),
    Current_time = erlang:system_time(millisecond),
    case persistent_term:get(sensor_role) of
        {master, TimeClock, _} ->
            Phase = get_phase(TimeClock, Current_time),
            if 
                Phase >= 0, Phase < 50 ->
                    get_measure(State, SensorName);
                true ->
                    timer:sleep((100 - Phase) + 5),
                    get_measure(State, SensorName)
            end;
        {slave, TimeClock, Offset} ->
            Phase = get_phase(TimeClock, Current_time, Offset),
            if 
                Phase >= 50, Phase < 100 ->
                    get_measure(State, SensorName);
                true ->
                    timer:sleep((100 - Phase) + 5),
                    get_measure(State, SensorName)
            end

        end.
            
%============================================================================================================================================
%============================================================== ROLE SETUP ==================================================================
%============================================================================================================================================

get_sensor_role() ->
    case persistent_term:get(osensor, none) of
        none -> % Is alone in a room
            io:format("[SONAR_SENSOR] No other sensor, sensor is master~n"),
            TimeClock = erlang:system_time(millisecond),
            persistent_term:put(sensor_role, {master, TimeClock, 0});                     
        Osensor -> % Start Handshake
            case persistent_term:get(sensor_role, none) of
                none -> % Classical sensor bootstrap
                    {ok, Priority} = get_rand_num(),
                    TimeClock = erlang:system_time(millisecond),
                    %io:format("[SONAR SENSOR] Random handshake Priority ~p~n", [Priority]),
                    role_handshake(Osensor, Priority, TimeClock),
                    1;
                _ ->  % Case where the sonar sensor crashed and was reloaded (need to find the latest seq number)
                    io:format("[SONAR_SENSOR] Recovering from crash"),
                    [{_, _, Seq, [_]}] = hera_data:get(distance),
                    io:format(Seq),
                    Seq+1
            end
    end.       

role_handshake(Osensor, Priority, TimeClock) ->
    hera_com:send_unicast(Osensor, "Handshake," ++ integer_to_list(Priority) ++ "," ++ integer_to_list(TimeClock), "UTF8"),
    receive
        {handshake, OPriority, OTimeClock} ->
            if
                Priority > OPriority ->
                    io:format("[SONAR_SENSOR] Local priority higher, sensor role : MASTER~n"),                    
                    wait_ack(Osensor),
                    persistent_term:put(sensor_role, {master, TimeClock, OTimeClock - TimeClock});
                Priority < OPriority ->
                    io:format("[SONAR_SENSOR] External priority higher, sensor role : SLAVE~n"),                    
                    wait_ack(Osensor),                  
                    persistent_term:put(sensor_role, {slave, OTimeClock, OTimeClock - TimeClock});
                true ->
                    io:format("[SONAR_SENSOR] Priority collision, retrying~n"),
                    {ok, New_Priority} = get_rand_num(),
                    role_handshake(Osensor, New_Priority, TimeClock)                    
            end;
        {ok, role} ->
            ok
    after 500 ->
        role_handshake(Osensor, Priority, TimeClock) 
    end.   

wait_ack(Osensor) ->
    hera_com:send_unicast(Osensor, "Ok,role", "UTF8"),
    receive
        {ok, _} -> ok;
        _ -> wait_ack(Osensor)
    after 500 ->
        wait_ack(Osensor)
    end.

%============================================================================================================================================
%========================================================= SONAR MEASURE  ===================================================================
%============================================================================================================================================

get_measure(State, SensorName) ->
    % Get the Pmod Maxsonar measure and transform it into the right format
    % @param State : the internal state of the module (tuple)
    % @param SensorName : the name of the current sensor (atom)
    Dist_inch = pmod_maxsonar:get(),
    Dist_cm = Dist_inch * 2.54,
    D = round(Dist_cm, 4),    
    %io:format("[SONAR_SENSOR] Sonar measure ~p : ~p~n", [Seq, D]),

    get_ground_distance(State, SensorName, D).

get_ground_distance(State, SensorName, D) ->
    % Uses the basic pythagorian formula to transform the distance based on the sensor's height
    % @param State : the internal state of the module (tuple)
    % @param SensorName : the name of the current sensor (atom)
    % @parma D : Sonar measure in cm (integer)


    % TODO: CORRECT GROUND DISTANCE, FOR NOW JUST STORING DIST
    Seq = maps:get(seq, State, 1),
    hera_data:store(distance, SensorName, Seq, [D]),
    NewState = State#{seq => Seq + 1},
    {ok, [D], distance, SensorName, NewState}.


    %Seq = maps:get(seq, State, 1),
    %case hera_data:get(pos, SensorName) of
    %    [{_, _, _, [_ , _, H, _]}] ->
    %        True_measure = round(math:sqrt(math:pow(D, 2) - math:pow((H*100) - ?ROBOT_HEIGHT, 2)), 3), % Taking the height of the sonar into account
    %
    %        %io:format("[SONAR_SENSOR] ground distance to robot : ~p : ~p~n", [Seq, True_measure]),
    %
    %        hera_data:store(distance, SensorName, Seq, [True_measure]),
    %        NewState = State#{seq => Seq + 1},
    %        {ok, [True_measure], distance, SensorName, NewState};
    %    Msg ->
    %        io:format("[SONAR_SENSOR] Cannot get sensor height : ~p~n",[Msg]),
    %        {stop, cannot_get_height}
    %end.

%============================================================================================================================================
%=========================================================== HELPER FUNC ====================================================================
%============================================================================================================================================
    
round(Number, Precision) ->
    % Rounds a number following parameters
    % @param Number : the number to round (float)
    % @param Precision : the precision of the wanted rounding (integer)
    Power = math:pow(10, Precision),
    round(Number * Power) / Power.

get_rand_num() ->
    % Returns a random number between 0 and 2000
    persistent_term:get(id),
    Seed = {erlang:monotonic_time(), erlang:unique_integer([positive]), erlang:phash2(node())},
    rand:seed(exsplus, Seed),
    {ok, rand:uniform(3000)}.   

get_phase(TimeClock, Current_time) ->
    Phase = (Current_time - TimeClock) rem 100,
    case Phase < 0 of
        true ->
            100 - Phase;
        false ->
            Phase
    end.

get_phase(TimeClock, Current_time, Offset) ->
    Phase = (Current_time + Offset - TimeClock) rem 100,
    case Phase < 0 of
        true ->
            100 - Phase;
        false ->
            Phase
    end.

