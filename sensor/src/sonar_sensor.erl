-module(sonar_sensor).

-behavior(hera_measure).

-define(ROBOT_HEIGHT, 23).
-export([init/1, measure/1]).

%============================================================================================================================================
%======================================================= HERA MEASURE BEHAVIOUR =============================================================
%============================================================================================================================================

init(_Args) ->
    io:format("~n[SONAR_SENSOR] Starting measurements~n"),
    get_sensor_role(),
    Init_seq = get_init_seq(),
    {ok, #{seq => Init_seq}, #{
        name => sonar_sensor,
        iter => infinity,
        timeout => 20
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
                    {undefined, State}
            end;
        {slave, TimeClock, Offset} ->
            Phase = get_phase(TimeClock, Current_time, Offset),
            if 
                Phase >= 100, Phase < 150 ->
                    get_measure(State, SensorName);
                true ->
                    {undefined, State}
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
                    role_handshake(Osensor, Priority, TimeClock);
                _ ->  % Case where the sonar sensor crashed and was reloaded (need to find the latest seq number)
                    io:format("[SONAR_SENSOR] Recovering from crash"),
                    ok
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

get_init_seq() ->
    SensorName = persistent_term:get(sensor_name),
    case hera_data:get(distance, SensorName) of
        [{_, _, Seq, [_]}] -> Seq +1;
        _ -> 1
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
    % @param D : Sonar measure in cm (integer)

    Seq = maps:get(seq, State, 1),
    case hera_data:get(pos, SensorName) of
        [{_, _, _, [_ , _, H, _]}] ->

            if
                H > ?ROBOT_HEIGHT ->
                    True_measure = round(math:sqrt(math:pow(D, 2) - math:pow((H*100)-?ROBOT_HEIGHT, 2)), 3); % Taking the height of the sonar into account
                true ->
                    True_measure = round(D, 3) % The robot is bigger than the sensor's height, no need for correction
            end,
    
            %io:format("[SONAR_SENSOR] ground distance to robot : ~p : ~p~n", [Seq, True_measure]),
            hera_com:send_unicast(server, "Distance,"++float_to_list(True_measure)++","++atom_to_list(SensorName), "UTF8"),
    
            hera_data:store(distance, SensorName, Seq, [True_measure]),
            NewState = State#{seq => Seq + 1},
            {ok, [True_measure], distance, SensorName, NewState};
        Msg ->
            io:format("[SONAR_SENSOR] Cannot get sensor height : ~p~n",[Msg]),
            {stop, cannot_get_height}
    end.

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

get_phase(Start, Now) ->
    Phase0 = (Now - Start) rem 200,
    Phase = if
        Phase0 < 0 -> Phase0 + 200;
        true      -> Phase0
    end,
    Phase.

get_phase(Start, Now, Offset) ->
    Phase0 = (Now + Offset - Start) rem 200,
    Phase = if
        Phase0 < 0 -> Phase0 + 200;
        true      -> Phase0
    end,
    Phase.


