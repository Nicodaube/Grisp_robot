-module(sonar_sensor).

-behavior(hera_measure).

-define(ROBOT_HEIGHT, 23).
-define(MEASURE_ACCEPTABLE_RANGE, 10).
-define(TIMESLOT_SIZE, 300).
-define(MIN_MEASURE_PERIOD, ?TIMESLOT_SIZE/2).

-export([init/1, measure/1]).

%============================================================================================================================================
%======================================================= HERA MEASURE BEHAVIOUR =============================================================
%============================================================================================================================================

init(_Args) ->
    io:format("~n[SONAR_SENSOR] Starting measurements~n"),
    get_sensor_role(),
    State = #{
        seq => get_init_seq(),
        last_measure => none,
        timestamp => none
    },
    {ok, State, #{
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
                Phase >= ?TIMESLOT_SIZE-25 ->
                    get_measure(State, SensorName);
                Phase < 25 ->
                    get_measure(State, SensorName);
                true ->
                    {undefined, State}
            end;
        {slave, TimeClock, Offset} ->
            Phase = get_phase(TimeClock, Current_time, Offset),
            if 
                Phase >= (?TIMESLOT_SIZE/2)-25, Phase < (?TIMESLOT_SIZE/2)+25 ->
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

    case get_ground_distance(SensorName, D) of
        {ok, Ground_measure} ->
            check_measure_range(Ground_measure, State, SensorName);
        {stop, cannot_get_height} ->
            {stop, cannot_get_height}
    end.

get_ground_distance(SensorName, D) ->
    % Uses the basic pythagorian formula to transform the distance based on the sensor's height
    % @param State : the internal state of the module (tuple)
    % @param SensorName : the name of the current sensor (atom)
    % @param D : Sonar measure in cm (integer)

    case hera_data:get(pos, SensorName) of
        [{_, _, _, [_ , _, H, _]}] ->

            if
                H > ?ROBOT_HEIGHT ->
                    Ground_measure = round(math:sqrt(math:pow(D, 2) - math:pow((H*100)-?ROBOT_HEIGHT, 2)), 3); % Taking the height of the sonar into account
                true ->
                    Ground_measure = round(D, 3) % The robot is bigger than the sensor's height, no need for correction
            end,           

            {ok, Ground_measure};
        Msg ->
            io:format("[SONAR_SENSOR] Cannot get sensor height : ~p~n",[Msg]),
            {stop, cannot_get_height}
    end.

check_measure_range(Ground_measure, State, SensorName) ->
    % Smoothes the measure (filters bad measures due to interferences)
    % @param Ground_measure : measure in cm (Integer)
    % @param State : the internal state of the module (tuple)
    % @param SensorName : the name of the current sensor (atom)
    #{
        seq := Seq,
        last_measure := Last_measure,
        timestamp := Timestamp
    } = State,

    Current_timestamp = hera:timestamp(),
    if
        Last_measure == none orelse abs(Last_measure - Ground_measure) < ?MEASURE_ACCEPTABLE_RANGE -> 
            % Only keep the measure if it is within MEASURE_ACCEPTABLE_RANGE of the last measure or if there was no measure for MIN_MEASURE_PERIOD
            accept_measure(Ground_measure, SensorName, Current_timestamp, Seq);
        Current_timestamp - Timestamp > ?MIN_MEASURE_PERIOD ->
            accept_measure(Ground_measure, SensorName, Current_timestamp, Seq);        
        true ->
            io:format("[SONAR_SENSOR] ground distance exceeds the acceptable range by ~p~n", [abs(Last_measure - Ground_measure) - ?MEASURE_ACCEPTABLE_RANGE]),
            {undefined, State}
end.

accept_measure(Ground_measure, SensorName, Current_timestamp, Seq) ->
    %io:format("[SONAR_SENSOR] ground distance to robot : ~p : ~p~n", [Seq, True_measure]),
    hera_com:send_unicast(server, "Distance,"++float_to_list(Ground_measure)++","++atom_to_list(SensorName), "UTF8"),

    hera_data:store(distance, SensorName, Seq, [Ground_measure]),
    NewState = #{
        seq => Seq + 1,
        last_measure => Ground_measure,
        timestamp => Current_timestamp
    },
    {ok, [Ground_measure], distance, SensorName, NewState}.


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
    Phase0 = (Now - Start) rem ?TIMESLOT_SIZE,
    Phase = if
        Phase0 < 0 -> Phase0 + ?TIMESLOT_SIZE;
        true      -> Phase0
    end,
    Phase.

get_phase(Start, Now, Offset) ->
    Phase0 = (Now + Offset - Start) rem ?TIMESLOT_SIZE,
    Phase = if
        Phase0 < 0 -> Phase0 + ?TIMESLOT_SIZE;
        true      -> Phase0
    end,
    Phase.