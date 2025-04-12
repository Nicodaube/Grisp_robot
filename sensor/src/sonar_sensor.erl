-module(sonar_sensor).

-behavior(hera_measure).

-export([init/1, measure/1]).

init(_Args) ->
    io:format("[SONAR_SENSOR] Starting measurements~n"),
    {ok, #{seq => 1}, #{
        name => sonar_sensor,
        iter => infinity,
        timeout => 1000
    }}.
    
measure(State) ->
    SensorName = persistent_term:get(sensor_name),
    {ok, N} = get_rand_num(),
    case persistent_term:get(osensor, none) of
        none ->
            {ok, Measure, NewState} = get_measure(State),            
            {ok, [Measure], distance, SensorName, NewState};
        Osensor ->
            receive
                {measure} ->
                    io:format("[SONAR_SENSO] possible collision waiting for ~p~n",[N]),
                    timer:sleep(N),
                    measure(State)
            after N ->
                hera_com:send_unicast(Osensor, "measure", "UTF8"),
                {ok, Measure, NewState} = get_measure(State),            
                {ok, [Measure], distance, SensorName, NewState}
            end
        end.            
            

get_measure(State) ->
    Seq = maps:get(seq, State, 1),
    Dist_inch = pmod_maxsonar:get(),
    Dist_cm = Dist_inch * 2.54,
    R_Dist_cm = round(Dist_cm, 4),    
    %io:format("[SONAR_SENSOR] Sonar measure ~p : ~p~n", [Seq, R_Dist_cm]),
    NewState = State#{seq => Seq + 1},
    {ok, R_Dist_cm, NewState}.
    

round(Number, Precision) ->
    Power = math:pow(10, Precision),
    round(Number * Power) / Power.

get_rand_num() ->
    Seed = {erlang:monotonic_time(), erlang:unique_integer([positive]), erlang:phash2(node())},
    rand:seed(exsplus, Seed),
    {ok, rand:uniform(2000)}.   