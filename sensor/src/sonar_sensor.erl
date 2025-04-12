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
    Dist_inch = pmod_maxsonar:get(),
    Dist_cm = Dist_inch * 2.54,
    R_Dist_cm = round(Dist_cm, 4),

    Seq = maps:get(seq, State, 1),
    io:format("[SONAR_SENSOR] Sonar measure ~p : ~p~n", [Seq, R_Dist_cm]),
    SensorName = persistent_term:get(sensor_name),
    hera_data:store(distance, SensorName, Seq, [R_Dist_cm]),

    
    NewState = State#{seq => Seq + 1},
    {ok, [R_Dist_cm], distance, SensorName, NewState}. 

round(Number, Precision) ->
    Power = math:pow(10, Precision),
    round(Number * Power) / Power.