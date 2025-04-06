-module(sonar_sensor).

-behavior(hera_measure).

-export([init/1, measure/1]).

init(_Args) ->
    io:format("[SONAR_SENSOR] Starting measurements~n"),
    {ok, undefined, #{
        name => sonar_sensor,
        iter => infinity,
        timeout => 1000
    }}.
    
measure(State) ->
    Dist_inch = pmod_maxsonar:get(),
    Dist_cm = Dist_inch * 2.54,
    R_Dist_cm = round(Dist_cm, 4),
    %io:format("[SONAR_SENSOR] Sonar measure : ~p~n", [R_Dist_cm]),
    {ok, [R_Dist_cm], State}.

round(Number, Precision) ->
    Power = math:pow(10, Precision),
    round(Number * Power) / Power.