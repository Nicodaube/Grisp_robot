-module(sonar_sensor).

-behavior(hera_measure).

-export([init/1, measure/1]).

init(_Args) ->
    Spec = #{
        name => sonar_sensor,
        iter => infinity,
        timeout => 1000},

    Initial_distance =  0,
    io:format("[SONAR SENSOR] Sonar Sensor Spawned with Pid : ~p.~n",[self()]),
    grisp_led:color(1, {0, 1, 0}),
	grisp_led:color(2, {0, 1, 0}),
    grisp:add_device(uart, pmod_maxsonar),

    {ok, Initial_distance, Spec}.
    
measure(Sensor_Pid) ->
    distance = pmod_maxsonar:get(),
    io:format("[SONAR SENSOR] mesured distance : ~p ~n", [distance]),
    Sensor_Pid ! {self(), distance}.
