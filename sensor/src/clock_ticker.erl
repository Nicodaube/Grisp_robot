-module(clock_ticker).

-export([init/1]).

-define(TIMESLOT_SIZE, 200).

init(TimeClock) ->
    Osensor = persistent_term:get(osensor),
    Sonar_Pid = persistent_term:get(sonar_sensor),
    loop(TimeClock, Sonar_Pid, Osensor, 1).

loop(TimeClock, Sonar_Pid, Osensor, Count)->
    Offset = (Count * ?TIMESLOT_SIZE) div 2,
    Next_measure = TimeClock + Offset,

    Now = erlang:monotonic_time(millisecond),
    Time_to_wait = Next_measure - Now,

    if 
        Time_to_wait > 0 -> 
            timer:sleep(Time_to_wait);
        true -> 
            loop(TimeClock, Sonar_Pid, Osensor, Count + 1)
    end,

    case Count rem 2 of
        0 -> 
            Sonar_Pid ! clock,
            hera_com:send_unicast(server, "Clock," ++ integer_to_list(Count) ++ "," ++ integer_to_list(Next_measure), "UTF8");
        1 ->
            hera_com:send_unicast(Osensor, "Clock," ++ integer_to_list(Count), "UTF8")
    end,

    loop(TimeClock, Sonar_Pid, Osensor, Count + 1).