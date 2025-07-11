-module(clock_ticker).

-export([init/1]).

-define(TIMESLOT_SIZE, 200).

init(TimeClock) ->
    io:format("[CLOCK_TICKER] Starting ...~n"),
    process_flag(priority, max),
    Osensor = persistent_term:get(osensor),
    Sonar_Pid = persistent_term:get(sonar_sensor),
    SensName = persistent_term:get(sensor_name),
    Room = get_sensor_room(SensName),
    loop(TimeClock, Sonar_Pid, Osensor, Room, SensName, 1).

loop(TimeClock, Sonar_Pid, Osensor, Room, SensName,Count)->
    Offset = (Count * ?TIMESLOT_SIZE) div 2,
    Next_measure = TimeClock + Offset,

    Now = erlang:monotonic_time(millisecond),
    Time_to_wait = Next_measure - Now,

    if 
        Time_to_wait > 0 -> 
            timer:sleep(Time_to_wait);
        true -> 
            loop(TimeClock, Sonar_Pid, Osensor, Room, SensName, Count + 1)
    end,

    case hera_data:get(robot_pos, SensName) of 
        [{_, _, _, [ _, _, _, RobotRoom]}] when RobotRoom =:= Room ->
            case Count rem 2 of
                0 -> 
                    Sonar_Pid ! clock,
                    hera_com:send_unicast(server, "Clock," ++ integer_to_list(Count) ++ "," ++ integer_to_list(Next_measure), "UTF8");
                1 ->
                    hera_com:send_unicast(Osensor, "Clock," ++ integer_to_list(Count), "UTF8")
            end;
        [] ->
            ok
    end,

    loop(TimeClock, Sonar_Pid, Osensor, Room, SensName, Count + 1).

get_sensor_room(SensName) ->
    case hera_data:get(room, SensName) of 
        [{_, _, _, [Room]}] ->
            Room;
        [] ->
            -1
    end.
    
