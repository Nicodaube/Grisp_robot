-module(clock_ticker).

-export([init/1]).

-define(TIMESLOT_SIZE, 300).
-define(STARTUP_MARGIN, 0.11).

init(TimeClock) ->
    io:format("[CLOCK_TICKER] Starting ...~n"),
    process_flag(priority, max),
    Osensor = persistent_term:get(osensor),
    Sonar_Pid = persistent_term:get(sonar_measure),
    SensName = persistent_term:get(sensor_name),
    {Ax, Ay, Bx, By} = get_sensor_room(SensName),
    loop(TimeClock, Sonar_Pid, Osensor, {Ax, Ay, Bx, By}, 1).

loop(TimeClock, Sonar_Pid, Osensor, RoomInfo, Count)->
    Offset = (Count * ?TIMESLOT_SIZE) div 2,
    Next_measure = TimeClock + Offset,

    case is_in_room(RoomInfo) of
        true -> 
            io:format("[CLOCK] ROBOT IN ROOM~n"),
            case Count rem 2 of %determine who measures (slave or master)
                0 -> 
                    case wait_for_time(Next_measure) of 
                        ok -> 
                            hera_com:send_unicast(server, "Clock," ++ integer_to_list(Count) ++ "," ++ integer_to_list(Next_measure), "UTF8"),
                            Sonar_Pid ! clock;
                        skip -> loop(TimeClock, Sonar_Pid, Osensor, RoomInfo, Count + 1)
                    end;
                1 ->
                    case wait_for_time(Next_measure) of 
                        ok -> hera_com:send_unicast(Osensor, "Clock," ++ integer_to_list(Count), "UTF8");
                        skip -> loop(TimeClock, Sonar_Pid, Osensor, RoomInfo, Count + 1)
                    end                    
            end;
        false ->
            case wait_for_time(Next_measure) of 
                ok -> ok;
                skip -> loop(TimeClock, Sonar_Pid, Osensor, RoomInfo, Count + 1)
            end,
            ok            
    end,

    loop(TimeClock, Sonar_Pid, Osensor, RoomInfo, Count + 1).

get_sensor_room(SensName) ->
    case hera_data:get(room, SensName) of 
        [{_, _, _, [Room]}] ->
            case hera_data:get(room_info, Room) of
                [{_, _, _, [Ax, Ay, Bx, By]}] ->
                    {Ax, Ay, Bx, By};
                [] ->
                    io:format("[CLOCK] Error in pos determination~n"),
                    {0, 0, 0, 0}
            end;
        [] ->
            io:format("[CLOCK] Error in room determination~n"),
            {0, 0, 0, 0}
    end.

is_in_room({Ax, Ay, Bx, By}) ->
    case hera_data:get(robot_pos, robot) of 
        [{_, _, _, [Xpos, Ypos, _, _]}] when ((Ax-?STARTUP_MARGIN < Xpos andalso Xpos < Bx + ?STARTUP_MARGIN) andalso (Ay - ?STARTUP_MARGIN < Ypos andalso Ypos < By + ?STARTUP_MARGIN))->
            true;  
        [{_, _, _, [_Xpos, _Ypos, _, _]}] ->
            false;
        _ ->
            false
    end.   

wait_for_time(Next_measure) ->
    Now = erlang:monotonic_time(millisecond),
    Time_to_wait = Next_measure - Now,

    if 
        Time_to_wait > 0 -> 
            timer:sleep(Time_to_wait),
            ok;
        true -> 
            skip
    end.