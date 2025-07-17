-module(clock_ticker).

-export([init/1]).

-define(TIMESLOT_SIZE, 300).
-define(STARTUP_MARGIN, 0.1).

init(TimeClock) ->
    io:format("[CLOCK_TICKER] Starting ...~n"),
    process_flag(priority, max),
    Osensor = persistent_term:get(osensor),
    Sonar_Pid = persistent_term:get(sonar_sensor),
    SensName = persistent_term:get(sensor_name),
    {Ax, Ay, Bx, By} = get_sensor_room(SensName),
    loop(TimeClock, Sonar_Pid, Osensor, {Ax, Ay, Bx, By}, 1).

loop(TimeClock, Sonar_Pid, Osensor, RoomInfo, Count)->
    Offset = (Count * ?TIMESLOT_SIZE) div 2,
    Next_measure = TimeClock + Offset,

    case is_in_room(RoomInfo) of
        true ->
            io:format("[CLOCK] ROBOT IN ROOM~n"),
            [grisp_led:color(L, green) || L <- [1, 2]],
            case Count rem 2 of %determine who measures (slave or master)
                0 -> 
                    io:format("[CLOCK] in 0 ~n"),
                    case wait_for_time(Next_measure) of 
                        ok -> ok;
                        skip -> loop(TimeClock, Sonar_Pid, Osensor, RoomInfo, Count + 1)
                    end,
                    io:format("[CLOCK] sending server ~n"),
                    hera_com:send_unicast(server, "Clock," ++ integer_to_list(Count) ++ "," ++ integer_to_list(Next_measure), "UTF8"),
                    io:format("[CLOCK] sending sonar ~n"),
                    Sonar_Pid ! clock;                    
                1 ->
                    io:format("[CLOCK] in 1 ~n"),
                    case wait_for_time(Next_measure) of 
                        ok -> ok;
                        skip -> loop(TimeClock, Sonar_Pid, Osensor, RoomInfo, Count + 1)
                    end,
                    io:format("[CLOCK] sending osensor ~n"),
                    hera_com:send_unicast(Osensor, "Clock," ++ integer_to_list(Count), "UTF8")
            end;
        false ->
            %io:format("[CLOCK] Not in room~n")
            [grisp_led:color(L, aqua) || L <- [1, 2]],
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
        [{_, _, _, [Xpos, Ypos, _, _]}] ->
            io:format("[CLOCK] Xpos : ~p~n Ypos: : ~p~n Ax : ~p~n Ay : ~p~n Bx : ~p~n By: ~p~n",[Xpos, Ypos, Ax, Ay, Bx, By]),
            false;
        _ ->
            false
    end.   

wait_for_time(Next_measure) ->
    io:format("[CLOCK] in wait ~n"),
    Now = erlang:monotonic_time(millisecond),
    Time_to_wait = Next_measure - Now,

    if 
        Time_to_wait > 0 -> 
            io:format("[CLOCK] waiting ~p ~n", [Time_to_wait]),
            timer:sleep(Time_to_wait),
            ok;
        true -> 
            io:format("[CLOCK] skipping ~n"),
            skip
    end.