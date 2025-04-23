-module(kalman_measure).

-behavior(hera_measure).

-export([init/1, measure/1]).

%============================================================================================================================================
%======================================================= HERA MEASURE BEHAVIOUR ==============================================================
%============================================================================================================================================

init(_Args) ->
    io:format("~n[KALMAN_MEASURE] Starting measurements~n"),
    {ok, #{seq => 2}, #{
        name => kalman_measure,
        iter => infinity,
        timeout => 300
    }}.
    
measure(State) ->
    SensorName = persistent_term:get(sensor_name),
    case hera_data:get(robot_pos, SensorName) of
        [{_, _, _, [OldX, OldY, OldAngle, OldRoom]}] ->
            Seq = maps:get(seq, State, 1),
            NewState = State#{seq => Seq +1},
            io:format("[KALMAN_MEASURE] New robot pos : (~p,~p) at ~p in room number ~p~n",[OldX, OldY, OldAngle, OldRoom]),
            send_robot_pos([OldX, OldY, OldAngle, OldRoom]),
            {ok, [OldX, OldY, OldAngle, OldRoom], robot_pos, SensorName, NewState};
        _ ->
            io:format("[KALMAN_MEASURE] Robot position not initialised~n"),
            {stop, no_robot_pos}
    end.

%============================================================================================================================================
%======================================================= HELPER FUNC ========================================================================
%============================================================================================================================================       

send_robot_pos(Pos) ->
    Pos_string = string:join([lists:flatten(io_lib:format("~p", [Val])) || Val <- Pos], ","),
    Msg = "Robot_pos," ++ Pos_string,
    hera_com:send_unicast(server, Msg, "UTF8").
            