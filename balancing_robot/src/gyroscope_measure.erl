-module(gyroscope_measure).

-behavior(hera_measure).

-export([init/1, measure/1]).

%============================================================================================================================================
%======================================================= HERA MEASURE BEHAVIOUR =============================================================
%============================================================================================================================================

init(_Args) ->
    io:format("[GYROSCOPE] Starting~n"),
    calibrate(),
    io:format("[GYROSCOPE] Starting measurements~n"),
    {ok, #{seq => 2}, #{
        name => gyroscope,
        iter => infinity,
        timeout => 5
    }}.
    
measure(State) ->
    [Gy, Ax, Az] = pmod_nav:read(acc, [out_y_g, out_x_xl, out_z_xl], #{g_unit => dps}),
    Seq = maps:get(seq, State),

    NewState = State#{seq => Seq +1},
    %io:format("[GYROSCOPE] measured Gy : ~p, Ax : ~p, Az : ~p~n", [Gy, Ax, Az]),

    hera_data:store(gyroscope, node(), Seq, [Gy, Ax, Az]),
    {ok, [Gy, Ax, Az], gyroscope, self(), NewState}.


%============================================================================================================================================
%======================================================= CONFIG FUNC ========================================================================
%============================================================================================================================================

calibrate() -> 
    % Calibration process for the gyroscope
    io:format("[GYROSCOPE_MEASURE] Calibrating... ~n"),
    N = 500,
    [grisp_led:color(L, red) || L <- [1, 2]],
    Data = [list_to_tuple(pmod_nav:read(acc, [out_x_g, out_y_g, out_z_g])) || _ <- lists:seq(1, N)],
    {_, Y, _} = lists:unzip3(Data),
    io:format("[GYROSCOPE_MEASURE] Done calibrating~n"),
    Gy0 = lists:sum(Y) / N,
    hera_data:store(gyroscope, node(), 1, [0.0, Gy0, 0.0]).