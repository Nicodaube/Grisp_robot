-module(gyroscope_measure).

-behavior(hera_measure).

-export([init/1, measure/1]).

%============================================================================================================================================
%======================================================= HERA MEASURE BEHAVIOUR =============================================================
%============================================================================================================================================

init(_Args) ->
    io:format("[GYROSCOPE] Starting measurements~n"),
    calibrate(),
    {ok, #{seq => 2}, #{
        name => gyroscope,
        iter => infinity,
        timeout => 50
    }}.
    
measure(State) ->
    [Gy, Ax, Az] = pmod_nav:read(acc, [out_y_g, out_x_xl, out_z_xl], #{g_unit => dps}),



calibrate() -> 
    % Calibration process for the gyroscope
    io:format("[GYROSCOPE_MEASURE] Calibrating... Do not move the pmod_nav!~n"),
    N = 500,
    [grisp_led:color(L, red) || L <- [1, 2]],
    Data = [list_to_tuple(pmod_nav:read(acc, [out_x_g, out_y_g, out_z_g])) || _ <- lists:seq(1, N)],
    {X, Y, Z} = lists:unzip3(Data),
    io:format("[GYROSCOPE_MEASURE] Done calibrating~n"),
    {Gx0, Gy0, Gz0} = [lists:sum(X) / N, lists:sum(Y) / N, lists:sum(Z) / N],
    hera_data:store(gyroscope, self(), 1, [Gx0, Gy0, Gz0]).