-module(balancing_robot).

-behavior(application).

-export([start/2, stop/1]).

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start(_Type, _Args) ->
    {ok, Supervisor} = balancing_robot_sup:start_link(),
    grisp_led:color(1, {1, 0, 1}),
    grisp_led:color(2, {1, 0, 1}),
	_ = grisp:add_device(spi2, pmod_nav),
    pmod_nav:config(acc, #{odr_g => {hz,238}}),
    numerl:init(),
    timer:sleep(5000),
    hera:start_measure(hera_interface, []),
    {ok, Supervisor}.

stop(_State) -> ok.