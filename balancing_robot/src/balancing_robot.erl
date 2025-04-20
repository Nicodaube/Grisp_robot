-module(balancing_robot).

-behavior(application).

-export([start_robot/0]).
-export([start/2, stop/1]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Function to start the robot
start_robot() ->
    timer:sleep(5000), % Waiting for setup of GRiSP application before launching the robot
    hera:start_measure(hera_interface, []), % Start measurement using hera interface
    ok.
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Application start callback
start(_Type, _Args) ->
    {ok, Supervisor} = balancing_robot_sup:start_link(), % Start the supervisor
    grisp_led:color(1, {1, 0, 1}), % Set LED 1 color to purple
    grisp_led:color(2, {1, 0, 1}), % Set LED 2 color to purple
    _ = grisp:add_device(spi2, pmod_nav), % Add PMOD Nav to Grisp Card on port SPI2

    pmod_nav:config(acc, #{odr_g => {hz,238}}), % Configure PMOD Nav accelerometer with output data rate of 238 Hz
    numerl:init(), % Initialize numerl library
    _ = spawn(balancing_robot, start_robot, []), % Spawn a process to start the robot
    {ok, Supervisor}. % Return ok and the supervisor

% Application stop callback
stop(_State) -> ok.