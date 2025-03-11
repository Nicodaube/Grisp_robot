% @doc sensor public API.
-module(sensor).

-behavior(application).

% Callbacks
-export([start/2, stop/1, start_sensor/0]).

%--- Callbacks -----------------------------------------------------------------

start_sensor() ->
    %timer:sleep(5000),
    
    %io:format("[SENSOR] WiFi setup complete~n"),
    %hera:start_measure(sonar_sensor, []),
    ok.

wifi_setup() ->
    io:format("[SENSOR] WiFi setup begin~n"),
    timer:sleep(10000),
    case inet:getifaddrs() of
        {ok, List} ->
            io:format("[SENSOR] WiFi setup done~n"),
            [grisp_led:color(L, green) || L <- [1, 2]],

            % Parse Ip from inet return
            {_, Parameters} = lists:nth(3, List),
            {_, IpTuple} = lists:nth(4, Parameters),

            io:format("inet return: ~p~n", [IpTuple]);
        {error, Reason} ->
            io:format("WiFi setup failed: ~p~n", [Reason]),
            [grisp_led:flash(L, red, 100) || L <- [1, 2]]
    end,
    ok.


% @private
start(_Type, _Args) -> 
    io:format("[SENSOR] start initialization sequence~n"),
    [grisp_led:flash(L, yellow, 500) || L <- [1, 2]],

    wifi_setup(),
    {ok, Supervisor} = sensor_sup:start_link(), % start the supervisor and get its Pid
    spawn(sensor, start_sensor, []),
    {ok, self()}.

% @private
stop(_State) -> ok.
