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
    timer:sleep(20000),
    case inet:getifaddrs() of
        {ok, List} ->
            io:format("[SENSOR] WiFi setup done~n"),
            [grisp_led:color(L, green) || L <- [1, 2]],

            % Parse Ip from inet return
            {_, Parameters} = lists:nth(3, List),
            {_, IpTuple} = lists:nth(4, Parameters),
            io:format("[SENSOR] Ip adress is : ~p~n", [IpTuple]),

            send_udp_message({192,168,136,1}, 5000, inet:ntoa(IpTuple)),
            
            loop();
            
        {error, Reason} ->
            io:format("WiFi setup failed: ~p~n", [Reason]),
            [grisp_led:flash(L, red, 100) || L <- [1, 2]]
    end,
    ok.

loop() ->
    Distance = pmod_maxsonar:get(),
    String = integer_to_list(Distance),
    io:format("[SENSOR] Calculated Distance : ~p ~n",[String]),
    timer:sleep(2000),
    send_udp_message({192,168,136,1}, 5000, String),
    loop().


send_udp_message(Host, Port, Message) ->
    {ok, Socket} = gen_udp:open(9000, [binary, {active, false}]),
    gen_udp:send(Socket, Host, Port, Message),
    gen_udp:close(Socket).

% @private
start(_Type, _Args) -> 
    io:format("[SENSOR] start initialization sequence~n"),
    {ok, _} = sensor_sup:start_link(),
    [grisp_led:flash(L, yellow, 500) || L <- [1, 2]],
    grisp:add_device(uart, pmod_maxsonar),

    wifi_setup(),
    spawn(sensor, start_sensor, []),
    {ok, self()}.

% @private
stop(_State) -> ok.
