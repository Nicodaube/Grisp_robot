% @doc sensor public API.
-module(sensor).

-behavior(application).

% Callbacks
-export([start/2, stop/1, start_sensor/0]).

%--- Callbacks -----------------------------------------------------------------

start_sensor() ->
    %timer:sleep(5000),
    
    %hera:start_measure(sonar_sensor, []),
    ok.

wifi_setup() ->
    io:format("[SENSOR] WiFi setup begin~n"),
    timer:sleep(20000),
    case inet:getifaddrs() of
        {ok, List} ->
            

            % Parse Ip from inet return
            {_, Parameters} = lists:nth(3, List),
            {_, IpTuple} = lists:nth(4, Parameters),

            % Check if the wifi setup has been done correctly
            case IpTuple of 
                {172,_,_,_} -> 
                    io:format("[SENSOR] WiFi setup done~n"),
                    [grisp_led:color(L, green) || L <- [1, 2]],
                    io:format("[SENSOR] Ip adress is : ~p~n", [IpTuple]),
                    {ok, Id} = get_grisp_id(),
                    
                    send_udp_message({172,20,10,8}, 5000, "Hello from " ++ Id),
                    
                    loop();
                {192,168,_,_} -> 
                    io:format("[SENSOR] WiFi setup done~n"),
                    [grisp_led:color(L, green) || L <- [1, 2]],
                    io:format("[SENSOR] Ip adress is : ~p~n", [IpTuple]),

                    {ok, Id} = get_grisp_id(),
                    
                    send_udp_message({172,20,10,8}, 5000, "Hello from " ++ Id),
                    
                    loop();
                _ ->
                    % error case
                    io:format("WiFi setup failed:~n"),
                    [grisp_led:flash(L, red, 1000) || L <- [1, 2]]
                end
    end,
    ok.


get_grisp_id() ->
    JMP1 = grisp_gpio:open(jumper_1, #{mode => input}),
    JMP2 = grisp_gpio:open(jumper_2, #{mode => input}) * 2,
    JMP3 = (grisp_gpio:open(jumper_3, #{mode => input}) * 2) * 2,
    JMP4 = ((grisp_gpio:open(jumper_4, #{mode => input}) * 2) * 2) * 2,
    JMP5 = ((grisp_gpio:open(jumper_5, #{mode => input}) * 2) * 2) * 2,

    SUM = JMP1 + JMP2 + JMP3 + JMP4 + JMP5,
    {ok, SUM}.

loop() ->
    Dist_inch = pmod_maxsonar:get(),
    Dist_cm = Dist_inch * 2.54,
    R_Dist_cm = round(Dist_cm, 4),
    String = float_to_list(R_Dist_cm),

    io:format("[SENSOR] Calculated Distance : ~p ~n",[R_Dist_cm]),
    timer:sleep(500),
    send_udp_message({172,20,10,8}, 5000, "Distance " ++ String),
    loop().

round(Number, Precision) ->
    Power = math:pow(10, Precision),
    round(Number * Power) / Power.

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
