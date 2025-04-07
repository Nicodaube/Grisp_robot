% @doc sensor public API.
-module(sensor).

-behavior(application).

% Callbacks
-export([start/2, stop/1]).

%--- Callbacks -----------------------------------------------------------------

% @private
start(_Type, _Args) -> 
    io:format("[SENSOR] start initialization sequence~n"),
    {ok, _} = sensor_sup:start_link(),
    hera_sub:subscribe(self()),
    [grisp_led:flash(L, yellow, 500) || L <- [1, 2]],
    grisp:add_device(uart, pmod_maxsonar),

    wifi_setup(),
    {ok, Id} = get_grisp_id(),
    loop(Id),
    {ok, self()}.

% @private
stop(_State) -> ok.

wifi_setup() ->
    io:format("[SENSOR] WiFi setup begin~n"),
    timer:sleep(12000),
    case inet:getifaddrs() of
        {ok, List} ->
            % Parse Ip from inet return
            {_, Parameters} = lists:nth(3, List),
            {_, IpTuple} = lists:nth(4, Parameters),

            % Check if the wifi setup has been done correctly
            case IpTuple of 
                {172,_,_,_} -> 
                    handle_success(IpTuple);                  
                {192,168,_,_} -> 
                    handle_success(IpTuple);
                _ ->
                    io:format("[SENSOR] WiFi setup failed:~n"),
                    [grisp_led:flash(L, red, 1000) || L <- [1, 2]],
                    error
                end;
        _ -> 
            io:format("[SENSOR] WiFi setup failed:~n"),
            [grisp_led:flash(L, red, 1000) || L <- [1, 2]],
            error
    end,
    ok.

handle_success(Ip) ->
    io:format("[SENSOR] WiFi setup done~n"),
    [grisp_led:color(L, green) || L <- [1, 2]],
    {ok, Id} = get_grisp_id(),
    io:format("[SENSOR] Sensor connected with IP : ~p and ID : ~p ~n", [Ip, Id]),
    send_udp_message({172,20,10,8}, 5000, "Hello from " ++ integer_to_list(Id)).

get_grisp_id() ->
    JMP1 = grisp_gpio:open(jumper_1, #{mode => input}),
    JMP2 = grisp_gpio:open(jumper_2, #{mode => input}),
    JMP3 = grisp_gpio:open(jumper_3, #{mode => input}),
    JMP4 = grisp_gpio:open(jumper_4, #{mode => input}),
    JMP5 = grisp_gpio:open(jumper_5, #{mode => input}),

    V1 = grisp_gpio:get(JMP1),
    V2 = grisp_gpio:get(JMP2),
    V3 = grisp_gpio:get(JMP3),
    V4 = grisp_gpio:get(JMP4),
    V5 = grisp_gpio:get(JMP5),

    SUM = (V1) + (V2 bsl 1) + (V3 bsl 2) + (V4 bsl 3) + (V5 bsl 4),
    {ok, SUM}.


send_udp_message(Host, Port, Message) ->
    {ok, Socket} = gen_udp:open(8000, [binary, {active, false}]),
    gen_udp:send(Socket, Host, Port, Message),
    gen_udp:close(Socket).

loop(Id) ->
    receive
        {hera_notify, ["Pos", Ids, Xs, Ys]} ->
            ParsedId = list_to_integer(Ids),
            X = list_to_float(Xs),
            Y = list_to_float(Ys),
            SensorName = list_to_atom("sensor" ++ Ids),
            hera_data:store(pos, SensorName, 0, [X, Y]),
            io:format("[SENSOR] Sensor's ~p position : (~p,~p)~n",[ParsedId,X,Y]),
            if Id == ParsedId ->                
                {ok, N} = get_rand_num(),
                timer:sleep(N),
                io:format("[SENSOR] Starting measures~n"),
                hera:start_measure(sonar_sensor, []),
                loop(Id);
               true ->
                loop(Id)
            end;
        {hera_notify, Msg} ->
            io:format("[SENSOR] Received unhandled message : ~p~n", [Msg]),
            loop(Id);
        Msg ->
            io:format("[SENSOR] receive ~p~n",[Msg]),
            loop(Id)
    end.

get_rand_num() ->
    Seed = {erlang:monotonic_time(), erlang:unique_integer([positive]), erlang:phash2(node())},
    rand:seed(exsplus, Seed),
    {ok, rand:uniform(2000)}.
    


