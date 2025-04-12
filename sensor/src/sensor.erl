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

    io:format("[SENSOR] WiFi setup begin~n"),
    wifi_setup(),
    {ok, Id} = get_grisp_id(),
    loop(Id),
    {ok, self()}.

% @private
stop(_State) -> ok.

wifi_setup() ->
    timer:sleep(20000),
    case inet:getifaddrs() of
        {ok, List} ->
            % Parse Ip from inet return
            {_, Parameters} = lists:nth(3, List),
            {_, IpTuple} = lists:nth(4, Parameters),

            % Check if the wifi setup has been done correctly
            case IpTuple of 
                {172,_,_,_} -> 
                    hera_com:add_device("SERVER", {172,20,10,8}, 5000),
                    handle_success();                  
                {192,168,_,_} -> 
                    handle_success();
                _ ->
                    io:format("[SENSOR] WiFi setup failed:~n"),
                    [grisp_led:flash(L, red, 750) || L <- [1, 2]],
                    wifi_setup()
                end;
        _ -> 
            io:format("[SENSOR] WiFi setup failed:~n"),
            [grisp_led:flash(L, red, 750) || L <- [1, 2]],
            wifi_setup()
    end,
    ok.

handle_success() ->
    io:format("[SENSOR] WiFi setup done~n"),
    [grisp_led:flash(L, green, 1000) || L <- [1, 2]],
    {ok, Id} = get_grisp_id(),
    ack_loop(Id).
    

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

ack_loop(Id) ->
    send_udp_message("SERVER", "Hello from " ++ integer_to_list(Id), "UTF8"),
    receive
        {hera_notify, ["Ack", _]} ->
            io:format("[SENSOR] Received ACK from server"),
            ok

    after 10000 ->
        ack_loop(Id)
    end.

send_udp_message(Name, Message, Type) ->
    hera_com:send_unicast(Name, Message, Type).

loop(Id) ->
    receive
        {hera_notify, ["Add_Device", Name, SIp, Port]} ->  % Received at config time to register all used devices           
            SelfName = "Sensor_" ++ integer_to_list(Id),           
            case Name of 
                SelfName -> % Don't register self
                    ok;
                _ -> 
                    io:format("[SENSOR] Discovered new device : ~p~n", [Name]),
                    {ok, Ip} = inet:parse_address(SIp),
                    IntPort = list_to_integer(Port),
                    hera_com:add_device(Name, Ip, IntPort)
            end,            
            loop(Id);
        {hera_notify, ["Pos", Ids, Xs, Ys]} -> % Received at config time To get all the sensors positions
            ParsedId = list_to_integer(Ids),
            X = list_to_float(Xs),
            Y = list_to_float(Ys),
            SensorName = list_to_atom("sensor" ++ Ids),
            hera_data:store(pos, SensorName, 0, [X, Y]),
            io:format("[SENSOR] Sensor's ~p position : (~p,~p)~n",[ParsedId,X,Y]),
            if Id == ParsedId ->

                {ok, N} = get_rand_num(),
                io:format("[SENSOR] Starting measures in ~p msec~n", [N]),
                [grisp_led:color(L, green) || L <- [1, 2]],
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
        {getID, From} ->
            From ! id,
            loop(id);
        Msg ->
            io:format("[SENSOR] receive ~p~n",[Msg]),
            loop(Id)
    end.

get_rand_num() ->
    Seed = {erlang:monotonic_time(), erlang:unique_integer([positive]), erlang:phash2(node())},
    rand:seed(exsplus, Seed),
    {ok, rand:uniform(2000)}.
    


