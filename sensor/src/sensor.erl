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
    Id = persistent_term:get(id),
    loop(Id),
    {ok, self()}.

% @private
stop(_State) -> ok.

wifi_setup() ->
    io:format("[SENSOR] WiFi setup begin...~n"),

    % Computing sensor id and storing it in persistent data
    {ok, Id} = get_grisp_id(),
    io:format("[SENSOR] sensor id :~p~n",[Id]),
    persistent_term:put(sensor_name, list_to_atom("sensor_" ++ integer_to_list(Id))),
    persistent_term:put(id, Id),
    
    % Waiting for IP negociation
    timer:sleep(18000),

    % Checking if IP was aquired
    case inet:getifaddrs() of
        {ok, List} ->
            % Parse Ip from inet return
            {_, Parameters} = lists:nth(3, List),
            {_, IpTuple} = lists:nth(4, Parameters),

            % Check if the wifi setup has been done correctly
            case IpTuple of 
                {172,_,_,_} -> 
                    hera_com:add_device("SERVER", {172,20,10,8}, 5000),
                    handle_success(Id);                  
                {192,168,_,_} -> 
                    handle_success(Id);
                _ ->
                    io:format("[SENSOR] WiFi setup failed:~n"),
                    [grisp_led:flash(L, red, 750) || L <- [1, 2]],
                    error
                end;
        _ -> 
            io:format("[SENSOR] WiFi setup failed:~n"),
            [grisp_led:flash(L, red, 750) || L <- [1, 2]],
            error
    end,
    ok.

handle_success(Id) ->
    io:format("[SENSOR] WiFi setup done~n"),
    [grisp_led:flash(L, green, 1000) || L <- [1, 2]],
    send_udp_message("SERVER", "Hello from " ++ integer_to_list(Id), "UTF8").

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

send_udp_message(Name, Message, Type) ->
    hera_com:send_unicast(Name, Message, Type).

loop(Id) ->
    receive

        {hera_notify, ["Add_Device", Name, SIp, Port]} ->  % Received at config time to register all used devices           
            SelfName = persistent_term:get(sensor_name),           
            case Name of 
                SelfName -> % Don't register self
                    ok;
                _ -> 
                    io:format("[SENSOR] Discovered new device : ~p~n", [Name]),
                    {ok, Ip} = inet:parse_address(SIp),
                    IntPort = list_to_integer(Port),
                    hera_com:add_device(list_to_atom(Name), Ip, IntPort)
            end,            
            loop(Id);

        {hera_notify, ["Pos", Ids, Xs, Ys, RoomS]} -> % Received at config time To get all the sensors positions
            ParsedId = list_to_integer(Ids),
            X = list_to_float(Xs),
            Y = list_to_float(Ys),
            Room = list_to_integer(RoomS),
            SensorName = list_to_atom("sensor_" ++ Ids),
            hera_data:store(room, SensorName, 1, [Room]),
            hera_data:store(pos, SensorName, 1, [X, Y]),
            io:format("[SENSOR] Sensor's ~p position : (~p,~p) in room n°~p~n",[ParsedId,X,Y, Room]),
            if Id == ParsedId ->

                {ok, N} = get_rand_num(),
                io:format("[SENSOR] Starting measures in ~p msec~n", [N]),
                [grisp_led:color(L, green) || L <- [1, 2]],
                timer:sleep(N),
                io:format("[SENSOR] Starting measures~n"),
                hera:start_measure(sonar_sensor, []),
                spawn(target_angle, start_link, [Id]),
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
    


