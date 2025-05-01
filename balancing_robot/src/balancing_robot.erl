-module(balancing_robot).

-behavior(application).

-export([start/2, stop/1]).

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRiSP STARTUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start(_Type, _Args) ->
    {ok, Supervisor} = balancing_robot_sup:start_link(),
    [grisp_led:flash(L, yellow, 500) || L <- [1, 2]],
	_ = grisp:add_device(spi2, pmod_nav),
    pmod_nav:config(acc, #{odr_g => {hz,238}}),
    numerl:init(),
    timer:sleep(2000),
    spawn(main_loop, robot_init, []),
    hera_subscribe:subscribe(self()),
    config(),
    {ok, Supervisor}.

stop(_State) -> ok.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% NETWORK CONFIGURATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

config() ->
    persistent_term:put(name, list_to_atom("robot")),
    await_connection(),
    io:format("[ROBOT] Waiting for start signal ...~n~n").
    
await_connection() ->
    % Waiting for HERA to notify succesful connection
    
    io:format("[ROBOT] WiFi setup starting...~n"),
    receive        
        {hera_notify, "connected"} -> % Received when hera_com managed to connect to the network
            io:format("[ROBOT] WiFi setup done~n~n"),
            grisp_led:flash(2, white, 1000),
            discover_server()
    after 18000 ->
        io:format("[ROBOT] WiFi setup failed:~n~n"),
        grisp_led:flash(2, red, 750),
        await_connection()
    end.

discover_server() ->
    % Waits forever until the server sends a Ping
    io:format("[ROBOT] Waiting for ping from server~n"),
    receive
        {hera_notify, ["ping", Name, SIp, Port]} -> % Received upon server ping reception
            {ok, Ip} = inet:parse_address(SIp),
            IntPort = list_to_integer(Port),
            hera_com:add_device(list_to_atom(Name), Ip, IntPort),
            ack_loop()
    after 9000 ->
        io:format("[ROBOT] no ping from server~n"),
        discover_server()
    end.

ack_loop() ->
    % Tries to pair with the server by a Hello -> Ack
    % @param Id : Sensor's Id set by the jumpers (Integer)
    send_udp_message(server, "Hello from robot", "UTF8"),
    receive
        {hera_notify, ["Ack", _]} -> % Ensures the discovery of the sensor by the server
            io:format("[ROBOT] Received ACK from server~n"),
            [grisp_led:flash(L, green, 1000) || L <- [1, 2]],
            ok
    after 5000 ->
        ack_loop()
    end.

send_udp_message(Name, Message, Type) ->
    % Sends message
    % @param Name : name of the device to send to (atom)
    % @param Message : message to be sent (String/Tuple)
    % @param Type : type of message, can be UTF8 or Binary (String)
    hera_com:send_unicast(Name, Message, Type).