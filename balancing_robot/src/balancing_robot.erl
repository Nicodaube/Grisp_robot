-module(balancing_robot).

-behavior(application).

-export([start/2, stop/1]).
    
%============================================================================================================================================
%========================================================= BASIC GRISP FUNC =================================================================
%============================================================================================================================================

% Application start callback
start(_Type, _Args) ->
    {ok, Supervisor} = balancing_robot_sup:start_link(), % Start the supervisor
    [grisp_led:flash(L, yellow, 500) || L <- [1, 2]],

    config(),
    hera:start_measure(hera_interface, []), % Start measurement using hera interface
    {ok, Supervisor}. % Return ok and the supervisor

% Application stop callback
stop(_State) -> ok.

%============================================================================================================================================
%========================================================= CONFIG TIME FUNC =================================================================
%============================================================================================================================================

config() ->
    who_am_i(),

    pmod_nav:config(acc, #{odr_g => {hz,238}}), % Configure PMOD Nav accelerometer with output data rate of 238 Hz
    numerl:init(), % Initialize numerl library
    _ = grisp:add_device(spi2, pmod_nav), % Add PMOD Nav to Grisp Card on port SPI2
    hera_sub:subscribe(self()),
    
    ok.

who_am_i() ->
    persistent_term:put(sensor_name, robot),
    await_connection().

await_connection() ->
    % Waiting for HERA to notify succesful connection
    % @param Id : Sensor's Id set by the jumpers (Integer)
    io:format("[ROBOT] WiFi setup starting...~n"),
    receive        
        {hera_notify, "connected"} -> % Received when hera_com managed to connect to the network
            io:format("[ROBOT] WiFi setup done~n~n"),
            [grisp_led:flash(L, white, 1000) || L <- [1, 2]],
            discover_server()
    after 18000 ->
        io:format("[ROBOT] WiFi setup failed:~n~n"),
        [grisp_led:flash(L, red, 750) || L <- [1, 2]],
        await_connection()
    end.

discover_server() ->
    % Waits forever until the server sends a Ping
    % @param Id : Sensor's Id set by the jumpers (Integer)
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
    