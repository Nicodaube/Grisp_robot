% @doc sensor public API.
-module(sensor).

-behavior(application).

% Callbacks
-export([start/2, stop/1]).

%============================================================================================================================================
%========================================================= BASIC GRISP FUNC =================================================================
%============================================================================================================================================

% @private
start(_Type, _Args) -> 
    io:format("[SENSOR] start initialization sequence~n"),
    {ok, _} = sensor_sup:start_link(),
    hera_subscribe:subscribe(self()),
    [grisp_led:flash(L, yellow, 500) || L <- [1, 2]],
    grisp:add_device(uart, pmod_maxsonar),

    config(),
    {ok, self()}.

% @private
stop(_State) -> ok.

%============================================================================================================================================
%========================================================= CONFIG TIME FUNC =================================================================
%============================================================================================================================================

config() ->
    who_am_i(),
    Id = persistent_term:get(id),
    io:format("[SENSOR] Waiting for start signal ...~n~n"),
    loop(Id).

who_am_i() ->
    % Computing sensor id and storing it in persistent data
    {ok, Id} = get_grisp_id(),
    io:format("[SENSOR] sensor id :~p~n",[Id]),
    persistent_term:put(sensor_name, list_to_atom("sensor_" ++ integer_to_list(Id))),
    persistent_term:put(id, Id),
    await_connection(Id).

await_connection(Id) ->
    % Waiting for HERA to notify succesful connection
    % @param Id : Sensor's Id set by the jumpers (Integer)
    io:format("[SENSOR] WiFi setup starting...~n"),
    receive        
        {hera_notify, "connected"} -> % Received when hera_com managed to connect to the network
            io:format("[SENSOR] WiFi setup done~n~n"),
            [grisp_led:flash(L, white, 1000) || L <- [1, 2]],
            discover_server(Id)
    after 18000 ->
        io:format("[SENSOR] WiFi setup failed:~n~n"),
        [grisp_led:flash(L, red, 750) || L <- [1, 2]],
        await_connection(Id)
    end.

discover_server(Id) ->
    % Waits forever until the server sends a Ping
    % @param Id : Sensor's Id set by the jumpers (Integer)
    io:format("[SENSOR] Waiting for ping from server~n"),
    receive
        {hera_notify, ["ping", Name, SIp, Port]} -> % Received upon server ping reception
            {ok, Ip} = inet:parse_address(SIp),
            IntPort = list_to_integer(Port),
            hera_com:add_device(list_to_atom(Name), Ip, IntPort),
            ack_loop(Id)
    after 9000 ->
        io:format("[SENSOR] no ping from server~n"),
        discover_server(Id)
    end.

ack_loop(Id) ->
    % Tries to pair with the server by a Hello -> Ack
    % @param Id : Sensor's Id set by the jumpers (Integer)
    send_udp_message(server, "Hello from " ++ integer_to_list(Id), "UTF8"),
    receive
        {hera_notify, ["Ack", _]} -> % Ensures the discovery of the sensor by the server
            io:format("[SENSOR] Received ACK from server~n"),
            [grisp_led:flash(L, green, 1000) || L <- [1, 2]],
            ok
    after 5000 ->
        ack_loop(Id)
    end.

get_grisp_id() ->
    % Computes the Id of the GRiSP board using the jumpers
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
    % Sends message
    % @param Name : name of the device to send to (atom)
    % @param Message : message to be sent (String/Tuple)
    % @param Type : type of message, can be UTF8 or Binary (String)
    hera_com:send_unicast(Name, Message, Type).
    
%============================================================================================================================================
%============================================================== LOOP ========================================================================
%============================================================================================================================================

loop(Id) ->
    % Main Sensor loop
    % @param Id : Sensor's Id set by the jumpers (Integer)
    receive
        {hera_notify, ["Add_Device", Name, SIp, Port]} ->  % Received at config time to register all used sensors 
            add_device(Id, Name, SIp, Port);          
        {hera_notify, ["Init_pos", SPosx, SPosy, SAngle, SRoom]} -> % Register Robot Device initial position
            store_robot_position(Id, SPosx, SPosy, SAngle, SRoom);            
        {hera_notify, ["Pos", Ids, Xs, Ys, Hs, As, RoomS]} -> % Received at config time To get all the sensors positions (X-Axis, Y-axis, Height, Angle, Room)           
            store_sensor_position(Id, Ids, Xs, Ys, Hs, As, RoomS);
        {hera_notify, ["Start", _]} -> % Received at the end of the configuration to launch the simulation
            start_measures(Id);
        {hera_notify, ["measure"]} -> % Received from an other sensor when it wants to measure
            wait_before_measure(Id);
        {hera_notify, ["Exit"]} ->
            reset_state(Id);
        {hera_notify, ["ping", _, _, _]} -> % Ignore the pings after server discovery
            loop(Id);
        {hera_notify, Msg} -> % Unhandled Message
            io:format("[SENSOR] Received unhandled message : ~p~n", [Msg]),
            loop(Id);
        Msg -> % Message not from hera_notify
            io:format("[SENSOR] receive strange message : ~p~n",[Msg]),
            loop(Id)
    end.

%============================================================================================================================================
%======================================================== LOOP FUNCTIONS ====================================================================
%============================================================================================================================================

add_device(Id, Name, SIp, SPort) ->
    % Adds a device to the list of known devices
    % @param Id : Sensor's Id set by the jumpers (Integer)
    % @param Name : name of the device to register (String)
    % @param SIp : IP adress (String)
    % @param SPort : Port (String)
    SelfName = persistent_term:get(sensor_name),
    case list_to_atom(Name) of 
        SelfName -> % Don't register self
            ok;
        OName -> 
            io:format("[SENSOR] Discovered new device : ~p~n", [Name]),
            {ok, Ip} = inet:parse_address(SIp),
            Port = list_to_integer(SPort),
            hera_com:add_device(OName, Ip, Port)
    end,            
    loop(Id).

store_robot_position(Id, SPosx, SPosy, SAngle, SRoom) ->
    % Stores the initial robot position
    % @param Id : Sensor's Id set by the jumpers (Integer)
    % @param SPosx : X axis position (String)
    % @param SPosY : Y axis position (String)
    % @param SAngle : Robot angle (String)
    % @param SRoom : Robot room position (String)
    SelfName = persistent_term:get(sensor_name),
    Posx = list_to_float(SPosx),
    Posy = list_to_float(SPosy),
    Angle = list_to_integer(SAngle),
    Room = list_to_integer(SRoom),            
    hera_data:store(robot_pos, SelfName, 1, [Posx, Posy, Angle, Room]),
    loop(Id).

store_sensor_position(Id, Ids, Xs, Ys, Hs, As, RoomS) ->
    % Store the position of a sensor
    % @param Id : Sensor's Id set by the jumpers (Integer)
    % @param Xs : X axis position (String)
    % @param Ys : Y axis position (String)
    % @param Hs : Z axis position (String)
    % @param As : Angle of sensor (String)
    % @param Rooms : Room number (String)
    X = list_to_float(Xs),
    Y = list_to_float(Ys),
    H = list_to_float(Hs),
    A = list_to_integer(As),
    Room = list_to_integer(RoomS),
    SensorName = list_to_atom("sensor_" ++ Ids),
    hera_data:store(room, SensorName, 1, [Room]),
    hera_data:store(pos, SensorName, 1, [X, Y, H, A]),
    %io:format("[SENSOR] Sensor's ~p position : (~p,~p) in room n°~p~n",[ParsedId,X,Y, Room]),
    loop(Id).

start_measures(Id) ->
    % Launch all the hera_measure modules to gather data
    % @param Id : Sensor's Id set by the jumpers (Integer)
    {ok, Angle_Pid} = hera:start_measure(target_angle, []),
    {ok, Sonar_Pid} = hera:start_measure(sonar_sensor, []),            
    {ok, Kalman_Pid} = hera:start_measure(kalman_measure, []),
    persistent_term:put(target_angle, Angle_Pid),
    persistent_term:put(sonar_sensor, Sonar_Pid),
    persistent_term:put(kalman_measure, Kalman_Pid),
    [grisp_led:color(L, green) || L <- [1, 2]],
    loop(Id).

wait_before_measure(Id) ->
    % Sends a message to the sonar module to wait before measuring
    % @param Id : Sensor's Id set by the jumpers (Integer)
    Pid = persistent_term:get(sonar_sensor, none),
    case Pid of
        none ->
            io:format("[SENSOR] Error : Sonar sensor has not spawned~n"),
            loop(Id);
        _ ->
            Pid ! {measure},
            loop(Id)
    end.

reset_state(Id) ->
    % Kills all hera_measures modules, resets all data and jump back to server discovery
    % @param Id : Sensor's Id set by the jumpers (Integer)
    exit_measure_module(sonar_sensor),
    exit_measure_module(target_angle),
    exit_measure_module(kalman_measure),

    timer:sleep(500),
    reset_data(),

    [grisp_led:flash(L, white, 1000) || L <- [1, 2]],      

    discover_server(Id),            
    io:format("[SENSOR] Waiting for start signal ...~n~n"),
    loop(Id).

reset_data() ->
    % Delete all config dependent and hera_measures data
    persistent_term:erase(osensor),
    persistent_term:erase(sonar_sensor),
    hera_data:reset(),
    io:format("[SENSOR] Data resetted~n~n").

exit_measure_module(Name) ->
    % Kills a module stored in persistent term
    % @param Name : the name of the module (atom)
    Pid = persistent_term:get(Name, none),    
    case Pid of
        none ->
            ok;
        _ -> 
            exit(Pid, shutdown)
    end.
