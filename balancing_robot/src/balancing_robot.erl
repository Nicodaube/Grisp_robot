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
    loop_config(),
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
        io:format("[ROBOT] no ping from server~n~n"),
        discover_server()
    end.

ack_loop() ->
    % Tries to pair with the server by a Hello -> Ack
    % @param Id : Sensor's Id set by the jumpers (Integer)
    send_udp_message(server, "Hello,robot", "UTF8"),
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

loop_config() ->
    receive 
        {hera_notify, ["Add_Device", Name, SIp, Port]} ->  % Received at config time to register all used sensors 
            add_device(Name, SIp, Port);          
        {hera_notify, ["Init_pos", SPosx, SPosy, SAngle, SRoom]} -> % Register Robot Device initial position
            store_robot_position(SPosx, SPosy, SAngle, SRoom);            
        {hera_notify, ["Pos", Ids, Xs, Ys, Hs, As, RoomS]} -> % Received at config time To get all the sensors positions (X-Axis, Y-axis, Height, Angle, Room)           
            store_sensor_position(Ids, Xs, Ys, Hs, As, RoomS);
        {hera_notify, ["Room_info", RoomId, TLx, TLy, BRx, BRy]} ->
            store_room_info(RoomId, TLx, TLy, BRx, BRy);
        {hera_notify, ["Start", _]} -> % Received at the end of the configuration to launch the simulation
            start_measures();
        {hera_notify, ["Exit"]} -> % Received when gracefully exited the controller
            io:format("~n[ROBOT] Exit message received~n"),
            reset_state();
        {hera_notify, ["ping", _, _, _]} -> % Ignore the pings after server discovery
            loop_config();
        {hera_notify, Msg} -> % Unhandled Message
            io:format("[ROBOT] Received unhandled message : ~p~n", [Msg]),
            loop_config();
        Msg -> % Message not from hera_notify
            io:format("[ROBOT] receive strange message : ~p~n",[Msg]),
            loop_config()
    end.

loop_run() ->
    receive
        {hera_notify, ["Start", _]} -> % Received at the end of the configuration to launch the simulation
            io:format("~n[ROBOT] Already started~n"),
            loop_run;
        {hera_notify, ["Exit"]} -> % Received when gracefully exited the controller
            io:format("~n[ROBOT] Exit message received~n"),
            reset_state();
        {hera_notify, ["ping", _, _, _]} -> % Ignore the pings after server discovery
            loop_run();
        {hera_notify, Msg} -> % Unhandled Message
            io:format("[ROBOT] Received unhandled message : ~p~n", [Msg]),
            loop_run();
        Msg -> % Message not from hera_notify
            io:format("[ROBOT] receive strange message : ~p~n",[Msg]),
            loop_run()
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOOP FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

add_device(Name, SIp, SPort) ->
    % Adds a device to the list of known devices
    % @param Id : Sensor's Id set by the jumpers (Integer)
    % @param Name : name of the device to register (String)
    % @param SIp : IP adress (String)
    % @param SPort : Port (String)
    ack_message("Add_device", Name),
    case list_to_atom(Name) of 
        robot -> % Don't register self
            ok;
        OName ->             
            {ok, Ip} = inet:parse_address(SIp),
            Port = list_to_integer(SPort),
            hera_com:add_device(OName, Ip, Port)      
    end,            
    loop_config().

store_robot_position(SPosx, SPosy, SAngle, SRoom) ->
    % Stores the initial robot position
    % @param SPosx : X axis position (String)
    % @param SPosY : Y axis position (String)
    % @param SAngle : Robot angle (String)
    % @param SRoom : Robot room position (String)
    Posx = list_to_float(SPosx),
    Posy = list_to_float(SPosy),
    Angle = list_to_integer(SAngle),
    Room = list_to_integer(SRoom),  
    ack_message("Pos", "robot"),
    case hera_data:get(robot_pos) of
        [{_, _, _, [_, _, _, _]}] ->
            ok;
        [] ->
            hera_data:store(robot_pos, robot, 1, [Posx, Posy, Angle, Room])
    end,
    loop_config().

store_sensor_position(Ids, Xs, Ys, Hs, As, RoomS) ->
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
    Device_name = "sensor_" ++ Ids,
    ack_message("Pos", Device_name),
    SensorName = list_to_atom(Device_name),
    case hera_data:get(room, SensorName) of
        [{_, _, _, [_]}] ->
            ok;
        [] ->
            hera_data:store(room, SensorName, 1, [Room])
    end,

    case hera_data:get(pos, SensorName) of 
        [{_, _, _, [_, _, _, _]}] ->
            ok;
        [] ->
            hera_data:store(pos, SensorName, 1, [X, Y, H, A])
    end,
    
    %io:format("[ROBOT] Sensor's ~p position : (~p,~p) in room n°~p~n",[ParsedId,X,Y, Room]),
    loop_config().

store_room_info(RoomIdS, TLxS, TLyS, BRxS, BRyS) ->
    % Store the dimension of a room
    % @param Id : Sensor's Id set by the jumpers (Integer)
    % @param RoomIdS : Room concerned (String)
    % @param TLxS : Top left X corner position (String)
    % @param TLyS : Top left Y corner position (String)
    % @param BRxS : Bottom right X corner position (String)
    % @param BRyS : Bottom right Y corner position (String)
    TLx = list_to_float(TLxS),
    TLy = list_to_float(TLyS),
    BRx = list_to_float(BRxS),
    BRy = list_to_float(BRyS),
    RoomId = list_to_integer(RoomIdS),

    hera_data:store(room_info, RoomId, 1, [TLx, TLy, BRx, BRy]),
    ack_message("Room_info", RoomIdS),
    loop_config(). 

start_measures() ->
    % Launch all the hera_measure modules to gather data
    % @param Id : Sensor's Id set by the jumpers (Integer)
    io:format("=================================================================================================~n"),
    io:format("~n~n[ROBOT] Start received, starting the computing phase~n"),            
    {ok, Kalman_Pid} = hera:start_measure(kalman_measure, []),
    persistent_term:put(kalman_measure, Kalman_Pid),
    [grisp_led:color(L, green) || L <- [1, 2]],
    loop_run(). 

reset_state() ->
    % Kills all hera_measures modules, resets all data and jump back to server discovery
    % @param Id : Sensor's Id set by the jumpers (Integer)
    exit_measure_module(kalman_measure),

    timer:sleep(500),
    reset_data(),

    grisp_led:flash(2, white, 1000),      
    grisp_led:flash(1, green, 1000),      

    discover_server(),            
    io:format("[ROBOT] Waiting for start signal ...~n~n"),
    loop_config().

reset_data() ->
    % Delete all config dependent and hera_measures data
    persistent_term:erase(kalman_measure),
    hera_com:reset_devices(), 
    hera_data:reset(),
    io:format("[ROBOT] Data resetted~n~n~n~n"),
    io:format("=================================================================================================~n").

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

ack_message(Message, Device) ->
    % Used to send the acknowledgment message to the controller.
    % @param Message : The initial type message received by the robot (String)
    % @param Device : The name of the device concerned by the message (String)
    Msg = "Ack," ++ Message ++ "," ++ Device ++ ",robot",
    send_udp_message(server, Msg, "UTF8").