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
        io:format("[SENSOR] no ping from server~n~n"),
        discover_server(Id)
    end.

ack_loop(Id) ->
    % Tries to pair with the server by a Hello -> Ack
    % @param Id : Sensor's Id set by the jumpers (Integer)
    send_udp_message(server, "Hello," ++ integer_to_list(Id), "UTF8"),
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
        {hera_notify, ["Room_info", RoomId, TLx, TLy, BRx, BRy]} ->
            store_room_info(Id, RoomId, TLx, TLy, BRx, BRy);
        {hera_notify, ["Start", _]} -> % Received at the end of the configuration to launch the simulation            
            start_measures(Id);
        {hera_notify, ["Handshake", OPriority, OTimeClock]} -> % Received from the other sensor in during the sonar sensors role distribution
            resolve_handshake(Id, OPriority, OTimeClock);
        {hera_notify, ["Ok", _]} -> % Received from the other sensor to acknowledge the roles of the sensors 
            end_handshake(Id);
        {hera_notify, ["Exit"]} -> % Received when the controller is exited
            io:format("~n[SENSOR] Exit message received~n"),
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
    ack_message("Add_device", Name, Id),
    SelfName = persistent_term:get(sensor_name),
    case list_to_atom(Name) of 
        SelfName -> % Don't register self
            ok;
        OName ->             
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
    ack_message("Pos", "robot", Id),  
    case hera_data:get(robot_pos) of
        [{_, _, _, [_, _, _, _]}] ->
            ok;
        [] ->
            hera_data:store(robot_pos, SelfName, 1, [Posx, Posy, Angle, Room])
    end,        
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
    Device_name = "sensor_" ++ Ids,
    ack_message("Pos", Device_name, Id),
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
    
    %io:format("[SENSOR] Sensor's ~p position : (~p,~p) in room n°~p~n",[ParsedId,X,Y, Room]),
    loop(Id).

store_room_info(Id, RoomIdS, TLxS, TLyS, BRxS, BRyS) ->
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
    ack_message("Room_info", RoomId, Id),
    loop(Id).    

start_measures(Id) ->
    % Launch all the hera_measure modules to gather data
    % @param Id : Sensor's Id set by the jumpers (Integer)
    io:format("=================================================================================================~n"),
    io:format("~n~n[SENSOR] Start received, starting the computing phase~n"),
    case find_other_sensor() of
        ok ->
            {ok, Sonar_Pid} = hera:start_measure(sonar_sensor, []),
            {ok, Angle_Pid} = hera:start_measure(target_angle, []),
            persistent_term:put(sonar_sensor, Sonar_Pid),
            persistent_term:put(target_angle, Angle_Pid);            
        _ -> 
            {ok, Sonar_Pid} = hera:start_measure(sonar_sensor, []),
            persistent_term:put(sonar_sensor, Sonar_Pid)
    end,    
           
    %{ok, Kalman_Pid} = hera:start_measure(kalman_measure, []),
    
    %persistent_term:put(kalman_measure, Kalman_Pid),
    [grisp_led:color(L, green) || L <- [1, 2]],
    loop(Id).

resolve_handshake(Id, OPriority, OTimeClock) ->
    % Sends a message with the informations concerning the sensors role definition
    % @param Id : Sensor's Id set by the jumpers (Integer)
    % @param OPriority : Other sensor's random priority (String)
    % @param OTimeclock : Other sensor's time clock (String)
    Pid = persistent_term:get(sonar_sensor, none),
    case Pid of
        none ->
            io:format("[SENSOR] Error : Sonar sensor has not spawned~n"),
            loop(Id);
        _ ->
            %io:format("[SENSOR] Sending handshake informations~n"),
            Pid ! {handshake, list_to_integer(OPriority), list_to_integer(OTimeClock)},
            loop(Id)
    end.

end_handshake(Id)->
% Sends a ok message to signify the end of the handshake procedure
    % @param Id : Sensor's Id set by the jumpers (Integer)
    Pid = persistent_term:get(sonar_sensor, none),
    case Pid of
        none -> 
            io:format("[SENSOR] Error : Sonar sensor has not spawned~n"),
            loop(Id);
        _ ->
            %io:format("[SENSOR] Sending handshake ok~n"),
            Pid ! {ok, role},
            loop(Id)
    end.

reset_state(Id) ->
    % Kills all hera_measures modules, resets all data and jump back to server discovery
    % @param Id : Sensor's Id set by the jumpers (Integer)
    exit_measure_module(sonar_sensor),
    exit_measure_module(target_angle),
    %exit_measure_module(kalman_measure),

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
    persistent_term:erase(sensor_role),
    hera_com:reset_devices(),    
    hera_data:reset(),
    io:format("[SENSOR] Data resetted~n~n~n~n"),
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

find_other_sensor() ->
    % Sets up the affiliation with the room's sensor
    timer:sleep(300),
    SensName = persistent_term:get(sensor_name),
    case hera_data:get(room, SensName) of
      [{_, _, _, [Room]}] ->        
        case get_Osensor(Room) of 
            [H|_] -> % Multiple sensors in a room
                io:format("[SENSOR] Other sensor is : ~p~n", [H]),
                persistent_term:put(osensor, H),
                ok;
            _ -> % No other sensor
                io:format("[SENSOR] No other sensor in the room~n"),
                {error, no_other_sensor}
        end;
        
      Msg ->
        io:format("[SENSOR] Error in getting sensor pos ~p~n",[Msg]),
        timer:sleep(500),
        find_other_sensor()
    end.

get_Osensor(Room) ->
    % Finds the other sensors in the current room
    % @param Room : Room to set (Integer)
    Devices = persistent_term:get(devices),
    lists:foldl(
        fun({Name, _, _}, Acc) ->
            case Name of
                _ ->
                    case hera_data:get(room, Name) of
                        [{_, _, _, [ORoom]}] when Room =:= ORoom ->
                                %io:format("[SENSOR] Sens : ~p is in the same room as this sensor~n", [Name]),
                                [Name | Acc];
                        _ ->
                            Acc
                    end
            end
        end,
        [],
        Devices
    ).

ack_message(Message, Device, Id) ->
    Msg = "Ack," ++ Message ++ "," ++ Device ++ "," ++ integer_to_list(Id),
    send_udp_message(server, Msg, "UTF8").