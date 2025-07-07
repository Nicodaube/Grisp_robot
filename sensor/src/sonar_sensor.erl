-module(sonar_sensor).

-behavior(hera_measure).

-define(ROBOT_HEIGHT, 23).
-define(LPF_ALPHA, 0.3).
-define(SMOOTHING_WINDOW, 11). % Must be odd
-define(HAMPEL_WIDOW, 8).
-define(N_SIG, 5.0).

-export([init/1, measure/1]).

%============================================================================================================================================
%======================================================= HERA MEASURE BEHAVIOUR =============================================================
%============================================================================================================================================

init(_Args) ->
    get_sensor_role(),
    timer:sleep(200),
    io:format("~n[SONAR_SENSOR] Starting measurements~n"),
    State = #{
        seq          => get_init_seq(),
        last_measure => none,
        buf_h        => [],
        buf_s        => [],
        w_h          => ?HAMPEL_WIDOW,
        n_sig        => ?N_SIG,
        w_s          => ?SMOOTHING_WINDOW
    },
    {ok, State, #{name=>sonar_sensor, iter=>infinity}}.
    
measure(State) ->  
    receive
        clock ->
            get_measure(State)
    end.
        
            
%============================================================================================================================================
%============================================================== ROLE SETUP ==================================================================
%============================================================================================================================================

get_sensor_role() ->
    case persistent_term:get(osensor, none) of
        none -> % Is alone in a room
            io:format("[SONAR_SENSOR] No other sensor, sensor is master~n"),
            persistent_term:put(sensor_role, master);                     
        Osensor -> % Start Handshake
            case persistent_term:get(sensor_role, none) of
                none -> % Classical sensor bootstrap
                    {ok, Priority} = get_rand_num(),
                    TimeClock = erlang:monotonic_time(millisecond),
                    %io:format("[SONAR SENSOR] Random handshake Priority ~p~n", [Priority]),
                    role_handshake(Osensor, Priority, TimeClock);
                _ ->  % Case where the sonar sensor crashed and was reloaded (need to find the latest seq number)
                    io:format("[SONAR_SENSOR] Recovering from crash"),
                    ok
            end
    end.       

role_handshake(Osensor, Priority, TimeClock) ->
    hera_com:send_unicast(Osensor, "Handshake," ++ integer_to_list(Priority) ++ "," ++ integer_to_list(TimeClock), "UTF8"),
    receive
        {handshake, OPriority, _} ->
            if
                Priority > OPriority ->
                    io:format("[SONAR_SENSOR] Local priority higher, sensor role : MASTER~n"),                    
                    wait_ack(Osensor),
                    Clock_Pid = spawn(clock_ticker, init, [TimeClock]),
                    persistent_term:put(clock, Clock_Pid),
                    persistent_term:put(sensor_role, master);
                Priority < OPriority ->
                    io:format("[SONAR_SENSOR] External priority higher, sensor role : SLAVE~n"),                    
                    wait_ack(Osensor),
                    persistent_term:put(sensor_role, slave);
                true ->
                    io:format("[SONAR_SENSOR] Priority collision, retrying~n"),
                    {ok, New_Priority} = get_rand_num(),
                    role_handshake(Osensor, New_Priority, TimeClock)                    
            end;
        {ok, role} ->
            ok
    after 500 ->
        role_handshake(Osensor, Priority, TimeClock) 
    end.   

wait_ack(Osensor) ->
    hera_com:send_unicast(Osensor, "Ok,role", "UTF8"),
    receive
        {ok, _} -> ok;
        _ -> wait_ack(Osensor)
    after 500 ->
        wait_ack(Osensor)
    end.

get_init_seq() ->
    SensorName = persistent_term:get(sensor_name),
    case hera_data:get(distance, SensorName) of
        [{_, _, Seq, [_]}] -> Seq +1;
        _ -> 1
    end.

%============================================================================================================================================
%========================================================= SONAR MEASURE  ===================================================================
%============================================================================================================================================

get_measure(State) ->
    % Get the Pmod Maxsonar measure and transform it into the right format
    % @param State : the internal state of the module (tuple)
    % @param SensorName : the name of the current sensor (atom)

    Dist_inch = pmod_maxsonar:get(),
    Dist_cm = Dist_inch * 2.54,    
    SensorName = persistent_term:get(sensor_name),

    case get_ground_distance(SensorName, Dist_cm) of
        {ok, Ground_measure} ->
            low_pass_filter(Ground_measure, State, SensorName);
        {stop, cannot_get_height} ->
            {stop, cannot_get_height}
    end.

get_ground_distance(SensorName, D) ->
    % Uses the basic pythagorian formula to transform the distance based on the sensor's height
    % @param State : the internal state of the module (tuple)
    % @param SensorName : the name of the current sensor (atom)
    % @param D : Sonar measure in cm (integer)

    case hera_data:get(pos, SensorName) of
        [{_, _, _, [_ , _, H, _]}] ->

            if
                H > ?ROBOT_HEIGHT ->
                    Ground_measure = math:sqrt(math:pow(D, 2) - math:pow((H*100)-?ROBOT_HEIGHT, 2)); % Taking the height of the sonar into account
                true ->
                    Ground_measure = D % The robot is bigger than the sensor's height, no need for correction
            end,           

            {ok, Ground_measure};
        Msg ->
            io:format("[SONAR_SENSOR] Cannot get sensor height : ~p~n",[Msg]),
            {stop, cannot_get_height}
    end.

low_pass_filter(Ground_measure, State, SensorName) ->
    #{
      seq          := Seq,
      last_measure := Last,
      buf_h        := BufH,
      buf_s        := BufS,
      w_h          := W_H,
      n_sig        := N_SIG,
      w_s          := W_S
    } = State,

    %% 1) LPF existant
    New1 = case Last of
      none -> Ground_measure;
      _    -> ?LPF_ALPHA * Last + (1-?LPF_ALPHA)*Ground_measure
    end,

    %% 2) Étape Hampel
    BufH1 = append_buf(BufH, New1, 2*W_H+1),
    MedH  = median(BufH1),
    MAD   = median([abs(X-MedH) || X <- BufH1]),
    ValH  = if abs(New1-MedH) > N_SIG * MAD -> MedH; true -> New1 end,

    %% 3) Lissage médian causal
    BufS1 = append_buf(BufS, ValH, W_S),
    Y     = median(BufS1),

    %% 4) envoi / stockage
    hera_com:send_unicast(server,
      "Distance,"++float_to_list(Y)++","++atom_to_list(SensorName),"UTF8"),
    hera_data:store(distance, SensorName, Seq, [Y]),

    NewState = State#{
      seq => Seq+1,
      last_measure => Y,
      buf_h => BufH1,
      buf_s => BufS1
    },
    {ok, [Y], distance, SensorName, NewState}.


%============================================================================================================================================
%=========================================================== HELPER FUNC ====================================================================
%============================================================================================================================================

get_rand_num() ->
    % Returns a random number between 0 and 2000
    persistent_term:get(id),
    Seed = {erlang:monotonic_time(), erlang:unique_integer([positive]), erlang:phash2(node())},
    rand:seed(exsplus, Seed),
    {ok, rand:uniform(3000)}.

append_buf(Buf, X, Len) ->
    Buf2 = lists:append(Buf, [X]),
    Excess = length(Buf2) - Len,
    case Excess > 0 of
      true  -> lists:sublist(Buf2, Excess+1, Len);
      false -> Buf2
    end.

median(List) when List =/= [] ->
    S = lists:sort(List),
    L = length(S),
    case L rem 2 of
      1 -> lists:nth((L+1) div 2, S);
      0 ->
        A = lists:nth(L div 2, S),
        B = lists:nth(L div 2+1, S),
        (A + B)/2.0
    end.