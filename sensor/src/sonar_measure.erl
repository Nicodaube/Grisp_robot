-module(sonar_measure).

-behavior(hera_measure).

-define(ROBOT_HEIGHT, 23).
-define(LPF_ALPHA, 0.2).
-define(SMOOTHING_WINDOW, 3).
-define(HAMPEL_WINDOW, 7).
-define(N_SIG, 2.0).

-export([init/1, measure/1]).

%============================================================================================================================================
%======================================================= HERA MEASURE BEHAVIOUR =============================================================
%============================================================================================================================================

init(_Args) ->
    get_sensor_role(),
    timer:sleep(200),
    io:format("~n[SONAR_MEASURE] Starting measurements~n"),
    State = #{
        seq => get_init_seq(),
        last_measure => none,
        hampel_buffer => [],
        smoothing_buffer => [],
        n_sig => ?N_SIG
    },
    {ok, State, #{name=>sonar_measure, iter=>infinity}}.
    
measure(State) ->  
    receive
        clock ->
            #{
                seq := Seq,
                last_measure := _,
                hampel_buffer := _,
                smoothing_buffer := _,
                n_sig := _
            } = State,
            [grisp_led:color(L, green) || L <- [1, 2]],

            SensorName = persistent_term:get(sensor_name),
            {Measure, LPFMeasure, Hampel_buffer, Smoothing_buffer} = get_measure(State),
            hera_com:send_unicast(server, "Distance,"++float_to_list(Measure)++","++atom_to_list(SensorName),"UTF8"),

            hera_data:store(distance, SensorName, Seq, [Measure]),

            NewState = State#{
                seq => Seq+1,
                last_measure => LPFMeasure,
                hampel_buffer => Hampel_buffer,
                smoothing_buffer => Smoothing_buffer,
                n_sig := ?N_SIG
            },
            {ok, [Measure], distance, SensorName, NewState}
    after 5000 ->
        [grisp_led:color(L, blue) || L <- [1, 2]],
        {undefined, State}
    end.
                 
%============================================================================================================================================
%============================================================== ROLE SETUP ==================================================================
%============================================================================================================================================

get_sensor_role() ->
    case persistent_term:get(osensor, none) of
        none -> % Is alone in a room
            io:format("[SONAR_MEASURE] No other sensor, sensor is master~n"),
            persistent_term:put(sensor_role, master);                     
        Osensor -> % Start Handshake
            case persistent_term:get(sensor_role, none) of
                none -> % Classical sensor bootstrap
                    {ok, Priority} = get_rand_num(),
                    TimeClock = erlang:monotonic_time(millisecond),
                    role_handshake(Osensor, Priority, TimeClock);
                _ ->  % Case where the sonar measure module crashed and was reloaded (need to find the latest seq number)
                    io:format("[SONAR_MEASURE] Recovering from crash"),
                    ok
            end
    end.       

role_handshake(Osensor, Priority, TimeClock) ->
    hera_com:send_unicast(Osensor, "Handshake," ++ integer_to_list(Priority) ++ "," ++ integer_to_list(TimeClock), "UTF8"),
    receive
        {handshake, OPriority, _} ->
            if
                Priority > OPriority ->
                    io:format("[SONAR_MEASURE] Local priority higher, sensor role : MASTER~n"),                    
                    wait_ack(Osensor),
                    Clock_Pid = spawn(clock_ticker, init, [TimeClock]),
                    persistent_term:put(clock, Clock_Pid),
                    persistent_term:put(sensor_role, master);
                Priority < OPriority ->
                    io:format("[SONAR_MEASURE] External priority higher, sensor role : SLAVE~n"),                    
                    wait_ack(Osensor),
                    persistent_term:put(sensor_role, slave);
                true ->
                    io:format("[SONAR_MEASURE] Priority collision, retrying~n"),
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
    io:format("SONAR DIST : ~p~n", [Dist_cm]),
    LPF_filtered = low_pass_filter(Dist_cm, State),
    {Hampel_Measure, Hampel_buffer} = hampel_filter(LPF_filtered, State),
    {Smoothed_measure, Smoothing_buffer} = smooth_measure(Hampel_Measure, State),

    case get_ground_distance(SensorName, Smoothed_measure) of
        {ok, Ground_measure} ->
            {Ground_measure, LPF_filtered, Hampel_buffer, Smoothing_buffer};
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
            Height_diff = (H*100)-?ROBOT_HEIGHT,
            if
                H*100 > ?ROBOT_HEIGHT ->
                    if 
                        D > Height_diff -> 
                            Ground_measure = math:sqrt(math:pow(D, 2) - math:pow(Height_diff, 2)); % Taking the height of the sonar into account
                        true ->
                            Ground_measure = Height_diff
                    end;
                true ->
                    Ground_measure = D % The robot is bigger than the sensor's height, no need for correction
            end,           

            {ok, Ground_measure};
        Msg ->
            io:format("[SONAR_MEASURE] Cannot get sensor height : ~p~n",[Msg]),
            {stop, cannot_get_height}
    end.

low_pass_filter(Ground_measure, State) ->
    #{
      seq := _,
      last_measure := Last,
      hampel_buffer := _,
      smoothing_buffer := _,
      n_sig := _
    } = State,

    case Last of
      none -> Ground_measure;
      _ -> ?LPF_ALPHA * Last + (1-?LPF_ALPHA)*Ground_measure
    end.

hampel_filter(Measure, State) ->
    #{
      seq := _,
      last_measure := _,
      hampel_buffer := BufH,
      smoothing_buffer := _,
      n_sig := N_sig
    } = State,

    New_BufH = append_buf(BufH, Measure, 2*?HAMPEL_WINDOW+1),
    Buffer_Median  = median(New_BufH),
    MAD = median([abs(X - Buffer_Median) || X <- New_BufH]),
    if 
        abs(Measure - Buffer_Median) > N_sig * MAD -> 
            {Buffer_Median, New_BufH};
        true ->
            {Measure, New_BufH}
    end.

smooth_measure(Measure, State) ->
    #{
      seq := _,
      last_measure := _,
      hampel_buffer := _,
      smoothing_buffer := BufS,
      n_sig := _
    } = State,

    New_BufS = append_buf(BufS, Measure, ?SMOOTHING_WINDOW),
    {median(New_BufS), New_BufS}.
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