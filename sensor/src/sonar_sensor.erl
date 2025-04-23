-module(sonar_sensor).

-behavior(hera_measure).

-define(ROBOT_HEIGHT, 30).
-export([init/1, measure/1]).

%============================================================================================================================================
%======================================================= HERA MEASURE BEHAVIOUR =============================================================
%============================================================================================================================================

init(_Args) ->
    timer:sleep(300),
    io:format("~n[SONAR_SENSOR] Starting measurements~n"),
    {ok, #{seq => 1}, #{
        name => sonar_sensor,
        iter => infinity,
        timeout => 300
    }}.
    
measure(State) ->
    SensorName = persistent_term:get(sensor_name),
    {ok, N} = get_rand_num(),
    case persistent_term:get(osensor, none) of
        none -> % Is alone in a room
            %io:format("[SONAR_SENOSR] Alone in room, measuring~n"),
            get_measure(State, SensorName);                      
        Osensor ->
            receive
                {measure} -> % Received when an other sensor wants to measure
                    io:format("[SONAR_SENSOR] possible collision, waiting for ~pms~n",[N]),
                    timer:sleep(N div 2 + 50),
                    get_measure(State, SensorName)            
            after N + 50 -> % Timeout, can measure
                %io:format("[SONAR_SENSOR] Timeout~n"),                
                hera_com:send_unicast(Osensor, "measure", "UTF8"),
                get_measure(State, SensorName)
            end
        end.            
            
%============================================================================================================================================
%========================================================= SONAR MEASURE  ===================================================================
%============================================================================================================================================

get_measure(State, SensorName) ->
    % Get the Pmod Maxsonar measure and transform it into the right format
    % @param State : the internal state of the module (tuple)
    % @param SensorName : the name of the current sensor (atom)
    Dist_inch = pmod_maxsonar:get(),
    Dist_cm = Dist_inch * 2.54,
    D = round(Dist_cm, 4),    
    %io:format("[SONAR_SENSOR] Sonar measure ~p : ~p~n", [Seq, D]),

    get_ground_distance(State, SensorName, D).

get_ground_distance(State, SensorName, D) ->
    % Uses the basic pythagorian formula to transform the distance based on the sensor's height
    % @param State : the internal state of the module (tuple)
    % @param SensorName : the name of the current sensor (atom)
    % @parma D : Sonar measure in cm (integer)
    Seq = maps:get(seq, State, 1),
    case hera_data:get(pos, SensorName) of
        [{_, _, _, [_ , _, H, _]}] ->
            True_measure = round(math:sqrt(math:pow(D, 2) - math:pow((H*100) - ?ROBOT_HEIGHT, 2)), 3), % Taking the height of the sonar into account

            %io:format("[SONAR_SENSOR] ground distance to robot : ~p : ~p~n", [Seq, True_measure]),

            hera_data:store(distance, SensorName, Seq, [True_measure]),
            NewState = State#{seq => Seq + 1},
            {ok, [True_measure], distance, SensorName, NewState};
        Msg ->
            io:format("[SONAR_SENSOR] Cannot get sensor height : ~p~n",[Msg]),
            {stop, cannot_get_height}
    end.

%============================================================================================================================================
%=========================================================== HELPER FUNC ====================================================================
%============================================================================================================================================
    
round(Number, Precision) ->
    % Rounds a number following parameters
    % @param Number : the number to round (float)
    % @param Precision : the precision of the wanted rounding (integer)
    Power = math:pow(10, Precision),
    round(Number * Power) / Power.

get_rand_num() ->
    % Returns a random number between 0 and 150
    Seed = {erlang:monotonic_time(), erlang:unique_integer([positive]), erlang:phash2(node())},
    rand:seed(exsplus, Seed),
    {ok, rand:uniform(150)}.   