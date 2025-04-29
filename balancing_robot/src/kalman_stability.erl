-module(kalman_stability).

-behavior(hera_measure).

-define(DEG_TO_RAD, math:pi() / 180).

-export([init/1, measure/1]).

%============================================================================================================================================
%======================================================= HERA MEASURE BEHAVIOUR ==============================================================
%============================================================================================================================================

init(_Args) ->
    io:format("~n[KALMAN_STABILITY] Starting measurements~n"),
    % Initialize Kalman filter matrices
    T0 = erlang:system_time()/1.0e6,    
    process_flag(priority, max), %Sets process priority to max
    X0 = mat:matrix([[0], [0]]), % Initial state matrix
    P0 = mat:matrix([[0.1, 0], [0, 0.1]]), % Initial covariance matrix

    hera_data:store(frequency, node(), 1, [0, 0, 200.0, T0]),
    hera_data:store(delta_time, node(), 1, [0.0]),
    hera_data:store(k_stability_state, node(), 1, [T0, X0, P0]),    
    [{_, _, _, [_, Gy0, _]}] = hera_data:get(gyroscope),
    persistent_term:put(gy0, Gy0),

    init_kalman_matrices(),
    %hera_data:store(comp_filter_state, node(), 1, [Gy0, 0.0, 0.0]),
    
    {ok, #{seq => 2}, #{
        name => kalman_stability,
        iter => infinity,
        timeout => 1
    }}.
    
measure(State) ->
    % Perform Kalman filter computation to estimate the angle.
    % The Kalman filter uses the current sensor data (accelerometer and gyroscope)
    % and the previous state (X0, P0) to compute the new state (X1, P1).
    Gyk = persistent_term:get(gy0),
    Robot = persistent_term:get(robot_main),

    [{_, Seq, _, [Tk, Xk, Pk]}] = hera_data:get(k_stability_state),
    [{_, _, _, [Gyk_1, Axk_1, Azk_1]}] = hera_data:get(gyroscope),
    %[{_, _, _, [Gyk, Angle_Complem, Angle_Rate]}] = hera_data:get(comp_filter_state),

    {Tk_1, Dt} = get_delta_time(Tk),
    {X1, P1} = kalman_angle(Dt, Axk_1, Azk_1, Gyk_1, Gyk, Xk, Pk),
    %[{ _, _, _, [_, _, Mean_Freq, _]}] = hera_data:get(frequency),

    % Compute the complementary filter angle.
    % The complementary filter combines gyroscope and accelerometer data to estimate the angle.
    % The filter uses a weighting factor (K) based on the mean frequency of the loop.
    %K = 1.25 / (1.25 + (1.0 / Mean_Freq)), % Compute the weighting factor.
    %{Angle_Complem_New, Angle_Rate_New} = helper_module:complem_angle({Dt, Axk_1, Azk_1, Gyk_1, Gyk, K, Angle_Complem, Angle_Rate}),
    %Seq = maps:get(seq, State),

    %hera_data:store(delta_time, node(), Seq+1, [Dt]),
    hera_data:store(k_stability_state, node(), Seq+1, [Tk_1, X1, P1]),
    %hera_data:store(comp_filter_state, node(), Seq, [Gyk, Angle_Complem_New, Angle_Rate_New]),

    %NewState = State#{seq => Seq +1},
    Robot ! {kalman_value, [Tk_1, X1, P1, Dt]},
    {ok, [{Tk_1, X1, P1}], kalman, self(), State}.


%============================================================================================================================================
%======================================================= HELPER FUNCTIONS ===================================================================
%============================================================================================================================================

get_delta_time(Tk) ->
    Tk_1 = erlang:system_time()/1.0e6,
    Dt = (Tk_1 - Tk)/1000.0,
    {Tk_1, Dt}.

% Measurement noise covariance matrix
init_kalman_matrices() ->
    R = mat:matrix([[3.0, 0.0], [0, 3.0e-6]]),

    % Process noise covariance matrix
    Q = mat:matrix([[3.0e-5, 0.0], [0.0, 10.0]]),

    % Measurement function
    H = fun (X) -> 
        [Th, W] = mat:to_array(X),
        mat:matrix([ 	[Th],  % Measurement of angle
                        [W ]   % Measurement of angular velocity
                    ])
    end,

    % Jacobian of the measurement function
    Jh = fun (X) -> 
        [_Th, _W] = mat:to_array(X),
        mat:matrix([  	[1, 0],  % Partial derivatives of the measurement function
                        [0, 1]  % with respect to the state variables
                    ])
        end,

    persistent_term:put(kalman_matrices, [R, Q, H, Jh]).


kalman_angle(Dt, Ax, Az, Gy, Gy0, X0, P0) ->
    [R, Q, H, Jh] = persistent_term:get(kalman_matrices),

    % Prediction Step
    [Th0, W0] = mat:to_array(X0),

    Xp = mat:matrix([
        [Th0 + Dt * W0],
        [W0]
    ]),

    Phi = mat:matrix([
        [1, Dt],
        [0, 1]
    ]),

    Pp = mat:eval([
        Phi, '*', P0, '*´', Phi, '+', Q
    ]),

    % Measurement vector
    Z = mat:matrix([
        [math:atan(Az / (-Ax))],
        [(Gy - Gy0) * ?DEG_TO_RAD]
    ]),

    % Kalman update
    kalman:ekf({Xp, Pp}, {fun(X) -> X end, fun(_X) -> Phi end}, {H, Jh}, Q, R, Z).
    

            