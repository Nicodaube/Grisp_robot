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
    %hera_data:store(comp_filter_state, node(), 1, [Gy0, 0.0, 0.0]),
    
    {ok, #{seq => 2}, #{
        name => kalman_stability,
        iter => infinity,
        timeout => 5
    }}.
    
measure(State) ->
    % Perform Kalman filter computation to estimate the angle.
    % The Kalman filter uses the current sensor data (accelerometer and gyroscope)
    % and the previous state (X0, P0) to compute the new state (X1, P1).
    Gyk = persistent_term:get(gy0),

    [{_, Seq, _, [Tk, Xk, Pk]}] = hera_data:get(k_stability_state),
    %[{_, _, _, [Gyk, Angle_Complem, Angle_Rate]}] = hera_data:get(comp_filter_state),

    {Tk_1, Dt} = get_delta_time(Tk),
    [{_, _, _, [Gyk_1, Axk_1, Azk_1]}] = hera_data:get(gyroscope),

    {X1, P1} = kalman_angle(Dt, Axk_1, Azk_1, Gyk_1, Gyk, Xk, Pk),
    %[{ _, _, _, [_, _, Mean_Freq, _]}] = hera_data:get(frequency),

    % Compute the complementary filter angle.
    % The complementary filter combines gyroscope and accelerometer data to estimate the angle.
    % The filter uses a weighting factor (K) based on the mean frequency of the loop.
    %K = 1.25 / (1.25 + (1.0 / Mean_Freq)), % Compute the weighting factor.
    %{Angle_Complem_New, Angle_Rate_New} = helper_module:complem_angle({Dt, Axk_1, Azk_1, Gyk_1, Gyk, K, Angle_Complem, Angle_Rate}),
    %Seq = maps:get(seq, State),

    hera_data:store(delta_time, node(), Seq+1, [Dt]),
    hera_data:store(k_stability_state, node(), Seq+1, [Tk_1, X1, P1]),
    %hera_data:store(comp_filter_state, node(), Seq, [Gyk, Angle_Complem_New, Angle_Rate_New]),

    NewState = State#{seq => Seq +1},
    {ok, [{Tk_1, X1, P1}], kalman, self(), NewState}.


%============================================================================================================================================
%======================================================= HELPER FUNCTIONS ===================================================================
%============================================================================================================================================

get_delta_time(Tk) ->
    Tk_1 = erlang:system_time()/1.0e6,
    Dt = (Tk_1 - Tk)/1000.0,
    {Tk_1, Dt}.


kalman_angle(Dt, Ax, Az, Gy, Gy0, X0, P0) ->
    % Measurement noise covariance matrix
    R = mat:matrix([[3.0, 0.0], [0, 3.0e-6]]),

    % Process noise covariance matrix
    Q = mat:matrix([[3.0e-5, 0.0], [0.0, 10.0]]),

    % State transition function
    F = fun (X) -> 
            [Th, W] = mat:to_array(X),
            mat:matrix([ 	[Th + Dt * W],  % Update angle based on angular velocity
                            [W          ]  % Angular velocity remains constant
                        ])
        end,

    % Jacobian of the state transition function
    Jf = fun (X) -> 
            [_Th, _W] = mat:to_array(X),
            mat:matrix([  	[1, Dt],  % Partial derivatives of the state transition function
                            [0, 1 ]  % with respect to the state variables
                        ])
            end,

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

    % Measurement vector
    Z = mat:matrix([[math:atan(Az / (-Ax))],  % Angle from accelerometer
                    [(Gy - Gy0) * ?DEG_TO_RAD]]),  % Angular velocity from gyroscope

    % Perform the Extended Kalman Filter update
    kalman:ekf({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z).

            