-module(helper_module).

-export([
    round/2, calibrate/0, kalman_angle/8, complem_angle/1, select_angle/3,
    speed_ref/2, turn_ref/2, frequency_computation/4, wait/1, get_byte/1
]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Macros
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

-define(DEG_TO_RAD, math:pi() / 180).
-define(RAD_TO_DEG, 180.0 / math:pi()).

-define(ADV_V_MAX, 30.0).     % Maximum forward/backward speed in cm/s
-define(TURN_V_MAX, 80.0).    % Maximum turning speed in deg/s
-define(COEF_FILTER, 0.667).  % Complementary filter weighting
-define(FREQ_WINDOW, 100).    % Window size for frequency smoothing
-define(MS_TO_S, 1.0e-3).     % Milliseconds to seconds conversion

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Math helpers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Rounds a floating-point number to a specified number of decimal places.
%% @param Number - The number to round.
%% @param Precision - The number of decimal places to round to.
%% @return - The rounded number.
round(Number, Precision) ->
    Power = math:pow(10, Precision),
    round(Number * Power) / Power.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Sensor Calibration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Calibrates the gyroscope by averaging N samples of raw sensor data.
%% @return - A tuple containing the average gyroscope readings for the x, y, and z axes.
calibrate() ->
    N = 500,
    Data = [list_to_tuple(pmod_nav:read(acc, [out_x_g, out_y_g, out_z_g])) || _ <- lists:seq(1, N)],
    {X, Y, Z} = lists:unzip3(Data),
    [lists:sum(X) / N, lists:sum(Y) / N, lists:sum(Z) / N].

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Angle estimation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Extended Kalman Filter for fusing accelerometer, gyroscope, and sonar measurements.
%% - Predicts the next state based on the gyroscope data and updates it with the accelerometer and sonar data.
%% @param {Dt, Ax, Az, Gy, Gy0, X0, P0, D_sonar} - A tuple containing the time delta (Dt), accelerometer readings (Ax, Az),
%%   gyroscope readings (Gy, Gy0), initial state (X0), initial covariance (P0), and sonar distance (D_sonar).
%% @return - The updated state and covariance matrix after applying the Kalman filter.
kalman_angle(Dt, Ax, Az, Gy, Gy0, X0, P0, D_sonar) ->
    % Measurement noise covariance matrix (confidence in sensors)
    R = mat:matrix([[3.0, 0.0, 0.0],
                    [0.0, 3.0e-6, 0.0],
                    [0.0, 0.0, 25.0]]),

    % Process noise covariance matrix (confidence in model prediction)
    Q = mat:matrix([[3.0e-5, 0.0, 0.0],
                    [0.0, 10.0, 0.0],
                    [0.0, 0.0, 100.0]]),

    % Predict next state
    F = fun(X) ->
            [Th, W, D] = mat:to_array(X),
            mat:matrix([[Th + Dt * W], [W], [D]])
        end,

    % Jacobian of the prediction function
    Jf = fun(_) ->
            mat:matrix([[1, Dt, 0],
                        [0, 1, 0],
                        [0, 0, 1]])
         end,

    % Measurement function: maps internal state to expected measurements
    H = fun(X) ->
            [Th, W, D] = mat:to_array(X),
            mat:matrix([[Th], [W], [D]])
        end,

    % Jacobian of the measurement function
    Jh = fun(_) ->
            mat:matrix([[1, 0, 0],
                        [0, 1, 0],
                        [0, 0, 1]])
         end,

    % Build measurement vector
    Z = mat:matrix([
        [math:atan(Az / (-Ax))],
        [(Gy - Gy0) * ?DEG_TO_RAD],
        [D_sonar]
    ]),

    kalman:ekf({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z).

%% @doc Complementary filter combining gyroscope integration and accelerometer absolute angle.
%% - High-frequency response from gyroscope (short-term stability).
%% - Low-frequency correction from accelerometer (long-term drift correction).
%% @param {Dt, Ax, Az, Gy, Gy0, K, Angle_Complem, Angle_Rate} - A tuple containing the time delta (Dt), accelerometer readings (Ax, Az),
%%   gyroscope readings (Gy, Gy0), filter coefficient (K), current angle (Angle_Complem), and angular rate (Angle_Rate).
%% @return - The updated angle and angular rate after applying the complementary filter.
complem_angle({Dt, Ax, Az, Gy, Gy0, K, Angle_Complem, Angle_Rate}) ->
    % Low-pass filter on gyroscope angular rate
    Angle_Rate_New = (Gy - Gy0) * ?COEF_FILTER + Angle_Rate * (1 - ?COEF_FILTER),
    Delta_Gyr = Angle_Rate_New * Dt,
    Angle_Acc = math:atan(Az / (-Ax)) * ?RAD_TO_DEG,
    % Weighted combination
    Angle_Complem_New = (Angle_Complem + Delta_Gyr) * K + Angle_Acc * (1 - K),
    {Angle_Complem_New, Angle_Rate_New}.

%% @doc Selects the angle estimation method.
%% @param Switch - Boolean flag to select the angle estimation method.
%% @param Angle_Kalman - The angle estimated by the Kalman filter.
%% @param Angle_Complem - The angle estimated by the complementary filter.
%% @return - The selected angle based on the `Switch` parameter.
select_angle(true, Angle_Kalman, _Angle_Complem) -> Angle_Kalman;
select_angle(false, _Angle_Kalman, Angle_Complem) -> Angle_Complem.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Motion reference helpers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Computes the target advance velocity based on forward/backward control flags.
%% @param Forward - Boolean flag for forward motion.
%% @param Backward - Boolean flag for backward motion.
%% @return - The target advance velocity based on the control flags.
speed_ref(true, _) -> ?ADV_V_MAX;
speed_ref(_, true) -> -?ADV_V_MAX;
speed_ref(_, _)    -> 0.0.

%% @doc Computes the target turning velocity based on left/right control flags.
%% @param Left - Boolean flag for left turn.
%% @param Right - Boolean flag for right turn.
%% @return - The target turning velocity based on the control flags.
turn_ref(true, false) -> -?TURN_V_MAX;
turn_ref(false, true) -> ?TURN_V_MAX;
turn_ref(_, _)        -> 0.0.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Timing helpers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Computes running mean loop frequency based on elapsed time between iterations.
%% - Resets every ?FREQ_WINDOW iterations to avoid accumulating floating point errors.
%% @param Dt - Elapsed time since last iteration.
%% @param N - Number of iterations since last reset.
%% @param Freq - Current frequency estimate.
%% @param Mean_Freq - Running mean frequency.
%% @return - A tuple containing the updated number of iterations, frequency estimate, and running mean frequency.
frequency_computation(Dt, N, Freq, Mean_Freq) ->
    if
        N == ?FREQ_WINDOW ->
            {0, 0, Freq};  % Reset window
        true ->
            {N + 1, ((Freq * N) + (1 / Dt)) / (N + 1), Mean_Freq}
    end.

%% @doc Active wait for T milliseconds.
%% Busy-waits by continuously checking the system clock.
%% @param T_ms - Time in milliseconds to wait.
%% @return - Returns `ok` after the specified time has elapsed.
wait(T_ms) ->
    Tend = erlang:system_time() * ?MS_TO_S + T_ms,
    wait_help(erlang:system_time() * ?MS_TO_S, Tend).

%% @doc Helper function for active wait.
%% Continuously checks the system clock until the specified time has elapsed.
%% @param Tnow - Current time in seconds.
%% @param Tend - Target time in seconds.
%% @return - Returns `ok` if the current time is greater than or equal to the target time.
wait_help(Tnow, Tend) when Tnow >= Tend -> ok;
wait_help(_, Tend) ->
    wait_help(erlang:system_time() * ?MS_TO_S, Tend).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Bit manipulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Converts a list of 8 bits (MSB first) into a single byte.
%% @param [A, B, C, D, E, F, G, H] - A list of 8 bits.
%% @return - The byte value represented by the list of bits.
get_byte([A, B, C, D, E, F, G, H]) ->
    A*128 + B*64 + C*32 + D*16 + E*8 + F*4 + G*2 + H.
