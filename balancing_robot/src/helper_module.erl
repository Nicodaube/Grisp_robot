-module(helper_module).

-export([calibrate/0, kalman_angle/7, complem_angle/1, select_angle/3, speed_ref/2, turn_ref/2, frequency_computation/4, wait/1, get_byte/1]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Macros
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

-define(DEG_TO_RAD, math:pi() / 180).

%Advance constant
-define(ADV_V_MAX, 30.0).

%Turning constant
-define(TURN_V_MAX, 80.0).

%Angle at which robot is considered "down"
-define(MAX_ANGLE, 25.0).

%Coefficient for the complementary filter
-define(COEF_FILTER, 0.667).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate() ->
    % Number of samples to collect for calibration
    N = 500,

    % Collect N samples of accelerometer data (x, y, z) from the pmod_nav module
    Data = [list_to_tuple(pmod_nav:read(acc, [out_x_g, out_y_g, out_z_g])) || _ <- lists:seq(1, N)],

    % Separate the collected data into three lists: X, Y, Z
    {X, Y, Z} = lists:unzip3(Data),

    % Compute the average of each axis to determine the calibration offsets
    [lists:sum(X) / N, lists:sum(Y) / N, lists:sum(Z) / N]. %[Gx0, Gy0, Gz0]

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

complem_angle({Dt, Ax, Az, Gy, Gy0, K, Angle_Complem, Angle_Rate}) ->

    % Compute the new angular rate using a low-pass filter
    Angle_Rate_New = (Gy - Gy0) * ?COEF_FILTER + Angle_Rate * (1 - ?COEF_FILTER),

    % Compute the angle increment from the gyroscope data
    Delta_Gyr = Angle_Rate_New * Dt,

    % Compute the absolute angle from the accelerometer data
    Angle_Acc = math:atan(Az / (-Ax)) * 180 / math:pi(),

    % Apply the complementary filter to combine gyroscope and accelerometer data
    Angle_Complem_New = (Angle_Complem + Delta_Gyr) * K + Angle_Acc * (1 - K), 
    
    % Return the updated complementary angle and angular rate
    {Angle_Complem_New, Angle_Rate_New}.

% Selects the angle based on the provided switch.
% If the switch is true, the Kalman filter angle is selected.
% Otherwise, the complementary filter angle is selected.
select_angle(Switch, Angle_Kalman, Angle_Complem) ->
    if
        Switch -> 
            Angle = Angle_Kalman;  % Use Kalman filter angle
        true ->
            Angle = Angle_Complem  % Use complementary filter angle
    end,
    Angle.

% Computes the speed reference based on the forward and backward inputs.
% If the forward input is true, the speed reference is set to the maximum forward speed.
% If the backward input is true, the speed reference is set to the maximum backward speed.
% If neither input is true, the speed reference is set to zero.
speed_ref(Forward, Backward) ->
    if
        Forward ->
            Adv_V_Goal = ?ADV_V_MAX;  % Set speed to maximum forward speed
        Backward ->
            Adv_V_Goal = - ?ADV_V_MAX;  % Set speed to maximum backward speed
        true ->
            Adv_V_Goal = 0.0  % Set speed to zero
    end,
    Adv_V_Goal.

%% @doc Determines the target turning velocity (`Turn_V_Goal`) based on the input flags `Left` and `Right`.
%%      - If `Right` is true, the target turning velocity is set to the maximum turning velocity (`?TURN_V_MAX`).
%%      - If `Left` is true, the target turning velocity is set to the negative of the maximum turning velocity (`-?TURN_V_MAX`).
%%      - If neither `Left` nor `Right` is true, the target turning velocity is set to 0.0.
%%
%% @param Left A boolean flag indicating whether to turn left.
%% @param Right A boolean flag indicating whether to turn right.
%% @return The target turning velocity (`Turn_V_Goal`) as a float.
turn_ref(Left, Right) ->
    if
        Right ->
            Turn_V_Goal = ?TURN_V_MAX;
        Left ->
            Turn_V_Goal = - ?TURN_V_MAX;
        true ->
            Turn_V_Goal = 0.0
    end,
    Turn_V_Goal.


%% @doc Computes the frequency and its mean over a series of time intervals.
%%      - If the counter `N` reaches 100, it resets the counter and frequency accumulator.
%%      - Otherwise, it updates the frequency and mean frequency using the provided time interval `Dt`.
%%
%% @param Dt The time interval in seconds.
%% @param N The current counter for the number of intervals.
%% @param Freq The current frequency accumulator.
%% @param Mean_Freq The current mean frequency.
%% @return A tuple `{N_New, Freq_New, Mean_Freq_New}` containing the updated counter, frequency, and mean frequency.
frequency_computation(Dt, N, Freq, Mean_Freq) ->
    if 
        % Reset the counter and frequency accumulator when N reaches 100
        N == 100 ->
            N_New = 0,
            Freq_New = 0,
            Mean_Freq_New = Freq;
        true ->
            % Update the counter, frequency, and mean frequency
            N_New = N + 1,
            Freq_New = ((Freq * N) + (1 / Dt)) / (N + 1),
            Mean_Freq_New = Mean_Freq
    end,
    {N_New, Freq_New, Mean_Freq_New}.

%% @doc Pauses execution for `T` milliseconds.
wait(T) -> wait_help(erlang:system_time() / 1.0e6, T).

wait_help(Tnow, Tend) when Tnow >= Tend -> ok;
wait_help(_, Tend) -> wait_help(erlang:system_time() / 1.0e6, Tend).

%% @doc Transforms a list of 8 bits into a single byte.
%% @spec get_byte([0 | 1, 0 | 1, 0 | 1, 0 | 1, 0 | 1, 0 | 1, 0 | 1, 0 | 1]) -> integer()
%% @param List A list of 8 binary digits (0s and 1s) representing the bits of a byte.
%% @return An integer representing the byte value calculated from the binary digits.
%Transforms a list of 8 bits into a byte
get_byte(List) ->
    [A, B, C, D, E, F, G, H] = List,
    A*128 + B*64 + C*32 + D*16 + E*8 + F*4 + G*2 + H. 

