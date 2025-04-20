-module(stability_engine).

-export([controller/4]).

-define(ADV_V_MAX, 30.0).
-define(ADV_ACCEL, 75.0).

-define(TURN_V_MAX, 80.0).
-define(TURN_ACCEL, 400.0).


%V_ref_new must be looped to V_ref
%% @doc
%% The `controller/4` function processes the control logic for the balancing robot.
%%
%% @param {Dt, Angle, Speed} - A tuple containing the time delta (Dt), the current angle of the robot (Angle), and the current speed of the robot (Speed).
%% @param {Pid_Speed, Pid_Stability} - A tuple containing the PID controller parameters for speed and stability.
%% @param {Adv_V_Goal, Adv_V_Ref} - A tuple containing the goal and reference values for the advance velocity.
%% @param {Turn_V_Goal, Turn_V_Ref} - A tuple containing the goal and reference values for the turning velocity.
%%
%% @return - The function returns a tuple with the updated control values for the robot.
controller({Dt, Angle, Speed}, {Pid_Speed, Pid_Stability}, {Adv_V_Goal, Adv_V_Ref}, {Turn_V_Goal, Turn_V_Ref}) ->

    % Saturate advance acceleration
    if   
        Adv_V_Goal > 0.0 ->
            % If the goal is positive, increase the reference velocity
            Adv_V_Ref_New = pid_controller:saturation(Adv_V_Ref + ?ADV_ACCEL * Dt, ?ADV_V_MAX);
        Adv_V_Goal < 0.0 ->
            % If the goal is negative, decrease the reference velocity
            Adv_V_Ref_New = pid_controller:saturation(Adv_V_Ref - ?ADV_ACCEL * Dt, ?ADV_V_MAX);
        true ->
            % If the goal is zero, gradually reduce the reference velocity to zero
            if
                Adv_V_Ref > 0.5  -> 
                    Adv_V_Ref_New = pid_controller:saturation(Adv_V_Ref - ?ADV_ACCEL * Dt, ?ADV_V_MAX);
                Adv_V_Ref < -0.5 -> 
                    Adv_V_Ref_New = pid_controller:saturation(Adv_V_Ref + ?ADV_ACCEL * Dt, ?ADV_V_MAX);
                true ->
                    Adv_V_Ref_New = 0.0
            end
    end,

    % Saturate turning acceleration
    if   
        Turn_V_Goal > 0.0 ->
            % If the goal is positive, increase the reference velocity
            Turn_V_Ref_New = pid_controller:saturation(Turn_V_Ref + ?TURN_ACCEL * Dt, ?TURN_V_MAX);
        Turn_V_Goal < 0.0 ->
            % If the goal is negative, decrease the reference velocity
            Turn_V_Ref_New = pid_controller:saturation(Turn_V_Ref - ?TURN_ACCEL * Dt, ?TURN_V_MAX);
        true ->
            % If the goal is zero, gradually reduce the reference velocity to zero
            if
                Turn_V_Ref > 0.5  -> 
                    Turn_V_Ref_New = pid_controller:saturation(Turn_V_Ref - ?TURN_ACCEL * Dt, ?TURN_V_MAX);
                Turn_V_Ref < -0.5 -> 
                    Turn_V_Ref_New = pid_controller:saturation(Turn_V_Ref + ?TURN_ACCEL * Dt, ?TURN_V_MAX);
                true ->
                    Turn_V_Ref_New = 0.0
            end
    end,

    % Speed PI control
    % Set the new reference velocity for the speed PID controller
    Pid_Speed ! {self(), {set_point, Adv_V_Ref_New}},
    % Provide the current speed as input to the speed PID controller
    Pid_Speed ! {self(), {input, Speed}},
    % Receive the control output from the speed PID controller, which is the target angle
    receive {_, {control, Target_angle}} -> ok end,

    % Stability PD control
    % Set the new reference angle for the stability PID controller
    Pid_Stability ! {self(), {set_point, Target_angle}},
    % Provide the current angle as input to the stability PID controller
    Pid_Stability ! {self(), {input, Angle}},
    % Receive the control output from the stability PID controller, which is the acceleration
    receive {_, {control, Acc}} -> ok end,

    % Return the control outputs: acceleration, new advance velocity reference, and new turning velocity reference
    {Acc, Adv_V_Ref_New, Turn_V_Ref_New}.

