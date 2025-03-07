-module(stability_engine).

-export([controller/4]).

-define(ADV_V_MAX, 30.0).
-define(ADV_ACCEL, 75.0).

-define(TURN_V_MAX, 80.0).
-define(TURN_ACCEL, 400.0).


%V_ref_new must be looped to V_ref
controller({Dt, Angle, Speed}, {Pid_Speed, Pid_Stability}, {Adv_V_Goal, Adv_V_Ref}, {Turn_V_Goal, Turn_V_Ref}) ->

    %Saturate advance acceleration
    if   
        Adv_V_Goal > 0.0 ->
            Adv_V_Ref_New = pid_controller:saturation(Adv_V_Ref+?ADV_ACCEL*Dt, ?ADV_V_MAX);
        Adv_V_Goal < 0.0 ->
            Adv_V_Ref_New = pid_controller:saturation(Adv_V_Ref- ?ADV_ACCEL*Dt, ?ADV_V_MAX);
        true ->
            if
                Adv_V_Ref > 0.5  -> 
                    Adv_V_Ref_New = pid_controller:saturation(Adv_V_Ref- ?ADV_ACCEL*Dt, ?ADV_V_MAX);
                Adv_V_Ref < -0.5 -> 
                    Adv_V_Ref_New = pid_controller:saturation(Adv_V_Ref+?ADV_ACCEL*Dt, ?ADV_V_MAX);
                true ->
                    Adv_V_Ref_New = 0.0
            end
    end,
    % Adv_V_Ref_New = Adv_V_Goal,

    %Saturate turning acceleration
    if   
        Turn_V_Goal > 0.0 ->
            Turn_V_Ref_New = pid_controller:saturation(Turn_V_Ref+?TURN_ACCEL*Dt, ?TURN_V_MAX);
        Turn_V_Goal < 0.0 ->
            Turn_V_Ref_New = pid_controller:saturation(Turn_V_Ref- ?TURN_ACCEL*Dt, ?TURN_V_MAX);
        true ->
            if
                Turn_V_Ref > 0.5  -> 
                    Turn_V_Ref_New = pid_controller:saturation(Turn_V_Ref- ?TURN_ACCEL*Dt, ?TURN_V_MAX);
                Turn_V_Ref < -0.5 -> 
                    Turn_V_Ref_New = pid_controller:saturation(Turn_V_Ref+?TURN_ACCEL*Dt, ?TURN_V_MAX);
                true ->
                    Turn_V_Ref_New = 0.0
            end
    end,
    % Turn_V_Ref_New = Turn_V_Goal,

    %Speed PI
    Pid_Speed ! {self(), {set_point, Adv_V_Ref_New}},
    Pid_Speed ! {self(), {input, Speed}},
    receive {_, {control, Target_angle}} -> ok end,

    %TODO: send Target_angle to log

    % io:format("~p~n",[Target_angle]),

    %Stability PD
    Pid_Stability ! {self(), {set_point, Target_angle}},
    Pid_Stability ! {self(), {input, Angle}},
    receive {_, {control, Acc}} -> ok end,

    {Acc, Adv_V_Ref_New, Turn_V_Ref_New}.

