-module(kalman_measure).

-behavior(hera_measure).

-export([calibrate/1])
-export([init/1, measure/1]).
 
% We need to find the good insertitude
-define(VAR_Q, 0.001).
-define(VAR_R, 0.01).

-define(VAR_S, 0.01). % (0.2/2)^2
-define(VAR_P, 0.0025). % (0.1/2)^2
-define(VAR_AL, 0.1).
-define(VAR_AZ, 0.1).


init(_Args) ->
    io:format("~n[KALMAN_MEASURE] Starting measurements~n"),
    T0 = hera:timestamp(),
    Xpos = mat:zeros(9, 1),
    Ppos = mat:eye(9),
    Xor = [[1],[0],[0],[0]],
    Por = mat:diag([10,10,10,10]),
    State = {T0, Xpos, Ppos, Xor, Por, R0},
    {ok, State, #{seq => 2}, #{
        name => kalman_measure,
        iter => infinity,
        timeout => 30
    }}.
    
measure(State) ->
    SensorName = persistent_term:get(sensor_name),
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Pour le hera data il faudra avoir dans les données aussi accélération, vitesse angulaire en X,y,Z
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case hera_data:get(robot_pos, SensorName) of
        [{_, _, _, [OldX, OldY, OldAngle, OldRoom]}] ->
            DataSonars = hera_data:get(sonar),
            %DataNav = hera_data:get(nav) ## Faudrait du coup avoir les différentes données de navigation

            %It is use to create a secure temporel windows
            Nav = [Data || {_,_,Ts,Data} <- DataNav, T0 < Ts, T1-Ts < 500],
            Sonars = [{Node,Data} || {Node,_,Ts,Data} <- DataSonars,
                T0 < Ts, T1-Ts < 500],
            {XS,YS,ZS} = xyzS(Sonars), %Données sonar en 3D
            if
                length(Sonars) + length(Nav) == 0 -> % no measure
                    {undefined, {T0, Xpos, Ppos, Xor, Por, R0}};
                true ->
                    Orientation = q2dcm(Xor), % quaternion to direction cosine matrix
                    {Acc, AccLin, Gyro, Mag} = process_nav(Nav, Orientation),
                    Dtpos = (T1-T0)/1000,

                    % postition variation with the time matrix
                    Fpos = [
                        [1,Dtpos,(Dtpos*Dtpos)/2,0,0,0,0,0,0], % X 
                        [0,1,Dtpos,0,0,0,0,0,0], % V_X
                        [0,0,1,0,0,0,0,0,0], % Acc_X
                        [0,0,0,1,Dtpos,(Dtpos*Dtpos)/2,0,0,0], % Y
                        [0,0,0,0,1,Dtpos,0,0,0], % V_Y
                        [0,0,0,0,0,1,0,0,0], % Acc_Y
                        [0,0,0,0,0,0,1,Dtpos,(Dtpos*Dtpos)/2], % Z
                        [0,0,0,0,0,0,0,1,Dtpos], % V_Z
                        [0,0,0,0,0,0,0,0,1] % Acc_Z
                    ],

                    % modelling noise matrix
                    Qpos = mat:diag([?VAR_P,?VAR_P,?VAR_AL, ?VAR_P,?VAR_P,?VAR_AL, ?VAR_P,?VAR_P,?VAR_AL]), 
                    
                    % Obersvation matrix
                    Hpos =  [[1,0,0,0,0,0,0,0,0] || _ <- XS] ++ 
                            [[0,0,0,1,0,0,0,0,0] || _ <- YS] ++
                            [[0,0,0,0,0,0,1,0,0] || _ <- ZS] ++ 
                            [[0,0,1,0,0,0,0,0,0] || _ <- AccLin] ++
                            [[0,0,0,0,0,1,0,0,0] || _ <- AccLin] ++
                            [[0,0,0,0,0,0,0,0,1] || _ <- AccLin],
                    
                    % Z = H . X + noise, mesure vector matrix
                    Zpos =  [[X] || X <- XS] ++
                            [[Y] || Y <- YS] ++
                            [[Z] || Z <- ZS] ++
                            [[Ax] || [Ax,_,_] <- AccLin] ++
                            [[Ay] || [_,Ay,_] <- AccLin] ++
                            [[Az] || [_,_,Az] <- AccLin],
                    
                    % noise measure matrix, weight of the different measures
                    % The lower the value, the more confidence the Kalman has in the measurement.
                    Rpos = mat:diag(
                            [?VAR_S || _ <- Sonars] ++
                            [?VAR_AZ || _ <- AccLin] ++
                            [?VAR_AZ || _ <- AccLin] ++
                            [?VAR_AZ || _ <- AccLin]
                    ),

                    % Prediction and MAJ of the values
                    {Xpos0, Ppos0} = kalman:kf_predict({Xpos,Ppos}, Fpos, Qpos),
                    {Xpos1,Ppos1} =  kalman:kf_update({Xpos0, Ppos0}, Hpos, Rpos, Zpos),

                    if 
                        length(Nav) == 0 -> 
                            Valor = unit([X || [X] <- Xor]),
                            XorNorm = [[X] || X <- Valor],

                            Valpos = lists:append(Xpos1),
                            
                            {ok, Valpos ++ Valor, {T1, Xpos1, Ppos1, XorNorm, Por, R0}};
                        true -> 
                            [_,_,[Accx],_,_,[Accy],_,_,[Accz]] = Xpos1,

                            % prediction of the acceleration by kalman
                            AccLin2 = [[Accx,Accy,Accz]],
                            AccRot2 = mat:'*'(AccLin2, Orientation),
                            [Acc1,Acc2,Acc3] = Acc, 
                            [[AccRotx,AccRoty,AccRotz]] = AccRot2, 
                            R1 = ahrs([Acc1-AccRotx,Acc2-AccRoty,Acc3-AccRotz], Mag),

                            % New quatenion matrix
                            Quat = dcm2quat(mat:'*'(R1, R0)),
                            Dtor = (T1-T0)/1000, 
                            [Wx,Wy,Wz] = Gyro,

                            % is used to derive a quaternion
                            Omega = [
                                [0,Wx,Wy,Wz],
                                [-Wx,0,-Wz,Wy],
                                [-Wy,Wz,0,-Wx],
                                [-Wz,-Wy,Wx,0]
                            ],

                            For = mat:'+'(mat:eye(4), mat:'*'(0.5*Dtor, Omega)),
                            Qor = mat:diag([?VAR_Q,?VAR_Q,?VAR_Q,?VAR_Q]),
                            Hor = mat:eye(4),
                            Zor = mat:tr([Quat]),
                            Ror = mat:diag([?VAR_R,?VAR_R,?VAR_R,?VAR_R]),

                            {Xor0, Por0} = kalman:kf_predict({Xor,Por}, For, Qor),
                            {Xor1, Por1} = case qdot(Zor, Xor0) > 0 of
                                true ->
                                    kalman:kf_update({Xor0, Por0}, Hor, Ror, Zor);
                                false ->
                                    kalman:kf_update({mat:'*'(-1,Xor0), Por0}, Hor, Ror, Zor)
                    end,

                    Valor = unit([X || [X] <- Xor1]),
                    Xor1Norm = [[X] || X <- Valor],

                    Valpos = lists:append(Xpos1),

                    {ok, Valpos ++ Valor, {T1, Xpos1, Ppos1, Xor1Norm, Por1, R0}}
            end
            % Je ne sais pas ce que tu veux que je laisse
            Seq = maps:get(seq, State, 1),
            NewState = State#{seq => Seq +1},

            io:format("[KALMAN_MEASURE] New robot pos : (~p,~p) at ~p in room number ~p~n",[OldX, OldY, OldAngle, OldRoom]),
            send_robot_pos([OldX, OldY, OldAngle, OldRoom]),
            {ok, [OldX, OldY, OldAngle, OldRoom], robot_pos, SensorName, NewState};
        _ ->
            io:format("[KALMAN_MEASURE] Robot position not initialised~n"),
            {stop, no_robot_pos}
    end.
       

send_robot_pos(Pos) ->
    Pos_string = string:join([lists:flatten(io_lib:format("~p", [Val])) || Val <- Pos], ","),
    Msg = "Robot_pos," ++ Pos_string,
    hera_com:send_unicast(server, Msg, "UTF8").
            

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

xyzS(Sonars) ->
    X = [Pos + R*Dir || {sensor_fusion@sonar_1,[R,Pos,Dir]} <- Sonars],
    Y = [Pos + R*Dir || {sensor_fusion@sonar_2,[R,Pos,Dir]} <- Sonars],
    Z = [Pos + R*Dir || {sensor_fusion@sonar_3,[R,Pos,Dir]} <- Sonars],
    {X,Y,Z}.

qdot([[Q11], [Q12], [Q13], [Q14]], [[Q21], [Q22], [Q23], [Q24]]) ->
    Q11*Q21 + Q12*Q22 + Q13*Q23 + Q14*Q24.

process_nav([],_) -> 
    {[],[],[],[]};
process_nav([Nav], R) ->
    {Acc, Next} = lists:split(3, Nav),
    {Gyro, Mag} = lists:split(3, Next),
    Accrot = mat:'*'([Acc], mat:tr(R)),
    RotAcc = mat:'-'(Accrot, [[0,0,-9.81]]),
    {Acc, RotAcc, Gyro, Mag}. 


unit(V) ->
    Norm = math:sqrt(lists:sum([X*X || X <- V])),
    [X/Norm || X <- V].


scale(List, Factor) ->
    [X*Factor || X <- List].


calibrate(Comp, Registers, N) ->
    Data = [list_to_tuple(pmod_nav:read(Comp, Registers))
        || _ <- lists:seq(1,N)],
    {X, Y, Z} = lists:unzip3(Data),
    [lists:sum(X)/N, lists:sum(Y)/N, lists:sum(Z)/N].


ahrs(Acc, Mag) ->
    Down = unit([-A || A <- Acc]),
    East = unit(cross_product(Down, unit(Mag))),
    North = unit(cross_product(East, Down)),
    mat:tr([North, East, Down]).


cross_product([U1,U2,U3], [V1,V2,V3]) -> 
    [U2*V3-U3*V2, U3*V1-U1*V3, U1*V2-U2*V1].


dcm2quat(R) ->
    [[R11,R12,R13],
     [R21,R22,R23],
     [R31,R32,R33]
    ] = R,
    Q12 = 0.25*(1+R11+R22+R33),
    Q1 = math:sqrt(Q12),
    V = [
        4*Q12,
        R32-R23,
        R13-R31,
        R21-R12
    ],
    scale(V, (0.25/Q1)).


q2dcm([[Q0], [Q1], [Q2], [Q3]]) -> 
    R00 = 2 * (Q0 * Q0 + Q1 * Q1) - 1,
    R01 = 2 * (Q1 * Q2 - Q0 * Q3),
    R02 = 2 * (Q1 * Q3 + Q0 * Q2),
     
    R10 = 2 * (Q1 * Q2 + Q0 * Q3),
    R11 = 2 * (Q0 * Q0 + Q2 * Q2) - 1,
    R12 = 2 * (Q2 * Q3 - Q0 * Q1),
     
    R20 = 2 * (Q1 * Q3 - Q0 * Q2),
    R21 = 2 * (Q2 * Q3 + Q0 * Q1),
    R22 = 2 * (Q0 * Q0 + Q3 * Q3) - 1,

    [[R00, R01, R02],
     [R10, R11, R12],
     [R20, R21, R22]].