-module(hera_interface).

-behavior(hera_measure).

-export([init/1, measure/1]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Initializes the Hera interface for the robot.
%% @param _Args - Initialization arguments (not used).
%% @return - A tuple containing the robot process ID, measurement specification, and initial time.
init(_Args) ->
    % Define the measurement specification for Hera (infinite iterations, async mode)
    Spec = #{name => kaltest, iter => infinity, sync => false},
    
    % Capture system start time [ms]
    T0 = erlang:system_time() / 1.0e6,
    
    % Spawn the robot control process (main_loop:robot_init/1)
    Robot_Pid = spawn(main_loop, robot_init, [self()]),
    
    % Define the names and number of fields for logged data
    Names = "T,Freq,Gy,Acc,CtrlByte,AngleAccelerometer,AngleKalman,AngleComplem,Vref,Select,Adv_Ref,Turn_Ref,Speed",
    Len = 13,

    io:format("[Hera] Pid of the Hera interface: ~p.~n", [self()]),
    
    % Set LEDs to yellow to indicate ready state
    grisp_led:color(1, {1, 1, 0}),
    grisp_led:color(2, {1, 1, 0}),

    % Return initialization tuple: robot state, specification, and initial time
    {ok, {Robot_Pid, {Names, Len, 1}, T0, 0, 0, 200, T0}, Spec}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Measurement Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Main measurement loop for the Hera interface.
%% @param {Robot_Pid, {Names, Len, File_Number}, StartTime, N, Freq, Mean_Freq, T0} - Current state of the robot and measurement parameters.
%% @return - A tuple containing the updated state of the robot and measurement parameters.
measure({Robot_Pid, {Names, Len, File_Number}, StartTime, _N, _Freq, _Mean_Freq, T0}) ->

    % Request sensor and control data from the robot
    Robot_Pid ! {self(), get_all_data},
    
    receive 
        {_, data, _Robot_Data} -> ok
    end,

    % Compute elapsed time since last measurement
    T1 = erlang:system_time() / 1.0e6,
    _Dt = (T1 - T0) / 1000.0,

    % Initialize updated loop counters (placeholder for now)
    {N_New, Freq_New, Mean_Freq_New} = {0.0, 0.0, 0.0},

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Logging Management
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    File_Name = "measures" ++ integer_to_list(File_Number) ++ ".txt",

    % Check for start/stop log commands (non-blocking)
    {Start_Log, Stop_Log} = 
        receive
            {_, start_log} -> {true, false};
            {_, stop_log}  -> {false, true}
        after 0 ->
            {false, false}
        end,

    % Start logging: write header to file
    if
        Start_Log ->
            io:format("[Hera] Start logging~n"),
            log_init(Names, File_Name);
        true -> ok
    end,

    % Stop logging: retrieve data and write it
    if
        Stop_Log ->
            File_Number_New = File_Number + 1,
            io:format("[Hera] New log file with name: ~s~n", [File_Name]),
            log_init(Names, File_Name),

            % Set LED to red during file writing
            grisp_led:color(1, {1, 0, 0}),

            % Request logged values from robot
            Robot_Pid ! {self(), log_values},

            receive 
                {_, log, Log_Values} -> ok
            end,

            % Write all logged values to file
            log(Log_Values, File_Name, Len),

            % Set LED back to yellow
            grisp_led:color(1, {1, 1, 0}),
            io:format("[Hera] Done writing to file!~n");
        true ->
            File_Number_New = File_Number
    end,

    % Return updated Hera measurement state
    {ok, [0, 1], {Robot_Pid, {Names, Len, File_Number_New}, StartTime, N_New, Freq_New, Mean_Freq_New, T1}}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Logging Helpers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Initialize a new log file by writing the column headers.
%% @param Names - The names of the columns to be logged.
%% @param File - The name of the log file.
%% @return - `ok` after writing the header to the file.
log_init(Names, File) ->
    file:write_file(File, io_lib:fwrite("~s\n", [Names])).

%% @doc Append lists of logged values to the log file.
%% @param Sub_Vals - The list of logged values to be written.
%% @param File - The name of the log file.
%% @param Len - The number of values in each sublist.
%% @return - `ok` after writing the values to the file.
log([], _, _) -> 
    ok;
log([Sub_Vals | Rest], File, Len) ->
    Vals = lists:map(fun(V) -> lists:flatten(io_lib:format("~p", [V])) end, Sub_Vals),
    S = string:join(Vals, ","),
    Bytes = io_lib:format("~s~n", [S]),
    ok = file:write_file(File, Bytes, [append]),
    log(Rest, File, Len).
