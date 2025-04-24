-module(hera_interface).

-behavior(hera_measure).

-export([init/1, measure/1]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Callbacks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

init(_Args) ->
	
	Init_time = erlang:system_time() / 1.0e6,% Get the current system time in milliseconds
		
	% Define the names and length of the data to be logged
	Names = "T,Freq,Gy,Acc,CtrlByte,AngleAccelerometer,AngleKalman,AngleComplem,Vref,Select,Adv_Ref,Turn_Ref,Speed",
	Len = 13,
	
	% Set the color of the LEDs
	[grisp_led:color(L, yellow) || L <- [1, 2]],

	{ok, {Robot_Pid, {Names, Len, 1}, Init_time, 0, 0, 200, Init_time}, #{name => kaltest, iter => infinity, sync => false}}.


measure({Robot_Pid, {Names, Len, File_Number}, StartTime, N, Freq, Mean_Freq, T0}) ->

	% Send a message to the robot process (main_loop) to get all data
	Robot_Pid ! {self(), get_all_data},
	
	% Wait for the robot process (main_loop) to send back the data
	receive 
		{_, data, Robot_Data} -> 
			ok 
	end,

	% Get the current system time in milliseconds
	T1 = erlang:system_time() / 1.0e6,
	
	% Calculate the time difference in seconds
	Dt = (T1 - T0) / 1000.0,

	% Initialize the new values for N, Freq, and Mean_Freq
	{N_New, Freq_New, Mean_Freq_New} = {0.0, 0.0, 0.0},

	%%%%%%%%%%%%%%%
	%%% Logging %%%
	%%%%%%%%%%%%%%%

	% Define the file name for logging
	File_Name = "measures" ++ integer_to_list(File_Number) ++ ".txt",

	% Retrieve start and end of logging sequence
	receive
		% If a start_log message is received, set Start_Log to true and Stop_Log to false
		{_, start_log} -> 
			Start_Log = true,
			Stop_Log = false;
		% If a stop_log message is received, set Start_Log to false and Stop_Log to true
		{_, stop_log} -> 
			Start_Log = false,
			Stop_Log = true
	after 0 ->
		% If no message is received, set both Start_Log and Stop_Log to false
		Start_Log = false,
		Stop_Log = false
	end,

	% Start of logging sequence: create new file to write data in.
	if
		Start_Log ->
			% Print a message indicating the start of logging
			io:format("[Hera] Start logging~n"),
			
			% Initialize the log file by writing the header (Names)
			log_init(Names, File_Name);
		true ->
			% If not starting logging, do nothing and return ok
			ok
	end,

	% End of logging sequence: retrieve data and write to file.
	if
		Stop_Log ->
			% Increment the file number for the new log file
			File_Number_New = File_Number + 1,
			
			% Print a message indicating the creation of a new log file
			io:format("[Hera] New log file with name: ~s~n", [File_Name]),
			
			% Initialize the new log file by writing the header (Names)
			log_init(Names, File_Name),
			
			% Set LED 1 to red to indicate logging
			grisp_led:color(1, {1, 0, 0}),
			
			% Send a message to the robot process (main loop) to log values
			Robot_Pid ! {self(), log_values},
			
			% Wait for the robot process (main loop) to send back the log values
			receive 
				{_, log, Log_Values} -> 
					ok 
			end,
			
			% Append the log values to the log file
			log(Log_Values, File_Name, Len),
			
			% Set LED 1 back to yellow to indicate logging is done
			grisp_led:color(1, {1, 1, 0}),
			
			% Print a message indicating the completion of writing to the file
			io:format("[Hera] Done writing to file!~n");
		true ->
			% If not stopping logging, keep the same file number
			File_Number_New = File_Number
	end,

	{ok, [0,1], {Robot_Pid, {Names, Len, File_Number_New}, StartTime, N_New, Freq_New, Mean_Freq_New, T1}}.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% API
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%
%Initialise the log file by writing the header
%
%
%Initialise the log file by writing the header
%
log_init(Names, File) ->
	% Write the header (Names) to the file
	file:write_file(File, io_lib:fwrite("~s\n", [Names])).

%
% Append the values to the log file
%
log([], _, _) -> 
	ok; % Base case: if the list is empty, do nothing and return ok
log(Val_List, File, Len) ->
	% Split the list into the first sublist (Sub_Vals) and the rest (Rest)
	[Sub_Vals | Rest] = Val_List,
	
	% Convert each value in Sub_Vals to a string
	Vals = lists:map(fun(V) -> lists:flatten(io_lib:format("~p", [V])) end, Sub_Vals),
	
	% Join the values into a single comma-separated string
	S = string:join(Vals, ","),
	
	% Format the string to be written to the file, adding a newline at the end
	Bytes = io_lib:format("~s~n", [S]),
	
	% Append the formatted string to the file
	ok = file:write_file(File, Bytes, [append]),
	
	% Recursively process the rest of the list
	log(Rest, File, Len).
