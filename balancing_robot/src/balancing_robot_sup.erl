% @private
% @doc balancing_robot top level supervisor.
% This module implements the top-level supervisor for the balancing_robot application.
% It starts and supervises the child processes required for the application.
%
% The supervisor uses the `one_for_all` strategy, which means if one child process terminates,
% all other child processes are terminated and then all child processes are restarted.
%
% The supervisor is started using the `start_link/0` function, which links the supervisor
% to the calling process.
%
% The `init/1` callback function initializes the supervisor with no child processes.
-module(balancing_robot_sup).

-behavior(supervisor).

% API
-export([start_link/0]).

% Callbacks
-export([init/1]).

%--- API -----------------------------------------------------------------------

start_link() -> supervisor:start_link({local, ?MODULE}, ?MODULE, []).

%--- Callbacks -----------------------------------------------------------------

% @private
% @doc Initializes the supervisor.
% This function is called when the supervisor is started.
% It sets up the supervisor with the `one_for_all` strategy,
% a maximum restart intensity of 0 restarts in 1 second, and no child processes.
%
% @spec init(Args :: list()) -> {ok, {SupFlags :: tuple(strategy, restart intensity, timing), ChildSpecs :: list()}}.
init([]) ->
    {ok, {{one_for_all, 0, 1}, []}}. % Returns an ok tuple to indicate successful initialization of the supervisor.

