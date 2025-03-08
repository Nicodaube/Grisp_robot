% @doc sensor public API.
-module(sensor).

-behavior(application).

% Callbacks
-export([start/2]).
-export([stop/1]).

%--- Callbacks -----------------------------------------------------------------

% @private
start(_Type, _Args) -> sensor_sup:start_link().

% @private
stop(_State) -> ok.
