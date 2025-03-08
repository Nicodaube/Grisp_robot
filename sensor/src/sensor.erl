% @doc sensor public API.
-module(sensor).

-behavior(application).

% Callbacks
-export([start/2]).
-export([stop/1]).

%--- Callbacks -----------------------------------------------------------------

% @private
start(_Type, _Args) -> 
    sensor_sup:start_link(),
    grisp_network:start(),
    io:format("WiFi setup complete~n"),
    {ok, self()}.

% @private
stop(_State) -> ok.
