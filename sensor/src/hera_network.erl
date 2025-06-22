-module(hera_network).
-on_load(init/0).
-export([hello/0]).

init()->
    ok  = erlang:load_nif(atom_to_list(?MODULE), 0).

hello() ->
    nif_not_loaded.
