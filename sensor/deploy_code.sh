#!/bin/bash
rebar3 compile
USER=$(whoami)
sudo rm -rf /media/"${USER}"/GRISP/sensor/lib/sensor-0.1.0/src/*

cp ./_build/default/lib/sensor/src/* /media/"${USER}"/GRISP/sensor/lib/sensor-0.1.0/src/
cp ./_build/default/lib/sensor/ebin/* /media/"${USER}"/GRISP/sensor/lib/sensor-0.1.0/ebin
umount /media/"${USER}"/GRISP