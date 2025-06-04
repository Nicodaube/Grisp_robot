#!/bin/bash
rebar3 compile
USER=$(whoami)
sudo rm -rf /media/"${USER}"/GRISP/balancing_robot/lib/balancing_robot-0.1.0/src/*

cp ./_build/default/lib/balancing_robot/src/* /media/"${USER}"/GRISP/balancing_robot/lib/balancing_robot-0.1.0/src/
cp ./_build/default/lib/balancing_robot/ebin/* /media/"${USER}"/GRISP/balancing_robot/lib/balancing_robot-0.1.0/ebin
umount /media/"${USER}"/GRISP