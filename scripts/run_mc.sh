#!/bin/bash

# enable multicast and add route for lcm out the top
sudo ifconfig enp1s0 multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev enp1s0

# configure libraries
sudo LD_LIBRARY_PATH=. ldconfig
#sudo LD_LIBRARY_PATH=. ldd ./robot
sudo LD_LIBRARY_PATH=. ./mit_ctrl m r
