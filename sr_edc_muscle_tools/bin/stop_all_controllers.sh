#!/bin/bash

for name in $(rosrun controller_manager controller_manager list | grep running | cut -d ' ' -f 1); do
    rosrun controller_manager controller_manager stop $name
done
