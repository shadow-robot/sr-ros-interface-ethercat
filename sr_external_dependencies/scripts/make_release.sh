#!/bin/bash

START_PATH=`pwd`

#We copy the released protocol files to the correct directory
cd ${START_PATH}
chmod -R a+w released/*

echo "Releasing protocol"
cp -r include/sr_external_dependencies/external/* released/external/
echo "Releasing firmware"
cp -r compiled_firmware/released_firmware/* released/released_firmware/

chmod -R a-w released/*

#TODO: write the current versions of the host software / firmware in a file.
