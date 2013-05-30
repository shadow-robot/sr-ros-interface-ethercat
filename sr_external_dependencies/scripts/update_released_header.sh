#!/bin/bash

START_PATH=`pwd`

#create the folder in case it doesn't exist
mkdir -p ../include/sr_external_dependencies/external

#We copy the released protocol files to the correct directory
cd ${START_PATH}
chmod -R a+w ../include/sr_external_dependencies/external/
echo "copying (in directory: " $START_PATH " ), from: ../released/external/* to ../include/sr_external_dependencies/external/"
cp -r ../released/external/* ../include/sr_external_dependencies/external/
chmod -R a-w ../include/sr_external_dependencies/external/

#create the folder in case it doesn't exist
mkdir -p ../compiled_firmware/released_firmware/

echo "copying (in directory: " $START_PATH " ), from: ../released/released_firmware/* to ../compiled_firmware/released_firmware/"
chmod -R a+w ../include/sr_external_dependencies/external/
cp -r ../released/released_firmware/* ../compiled_firmware/released_firmware/
chmod -R a-w ../include/sr_external_dependencies/external/
