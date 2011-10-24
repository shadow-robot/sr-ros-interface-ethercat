#!/bin/bash

START_PATH=`pwd`

#We copy the released protocol files to the correct directory
cd ${START_PATH}
chmod -R a+w ../include/sr_external_dependencies/external/
echo "copying (in directory: " $START_PATH " ), from: ../released/external/* to ../include/sr_external_dependencies/external/"
cp -r ../released/external/* ../include/sr_external_dependencies/external/
chmod -R a-w ../include/sr_external_dependencies/external/

echo "copying (in directory: " $START_PATH " ), from: ../released/released_firmware/* to ../compiled_firmware/released_firmware/"
chmod -R a+w ../include/sr_external_dependencies/external/
cp -r ../released/released_firmware/* ../compiled_firmware/released_firmware/
chmod -R a-w ../include/sr_external_dependencies/external/
