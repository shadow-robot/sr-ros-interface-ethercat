#!/bin/bash

START_PATH=`pwd`

#create the folder in case it doesn't exist
mkdir -p ../include/sr_external_dependencies/external

#We copy the provisional development protocol files to the correct directory
cd ${START_PATH}
chmod -R a+w ../include/sr_external_dependencies/external/
mkdir -p ../include/sr_external_dependencies/external/common/
echo "copying (in directory: " $START_PATH " ), from: ../include/sr_external_dependencies/provisional/0220_palm_edc_ethercat_protocol.h to ../include/sr_external_dependencies/external/0220_palm_edc/"
cp -r ../include/sr_external_dependencies/provisional/0220_palm_edc_ethercat_protocol.h ../include/sr_external_dependencies/external/0220_palm_edc/
echo "copying (in directory: " $START_PATH " ), from: ../include/sr_external_dependencies/provisional/0320_palm_edc_ethercat_protocol.h to ../include/sr_external_dependencies/external/0320_palm_edc_muscle/"
cp -r ../include/sr_external_dependencies/provisional/0320_palm_edc_ethercat_protocol.h ../include/sr_external_dependencies/external/0320_palm_edc_muscle/
echo "copying (in directory: " $START_PATH " ), from: ../include/sr_external_dependencies/provisional/common_edc_ethercat_protocol.h to ../include/sr_external_dependencies/external/common/"
cp -r ../include/sr_external_dependencies/provisional/common_edc_ethercat_protocol.h ../include/sr_external_dependencies/external/common/
echo "copying (in directory: " $START_PATH " ), from: ../include/sr_external_dependencies/provisional/ethercat_can_bridge_protocol.h to ../include/sr_external_dependencies/external/common/"
cp -r ../include/sr_external_dependencies/provisional/ethercat_can_bridge_protocol.h ../include/sr_external_dependencies/external/common/
chmod -R a-w ../include/sr_external_dependencies/external/

