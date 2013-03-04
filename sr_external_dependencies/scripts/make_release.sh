#!/bin/bash

START_PATH=`pwd`

#We copy the released protocol files to the correct directory
cd ${START_PATH}
chmod -R a+w released/*

#copy the currently used protocol and compiled firmware to the release
echo "Releasing protocol"
cp -r include/sr_external_dependencies/external/* released/external/
echo "Releasing firmware"
cp -r compiled_firmware/released_firmware/* released/released_firmware/

chmod -R a-w released/*

#TODO: write the current versions of the host software / firmware in a file.
VERS_0220=`svnversion include/sr_external_dependencies/external/0220_palm_edc/`
VERS_0320_MUSCLE=`svnversion include/sr_external_dependencies/external/0320_palm_edc_muscle/`
VERS_BOOTLD=`svnversion include/sr_external_dependencies/external/simplemotor-bootloader/`
VERS_FW=`svnversion compiled_firmware/released_firmware/`
VERS_GIT_ETHERCAT=`git --git-dir \`rosstack find shadow_robot_ethercat\`/.git log --pretty=format:%h -n 1`
VERS_GIT_SHADOW=`git --git-dir \`rosstack find shadow_robot\`/.git log --pretty=format:%h -n 1`

echo "protocol_motor:"${VERS_0220} "; protocol_muscle:"${VERS_0320_MUSCLE} "; bootloader:"${VERS_BOOTLD}" ; firmware:"${VERS_FW}" ; git_shadow:"${VERS_GIT_SHADOW} "; git_ethercat:"${VERS_GIT_ETHERCAT} > tested_version.yaml

git tag --force "protocol_motor_"${VERS_0220}"__protocol_muscle_"${VERS_0320_MUSCLE}"__bootloader_"${VERS_BOOTLD}"__firmware_"${VERS_FW}"__git_shadow_"${VERS_GIT_SHADOW}"__git_ethercat_"${VERS_GIT_ETHERCAT}
