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
VERS_BOOTLD=`svnversion include/sr_external_dependencies/external/simplemotor-bootloader/`
VERS_FW=`svnversion compiled_firmware/released_firmware/`
VERS_BZR_ETHERCAT=`bzr revno \`rosstack find shadow_robot_ethercat\``
VERS_BZR_SHADOW=`bzr revno \`rosstack find shadow_robot\``

echo "protocol:"${VERS_0220} "; bootloader:"${VERS_BOOTLD}" ; firmware:"${VERS_FW}" ; bzr_shadow:"${VERS_BZR_SHADOW} "; bzr_ethercat:"${VERS_BZR_ETHERCAT} > tested_version.yaml

bzr tag --force "protocol"${VERS_0220}"__bootloader_"${VERS_BOOTLD}"__firmware_"${VERS_FW}"__bzr_shadow_"${VERS_BZR_SHADOW}"__bzr_ethercat_"${VERS_BZR_ETHERCAT}