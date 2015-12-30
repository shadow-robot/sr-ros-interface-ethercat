#!/bin/bash

PREVIOUS_PATH=`pwd`
START_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd ${START_PATH}

#TODO: write the current versions of the host software / firmware in a file.
rm -rf /tmp/firmware
rm -rf /tmp/sr_common
rm -rf /tmp/sr_core

git clone https://github.com/shadow-robot/hand-firmware.git /tmp/firmware
if [ $? -ne 0 ]; then
    echo "Error: Could not clone hand-firmware repository. Exiting script!"
    cd ${PREVIOUS_PATH}
    exit 1
fi

git clone https://github.com/shadow-robot/sr_common.git /tmp/sr_common
git clone https://github.com/shadow-robot/sr_core.git /tmp/sr_core

VERS_GIT_FIRMWARE=`git --git-dir /tmp/firmware/.git tag | tail -1`

VERS_GIT_COMMON=`git --git-dir /tmp/sr_common/.git log --pretty=format:%h -n 1`
VERS_GIT_CORE=`git --git-dir /tmp/sr_core/.git log --pretty=format:%h -n 1`
VERS_GIT_ETHERCAT=`git --git-dir $(dirname $(rosstack find shadow_robot_ethercat))/.git log --pretty=format:%h -n 1`
VERS_GIT_SHADOW=`git --git-dir $(dirname $(rosstack find shadow_robot))/.git log --pretty=format:%h -n 1`

rm -rf /tmp/firmware
rm -rf /tmp/sr_common
rm -rf /tmp/sr_core

export tag_name="git_firmware_"${VERS_GIT_FIRMWARE}"__git_sr_common_"${VERS_GIT_COMMON}"__git_sr_core_"${VERS_GIT_CORE}"__git_shadow_"${VERS_GIT_SHADOW}"__git_ethercat_"${VERS_GIT_ETHERCAT}

echo "git_firmware:"${VERS_GIT_FIRMWARE}" ; git_sr_common:"${VERS_GIT_COMMON}" ; git_sr_core:"${VERS_GIT_CORE}" ; git_shadow:"${VERS_GIT_SHADOW}"; git_ethercat:"${VERS_GIT_ETHERCAT} > ../tested_version.yaml

git tag --force $tag_name

git push origin $tag_name --force

cd ${PREVIOUS_PATH}
