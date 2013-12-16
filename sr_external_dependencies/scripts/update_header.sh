#!/bin/bash

PREVIOUS_PATH=`pwd`
START_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd ${START_PATH}

mkdir -p ../include/sr_external_dependencies/external
cd "../include/sr_external_dependencies/external"

function checkout_or_update {
    DIR_NAME=${1}
    SVN_PATH=${2}
    if [ -d ${DIR_NAME} ] ; then
	chmod -R a+w ${DIR_NAME}
	echo "Updating ${DIR_NAME}"
	pushd ${DIR_NAME}
	svn up
	popd

    #removes write access
	chmod -R a-w ${DIR_NAME}
    else
	chmod -R a+w .
	echo "Checking out ${SVN_PATH} in ${DIR_NAME}"
	svn co ${SVN_PATH} ${DIR_NAME}

    #removes write access
	chmod -R a-w ${DIR_NAME}
	chmod -R a-w .
    fi
}

checkout_or_update common svn://thoth:9998/Pic32/trunk/nodes/common
checkout_or_update 0220_palm_edc svn://thoth:9998/Pic32/trunk/nodes/0220_palm_edc
checkout_or_update 0320_palm_edc_muscle svn://thoth:9998/Pic32/trunk/nodes/0320_palm_edc_muscle
checkout_or_update 0230_palm_edc_TS svn://thoth:9998/Pic32/trunk/nodes/0230_palm_edc_TS
checkout_or_update simplemotor-bootloader svn://thoth:9998/EDC/simplemotor-bootloader/

cd ${START_PATH}
cd "../compiled_firmware"
checkout_or_update released_firmware svn://thoth:9998/EDC/simplemotor/released_firmware/
cd ${PREVIOUS_PATH}
