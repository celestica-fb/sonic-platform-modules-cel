#!/bin/bash

######################################################################################################
#Date       :
#Author     :
#Mail       :
#Function   :
#Version    :
#Update     :
######################################################################################################

DEVICE_DIR=/usr/share/sonic/device

CONFIGURE_MODE=NO
CONFIGURE_FILE=NO

function set_port_mode()
{
    local platfrom
	local configure_file_dir
	local configure_file
	
	platfrom=$( show version | grep "Platform:" | awk -F ": " '{print $2}' | grep "x86")
	configure_file_dir=${DEVICE_DIR}/${platfrom} 
		
	
    case ${CONFIGURE_MODE}x in
         "8x200G+64x25Gx")
             CONFIGURE_FILE="bcm56780_a0-generic-8x200_64x25.config"
             ;;
         "16x200G+48x50Gx")
             CONFIGURE_FILE="bcm56780_a0-generic-16x200_48x50.config"
             ;;
         "16x200G+48x100Gx")
             CONFIGURE_FILE="bcm56780_a0-generic-16x200_48x100.config"
             ;;
	     "16x200G+24x100Gx")
             CONFIGURE_FILE="bcm56780_a0-generic-16x200_24x100.config"
			 ;;
         *)
	         echo "Not support mode: ${CONFIGURE_MODE}"
	         exit 1
             ;;	 
     esac

    CONFIGURE_FILE=${DEVICE_DIR}/${platfrom}/${CONFIGURE_FILE}.yml	 
	if [ ! -f ${CONFIGURE_FILE} ] ;then
	    echo "Not found configure file: ${CONFIGURE_FILE} in ${configure_file_dir}"
		exit 1
	fi
	
	if [ ${platfrom} == "x86_64-alibaba_as14-40d-cl-r0" ] ;then
        if [ -d /usr/local/CPU_Diag/utility/Shamu_SDK ];then
		    cd /usr/local/CPU_Diag/utility/Shamu_SDK
			echo "configure port mode: ${CONFIGURE_MODE}"
			./auto_load_user.sh ${CONFIGURE_FILE}
		else
		    echo "Not found SDK package: /usr/local/CPU_Diag/utility/Shamu_SDK, please install the Diag"
		fi
	else
	    echo "Not support the platform: ${platform}"
		exit 2
	fi	
	

}



function usage()
{
    echo "Usage: `basename $0` options (-h|-s)"
    echo "          -h show this help"
    echo "          -s  <mode> set SDK port mode
	                Shamu mode: 8x200G+64x25G, 16x200G+48x50G, 16x200G+48x100G, 16x200G+24x100G"
    echo 
}


if [ $# -lt 1 ];then
    echo "too few arguments"
    usage
    exit
fi

while getopts "hs:" arg ; do
    case ${arg} in
        h)
            OPERATION="help"
            ;;
        s)
            OPERATION="set"
            CONFIGURE_MODE=${OPTARG}
            ;;
        ?)
            echo "unknown option"
            usage
            exit 1
            ;;
    esac
done


if [ ${OPERATION} == help ]; then
    usage
elif [ ${OPERATION} == set ]; then
    set_port_mode
else
    echo "unknown option"
fi
