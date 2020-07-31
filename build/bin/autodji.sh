#!/bin/bash

#export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

pid=`ps -ef | grep -v "grep" | grep "./djiosdk-Mobius" | wc -l`

echo "$pid"

if [ "$pid" == "0" ]; then

  echo $(date)

  cd /home/pi/nCube-sparrow

  node thyme

  cd /home/pi/2Onboard-SDK/Onboard-SDK/build/bin

  echo `./djiosdk-Mobius UserConfig.txt`

  echo "start"

fi


