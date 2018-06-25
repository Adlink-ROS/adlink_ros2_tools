#!/bin/bash
# Copyright 2018 ADLINK Technology, Inc.
# Developer: 
# * HaoChih, LIN (haochih.lin@adlinktech.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

echo "Switch your cpu to performance mode."
# Test if in sudo mode
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root" 
   exit 1
fi

# Final check
echo "[Warning] This script will change your sys file!"
read -p 'Are you sure? [True]: ' ans
if [ "$ans" != "True" ]; then
    exit 1
fi

number_of_cores=`cat /proc/cpuinfo | grep "cpu cores" | wc -l`
count=0
while [ $count -lt $number_of_cores ];do
    sudo echo 'performance' > /sys/devices/system/cpu/cpu$count/cpufreq/scaling_governor
    let count=count+1
done

echo "Finished!"



