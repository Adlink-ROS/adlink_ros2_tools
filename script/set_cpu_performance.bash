#!/bin/bash
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



