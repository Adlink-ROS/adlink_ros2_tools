# ADLINK ROS 2.0 Performance Testing Tools  
  
## Developers  
* HaoChih, LIN (haochih.lin@adlinktech.com)  
* Alan Chen (alan.chen@adlinktech.com)  
* Chester Tseng (chester.tseng@adlinktech.com)  

## License  
Apache 2.0 (Copyright 2017 ADLINK Technology, Inc.)  
  
# Dependencies  
* adlink_ros2_msgs  

## Compile (examples)   
$ cd ~/ros2_ws  
$ ament build  
OR (build only)  
$ ament build --only-packages adlink_ros2_tools  

For isolated build  
$ ament build --isolated --build-tests --symlink-install --only adlink_ros2_tools
  
## CPU Performance
You need to set your cpu as "performance" mode.  
$ sudo ./CURRENT_PKG/script/set_cpu_performance.bash  
Verify:
$ cat /proc/cpuinfo | grep MHz  
  
## Latency (Ping-Pong)  
Usage (help function):  
$ run adlink_ros2_tools RoundTrip_ping -h

Terminal 1:  
$ export RMW_IMPLEMENTATION=rmw_opensplice_cpp  
$ RCL_ASSERT_RMW_ID_MATCHES=rmw_opensplice_cpp ros2 run adlink_ros2_tools RoundTrip_pong   

Terminal 2:
$ ros2 run adlink_ros2_training RoundTrip_ping -h  
$ export RMW_IMPLEMENTATION=rmw_opensplice_cpp  
$ RCL_ASSERT_RMW_ID_MATCHES=rmw_opensplice_cpp ros2 run adlink_ros2_tools RoundTrip_ping -t 10 -p 1024  


