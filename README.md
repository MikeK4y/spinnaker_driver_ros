# spinnaker_driver_ros

A ROS driver for ***synchronized* stereo pairs** of Spinnaker cameras

Supports two types of triggering:

## Software triggering

Uses software to trigger the cameras. **The cameras will never be in perfect sync**. The left camera is always going to be triggered first. The basic camera configuration (exposure time, gain and fps) can be changed with a dynamic reconfigure server.

## Hardware triggering

Uses hardware to trigger the cameras. The cameras should be in perfect sync as long as the triggering signal they receive is in perfect sync. The nodelet assumes that the triggering is handled by a PixHawk FCU and that it will also provide the triggering time in the *"/mavros/cam_imu_sync/cam_imu_stamp"* ROS topic. It also assumes that it can control and configure the trigger using the ROS services *"/mavros/cmd/trigger_interval"* and *"/mavros/cmd/trigger_control"*.

Tested with Spinnaker 2.4.0.143, Blackfly and Blackfly S USB3 cameras and ROS Noetic
