#include "spinnaker_driver_ros/stereo_camera_manager_node.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "stereo_camera_manager_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport image_t(nh);

  StereoCameraManagerNode camera_mngr(nh, image_t);

  ros::spin();

  camera_mngr.frame_grab_worker.join();

  return 0;
}
