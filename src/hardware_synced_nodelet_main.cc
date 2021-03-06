#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "hardware_synced_nodelet");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  nodelet.load(nodelet_name, "spinnaker_driver_ros/hardware_synced_nodelet", remap,
               nargv);

  ros::spin();

  return 0;
}
