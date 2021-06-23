#include "spinnaker_driver_ros/stereo_camera_manager_node.h"

// ROS
#include "camera_info_manager/camera_info_manager.h"

StereoCameraManagerNode::StereoCameraManagerNode(
    ros::NodeHandle &nh, image_transport::ImageTransport &image_t) {
  // Initialize Spinnaker handles
  system = Spinnaker::System::GetInstance();
  camera_list = system->GetCameras();

  if (camera_list.GetSize() < 2) {
    ROS_ERROR("At least two cameras need to be connected. Cameras found = %d",
              camera_list.GetSize());
    exit(-1);
  }

  // Load parameters from ROS
  loadParameters();

  // Connect to cameras
  l_camera->setSerial(l_cam_serial);
  r_camera->setSerial(r_cam_serial);

  if (!l_camera->connect(camera_list) || !r_camera->connect(camera_list)) {
    ROS_ERROR(
        "Could not connect to the cameras with the provided serial numbers");
    exit(-1);
  }

  // Initialize cameras
  l_camera->configure();
  r_camera->configure();

  if (!l_camera->startAcquisition() || !r_camera->startAcquisition()) {
    ROS_ERROR("Could not start frame acquisition");
    exit(-1);
  }

  // Setup Publishers
  image_transport::Publisher l_image_pub =
      image_t.advertise("left_camera/image_raw", 1);
  ros::Publisher l_cam_info_pub =
      nh.advertise<sensor_msgs::CameraInfo>("left_camera/camera_info", 1);

  image_transport::Publisher r_image_pub =
      image_t.advertise("right_camera/image_raw", 1);
  ros::Publisher r_cam_info_pub =
      nh.advertise<sensor_msgs::CameraInfo>("right_camera/camera_info", 1);

  // publishImage(*l_camera);

  l_cam_worker =
      std::thread(&StereoCameraManagerNode::publishImage, this,
                  std::ref(*l_camera), l_image_pub);  //, l_image_pub, l_cam_info_pub);
  // r_cam_worker = std::thread(&StereoCameraManagerNode::publishImage, this,
  //                            *r_camera, r_image_pub, r_cam_info_pub);
}

StereoCameraManagerNode::~StereoCameraManagerNode() {
  // Clear camera list before releasing system
  camera_list.Clear();

  // Release system
  system->ReleaseInstance();
}

void StereoCameraManagerNode::loadParameters() {
  ros::NodeHandle nh_lcl("~");

  nh_lcl.param("left_camera_serial", l_cam_serial, std::string("01234567"));
  nh_lcl.param("right_camera_serial", r_cam_serial, std::string("01234567"));
}

void StereoCameraManagerNode::publishImage(
    SpinnakerCamera &camera, image_transport::Publisher image_pub) {
  while (ros::ok()) {
    cv::Mat cap;
    uint64_t time_stamp;
    if (camera.grabFrame(cap, time_stamp)) {
      cv_bridge::CvImage ros_mat;
      sensor_msgs::Image ros_image;

      ros_mat.image = cap;
      ros_mat.header.stamp.sec = time_stamp;
      ros_mat.header.stamp.nsec = time_stamp * 1e-9;
      ros_mat.encoding = "mono8";
      // TODO: Add header.frame_id

      ros_mat.toImageMsg(ros_image);

      image_pub.publish(ros_image);
    } else {
      ROS_WARN("Did not received a frame! Trying again");
    }
  }
}