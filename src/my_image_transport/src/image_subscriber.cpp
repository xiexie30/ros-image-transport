#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// 使用image/compressed压缩图像的命令为：
    // rosrun my_image_transport image_subscriber _image_transport:=compressed

void imageCallback(const sensor_msgs::ImageConstPtr& msg, bool imshow=false)
{
  try
  {
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    if (image.empty()) {
      ROS_ERROR("image is empty!!");
      return;
    }

    // 显示延时
    double time_now = ros::Time::now().toSec();
    // float delay = (time_now.nsec - msg->header.stamp.nsec) / 1.0 / 1e9;
    double delay = ros::Time::now().toSec() - msg->header.stamp.toSec();
    ROS_INFO("Time:%d.%d; Delay:%f; Seq:%d; Frame:%s:\n\t",msg->header.stamp.sec,msg->header.stamp.nsec, delay, 
        msg->header.seq,msg->header.frame_id.c_str());
    
    if (imshow) {
      cv::imshow("view", image);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  bool imshow = false;
  if (argc > 1 && (strcmp(argv[1], "imshow") == 0 || strcmp(argv[1], "show") == 0)) {
    imshow = true;
  }
  std::cout << imshow << std::endl;
  if (imshow) {
    cv::namedWindow("view");
    cv::startWindowThread();
  }
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/image", 1, boost::bind(&imageCallback, _1, imshow));
  ros::spin();
  if (imshow) cv::destroyWindow("view");

  return 0;
}
