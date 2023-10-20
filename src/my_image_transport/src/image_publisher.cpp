#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgcodecs/legacy/constants_c.h"
#include "std_msgs/Header.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/image", 1);

    std::string img_path = "src/my_image_transport/res/bird.jpg";
    if (argc > 1) {
        img_path = "src/my_image_transport/res/" + std::string(argv[1]) + ".jpg";
    }

    cv::Mat image = cv::imread(img_path, CV_LOAD_IMAGE_COLOR);
    if (image.empty()) {
        ROS_ERROR("image is empty!!!");
        return -1;
    }
    std_msgs::Header image_header;
    image_header.seq = 1;
    image_header.stamp = ros::Time::now();
    image_header.frame_id = "earth"; // 我这里选择earth类型的坐标系
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(image_header, "bgr8", image).toImageMsg();

    ros::Rate loop_rate(30);
    while (nh.ok()) {
        pub.publish(msg);
        image_header.seq+1;
        image_header.stamp = ros::Time::now();
        msg = cv_bridge::CvImage(image_header, "bgr8", image).toImageMsg();
        // ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
