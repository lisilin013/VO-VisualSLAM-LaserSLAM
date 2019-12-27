//
// Created by lisilin on 19-12-27.
//

// ros
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>

// opencv
#include <opencv2/opencv.hpp>

// std
#include <mutex>
#include <cstdlib>     /* atoi */


// global vars
nav_msgs::Odometry odom_now;
std::string img_save_path = "";
double dist_thresh = 1;
double yaw_thresh = 20;

// store image per 0.5m
void ImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    cv::imshow("camera image", cv_ptr->image);
    int key_val = cv::waitKey(1);


    // save image when odom_delt >= 1 m or delt_yaw>=20 degree
    // get delt distance
    static nav_msgs::Odometry odom_pre = odom_now;
    double x = odom_now.pose.pose.position.x - odom_pre.pose.pose.position.x;
    double y = odom_now.pose.pose.position.y - odom_pre.pose.pose.position.y;
    double delt_odom_distance = std::sqrt(x*x + y*y);

    // get delt yaw
    tf::Matrix3x3 tf_rotate_now(tf::Quaternion(odom_now.pose.pose.orientation.x,
            odom_now.pose.pose.orientation.y,
            odom_now.pose.pose.orientation.z,
            odom_now.pose.pose.orientation.w));
    tf::Matrix3x3 tf_rotate_pre(tf::Quaternion(odom_pre.pose.pose.orientation.x,
            odom_pre.pose.pose.orientation.y,
            odom_pre.pose.pose.orientation.z,
            odom_pre.pose.pose.orientation.w));
    tf::Matrix3x3 delt_tf_rotate = tf_rotate_pre.inverse()*tf_rotate_now;
    double delt_odom_yaw = 0, roll, pitch;
    delt_tf_rotate.getRPY(roll, pitch, delt_odom_yaw);
    delt_odom_yaw *= 180.0/M_PI;


    // save images
    static int id = 0;
    if (delt_odom_distance >= dist_thresh || delt_odom_yaw >= yaw_thresh) {
        bool save_ok = cv::imwrite(img_save_path + std::to_string(id) + ".jpg", cv_ptr->image);
        if (!save_ok) {
            ROS_ERROR("cannot save image!");
            ros::shutdown();
            return;
        }

        ROS_INFO("save image id: %d", id++);
        odom_pre = odom_now; // update odom_pre
    }


    // handle keyboard hit event
    if (key_val == 27) {//ESC
        ROS_WARN("Pressed ESC, shutdown ros");
        ros::shutdown();
    }
}

void OdomCallback(const nav_msgs::OdometryConstPtr &msg) {
    odom_now = *msg;
}


int main(int argc, char **argv) {
    // read default params
    if (argc != 4) {
        std::cout << "Usage: rosrun xx_pkg capture_image dist_thresh yaw_thresh img_save_path" << std::endl;
    }
    dist_thresh = atof(argv[1]);
    yaw_thresh = atof(argv[2]);
    img_save_path = argv[3];

    // add postfix
    if (img_save_path.back() != '/')
        img_save_path += '/';

    std::cout << "dist_thresh: " << dist_thresh << std::endl;
    std::cout << "yaw_thresh: " << yaw_thresh << std::endl;
    std::cout << "img_save_path: " << img_save_path << std::endl;

    // init ros
    ros::init(argc, argv, "capture_image_node");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, &OdomCallback);
    ros::Subscriber image_sub = nh.subscribe("/camera_array/cam0/image_raw", 1, &ImageCallback);

    ros::spin();
//    ros::AsyncSpinner spinner(2);
//    spinner.start();
//    ros::waitForShutdown();
    return 0;
}
