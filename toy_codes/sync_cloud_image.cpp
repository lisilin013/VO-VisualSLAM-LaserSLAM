#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>


const std::string img_save_path   = "/home/nrsl/code/lifelong-slam/plycal/data/image_orig/";
const std::string cloud_save_path = "/home/nrsl/code/lifelong-slam/plycal/data/pointcloud/";
int id = 0;

void CloudImageSyncCallback(const sensor_msgs::PointCloud2ConstPtr &msg_pc,
                            const sensor_msgs::ImageConstPtr &msg_img) {
    ROS_DEBUG("Velodyne scan received at %f", msg_pc->header.stamp.toSec());
    ROS_DEBUG("Image received at %f", msg_img->header.stamp.toSec());

    pcl::PointCloud<pcl::PointXYZI> cloud;
    fromROSMsg(*msg_pc, cloud);

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow("camera image", cv_ptr->image);
    int key_val = cv::waitKey(10);

    // handle keyboard hit event
    if (key_val == 27) {//ESC
        ROS_WARN("Pressed ESC, shutdown ros");
        ros::shutdown();
    }
    else if (key_val == 32) {//space
        pcl::io::savePCDFileBinary(cloud_save_path + std::to_string(id) + ".pcd", cloud);
        cv::imwrite(img_save_path + std::to_string(id) + ".jpg", cv_ptr->image);
        ROS_INFO("save cloud and image id: %d", id++);
    }
    else {
        ROS_DEBUG("key: %d\n", key_val);
    }

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "cloud_image_sync_callback_node");
    ros::NodeHandle n;

    ROS_INFO("init cloud_image_sync_callback_node");
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, "/velodyne_points", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/camera_array/cam0/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, image_sub);
    sync.registerCallback(boost::bind(&CloudImageSyncCallback, _1, _2));

    ros::spin();
    return 0;
}

