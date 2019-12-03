//
// Created by nrsl on 19-12-1.
//
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>

#include "utils.h"

using namespace std;

struct ProjectParams {
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Mat lidar2camera;
};

ostream &operator<<(ostream &os, const ProjectParams &param) {
    os << "-----------------------------------------------" << endl;
    os << "Project parameters are:\n" << endl;
    os << "\ncamera_matrix: \n" << param.camera_matrix << endl;
    os << "\ndist_coeffs  : \n" << param.dist_coeffs << endl;
    os << "\nlidar2camera : \n" << param.lidar2camera << endl;
    os << "-----------------------------------------------" << endl;
    return os;
}


/**
 * @brief: project cloud into image plane
 * @param cloud
 * @param img
 * @param trans: camera is target coordinators
 */
cv::Mat ProjectCloud2Image(const pcl::PointCloud<pcl::PointXYZ> &pc,
                           const cv::Mat &img, const ProjectParams &params) {
    const double resolution = 0.1;
    const double depth_min = 1.0;
    const double depth_gap = 10.0;
    uint32_t num = static_cast<uint32_t>(depth_gap/resolution);
    utils::color::rgbs colors = utils::color::get_rgbs(num);

    // undistortion
    cv::Mat im = img.clone();
    cv::undistort(img, im, params.camera_matrix, params.dist_coeffs);

    // projection
    for (auto &p : pc.points) {
        cv::Mat point(4, 1, CV_64F);
        point.at<double>(0, 0) = p.x;
        point.at<double>(1, 0) = p.y;
        point.at<double>(2, 0) = p.z;
        point.at<double>(3, 0) = 1;

        cv::Mat cam_pt = params.lidar2camera*point;
        cv::Mat pt = params.camera_matrix*cam_pt.rowRange(0, 3);// 3*1
        if (pt.at<double>(2) < 0.5) { //Z >=0.5
            continue;
        }

        int32_t u = static_cast<int32_t>(pt.at<double>(0, 0)/pt.at<double>(2, 0));
        int32_t v = static_cast<int32_t>(pt.at<double>(1, 0)/pt.at<double>(2, 0));

        if (u < 0 || u >= img.cols || v < 0 || v >= img.rows) {
            continue;
        }
//        cv::Vec3b color = im.at<cv::Vec3b>(cv::Point(u, v));
//        p.r = color[2];
//        p.g = color[1];
//        p.b = color[0];

        double f = std::sqrt(p.x*p.x + p.y*p.y) - depth_min;
        uint32_t idx = static_cast<uint32_t>(f/resolution);
        if (idx >= num) {
            idx = num - 1;
        }
        auto &c = colors[idx];
        cv::circle(im, cv::Point2d(u, v), 2, cv::Scalar(c[2], c[1], c[0]), -1);
    }

    return im;
}

cv::Mat ProjectCloud2Image2(const pcl::PointCloud<pcl::PointXYZ> &pc,
                            const cv::Mat &img, const ProjectParams &params) {
    const double resolution = 0.1;
    const double depth_min = 1.0;
    const double depth_gap = 10.0;
    uint32_t num = static_cast<uint32_t>(depth_gap/resolution);
    utils::color::rgbs colors = utils::color::get_rgbs(num);

    // undistortion
    cv::Mat im;
    img.copyTo(im);
    cv::undistort(img, im, params.camera_matrix, params.dist_coeffs);

    vector<cv::Point3f> obj_points;
    for (auto &p : pc.points) {
        cv::Mat point(4, 1, CV_64F);
        point.at<double>(0, 0) = p.x;
        point.at<double>(1, 0) = p.y;
        point.at<double>(2, 0) = p.z;
        point.at<double>(3, 0) = 1;

        cv::Mat cam_pt = params.lidar2camera*point;
        cv::Mat pt = params.camera_matrix*cam_pt.rowRange(0, 3);// 3*1
        if (pt.at<double>(2) < 0.5) { //Z >=0.5
            continue;
        }

        int32_t u = static_cast<int32_t>(pt.at<double>(0, 0)/pt.at<double>(2, 0));
        int32_t v = static_cast<int32_t>(pt.at<double>(1, 0)/pt.at<double>(2, 0));

        if (u < 0 || u >= img.cols || v < 0 || v >= img.rows) {
            continue;
        }

        obj_points.push_back(cv::Point3f(p.x, p.y, p.z));
    }
    cv::Mat r_vec, t_vec;
    cv::Rodrigues(params.lidar2camera.rowRange(0, 3).colRange(0, 3), r_vec);
    t_vec = params.lidar2camera.rowRange(0, 3).col(3);
    vector<cv::Point2f> image_points;
    cv::projectPoints(obj_points, r_vec, t_vec, params.camera_matrix, params.dist_coeffs, image_points);

    for (int i = 0; i < image_points.size(); ++i) {
        int32_t u = static_cast<int32_t >(image_points[i].x);
        int32_t v = static_cast<int32_t >(image_points[i].y);
        cv::circle(im, cv::Point2d(u, v), 2, cv::Scalar(0, 0, 255), -1);
    }
    return im;
}
//----------------------------------------
// global var
//----------------------------------------
ProjectParams params;
void CloudImageSyncCallback(const sensor_msgs::PointCloud2ConstPtr &msg_pc,
                            const sensor_msgs::ImageConstPtr &msg_img) {
    ROS_DEBUG("Velodyne scan received at %f", msg_pc->header.stamp.toSec());
    ROS_DEBUG("Image received at %f", msg_img->header.stamp.toSec());

    pcl::PointCloud<pcl::PointXYZ> cloud;
    fromROSMsg(*msg_pc, cloud);

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat project_img = ProjectCloud2Image(cloud, cv_ptr->image, params);
    if (project_img.empty()) {
        ROS_ERROR("project image is empty!");
        return;
    }
    cv::imshow("project img", project_img);
    int key_val = cv::waitKey(10);

    // quit
    if (key_val == 27) {
        ros::shutdown();
    }
}


void TestPlyCali() {
    const std::string img_save_path = "/home/nrsl/code/lifelong-slam/plycal/data_copy/image_orig/";
    const std::string cloud_save_path = "/home/nrsl/code/lifelong-slam/plycal/data_copy/pointcloud/";
    vector<string> img_path, cloud_path;
    for (int i = 0; i < 8; ++i) {
        cv::Mat img = cv::imread(img_save_path + to_string(i) + ".jpg", CV_LOAD_IMAGE_UNCHANGED);
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::io::loadPCDFile(cloud_save_path + to_string(i) + ".pcd", cloud);

        cv::Mat project_img = ProjectCloud2Image2(cloud, img, params);
        if (project_img.empty()) {
            ROS_ERROR("project image is empty!");
            return;
        }
        cv::imshow("project img", project_img);
        int key_val = cv::waitKey(0);

        // quit
        if (key_val == 27) {
            ros::shutdown();
        }
    }

};


//------------------------------------------------------
// test kitti data, project cloud to image
//------------------------------------------------------
void ReadKittiBinData2Cloud(const std::string &in_file, pcl::PointCloud<pcl::PointXYZI> &points) {
    // load point cloud
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if (!input.good()) {
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);
    for (int i = 0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        points.push_back(point);
    }
    input.close();
}

cv::Mat Project(const pcl::PointCloud<pcl::PointXYZI> &pc, const cv::Mat &img, const cv::Mat &trans_mat_34) {
    const double resolution = 0.1;
    const double depth_min = 1.0;
    const double depth_gap = 10.0;
    uint32_t num = static_cast<uint32_t>(depth_gap/resolution);
    utils::color::rgbs colors = utils::color::get_rgbs(num);

    cv::Mat im = img.clone();

    // projection
    for (const auto &p : pc.points) {
        cv::Mat point(4, 1, CV_64F);
        point.at<double>(0, 0) = p.x;
        point.at<double>(1, 0) = p.y;
        point.at<double>(2, 0) = p.z;
        point.at<double>(3, 0) = 1;

        cv::Mat pt = trans_mat_34*point;
        if (pt.at<double>(2) < 0.5) { //Z >=0
            continue;
        }

        // normalize homogeneous coordinates:
        int32_t u = static_cast<int32_t>(pt.at<double>(0, 0)/pt.at<double>(2, 0));
        int32_t v = static_cast<int32_t>(pt.at<double>(1, 0)/pt.at<double>(2, 0));

        if (u < 0 || u >= img.cols || v < 0 || v >= img.rows) {
            continue;
        }

        double f = std::sqrt(p.x*p.x + p.y*p.y) - depth_min;
        uint32_t idx = static_cast<uint32_t>(f/resolution);
        if (idx >= num) {
            idx = num - 1;
        }
        auto &c = colors[idx];
        cv::circle(im, cv::Point2d(u, v), 2, cv::Scalar(c[2], c[1], c[0]), -1);
    }

    return im;
}
void TestKitti() {
    const string base_path = "/home/nrsl/code/data/kitti/";
    const string image0_path = base_path + "image0/00000";
    const string image2_path = base_path + "image2/00000";
    const string velodyne_path = base_path + "velodyne/00000";

    // load extrinsic params
    string config_file = ros::package::getPath("lidar_camera_calibration") + "/src/config_kitti.yaml";
    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    cv::Mat P0_34, P1_34, P2_34, P3_34, Tr_34;//3*4
    fs["P0"] >> P0_34;
    fs["P1"] >> P1_34;
    fs["P2"] >> P2_34;
    fs["P3"] >> P3_34;
    fs["Tr"] >> Tr_34;

    cv::Mat P0, P1, P2, P3, Tr_velo_to_cam;
    P0 = P1 = P2 = P3 = Tr_velo_to_cam = cv::Mat::eye(4, 4, CV_64F); //4*4
    P0_34.copyTo(P0(cv::Rect(0, 0, 4, 3)));
    P2_34.copyTo(P2(cv::Rect(0, 0, 4, 3)));
    Tr_34.copyTo(Tr_velo_to_cam(cv::Rect(0, 0, 4, 3)));

    cv::Mat velo2cam0 = P0_34*Tr_velo_to_cam; //3*4
    cv::Mat velo2cam2 = P2_34*Tr_velo_to_cam;//3*4

    for (int i = 0; i < 10; ++i) {
        cv::Mat img0 = cv::imread(image0_path + to_string(i) + ".png", CV_LOAD_IMAGE_COLOR);
        cv::Mat img2 = cv::imread(image2_path + to_string(i) + ".png", CV_LOAD_IMAGE_COLOR);
        pcl::PointCloud<pcl::PointXYZI> cloud;
        ReadKittiBinData2Cloud(velodyne_path + to_string(i) + ".bin", cloud);

        // project
        cv::Mat im0 = Project(cloud, img0, velo2cam0);
        cv::Mat im2 = Project(cloud, img2, velo2cam2);

        cv::imshow("project img0", im0);
        cv::imshow("project img2", im2);
        int key_val = cv::waitKey(0);

        // quit
        if (key_val == 27) {
            ros::shutdown();
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "project_cloud2img_node");
    ros::NodeHandle n("~");

    // read settings
    string config_file = ros::package::getPath("lidar_camera_calibration") + "/src/config.yaml";
    cv::FileStorage fs(config_file, cv::FileStorage::READ);
    fs["camera_matrix"] >> params.camera_matrix;
    fs["distortion_coefficients"] >> params.dist_coeffs;
    fs["lidar2camera"] >> params.lidar2camera;

    cout << params;

//    TestPlyCali();
    TestKitti();

    // ros subscribers
   // ROS_INFO("init cloud_image_sync_callback_node");
   // message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(n, "/velodyne_points", 1);
   // message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/camera_array/cam0/image_raw", 1);

   // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;
   // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, image_sub);
   // sync.registerCallback(boost::bind(&CloudImageSyncCallback, _1, _2));

   // ros::spin();

    return 0;
}