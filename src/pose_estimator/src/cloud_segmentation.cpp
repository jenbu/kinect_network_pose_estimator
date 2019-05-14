


#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include "pose_estimator.h"

#include <darknet_ros_msgs_eb/BoundingBox.h>
#include <darknet_ros_msgs_eb/BoundingBoxes.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>


#include <ros/ros.h>
#include <ros/spinner.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl/filters/extract_indices.h>


using namespace std;
using namespace sensor_msgs;
using namespace message_filters;


class Receiver
{
private:
    std::mutex lock;
    bool running, save, running_viewers, enablePE, recordData;
    bool updateImage[6], updateCloud[6];
    size_t frame;
    int PCLayers;
    double box_xWidth, box_yWidth, pin_xWidth, pin_yWidth;

    ros::NodeHandle nh;
    ros::Publisher pub;

    PoseEstimator pose_est;
    pcl::VoxelGrid<pcl::PointXYZRGBA> vg_a;

    ros::AsyncSpinner spinner;

    cv::Mat color_image[6], depth_image[6];
    cv::Mat cameraMatrixColor[6], cameraMatrixDepth[6];
    cv::Mat lookupX[6], lookupY[6];

    darknet_ros_msgs_eb::BoundingBoxes detection_inf[6];


    //Transformation matrix
    Eigen::Matrix4f to_world_matrix[6];
    Eigen::Matrix4f j2_toworld = Eigen::Matrix4f::Identity(); Eigen::Matrix4f j3_toworld = Eigen::Matrix4f::Identity();
    Eigen::Matrix4d pin_transformation_matrix, box_transformation_matrix, pin_prealigned_tfmat, box_prealigned_tfmat;
    Eigen::Matrix4d icp_transformation_box, icp_transformation_pin;
    std::vector<double> pose_pin, pose_box, pose_pin_gt, pose_box_gt;

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud[6], box_cloud[6], pin_cloud[6];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_world[6], box_cloud_world[6], pin_cloud_world[6];
    pcl::PointCloud<pcl::PointXYZ>::Ptr concatenated_world, concatenated_pin, concatenated_box, segmented_box, segmented_pin, aligned_box, aligned_pin, concatenated_box_filtered, concatenated_pin_filtered;
    pcl::PointCloud<pcl::PointNormal>::Ptr aligned_boxEnd, filtered_cloud, segmented_box_normal, segmented_pin_normal;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_box_xyz, aligned_pin_xyz, cloud_xyz_box, cloud_xyz_pin;

    pcl::PCDWriter writer;
    pcl::PLYReader reader;
    std::ostringstream oss;
    ofstream poseData;


    typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerT;

    const int QHD_WIDTH = 960;
    const int QHD_HEIGHT = 540;
    int QHD_WIDTH_DZ = 20;//dødsone som ikke overlapper med depthmap på hver side
    int IR_HEIGHT_DZ = 34;//Sone på topp og bunn på depthmap som ikke overlapper med hd
    int boundry_rmin = 10;
    int boundry_rmax = 0;


public:
    Receiver() :
            running(false), nh("a"), spinner(0), running_viewers(true), enablePE(false), PCLayers(1), recordData(false)
    {
        for(int i = 0; i < 6; i++)
        {
            cameraMatrixColor[i] = cv::Mat::zeros(3, 3, CV_64F);
            cameraMatrixDepth[i] = cv::Mat::zeros(3, 3, CV_64F);
            pose_box.push_back(0.0);
            pose_pin.push_back(0.0);
        }



    }

    ~Receiver()
    {

    }

    void run()
    {
        cout << "Hei" << endl;
        start();
        close();
    }

private:

    void close()
    {
        spinner.stop();
        running = false;


    }
    void start()
    {
        running = true;
        save = false;

        for(int i= 0; i < 6; i++)
        {
            updateCloud[i] = false;
            updateImage[i] = false;
        }

        //Setting up the subscribers
        typedef message_filters::sync_policies::ApproximateTime<CompressedImage, CameraInfo, Image, CameraInfo, darknet_ros_msgs_eb::BoundingBoxes> ApproxSyncPolicy;

        message_filters::Subscriber<CompressedImage> image_color_subs[6] = {{nh, "/jetson1/qhd/image_color/compressed", 1}, {nh, "/jetson2/qhd/image_color/compressed", 1},
                                                                            {nh, "/jetson3/qhd/image_color/compressed", 1}, {nh, "/jetson4/qhd/image_color/compressed", 1},
                                                                            {nh, "/jetson5/qhd/image_color/compressed", 1}, {nh, "/jetson6/qhd/image_color/compressed", 1}};
        message_filters::Subscriber<CameraInfo> info_color_subs[6] = {{nh, "/jetson1/sd/camera_info", 1}, {nh, "/jetson2/sd/camera_info", 1}, {nh, "/jetson3/sd/camera_info", 1},
                                                                      {nh, "/jetson4/sd/camera_info", 1}, {nh, "/jetson5/sd/camera_info", 1}, {nh, "/jetson6/sd/camera_info", 1}};
        message_filters::Subscriber<Image> image_depth_subs[6] = {{nh, "/jetson1/sd/image_depth", 1}, {nh, "/jetson2/sd/image_depth", 1}, {nh, "/jetson3/sd/image_depth", 1},
                                                                  {nh, "/jetson4/sd/image_depth", 1}, {nh, "/jetson5/sd/image_depth", 1}, {nh, "/jetson6/sd/image_depth", 1}};
        message_filters::Subscriber<darknet_ros_msgs_eb::BoundingBoxes> bounding_boxes[6] = {{nh, "/jetson1/bounding_boxes", 1}, {nh, "/jetson2/bounding_boxes", 1},
                                                                                             {nh, "/jetson3/bounding_boxes", 1}, {nh, "/jetson4/bounding_boxes", 1},
                                                                                             {nh, "/jetson5/bounding_boxes", 1}, {nh, "/jetson6/bounding_boxes", 1}};
        message_filters::Synchronizer<ApproxSyncPolicy> *syncApprox[6];


        for(int i = 0; i < 6; i++)
        {
            syncApprox[i] = new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(5), image_color_subs[i], info_color_subs[i], image_depth_subs[i], info_color_subs[i], bounding_boxes[i]);
            syncApprox[i]->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4, _5, i));
        }

        std::chrono::milliseconds duration(1);


        initKinectToWorldTransMatrices(to_world_matrix);

        segmented_box_normal = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
        segmented_pin_normal = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
        cloud_xyz_box = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_xyz_pin = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        aligned_pin_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        aligned_box_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        aligned_boxEnd = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
        aligned_pin = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        concatenated_world = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        concatenated_box = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        concatenated_pin = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        segmented_box = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        segmented_pin = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        aligned_box = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        concatenated_pin_filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        concatenated_box_filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        temp = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());


        reader.read("/home/erlendb/Blender/box_end7_m.ply", *aligned_boxEnd);
        reader.read("/home/erlendb/Blender/pin_end_013.ply", *aligned_pin);
        pcl::copyPointCloud(*aligned_boxEnd, *aligned_box);

        //Transforming the models to have an upright position
        pin_transformation_matrix = Eigen::Matrix4d::Identity();
        box_transformation_matrix = Eigen::Matrix4d::Identity();
        icp_transformation_box = Eigen::Matrix4d::Identity();
        icp_transformation_pin = Eigen::Matrix4d::Identity();
        pin_prealigned_tfmat = Eigen::Matrix4d::Identity();
        box_prealigned_tfmat = Eigen::Matrix4d::Identity();

        Eigen::Matrix4d transformation_matrix;
        double theta = -M_PI/2;
        transformation_matrix << 1,              0,           0, 0,
                0,     cos(theta),  sin(theta), 0,
                0,     -sin(theta), cos(theta), 0,
                0,              0,           0, 1;
        pcl::transformPointCloud(*aligned_box, *aligned_box, transformation_matrix);
        theta = M_PI/2;
        transformation_matrix << 1,              0,           0, 0,
                                 0,     cos(theta),  sin(theta), 0,
                                 0,     -sin(theta), cos(theta), 0.439,
                                 0,              0,           0, 1;
        pcl::transformPointCloud(*aligned_pin, *aligned_pin, transformation_matrix);

        double xmin_box = 1000.0; double xmax_box = -1000.0; double ymin_box = 1000.0; double ymax_box = -1000.0;
        for(int i = 0; i < aligned_box->points.size(); i++)
        {
            if(aligned_box->points[i].x > xmax_box)
                xmax_box = aligned_box->points[i].x;
            if(aligned_box->points[i].x < xmin_box)
                xmin_box = aligned_box->points[i].x;

            if(aligned_box->points[i].y > ymax_box)
                ymax_box = aligned_box->points[i].y;
            if(aligned_box->points[i].y < ymin_box)
                ymin_box = aligned_box->points[i].y;

        }
        box_xWidth = xmax_box-xmin_box;
        box_yWidth = ymax_box-ymin_box;
        cout << "box xwidth: " << xmax_box - xmin_box << endl;
        cout << "box ywidth: " << ymax_box - ymin_box << endl;

        double xmin_pin = 1000.0; double xmax_pin = -1000.0; double ymin_pin = 1000.0; double ymax_pin = -1000.0;
        for(int i = 0; i < aligned_pin->points.size(); i++)
        {
            if(aligned_pin->points[i].x > xmax_pin)
                xmax_pin = aligned_pin->points[i].x;
            if(aligned_pin->points[i].x < xmin_pin)
                xmin_pin = aligned_pin->points[i].x;
            if(aligned_pin->points[i].y > ymax_pin)
                ymax_pin = aligned_pin->points[i].y;
            if(aligned_pin->points[i].y < ymin_pin)
                ymin_pin = aligned_pin->points[i].y;

        }
        cout << "pin xwidth: " << xmax_pin - xmin_pin << endl;
        cout << "pin ywidth: " << ymax_pin - ymin_pin<< endl;
        pin_xWidth = xmax_pin - xmin_pin;
        pin_yWidth = ymax_pin - ymin_pin;

        transformation_matrix << 1,              0,           0, -box_xWidth/2,
                                 0,              1,           0, box_yWidth/2,
                                 0,              0,           1, -0.23,
                                 0,              0,           0, 1;
        pcl::transformPointCloud(*aligned_box, *aligned_box, transformation_matrix);

        transformation_matrix << 1,              0,           0, -pin_xWidth/2,
                                 0,              1,           0, -pin_yWidth/2,
                                 0,              0,           1, -0.327,
                                 0,              0,           0, 1;
        pcl::transformPointCloud(*aligned_pin, *aligned_pin, transformation_matrix);

        //Ground truth position of box and pin
        pose_box_gt.push_back(5.0); pose_box_gt.push_back(5.0); pose_box_gt.push_back(0.08); pose_box_gt.push_back(0.0); pose_box_gt.push_back(0.0); pose_box_gt.push_back(0.0);
        pose_pin_gt.push_back(5.0); pose_pin_gt.push_back(5.0); pose_pin_gt.push_back(0.37); pose_pin_gt.push_back(0.0); pose_pin_gt.push_back(0.0); pose_pin_gt.push_back(0.0);

        spinner.start();

        while (!updateImage[1] || !updateCloud[1]/* || !updateImage[2] || !updateCloud[2]*/) {
            if (!ros::ok()) {
                return;
            }
            cout << "Starting up" << endl;
            std::this_thread::sleep_for(duration);
        }
        cout << "får meldinger" << endl;



        for(int i = 0; i < 6; i++)
        {
            updateCloud[i] = false;
            updateImage[i] = false;

            cloud[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
            box_cloud[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
            pin_cloud[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
            cloud_world[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
            box_cloud_world[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
            pin_cloud_world[i] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());


            cloud[i]->height = color_image[i].rows;
            cloud[i]->width = color_image[i].cols;
            cloud[i]->is_dense = false;
            cloud[i]->points.resize(cloud[i]->height*cloud[i]->width);
            box_cloud[i]->height = depth_image[i].rows;
            box_cloud[i]->width = depth_image[i].cols;
            box_cloud[i]->is_dense = false;
            box_cloud[i]->points.resize(box_cloud[i]->height*box_cloud[i]->width);
            pin_cloud[i]->height = depth_image[i].rows;
            pin_cloud[i]->width = depth_image[i].cols;
            pin_cloud[i]->is_dense = false;
            pin_cloud[i]->points.resize(pin_cloud[i]->height*pin_cloud[i]->width);


        }

        createLookup(this->color_image[1].cols, this->color_image[1].rows);

        cloudViewer();

    }

    void callback(const sensor_msgs::CompressedImage::ConstPtr &img_color, const sensor_msgs::CameraInfo::ConstPtr info_color, const sensor_msgs::Image::ConstPtr &img_depth,
                  const sensor_msgs::CameraInfo::ConstPtr info_depth, const darknet_ros_msgs_eb::BoundingBoxes::ConstPtr det_info, int jetson_index)
    {
        //cout << "Jetson" << (jetson_index+1) << " callback" << endl;
        cv::Mat color, depth;
        cv::Mat color_cropped, depth_cropped;

        readCameraInfo(info_color, cameraMatrixColor[jetson_index]);
        readCameraInfo(info_depth, cameraMatrixDepth[jetson_index]);

        readImage(img_color, color, img_depth, depth);


        if(color.type() == CV_16U)
        {
            cv::Mat tmp;
            color.convertTo(tmp, CV_8U, 0.02);
            cv::cvtColor(tmp, color, CV_GRAY2BGR);
        }


        lock.lock();
        this->color_image[jetson_index] = color;
        this->depth_image[jetson_index] = depth;
        this->detection_inf[jetson_index] = *det_info;
        updateImage[jetson_index] = true;
        updateCloud[jetson_index] = true;
        lock.unlock();

    }

    void cloudViewer()
    {
        darknet_ros_msgs_eb::BoundingBoxes det_inf[6];
        cv::Mat color[6], depth[6];
        CvPoint p1, p2;
        ostringstream os;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor_seg;
        sor_seg.setMeanK (50);
        sor_seg.setStddevMulThresh (0.6);


        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Visualizer"));
        //pcl::visualization::PCLVisualizer::Ptr visualizer2(new pcl::visualization::PCLVisualizer("Visualizer2"));
        std::string box_cloudName[6];
        std::string cloud_name[6];
        std::string pin_cloudName[6];
        Eigen::Affine3f affine_trans_box = Eigen::Affine3f::Identity(); Eigen::Affine3f affine_trans_pin = Eigen::Affine3f::Identity();

        std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
        std::chrono::time_point<std::chrono::high_resolution_clock> now_time;
        double elapsed;

        lock.lock();
        for(int i = 0; i < 6; i++)
        {
            color[i] = this->color_image[i];
            depth[i] = this->depth_image[i];
            det_inf[i] = this->detection_inf[i];
            updateCloud[i] = false;
            updateImage[i] = false;
        }
        lock.unlock();

        //cv::imshow("abc", color[1]);
        //cv::waitKey(3000);
        int v1 = 1; double ymax = 0.5; double ymin = 0.0; double width = 1.0/3.0; int k = 0;
        for(int i = 0; i < 6; i++)
        {
            createCloud(depth[i], color[i], box_cloud[i], pin_cloud[i], cloud[i], det_inf[i], i);
            //*concatenated_world += *cloud_world[i];
            *concatenated_pin += *pin_cloud_world[i];
            *concatenated_box += *box_cloud_world[i];

            /*
            if(i == 3)
            {
                ymax = 1; ymin = 0.5;
                k = 0;
            }
            os << "Cloud" << i;  cloud_name[i] = os.str(); os.str(std::string());
            os << "Box-Cloud" << i; box_cloudName[i] = os.str(); os.str(std::string());
            os << "Pin-Cloud" << i; pin_cloudName[i] = os.str(); os.str(std::string());
            cout << "cloud_name: " << cloud_name[i] << " box_cloudName: " << box_cloudName[i] << " pin_cloudName: " << pin_cloudName[i] << endl;
            visualizer2->createViewPort(k*width, ymin, (k+1)*width, ymax, v1);
            visualizer2->addPointCloud(cloud_world[i], ColorHandlerT(cloud_world[i], 255.0, 255.0, 255.0), cloud_name[i], v1);
            visualizer2->addPointCloud(pin_cloud_world[i], ColorHandlerT(pin_cloud_world[i], 255.0, 0.0, 0.0), pin_cloudName[i], v1);
            visualizer2->addPointCloud(box_cloud_world[i], ColorHandlerT(box_cloud_world[i], 0.0, 255.0, 0.0), box_cloudName[i], v1);
            k++;
            */
        }

        //visualizer->setBackgroundColor(255.0, 255.0, 255.0);
        //visualizer->addPointCloud(cloud_world[2], ColorHandlerT(cloud_world[2], 255.0, 255.0, 255.0), "World cloud");
        //visualizer->addPointCloud(pin_cloud_world[2], ColorHandlerT(cloud_world[2], 255.0, 0.0, 0.0), "Pin World cloud");
        //visualizer->addPointCloud(box_cloud_world[2], ColorHandlerT(box_cloud_world[2], 0.0, 255.0, 0.0), "Box World cloud");
        //visualizer->addPointCloud(concatenated_world, ColorHandlerT(concatenated_world, 255.0, 255.0, 255.0), "World cloud");
        visualizer->addPointCloud(concatenated_box, ColorHandlerT(concatenated_box, 0.0, 255.0, 0.0), "Box cloud");
        visualizer->addPointCloud(concatenated_pin, ColorHandlerT(concatenated_pin, 255.0, 0.0, 0.0), "Pin cloud");
        visualizer->addPointCloud(concatenated_box_filtered, ColorHandlerT(concatenated_box_filtered, 0.0, 0.0, 255.0), "box-filtered");
        visualizer->addPointCloud(concatenated_pin_filtered, ColorHandlerT(concatenated_pin_filtered, 0.0, 0.0, 255.0), "pin-filtered");
        //visualizer->addPointCloud(segmented_box, ColorHandlerT(segmented_box, 255.0, 0.0, 0.0), "segmented_box");
        //visualizer->addPointCloud(segmented_pin, ColorHandlerT(segmented_pin, 255.0, 0.0, 0.0), "segmented_pin");
        visualizer->addPointCloud(aligned_box, ColorHandlerT(aligned_box, 255.0, 0.0, 255.0), "box-model");
        visualizer->addPointCloud(aligned_pin, ColorHandlerT(aligned_pin, 255.0, 0.0, 255.0), "pin-model");
        visualizer->addCoordinateSystem(0.3, 0.0, 0.0, 0.0, "box_coordinate");
        visualizer->addCoordinateSystem(0.3, 0.0, 0.0, 0.0, "pin_coordinate");


        //Add pose text
        os << "Box-end\n" << "x: " << -1  << " Rot_x: " << -1 << endl << "y: " << -1 << " Rot_y: " << -1 << endl << "z: " << -1 << " Rot_z: " << -1 << endl;
        visualizer->addText(os.str(), 10, 10, "box_text"); os.str(std::string());
        os << "Pin-end\n" << "x: " << -1  << " Rot_x: " << -1 << endl << "y: " << -1 << " Rot_y: " << -1 << endl << "z: " << -1 << " Rot_z: " << -1 << endl;
        visualizer->addText(os.str(), 10, 60, "pin_text"); os.str(std::string());
        visualizer->addCoordinateSystem(2.0);
        visualizer->initCameraParameters();
        //visualizer->setBackgroundColor(0, 0, 0);
        visualizer->setShowFPS(true);
        visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
        visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);

        int iterator = 0;
        double avgX, avgY;

        while(running && ros::ok())
        {
            cout << "Waiting for messages" << endl;
            if(updateCloud[0] && updateCloud[1] && updateCloud[2] && updateCloud[3] &&  updateCloud[4] && updateCloud[5])
            {

                if(iterator == 1)
                {
                    start_time = std::chrono::high_resolution_clock::now();
                    cout << "updated start time" << endl;
                }


                lock.lock();
                for(int i = 0; i < 6; i++)
                {
                    color[i] = this->color_image[i];
                    depth[i] = this->depth_image[i];
                    det_inf[i] = this->detection_inf[i];
                    updateCloud[i] = false;
                    updateImage[i] = false;
                }
                lock.unlock();


                cout << "Building cloud, layer " << iterator << endl;


                for(int i = 0; i < 6; i++)
                {
                    createCloud(depth[i], color[i], box_cloud[i], pin_cloud[i], cloud[i], det_inf[i], i);
                    //*concatenated_world += *cloud_world[i];
                    *concatenated_pin += *pin_cloud_world[i];
                    *concatenated_box += *box_cloud_world[i];
                }

                for(int k = 0; k < 6; k++)
                {
                    for(int i = 0; i < det_inf[k].bounding_boxes.size(); i++)
                    {
                        p1.x = det_inf[k].bounding_boxes[i].xmin; p1.y = det_inf[k].bounding_boxes[i].ymin;
                        p2.x = det_inf[k].bounding_boxes[i].xmax; p2.y = det_inf[k].bounding_boxes[i].ymax;
                        cv::rectangle(color[k], p1, p2, (255.0, 0.0, 0.0));
                        p1.x -= 30; p1.y -= 5;
                        cv::putText(color[k], det_inf[k].bounding_boxes[i].Class, p1, cv::FONT_HERSHEY_DUPLEX, 1.0, cvScalar(255.0, 255.0, 255.0));
                    }
                }


                if(iterator==PCLayers)
                {

                    cout << "Begin processing" << endl;

                    if(enablePE)
                    {
                        pcl::copyPointCloud(*concatenated_box, *concatenated_box_filtered);
                        pcl::copyPointCloud(*concatenated_pin, *concatenated_pin_filtered);
                        poseEstimation(concatenated_box_filtered, concatenated_pin_filtered);
                    }

                    affine_trans_box.matrix() = box_transformation_matrix.cast<float>();
                    affine_trans_pin.matrix() = pin_transformation_matrix.cast<float>();
                    visualizer->updateCoordinateSystemPose("box_coordinate", affine_trans_box);
                    visualizer->updateCoordinateSystemPose("pin_coordinate", affine_trans_pin);

                    //visualizer->setBackgroundColor(255.0, 255.0, 255.0);
                    //visualizer->updatePointCloud(concatenated_world, ColorHandlerT(concatenated_world, 255.0, 255.0, 255.0), "World cloud");
                    visualizer->updatePointCloud(concatenated_box, ColorHandlerT(concatenated_box, 0.0, 255.0, 0.0), "Box cloud");
                    visualizer->updatePointCloud(concatenated_pin, ColorHandlerT(concatenated_pin, 255.0, 0.0, 0.0), "Pin cloud");
                    visualizer->updatePointCloud(concatenated_box_filtered, ColorHandlerT(concatenated_box_filtered, 0.0, 0.0, 255.0), "box-filtered");
                    visualizer->updatePointCloud(concatenated_pin_filtered, ColorHandlerT(concatenated_pin_filtered, 0.0, 0.0, 255.0), "pin-filtered");
                    visualizer->updatePointCloud(aligned_pin, ColorHandlerT(aligned_pin, 255.0, 0.0, 255.0), "pin-model");
                    visualizer->updatePointCloud(aligned_box, ColorHandlerT(aligned_box, 255.0, 0.0, 255.0), "box-model");
                    //visualizer->updatePointCloud(segmented_box, ColorHandlerT(segmented_box, 255.0, 0.0, 0.0), "segmented_box");
                    //visualizer->updatePointCloud(segmented_pin, ColorHandlerT(segmented_pin, 255.0, 0.0, 0.0), "segmented_pin");

                    cout << "pose_box size: " << pose_box.size() << " pose_pin size: " << pose_pin.size() << endl;
                    if(pose_box.size() > 0)
                    {
                        os  << "Box-end\n" << "x: " <<  pose_box[0]  << " Rot_x: " << pose_box[3]  << endl << "y: " << pose_box[1]  << " Rot_y: " << pose_box[4]
                        << endl << "z: " << pose_box[2]  << " Rot_z: " << pose_box[5]  << endl;
                        visualizer->updateText(os.str(), 10, 10, "box_text"); os.str(std::string());
                    }
                    else
                    {
                        os << "Box-end\n" << "x: " << -1  << " Rot_x: " << -1 << endl << "y: " << -1 << " Rot_y: " << -1 << endl << "z: " << -1 << " Rot_z: " << -1 << endl;
                        visualizer->updateText(os.str(), 10, 10, "box_text"); os.str(std::string());
                    }
                    if(pose_pin.size() > 0)
                    {
                        os << "Pin-end\n" << "x: " <<  pose_pin[0]  << " Rot_x: " << pose_pin[3]  << endl << "y: " << pose_pin[1]  << " Rot_y: " << pose_pin[4]
                           << endl << "z: " << pose_pin[2]  << " Rot_z: " << pose_pin[5]  << endl;
                        visualizer->updateText(os.str(), 10, 60, "pin_text"); os.str(std::string());
                    }
                    else{
                        os << "Pin-end\n" << "x: " << -1  << " Rot_x: " << -1 << endl << "y: " << -1 << " Rot_y: " << -1 << endl << "z: " << -1 << " Rot_z: " << -1 << endl;
                        visualizer->updateText(os.str(), 10, 60, "pin_text"); os.str(std::string());
                    }


                    if(save)
                    {
                        save = false;
                        saveCloudAndImages(concatenated_box, concatenated_pin, color);
                    }
                    concatenated_world->clear();
                    concatenated_world->resize((size_t)depth[1].cols*depth[1].rows);
                    concatenated_pin->clear();
                    concatenated_pin->resize((size_t)depth[1].cols*depth[1].rows);
                    concatenated_box->clear();
                    concatenated_box->resize((size_t)depth[1].cols*depth[1].rows);
                    segmented_box->clear();
                    segmented_box->resize((size_t)depth[1].cols*depth[1].rows);
                    concatenated_box_filtered->clear();
                    concatenated_box_filtered->resize((size_t)depth[1].cols*depth[1].rows);
                    concatenated_pin_filtered->clear();
                    concatenated_pin_filtered->resize(3000);

                    now_time = std::chrono::high_resolution_clock::now();
                    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now_time - start_time).count() / 1000.0;
                    cout << "Time lapsed: " << elapsed << endl;
                    iterator = 0;
                }

                cv::imshow("ROI detector", color[2]);
                //cv::imshow("depth", depth);
                iterator += 1;

            }


            cv::waitKey(10);
            visualizer->spinOnce(2000);


        }
        visualizer->close();
        cv::destroyAllWindows();
        cv::waitKey(100);
    }

    void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_box, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_pin,
            pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const darknet_ros_msgs_eb::BoundingBoxes &det_inf, int index) const
    {

        const float badPoint = std::numeric_limits<float>::quiet_NaN();

        int r_min, r_max, c_min, c_max;

        double score[2] = {0.0, 0.0};
        int class_index[2] = {-1, -1};

        for(int o = 0; o < det_inf.bounding_boxes.size(); o++)
        {
            if(det_inf.bounding_boxes[o].Class == "pin-end")
            {
                if(det_inf.bounding_boxes[o].probability > score[1])
                {
                    score[1] = det_inf.bounding_boxes[o].probability;
                    class_index[1] = o;
                }
            }

            if(det_inf.bounding_boxes[o].Class == "box-end")
            {
                if(det_inf.bounding_boxes[o].probability > score[0])
                {
                    score[0] = det_inf.bounding_boxes[o].probability;
                    class_index[0] = o;
                }
            }
        }


        cloud_box->clear();
        cloud_box->resize((size_t)depth.cols*depth.rows);
        cloud_pin->clear();
        cloud_pin->resize((size_t)depth.cols*depth.rows);
        cloud->clear();
        cloud->resize((size_t)depth.cols*depth.rows);
        cloud_world[index]->clear();
        cloud_world[index]->resize((size_t)depth.cols*depth.rows);
        int width;
        pcl::PointXYZ *itP;
        cv::Vec3b color_pixel;
        uint16_t depth_pixel;
        int x1, x2, y1, y2;


        if(det_inf.bounding_boxes.size() > 0)
        {
            for(int t = 0; t < det_inf.bounding_boxes.size(); t++)
            {
                //Reject proposals out of bounds with the IR depth image
                if(det_inf.bounding_boxes[t].xmin < QHD_WIDTH_DZ || det_inf.bounding_boxes[t].xmax > QHD_WIDTH-QHD_WIDTH_DZ)
                    continue;

                if(t != class_index[0] && t != class_index[1])
                    continue;

                //Convert to from qhd coordinates to sd
                x1 = det_inf.bounding_boxes[t].xmin; x2 = det_inf.bounding_boxes[t].xmax;
                y1 = det_inf.bounding_boxes[t].ymin; y2 = det_inf.bounding_boxes[t].ymax;
                qhdTosd(x1, x2, y1, y2);


                r_min = y1-10;//boundry_rmin;
                r_max = y2+20;//boundry_rmax;//+30;
                c_min = x1-20;
                c_max = x2+20;
                width = c_max - c_min;
                if(r_min < 0)
                    r_min = 0;
                if(r_max > depth.rows)
                    r_max = depth.rows;
                if(c_min < 0)
                    c_min = 0;
                if(c_max > depth.cols)
                    c_max = depth.cols;


                for(int r = r_min; r < r_max; ++r)
                {
                    if(det_inf.bounding_boxes[t].Class == "box-end")
                        itP = &cloud_box->points[r * width];
                    else if(det_inf.bounding_boxes[t].Class == "pin-end")
                        itP = &cloud_pin->points[r * width];



                    const float y = lookupY[index].at<float>(0, r);
                    const float *itX = lookupX[index].ptr<float>();

                    for(int i = 0; i < c_min; i++)
                        itX++;

                    for(size_t c = c_min; c < c_max; ++c, ++itP, ++itX)
                    {

                        color_pixel = color.at<cv::Vec3b>(r,c);
                        depth_pixel = depth.at<uint16_t>(r,c);

                        register const float depthValue = depth_pixel / 1000.0f;
                        // Check for invalid measurements
                        if(depth_pixel == 0)
                        {
                            // not valid
                            itP->x = itP->y = itP->z = badPoint;
                            continue;
                        }

                        itP->z = depthValue;
                        itP->x = *itX * depthValue;
                        itP->y = y * depthValue;
                    }
                }
            }
            pcl::transformPointCloud(*cloud_pin, *pin_cloud_world[index], to_world_matrix[index]);
            pcl::transformPointCloud(*cloud_box, *box_cloud_world[index], to_world_matrix[index]);
        }


        if(det_inf.bounding_boxes.size() == 0)
        {
            for(int r = 0; r < depth.rows; ++r)
            {

                itP = &cloud->points[r * depth.cols];


                const float y = lookupY[index].at<float>(0, r);
                const float *itX = lookupX[index].ptr<float>();


                for(size_t c = 0; c < depth.cols; ++c, ++itP, ++itX)
                {
                    depth_pixel = depth.at<uint16_t>(r,c);

                    register const float depthValue = depth_pixel / 1000.0f;
                    // Check for invalid measurements
                    if(depth_pixel == 0)
                    {
                        // not valid
                        itP->x = itP->y = itP->z = badPoint;
                        continue;
                    }

                    itP->z = depthValue;
                    itP->x = *itX * depthValue;
                    itP->y = y * depthValue;

                }
            }
            pcl::transformPointCloud(*cloud, *cloud_world[index], to_world_matrix[index]);
        }



    }

    void qhdTosd(int &x1, int &x2, int &y1, int &y2) const
    {
        x1 -= QHD_WIDTH_DZ; x2 -= QHD_WIDTH_DZ;
        y1 += IR_HEIGHT_DZ; y2 += IR_HEIGHT_DZ;
        //cout << "QHD_WIDTH_DZ in qhdTosd()" << QHD_WIDTH_DZ << endl;
        double qhd_height = this->color_image[1].rows;
        double qhd_width = this->color_image[1].cols-2*QHD_WIDTH_DZ;
        double sd_height = this->depth_image[1].rows-2*IR_HEIGHT_DZ; //pga IR kamera er høyere enn hd kameraet
        double sd_width = this->depth_image[1].cols;

        double width_factor = sd_width/qhd_width;
        double height_factor = sd_height/qhd_height;

        x1 = (int)x1*width_factor;
        x2 = (int)x2*width_factor;
        y1 = (int)y1*height_factor;
        y2 = (int)y2*height_factor;

        //cout << "width factor: " << width_factor << " x1: " << x1 << " x2: " << x2 << " y1: " << y1 << " y2: " << y2 << endl;
    }

    void readImage(const sensor_msgs::CompressedImage::ConstPtr img, cv::Mat &outImg, const sensor_msgs::Image::ConstPtr depth, cv::Mat &outDepth)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(depth, depth->encoding);
        cv_ptr->image.copyTo(outDepth);
        outImg = cv::imdecode(cv::Mat(img->data), 1);
    }

    void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const {
        double *itC = cameraMatrix.ptr<double>(0, 0);
        for (size_t i = 0; i < 9; ++i, ++itC) {
            *itC = cameraInfo->K[i];
        }
    }

    void createLookup(size_t width, size_t height)
    {

        for(int i = 0; i < 6; i++)
        {
            //fx og fy er focal lengde, lengde/piksel. er begge like har man helt kvadratisk bilde.
            const float fx = 1.0f / cameraMatrixColor[i].at<double>(0,0);
            const float fy = 1.0f / cameraMatrixColor[i].at<double>(1,1);
            //Principal point cx og cy sier hvor på bilde-planet pinhole treffer.
            const float cx = cameraMatrixColor[i].at<double>(0,2);
            const float cy = cameraMatrixColor[i].at<double>(1,2);
            float *it;

            lookupY[i] = cv::Mat(1, height, CV_32F);
            it = lookupY[i].ptr<float>();
            for(size_t r = 0; r < height; ++r, ++it)
            {
                *it = (r - cy) * fy;
            }

            lookupX[i] = cv::Mat(1, width, CV_32F);
            it = lookupX[i].ptr<float>();
            for(size_t c = 0; c < width; ++c, ++it)
            {
                *it = (c - cx) * fx;
            }
        }

    }

    void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
    {
        if(event.keyUp())
        {
            switch(event.getKeyCode())
            {
                case 27:
                case 'q':
                    running = false;
                    break;
                case ' ':
                case 's':
                    save = true;
                    break;
                case 'k':
                    //QHD_WIDTH_DZ -= 1;
                    //cout << "QHD_WIDTH_DZ: " << QHD_WIDTH_DZ << endl;
                    boundry_rmin += 1;
                    break;
                case 'l':
                    //QHD_WIDTH_DZ += 1;
                    //cout << "QHD_WIDTH_DZ: " << QHD_WIDTH_DZ << endl;
                    boundry_rmax -=1;
                    break;
                case 'n':
                    IR_HEIGHT_DZ -= 1;
                    cout << "IR_HEIGHT_DZ: " << IR_HEIGHT_DZ << endl;
                    break;
                case 'm':
                    IR_HEIGHT_DZ += 1;
                    cout << "IR_HEIGHT_DZ: " << IR_HEIGHT_DZ << endl;
                    break;
                case '0':
                    enablePE = !enablePE;
                    if(enablePE)
                        cout << "Enabling Pose estimation" << endl;
                    else
                        cout << "Disabling Pose estimation" << endl;
                    break;
                case '1':
                    PCLayers -= 1;
                    if(PCLayers < 1)
                        PCLayers = 1;
                    cout << "Removed a pointcloud layer, current layers: " << PCLayers << endl;
                    break;
                case '2':
                    PCLayers += 1;
                    cout << "Added pointcloud layer, current layers: " << PCLayers << endl;
                    break;
                case 'd':

                    if(poseData.is_open())
                    {
                        cout << "Disable data record" << endl;
                        poseData.close();
                    }
                    else{
                        cout << "Enable data record" << endl;
                        poseData.open("Vertical_quarternions.txt");
                        poseData << "Box_x " << "Box_y " << "Box_z " << "Box_quat x " << "Box_quat y " << "Box_quat z " << "Box_quat w " << "Box_Rotx " << "Box_Roty " << "Box_Rotz ";
                        poseData << "Pin_x " << "Pin_y " << "Pin_z " << "Pin_quat x " << "Pin_quat y " << "Pin_quat z " << "Pin_quat w " << "Pin_Rotx " << "Pin_Roty " << "Pin_Rotz\n";
                    }

                    recordData = !recordData;
                    break;

            }
        }
    }

    void saveCloudAndImages(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_box, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_pin, const cv::Mat color[6]/*, const cv::Mat &depth*/)
    {
        time_t t = time(NULL);
        tm* tPtr = localtime(&t);
        std::string tid;

        oss.str("");
        oss << tPtr->tm_mon + 1 << "." << tPtr->tm_mday << ":"<<tPtr->tm_hour << "." << tPtr->tm_min << "." << tPtr->tm_sec << std::setfill('0') << std::setw(4) << frame;
        const std::string baseName = oss.str();

        cout << baseName << endl;

        const std::string path = "/home/erlendb/Pictures/";
        const std::string cloudNameBox = path /*+ "Cloud/"*/ + baseName + "box" + "_cloud.pcd";
        const std::string cloudNamePin = path /*+ "Cloud/"*/ + baseName + "pin" + "_cloud.pcd";
        const std::string depthName = path + baseName + "_depth.jpg";

        /*
        for(int i = 0; i < 6; i++)
        {
            oss.str("");
            oss << tPtr->tm_mon + 1 << "." << tPtr->tm_mday << ":"<<tPtr->tm_hour << "." << tPtr->tm_min << "." << tPtr->tm_sec << "j" << i << "_" << frame;
            const std::string baseName = oss.str();
            cout << baseName << endl;

            const std::string path = "/home/erlendb/Pictures/Masteroppgave_bilder/Rapport/";
            const std::string colorName = path + baseName + "_color.jpg";
            cv::imwrite(colorName, color[i]);
        }*/

        cout << "saved shit" << endl;
        //cout << "Saving cloud: " << cloudName << endl;
        writer.writeBinary(cloudNameBox, *cloud_box);
        writer.writeBinary(cloudNamePin, *cloud_pin);
        //OUT_INFO("saving color: " << colorName);
        //cv::imwrite(colorName, color);
        //OUT_INFO("saving depth: " << depthName);
        //cv::imwrite(depthName, depth);
        //OUT_INFO("saving depth: " << depthColoredName);
        //cv::imwrite(depthColoredName, depthColored, params);*/
        //OUT_INFO("saving complete!");
        ++frame;
    }

    void initKinectToWorldTransMatrices(Eigen::Matrix4f m[6])
    {
        float j1_x = 7.798;
        float j1_y = 0.496;
        float j1_z = 4.175;

        Eigen::Quaterniond q_j1(-0.349, 0.872, 0.317, -0.133);
        Eigen::Matrix3d j1_rot;
        j1_rot = q_j1.normalized().toRotationMatrix();
        m[0] << j1_rot(0, 0), j1_rot(0, 1), j1_rot(0, 2), j1_x,
                j1_rot(1, 0), j1_rot(1, 1), j1_rot(1, 2), j1_y,
                j1_rot(2, 0), j1_rot(2, 1), j1_rot(2, 2), j1_z,
                0.0,            0.0,          0.0,         1.0;
        //m[0] = Eigen::Matrix4f::Identity();

        float j2_x = 1.729;
        float j2_y = 0.501;
        float j2_z = 4.135;

        Eigen::Quaterniond q_j2(-0.311, 0.869, -0.369, 0.111);
        Eigen::Matrix3d j2_rot;
        j2_rot = q_j2.normalized().toRotationMatrix();
        m[1] <<   j2_rot(0, 0), j2_rot(0, 1), j2_rot(0, 2), j2_x,
                j2_rot(1, 0), j2_rot(1, 1), j2_rot(1, 2), j2_y,
                j2_rot(2, 0), j2_rot(2, 1), j2_rot(2, 2), j2_z,
                0.0,            0.0,          0.0,         1.0;
        //m[1] = Eigen::Matrix4f::Identity();

        float j3_x = 9.522;
        float j3_y = 5.275;
        float j3_z = 4.353;
        Eigen::Quaterniond q_j3(-0.225, 0.680, 0.662, -0.218);
        Eigen::Matrix3d j3_rot;
        j3_rot = q_j3.normalized().toRotationMatrix();
        cout << j3_rot << endl;
        m[2] <<   j3_rot(0, 0), j3_rot(0, 1), j3_rot(0, 2), j3_x,
                j3_rot(1, 0), j3_rot(1, 1), j3_rot(1, 2), j3_y,
                j3_rot(2, 0), j3_rot(2, 1), j3_rot(2, 2), j3_z,
                0.0,            0.0,          0.0,         1.0;

        //m[2] = Eigen::Matrix4f::Identity();

        float j4_x = 0.553;
        float j4_y = 4.995;
        float j4_z = 4.353;
        Eigen::Quaterniond q_j4(-0.208, 0.681, -0.67, 0.211);
        Eigen::Matrix3d j4_rot;
        j4_rot = q_j4.normalized().toRotationMatrix();
        m[3] << j4_rot(0, 0), j4_rot(0, 1), j4_rot(0, 2), j4_x,
                j4_rot(1, 0), j4_rot(1, 1), j4_rot(1, 2), j4_y,
                j4_rot(2, 0), j4_rot(2, 1), j4_rot(2, 2), j4_z,
                0.0,            0.0,          0.0,         1.0;

        //m[3] = Eigen::Matrix4f::Identity();

        float j5_x = 8.879;
        float j5_y = 9.192;
        float j5_z = 4.194;
        Eigen::Quaterniond q_j5(-0.166, 0.417, 0.840, -0.306);
        Eigen::Matrix3d j5_rot;
        j5_rot = q_j5.normalized().toRotationMatrix();
        m[4] << j5_rot(0, 0), j5_rot(0, 1), j5_rot(0, 2), j5_x,
                j5_rot(1, 0), j5_rot(1, 1), j5_rot(1, 2), j5_y,
                j5_rot(2, 0), j5_rot(2, 1), j5_rot(2, 2), j5_z,
                0.0,            0.0,          0.0,         1.0;

        //m[4] = Eigen::Matrix4f::Identity();

        float j6_x = 0.559;
        float j6_y = 9.136;
        float j6_z = 4.145;
        Eigen::Quaterniond q_j6(0.174, -0.457, 0.812, -0.318);
        Eigen::Matrix3d j6_rot;
        j6_rot = q_j6.normalized().toRotationMatrix();
        m[5] << j6_rot(0, 0), j6_rot(0, 1), j6_rot(0, 2), j6_x,
                j6_rot(1, 0), j6_rot(1, 1), j6_rot(1, 2), j6_y,
                j6_rot(2, 0), j6_rot(2, 1), j6_rot(2, 2), j6_z,
                0.0,            0.0,          0.0,         1.0;
        //m[5] = Eigen::Matrix4f::Identity();


    }

    void poseEstimation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_box_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pin_cloud)
    {
        Eigen::Matrix4d transformation_matrix_box = Eigen::Matrix4d::Identity(), transformation_matrix_pin = Eigen::Matrix4d::Identity();
        box_transformation_matrix = Eigen::Matrix4d::Identity(); pin_transformation_matrix = Eigen::Matrix4d::Identity();

        //cout << "Pin-end pose estimation" << endl;
        pcl::copyPointCloud(*input_box_cloud, *cloud_xyz_box);
        pcl::copyPointCloud(*aligned_box, *aligned_box_xyz);
        pose_est.setModel(aligned_box_xyz);
        pose_est.start(cloud_xyz_box, aligned_box_xyz, box_prealigned_tfmat, icp_transformation_box);
        pcl::copyPointCloud(*cloud_xyz_box, *input_box_cloud);
        pcl::copyPointCloud(*aligned_box_xyz, *aligned_box);
        box_transformation_matrix *= icp_transformation_box;
        box_transformation_matrix *= box_prealigned_tfmat;


        //cout << "Box-end pose estimation" << endl;
        pcl::copyPointCloud(*input_pin_cloud, *cloud_xyz_pin);
        pcl::copyPointCloud(*aligned_pin, *aligned_pin_xyz);
        pose_est.setModel(aligned_pin_xyz);
        pose_est.start(cloud_xyz_pin, aligned_pin_xyz, pin_prealigned_tfmat, icp_transformation_pin);
        pcl::copyPointCloud(*cloud_xyz_pin, *input_pin_cloud);
        pcl::copyPointCloud(*aligned_pin_xyz, *aligned_pin);
        pin_transformation_matrix *= icp_transformation_pin;
        pin_transformation_matrix *= pin_prealigned_tfmat;

        pose_box.erase(pose_box.begin(), pose_box.begin()+6);

        //Calculate angles and add pose to pose vectors.
        double theta_y;
        double theta_y1 = -sin(box_transformation_matrix(2,0));
        double theta_y2 = M_PI + sin(theta_y1);
        theta_y = theta_y1;


        double theta_x = atan2(box_transformation_matrix(2,1), box_transformation_matrix(2,2)); //Fiks når cos(theta_y) = 0
        double theta_z = atan2(box_transformation_matrix(1,0), box_transformation_matrix(0,0));

        pose_box.push_back(box_transformation_matrix(0,3));
        pose_box.push_back(box_transformation_matrix(1,3));
        pose_box.push_back(box_transformation_matrix(2,3)); //plus factor is distance up to neck
        pose_box.push_back(theta_x);
        pose_box.push_back(theta_y);
        pose_box.push_back(theta_z);

        pose_pin.erase(pose_pin.begin(), pose_pin.begin()+6);
        theta_y1 = -sin(pin_transformation_matrix(2,0));
        theta_y2 = M_PI + sin(theta_y1);
        theta_y = theta_y1;

        theta_x = atan2(pin_transformation_matrix(2,1), pin_transformation_matrix(2,2)); //Fiks når cos(theta_y) = 0
        theta_z = atan2(pin_transformation_matrix(1,0), pin_transformation_matrix(0,0));
        pose_pin.push_back(pin_transformation_matrix(0,3));
        pose_pin.push_back(pin_transformation_matrix(1,3)); //Gimbal lock ved pi/2 rundt y
        pose_pin.push_back(pin_transformation_matrix(2,3));
        pose_pin.push_back(theta_x);
        pose_pin.push_back(theta_y);
        pose_pin.push_back(theta_z);



        bool recordQuaternion = true;
        if(recordData)
        {
            if(recordQuaternion)
            {
                Eigen::Matrix3d box_rot, pin_rot;
                box_rot << box_transformation_matrix(0, 0), box_transformation_matrix(0, 1), box_transformation_matrix(0, 2),
                           box_transformation_matrix(1, 0), box_transformation_matrix(1, 1), box_transformation_matrix(1, 2),
                           box_transformation_matrix(2, 0), box_transformation_matrix(2, 1), box_transformation_matrix(2, 2);
                pin_rot << pin_transformation_matrix(0, 0), pin_transformation_matrix(0, 1), pin_transformation_matrix(0, 2),
                           pin_transformation_matrix(1, 0), pin_transformation_matrix(1, 1), pin_transformation_matrix(1, 2),
                           pin_transformation_matrix(2, 0), pin_transformation_matrix(2, 1), pin_transformation_matrix(2, 2);

                Eigen::Quaterniond q_box(box_rot);
                Eigen::Quaterniond q_pin(pin_rot);

                poseData << pose_box[0] << " " << pose_box[1] << " " << pose_box[2] << " " << q_box.x()  << " " << q_box.y() << " " << q_box.z() << " " << q_box.w() << " " << pose_box[3] << " " << pose_box[4] << " " << pose_box[5] << " "
                         << pose_pin[0] << " " << pose_pin[1] << " " << pose_pin[2] << " " << q_pin.x()  << " " << q_pin.y() << " " << q_pin.z() << " " << q_pin.w() << " " << pose_pin[3] << " " << pose_pin[4] << " " << pose_pin[5] << "\n";
            }
            else
            {
                poseData << pose_box[0] << " " << pose_box[1] << " " << pose_box[2] << " "
                         << pose_box[3] << " " << pose_box[4] << " " << pose_box[5] << " ";
                poseData << pose_pin[0] << " " << pose_pin[1] << " " << pose_pin[2] << " "
                         << pose_pin[3] << " " << pose_pin[4] << " " << pose_pin[5] << "\n";
            }

        }

    }


    void printTransformationMatrix(Eigen::Matrix4d matrix)
    {
        pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1),
                                 matrix(0, 2));
        pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1),
                                 matrix(1, 2));
        pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1),
                                 matrix(2, 2));
        pcl::console::print_info("\n");
        pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", matrix(0, 3), matrix(1, 3),
                                 matrix(2, 3));
        pcl::console::print_info("\n");
    }
};




int main(int argc, char** argv)
{
    cout << "start" << endl;
    ros::init(argc, argv, "Pipe_detector");
    cout << "etter init" << endl;
    Receiver a;
    a.run();


    return 1;
}

