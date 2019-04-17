#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include "pose_estimator.h"

#include <ros/ros.h>
#include <ros/spinner.h>



#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

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
    bool running, save, running_viewers;
    bool updateImage, updateCloud;
    size_t frame;

    ros::NodeHandle nh;
    ros::Publisher pub;

    ros::AsyncSpinner spinner;

    cv::Mat color_image1, color_image2, color_image3, color_image4, color_image5, color_image6;
    cv::Mat depth_image5, depth_image6;
    cv::Mat cameraMatrixColor, cameraMatrixDepth;
    cv::Mat lookupX, lookupY;
    cv::Mat color_images[7];

    PoseEstimator pose_est_box;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, box_cloud, pin_cloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal, aligned_boxEnd, filtered_cloud, segmented;
    pcl::PCDWriter writer;
    pcl::PLYReader reader;
    std::ostringstream oss;

    typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> ColorHandlerT;


public:
    Receiver() :
            running(false), updateImage(false), updateCloud(false), nh("a"), spinner(0), running_viewers(true)
    {
        cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
        cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
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

        message_filters::Subscriber<Image> image_color_sub_j6(nh, "/jetson6/qhd/image_color", 1);
        message_filters::Subscriber<Image> image_depth_sub_j6(nh, "/jetson6/qhd/image_depth_rect", 1);
        message_filters::Subscriber<Image> image_color_sub_j5(nh, "/jetson5/qhd/image_color", 1);
        message_filters::Subscriber<Image> image_depth_sub_j5(nh, "/jetson5/qhd/image_depth_rect", 1);
        message_filters::Subscriber<Image> image_color_sub_j4(nh, "/jetson4/qhd/image_color", 1);
        message_filters::Subscriber<Image> image_depth_sub_j4(nh, "/jetson4/qhd/image_depth_rect", 1);
        message_filters::Subscriber<Image> image_color_sub_j3(nh, "/jetson3/qhd/image_color", 1);
        message_filters::Subscriber<Image> image_depth_sub_j3(nh, "/jetson3/qhd/image_depth_rect", 1);
        message_filters::Subscriber<Image> image_color_sub_j2(nh, "/jetson2/qhd/image_color", 1);
        message_filters::Subscriber<Image> image_depth_sub_j2(nh, "/jetson2/qhd/image_depth_rect", 1);
        message_filters::Subscriber<Image> image_color_sub_j1(nh, "/jetson1/qhd/image_color", 1);
        message_filters::Subscriber<Image> image_depth_sub_j1(nh, "/jetson1/qhd/image_depth_rect", 1);
        message_filters::Subscriber<Image> image_color_kinect(nh, "/kinect2/qhd/image_color_rect", 1);

        typedef sync_policies::ApproximateTime<Image, Image, Image, Image, Image, Image> ApproxSyncPolicy;
        message_filters::Synchronizer<ApproxSyncPolicy> *syncApprox;
        syncApprox = new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(5), image_color_sub_j6, image_color_sub_j5,
                                                                         image_color_sub_j4, image_color_sub_j3, image_color_sub_j2, image_color_sub_j1);
        syncApprox->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4, _5, _6));


        std::chrono::milliseconds duration(1);




        spinner.start();

        //sover helt til man får tak i data fra publishere
        while (!updateImage || !updateCloud) {
            if (!ros::ok()) {
                return;
            }
            cout << "Starting up" << endl;
            std::this_thread::sleep_for(duration);
        }

        cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        box_cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pin_cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        cloud_normal = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
        aligned_boxEnd = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
        filtered_cloud = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
        segmented = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
        reader.read("/home/erlendb/Blender/box_end7_m.ply", *aligned_boxEnd);
        pose_est_box.setModel(aligned_boxEnd);
        /*
        cloud->height = color_image.rows;
        cloud->width = color_image.cols;
        cloud->is_dense = false;
        cloud->points.resize(cloud->height*cloud->width);
        box_cloud->height = color_image.rows;
        box_cloud->width = color_image.cols;
        box_cloud->is_dense = false;
        box_cloud->points.resize(box_cloud->height*box_cloud->width);*/

        //createLookup(this->color_image.cols, this->color_image.rows);

        //Different viewers
        //imageViewer();
        //cloudViewer();
        //poseViewer();
        rgbViewer();
    }

    void callback(const sensor_msgs::Image::ConstPtr &img_color6, const sensor_msgs::Image::ConstPtr &img_color5, const sensor_msgs::Image::ConstPtr &img_color4,
                  const sensor_msgs::Image::ConstPtr &img_color3, const sensor_msgs::Image::ConstPtr &img_color2, const sensor_msgs::Image::ConstPtr &img_color1)
    {
        cv::Mat color5, depth5, color6, depth6;

        cv::Mat colorMatlist[6];

        sensor_msgs::Image::ConstPtr colorImagesList[7];
        //colorImagesList[5] = img_color_kinect;
        colorImagesList[5] = img_color6;
        colorImagesList[4] = img_color5;
        colorImagesList[3] = img_color4;
        colorImagesList[2] = img_color3;
        colorImagesList[1] = img_color2;
        colorImagesList[0] = img_color1;

        //readCameraInfo(info_color, cameraMatrixColor);
        //readCameraInfo(info_depth, cameraMatrixDepth);

        for(int k = 0; k < 6; k++)
        {
            readImage(colorImagesList[k], colorMatlist[k]);

            if(colorMatlist[k].type() == CV_16U)
            {
                cv::Mat tmp;
                colorMatlist[k].convertTo(tmp, CV_8U, 0.02);
                cv::cvtColor(tmp, colorMatlist[k], CV_GRAY2BGR);
            }
        }

        //readImage(img_depth5, depth5); readImage(img_depth6, depth6);


        lock.lock();
        for(int i = 0; i < 6; i++)
            this->color_images[i] = colorMatlist[i];

        //this->color_image5 = color5;
        //this->depth_image5 = depth5;
        //this->color_image6 = color6;
        //this->depth_image6 = depth6;
        //this->detection_inf = *det_info;
        updateImage = true;
        updateCloud = true;
        lock.unlock();

    }

    void rgbViewer()
    {
        std::string cvWindows[3];
        cv::Mat colorImgList[7];
        cvWindows[0] = "RGBJ1";
        cvWindows[1] = "RGBJ2";
        cvWindows[2] = "RGBJ3";

        cv::Mat color5, color6;
        //yolo_node::detection_info det_inf;

        lock.lock();
        color5 = this->color_images[4];
        color6 = this->color_images[5];
        //det_inf = this->detection_inf;
        updateImage = false;
        lock.unlock();



        for(int i = 0; i < 3; i++)
        {
            cv::namedWindow(cvWindows[i]);
        }

        while(running && ros::ok())
        {
            if(updateImage)
            {
                lock.lock();
                //color5 = this->color_image5;
                colorImgList[0] = this->color_images[0];
                colorImgList[1] = this->color_images[1];
                colorImgList[2] = this->color_images[2];
                colorImgList[3] = this->color_images[3];
                colorImgList[4] = this->color_images[4];
                colorImgList[5] = this->color_images[5];

                //det_inf = this->detection_inf;
                updateImage = false;
                lock.unlock();

                for(int i = 0; i < 3; i++)
                {
                    cv::imshow(cvWindows[i], colorImgList[i]);
                }


                if(save)
                {
                    save = false;
                    saveCloudAndImages(colorImgList);
                }
            }
            int key = cv::waitKey(10);
            switch (key & 0xFF) {
                case 27:
                case 'q':
                    running = false;
                    break;
                case 's':
                    save = true;
                    break;
            }
        }


    }

    void readImage(const sensor_msgs::Image::ConstPtr img, cv::Mat &outImg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(img, img->encoding);
        cv_ptr->image.copyTo(outImg);
    }

    void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const {
        double *itC = cameraMatrix.ptr<double>(0, 0);
        for (size_t i = 0; i < 9; ++i, ++itC) {
            *itC = cameraInfo->K[i];
        }
    }

    void saveCloudAndImages(const cv::Mat color[6])
    {
        time_t t = time(NULL);
        tm* tPtr = localtime(&t);
        std::string tid;

        for(int i = 0; i < 6; i++)
        {
            oss.str("");
            oss << tPtr->tm_mon + 1 << "." << tPtr->tm_mday << ":"<<tPtr->tm_hour << "." << tPtr->tm_min << "." << tPtr->tm_sec << "j" << i << "_" << frame;
            const std::string baseName = oss.str();
            cout << baseName << endl;

            const std::string path = "/home/erlendb/Pictures/Masteroppgave_bilder/KinectNetwork/bilder17_04/";
            const std::string cloudName = path /*+ "Cloud/"*/ + baseName + "_cloud.pcd";
            const std::string colorName = path /*+ "Color/"*/ + baseName + "_color.jpg";
            cv::imwrite(colorName, color[i]);
        }









        //cout << "Saving cloud: " << cloudName << endl;
        //writer.writeBinary(cloudName, *cloud);


        /*OUT_INFO("saving depth: " << depthName);
        cv::imwrite(depthName, depth, params);
        OUT_INFO("saving depth: " << depthColoredName);
        cv::imwrite(depthColoredName, depthColored, params);*/
        //OUT_INFO("saving complete!");
        ++frame;
    }

};



int main(int argc, char** argv)
{
    cout << "start" << endl;
    ros::init(argc, argv, "kyssMegiRaeva");
    cout << "etter init" << endl;
    Receiver a;
    a.run();


    return 1;
}
