


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
    bool running, save, running_viewers;
    bool updateImage[6], updateCloud[6];
    size_t frame;

    ros::NodeHandle nh;
    ros::Publisher pub;

    PoseEstimator pose_est;

    ros::AsyncSpinner spinner;

    cv::Mat color_image[6], depth_image[6];
    cv::Mat cameraMatrixColor[6], cameraMatrixDepth[6];
    cv::Mat lookupX[6], lookupY[6];

    darknet_ros_msgs_eb::BoundingBoxes detection_inf[6];

    //Transformation matrix
    Eigen::Matrix4f to_world_matrix[6];
    Eigen::Matrix4f j2_toworld = Eigen::Matrix4f::Identity(); Eigen::Matrix4f j3_toworld = Eigen::Matrix4f::Identity();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud[6], box_cloud[6], pin_cloud[6];
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_world[6], box_cloud_world[6], pin_cloud_world[6];
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr concatenated_world, concatenated_pin, concatenated_box, segmented_box, aligned_box;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal_box, cloud_normal_pin, aligned_boxEnd, filtered_cloud, segmented_box_normal;
    pcl::PCDWriter writer;
    pcl::PLYReader reader;
    std::ostringstream oss;

    typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> ColorHandlerT;

    const int QHD_WIDTH = 960;
    const int QHD_HEIGHT = 540;
    int QHD_WIDTH_DZ = 90;//dødsone som ikke overlapper med depthmap på hver side
    int IR_HEIGHT_DZ = 40;//Sone på topp og bunn på depthmap som ikke overlapper med hd


public:
    Receiver() :
            running(false), nh("a"), spinner(0), running_viewers(true)
    {
        for(int i = 0; i < 6; i++)
        {
            cameraMatrixColor[i] = cv::Mat::zeros(3, 3, CV_64F);
            cameraMatrixDepth[i] = cv::Mat::zeros(3, 3, CV_64F);
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


        typedef message_filters::sync_policies::ApproximateTime<Image, CameraInfo, Image, CameraInfo, darknet_ros_msgs_eb::BoundingBoxes> ApproxSyncPolicy;


        message_filters::Subscriber<Image> image_color_sub_j2(nh, "/jetson2/qhd/image_color", 1);
        message_filters::Subscriber<CameraInfo> info_color_sub_j2(nh, "/jetson2/sd/camera_info", 1);
        message_filters::Subscriber<Image> image_depth_sub_j2(nh, "/jetson2/sd/image_depth", 1);
        message_filters::Subscriber<CameraInfo> info_depth_sub_j2(nh, "/jetson2/sd/camera_info", 1);
        message_filters::Subscriber<darknet_ros_msgs_eb::BoundingBoxes> bounding_boxes_j2(nh, "/jetson2/bounding_boxes", 1);

        message_filters::Synchronizer<ApproxSyncPolicy> *syncApprox_j2;
        syncApprox_j2 = new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(5), image_color_sub_j2, info_color_sub_j2, image_depth_sub_j2, info_depth_sub_j2, bounding_boxes_j2);
        syncApprox_j2->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4, _5, 1));

        message_filters::Subscriber<Image> image_color_sub_j3(nh, "/jetson3/qhd/image_color", 1);
        message_filters::Subscriber<CameraInfo> info_color_sub_j3(nh, "/jetson3/sd/camera_info", 1);
        message_filters::Subscriber<Image> image_depth_sub_j3(nh, "/jetson3/sd/image_depth", 1);
        message_filters::Subscriber<CameraInfo> info_depth_sub_j3(nh, "/jetson3/sd/camera_info", 1);
        message_filters::Subscriber<darknet_ros_msgs_eb::BoundingBoxes> bounding_boxes_j3(nh, "/jetson3/bounding_boxes", 1);

        message_filters::Synchronizer<ApproxSyncPolicy> *syncApprox_j3;
        syncApprox_j3 = new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(5), image_color_sub_j3, info_color_sub_j3, image_depth_sub_j3, info_depth_sub_j3, bounding_boxes_j3);
        syncApprox_j3->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4, _5, 2));


        std::chrono::milliseconds duration(1);


        initKinectToWorldTransMatrices(to_world_matrix[1], to_world_matrix[2]);

        segmented_box_normal = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
        cloud_normal_box = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
        cloud_normal_pin = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
        aligned_boxEnd = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
        reader.read("/home/erlendb/Blender/box_end7_m.ply", *aligned_boxEnd);
        pose_est.setModel(aligned_boxEnd);

        concatenated_world = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        concatenated_box = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        concatenated_pin = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        segmented_box = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        aligned_box = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::copyPointCloud(*aligned_boxEnd, *aligned_box);
        spinner.start();

        //sover helt til man får tak i data fra publishere
        while (!updateImage[1] || !updateCloud[1] || !updateImage[2] || !updateCloud[2]) {
            if (!ros::ok()) {
                return;
            }
            cout << "Starting up" << endl;
            std::this_thread::sleep_for(duration);
        }
        cout << "får messeges" << endl;

        for(int i = 0; i < 6; i++)
        {
            updateCloud[i] = false;
            updateImage[i] = false;

            cloud[i] = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
            box_cloud[i] = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
            pin_cloud[i] = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
            cloud_world[i] = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
            box_cloud_world[i] = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
            pin_cloud_world[i] = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());


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

    void callback(const sensor_msgs::Image::ConstPtr &img_color, const sensor_msgs::CameraInfo::ConstPtr info_color, const sensor_msgs::Image::ConstPtr &img_depth,
                  const sensor_msgs::CameraInfo::ConstPtr info_depth, const darknet_ros_msgs_eb::BoundingBoxes::ConstPtr det_info, int jetson_index)
    {
        cout << "Jetson" << jetson_index << " callback" << endl;
        cv::Mat color, depth;
        cv::Mat color_cropped, depth_cropped;

        readCameraInfo(info_color, cameraMatrixColor[jetson_index]);
        readCameraInfo(info_depth, cameraMatrixDepth[jetson_index]);

        readImage(img_color, color);
        readImage(img_depth, depth);


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

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Visualizer"));
        std::string box_cloudName[6];
        std::string cloud_name[6];
        std::string pin_cloudName[6];

        lock.lock();
        for(int i = 1; i < 3; i++)
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
        for(int i = 1; i < 3; i++)
        {
            createCloud(depth[i], color[i], box_cloud[i], pin_cloud[i], cloud[i], det_inf[i], i);
            *concatenated_world += *cloud_world[i];
            *concatenated_pin += *pin_cloud_world[i];
            *concatenated_box += *box_cloud_world[i];
            os << "Cloud" << i;  cloud_name[i] = os.str(); os.str(std::string());
            os << "Box-Cloud" << i; box_cloudName[i] = os.str(); os.str(std::string());
            os << "Pin-Cloud" << i; pin_cloudName[i] = os.str(); os.str(std::string());
            cout << "cloud_name: " << cloud_name[i] << " box_cloudName: " << box_cloudName[i] << " pin_cloudName: " << pin_cloudName[i] << endl;
        }
        poseEstimation(concatenated_box, concatenated_pin);

        //visualizer->addPointCloud(concatenated_world, ColorHandlerT(concatenated_world, 255.0, 255.0, 255.0), "World cloud");
        visualizer->addPointCloud(concatenated_box, ColorHandlerT(concatenated_box, 0.0, 255.0, 0.0), "Box cloud");
        //visualizer->addPointCloud(concatenated_pin, ColorHandlerT(concatenated_pin, 0.0, 255.0, 0.0), "Pin cloud");
        //visualizer->addPointCloud(segmented_box, ColorHandlerT(segmented_box, 255.0, 255.0, 0.0), "seg cloud");
        visualizer->addPointCloud(aligned_box, ColorHandlerT(aligned_box, 255.0, 0.0, 255.0), "box-model");

        //visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Visualizer");
        visualizer->initCameraParameters();
        visualizer->setBackgroundColor(0, 0, 0);
        //visualizer->setSize(color[1].cols, color[1].rows);
        visualizer->setShowFPS(true);
        visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
        visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);

        int iterator = 0;

        while(running && ros::ok())
        {
            if(updateCloud[1] && updateCloud[2])
            {

                lock.lock();
                for(int i = 1; i < 3; i++)
                {
                    color[i] = this->color_image[i];
                    depth[i] = this->depth_image[i];
                    det_inf[i] = this->detection_inf[i];
                    updateCloud[i] = false;
                    updateImage[i] = false;
                }
                lock.unlock();





                cout << "inni i loop" << endl;

                cout << "QHD DZ:" <<QHD_WIDTH_DZ << endl;
                cout << "IR DZ:" << IR_HEIGHT_DZ << endl;

                for(int i = 1; i < 3; i++)
                {
                    createCloud(depth[i], color[i], box_cloud[i], pin_cloud[i], cloud[i], det_inf[i], i);
                    *concatenated_world += *cloud_world[i];
                    *concatenated_pin += *pin_cloud_world[i];
                    *concatenated_box += *box_cloud_world[i];

                    //visualizer->updatePointCloud(cloud_world[i], ColorHandlerT(cloud_world[i], 255.0, 255.0, 255.0), cloud_name[i]);
                    //visualizer->updatePointCloud(box_cloud_world[i], ColorHandlerT(box_cloud_world[i], 0.0, 255.0, 0.0), box_cloudName[i]);
                    //visualizer->updatePointCloud(pin_cloud_world[i], ColorHandlerT(pin_cloud_world[i], 255.0, 0.0, 0.0), pin_cloudName[i]);
                }

                if(iterator==3)
                {
                    poseEstimation(concatenated_box, concatenated_pin);
                    //visualizer->updatePointCloud(concatenated_world, ColorHandlerT(concatenated_world, 255.0, 255.0, 255.0), "World cloud");
                    visualizer->updatePointCloud(concatenated_box, ColorHandlerT(concatenated_box, 255.0, 0.0, 0.0), "Box cloud");
                    //visualizer->updatePointCloud(concatenated_pin, ColorHandlerT(concatenated_pin, 0.0, 255.0, 0.0), "Pin cloud");
                    //visualizer->updatePointCloud(segmented_box, ColorHandlerT(segmented_box, 255.0, 255.0, 0.0), "seg cloud");
                    visualizer->updatePointCloud(aligned_box, ColorHandlerT(aligned_box, 255.0, 0.0, 255.0), "box-model");

                    concatenated_world->clear();
                    concatenated_world->resize((size_t)depth[1].cols*depth[1].rows);
                    concatenated_pin->clear();
                    concatenated_pin->resize((size_t)depth[1].cols*depth[1].rows);
                    concatenated_box->clear();
                    concatenated_box->resize((size_t)depth[1].cols*depth[1].rows);
                    segmented_box->clear();
                    segmented_box->resize((size_t)depth[1].cols*depth[1].rows);


                    iterator = 0;
                }


                for(int i = 0; i < det_inf[1].bounding_boxes.size(); i++)
                {
                    p1.x = det_inf[1].bounding_boxes[i].xmin; p1.y = det_inf[1].bounding_boxes[i].ymin;
                    p2.x = det_inf[1].bounding_boxes[i].xmax; p2.y = det_inf[1].bounding_boxes[i].ymax;
                    cv::rectangle(color[1], p1, p2, (255.0, 0.0, 0.0));
                    cv::putText(color[1], det_inf[1].bounding_boxes[i].Class, p1, cv::FONT_HERSHEY_PLAIN, 0.5, (255.0, 255.0, 255.0));

                }

                cv::imshow("ROI detector", color[1]);
                //cv::imshow("depth", depth);
                iterator += 1;

            }
            if(save)
            {
                save = false;
                //saveCloudAndImages(pin_cloud, color, depth, depth);
            }
            cv::waitKey(10);
            visualizer->spinOnce(1000);


        }
        visualizer->close();
        cv::destroyAllWindows();
        cv::waitKey(100);
    }

    void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_box, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_pin,
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const darknet_ros_msgs_eb::BoundingBoxes &det_inf, int index) const
    {

        //cout << det_inf.bounding_boxes[0] << endl;
        const float badPoint = std::numeric_limits<float>::quiet_NaN();

        int r_min, r_max, c_min, c_max;

        cloud_box->clear();
        cloud_box->resize((size_t)depth.cols*depth.rows);
        //cout << "det_inf.class_type size: " << det_inf.bounding_boxes[0].Class << endl;
        //cout << "QHD width" << color.cols << "\nQHD height: " << color.rows << endl;
        cloud_pin->clear();
        cloud_pin->resize((size_t)depth.cols*depth.rows);
        cloud->clear();
        cloud->resize((size_t)depth.cols*depth.rows);
        cloud_world[index]->clear();
        cloud_world[index]->resize((size_t)depth.cols*depth.rows);
        int width;
        pcl::PointXYZRGBA *itP;
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

                //Convert to from qhd coordinates to sd
                x1 = det_inf.bounding_boxes[t].xmin; x2 = det_inf.bounding_boxes[t].xmax;
                y1 = det_inf.bounding_boxes[t].ymin; y2 = det_inf.bounding_boxes[t].ymax;
                qhdTosd(x1, x2, y1, y2);


                r_min = y1;
                r_max = y2+30;
                c_min = x1;
                c_max = x2+30;
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
                            itP->rgba = 0;
                            continue;
                        }

                        itP->z = depthValue;
                        itP->x = *itX * depthValue;
                        itP->y = y * depthValue;
                        itP->b = 255;//color_pixel.val[0];
                        itP->g = 255;//color_pixel.val[1];
                        itP->r = 255;//color_pixel.val[2];
                        itP->a = 255;
                    }
                }
            }
        }



        for(int r = 0; r < depth.rows; ++r)
        {

            itP = &cloud->points[r * depth.cols];


            const float y = lookupY[index].at<float>(0, r);
            const float *itX = lookupX[index].ptr<float>();


            for(size_t c = 0; c < depth.cols; ++c, ++itP, ++itX)
            {

                color_pixel = color.at<cv::Vec3b>(r,c);
                depth_pixel = depth.at<uint16_t>(r,c);

                register const float depthValue = depth_pixel / 1000.0f;
                // Check for invalid measurements
                if(depth_pixel == 0)
                {
                    // not valid
                    itP->x = itP->y = itP->z = badPoint;
                    itP->rgba = 0;
                    continue;
                }

                itP->z = depthValue;
                itP->x = *itX * depthValue;
                itP->y = y * depthValue;
                itP->b = 255;//color_pixel.val[0];
                itP->g = 255;//color_pixel.val[1];
                itP->r = 255;//color_pixel.val[2];
                itP->a = 255;
            }
        }

        pcl::transformPointCloud(*cloud, *cloud_world[index], to_world_matrix[index]);
        pcl::transformPointCloud(*cloud_pin, *pin_cloud_world[index], to_world_matrix[index]);
        pcl::transformPointCloud(*cloud_box, *box_cloud_world[index], to_world_matrix[index]);

    }

    void qhdTosd(int &x1, int &x2, int &y1, int &y2) const
    {
        x1 -= QHD_WIDTH_DZ; x2 -= QHD_WIDTH_DZ;

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


    void createLookup(size_t width, size_t height)
    {

        for(int i = 1; i < 3; i++)
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
            }
        }
    }

    void saveCloudAndImages(/*const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud,*/ const cv::Mat &color, const cv::Mat &depth)
    {
        time_t t = time(NULL);
        tm* tPtr = localtime(&t);
        std::string tid;

        oss.str("");
        oss << tPtr->tm_mon + 1 << "." << tPtr->tm_mday << ":"<<tPtr->tm_hour << "." << tPtr->tm_min << "." << tPtr->tm_sec << std::setfill('0') << std::setw(4) << frame;
        const std::string baseName = oss.str();

        cout << baseName << endl;

        const std::string path = "/home/erlendb/Pictures/";
        const std::string cloudName = path /*+ "Cloud/"*/ + baseName + "_cloud.pcd";
        const std::string colorName = path /*+ "Color/"*/ + baseName + "_color.jpg";
        const std::string depthName = path + baseName + "_depth.jpg";


        //cout << "Saving cloud: " << cloudName << endl;
        //writer.writeBinary(cloudName, *cloud);
        //OUT_INFO("saving color: " << colorName);
        cv::imwrite(colorName, color);
        //OUT_INFO("saving depth: " << depthName);
        cv::imwrite(depthName, depth);
        //OUT_INFO("saving depth: " << depthColoredName);
        //cv::imwrite(depthColoredName, depthColored, params);*/
        //OUT_INFO("saving complete!");
        ++frame;
    }

    void initKinectToWorldTransMatrices(Eigen::Matrix4f &m2, Eigen::Matrix4f &m3)
    {
        float j2_x = 1.729;
        float j2_y = 0.501;
        float j2_z = 4.135;
        //Eigen::Quaterniond q_j2(0.311, 0.869, -0.369, 0.111);
        Eigen::Quaterniond q_j2(-0.311, 0.869, -0.369, 0.111);
        Eigen::Matrix3d j2_rot;
        j2_rot = q_j2.normalized().toRotationMatrix();
        m2 <<   j2_rot(0, 0), j2_rot(0, 1), j2_rot(0, 2), j2_x,
                j2_rot(1, 0), j2_rot(1, 1), j2_rot(1, 2), j2_y,
                j2_rot(2, 0), j2_rot(2, 1), j2_rot(2, 2), j2_z,
                0.0,            0.0,          0.0,         1.0;

        float j3_x = 9.522;
        float j3_y = 5.275;
        float j3_z = 4.353;
        //Eigen::Quaterniond q_j3(0.225, 0.680, 0.662, -0.218);
        Eigen::Quaterniond q_j3(-0.225, 0.680, 0.662, -0.218);
        Eigen::Matrix3d j3_rot;
        j3_rot = q_j3.normalized().toRotationMatrix();
        cout << j3_rot << endl;
        m3 <<   j3_rot(0, 0), j3_rot(0, 1), j3_rot(0, 2), j3_x,
                j3_rot(1, 0), j3_rot(1, 1), j3_rot(1, 2), j3_y,
                j3_rot(2, 0), j3_rot(2, 1), j3_rot(2, 2), j3_z,
                0.0,            0.0,          0.0,         1.0;

        cout << m3 << endl;

    }

    void poseEstimation(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &input_box_cloud, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &input_pin_cloud)
    {
        pcl::copyPointCloud(*input_box_cloud, *cloud_normal_box);
        pose_est.start(cloud_normal_box, aligned_boxEnd);
        pose_est.getSegmented(segmented_box_normal);
        pcl::copyPointCloud(*segmented_box_normal, *segmented_box);
        pcl::copyPointCloud(*aligned_boxEnd, *aligned_box);

        pcl::copyPointCloud(*input_pin_cloud, *cloud_normal_pin);

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

