


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


#include <ros/ros.h>
#include <ros/spinner.h>

#include <darknet_ros_msgs_eb/BoundingBox.h>
#include <darknet_ros_msgs_eb/BoundingBoxes.h>

#include <../pose_estimator.cpp>


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

    cv::Mat color_image, depth_image;
    cv::Mat cameraMatrixColor, cameraMatrixDepth;
    cv::Mat lookupX, lookupY;

    darknet_ros_msgs_eb::BoundingBoxes detection_inf;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, box_cloud, pin_cloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal, aligned_boxEnd, filtered_cloud, segmented;
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

        bool bridge_source = true;





        message_filters::Subscriber<Image> image_color_sub(nh, "/jetson2/qhd/image_color", 1);
        message_filters::Subscriber<CameraInfo> info_color_sub(nh, "/jetson2/sd/camera_info", 1);
        message_filters::Subscriber<Image> image_depth_sub(nh, "/jetson2/sd/image_depth", 1);
        message_filters::Subscriber<CameraInfo> info_depth_sub(nh, "/jetson2/sd/camera_info", 1);
        message_filters::Subscriber<darknet_ros_msgs_eb::BoundingBoxes> bounding_boxes_j2(nh, "/jetson2/bounding_boxes", 1);
        typedef message_filters::sync_policies::ApproximateTime<Image, CameraInfo, Image, CameraInfo, darknet_ros_msgs_eb::BoundingBoxes> ApproxSyncPolicy;
        message_filters::Synchronizer<ApproxSyncPolicy> *syncApprox;

        syncApprox = new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(5), image_color_sub, info_color_sub, image_depth_sub, info_depth_sub, bounding_boxes_j2);
        syncApprox->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4, _5));

        //TimeSynchronizer</*Image,*/ CameraInfo, Image, CameraInfo/*, darknet_ros_msgs::BoundingBoxes*/> sync(/*image_color_sub,*/ info_color_sub, image_depth_sub,  info_depth_sub, /*bounding_boxes_j2,*/ 10);
        //sync.registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3/*, _4, _5*/));

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
        cout << "får messeges" << endl;
        cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        box_cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pin_cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());


        cloud->height = color_image.rows;
        cloud->width = color_image.cols;
        cloud->is_dense = false;
        cloud->points.resize(cloud->height*cloud->width);
        box_cloud->height = depth_image.rows;
        box_cloud->width = depth_image.cols;
        box_cloud->is_dense = false;
        box_cloud->points.resize(box_cloud->height*box_cloud->width);

        pin_cloud->height = depth_image.rows;
        pin_cloud->width = depth_image.cols;
        pin_cloud->is_dense = false;
        pin_cloud->points.resize(pin_cloud->height*pin_cloud->width);


        createLookup(this->color_image.cols, this->color_image.rows);

        cloudViewer();

    }

    void callback(const sensor_msgs::Image::ConstPtr &img_color, const sensor_msgs::CameraInfo::ConstPtr info_color, const sensor_msgs::Image::ConstPtr &img_depth,
                  const sensor_msgs::CameraInfo::ConstPtr info_depth, const darknet_ros_msgs_eb::BoundingBoxes::ConstPtr det_info)
    {
        //cout << "hei" << endl;
        cv::Mat color, depth;
        cv::Mat color_cropped, depth_cropped;

        readCameraInfo(info_color, cameraMatrixColor);
        readCameraInfo(info_depth, cameraMatrixDepth);

        readImage(img_color, color);
        readImage(img_depth, depth);


        if(color.type() == CV_16U)
        {
            cv::Mat tmp;
            color.convertTo(tmp, CV_8U, 0.02);
            cv::cvtColor(tmp, color, CV_GRAY2BGR);
        }



        lock.lock();
        this->color_image = color;
        this->depth_image = depth;
        this->detection_inf = *det_info;
        updateImage = true;
        updateCloud = true;
        lock.unlock();

    }

    void cloudViewer()
    {
        darknet_ros_msgs_eb::BoundingBoxes det_inf;
        cv::Mat color, depth;
        CvPoint p1, p2;

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud viewer"));
        const std::string cloudName = "rendered";

        lock.lock();
        color = this->color_image;
        depth = this->depth_image;
        det_inf = this->detection_inf;
        updateCloud = false;
        updateImage = false;
        lock.unlock();

        createCloud(depth, color, box_cloud, pin_cloud, cloud, det_inf);
        visualizer->addPointCloud(cloud, ColorHandlerT(cloud, 255.0, 255.0, 255.0) ,"Full cloud");
        visualizer->addPointCloud(box_cloud, ColorHandlerT(cloud, 0.0, 255.0, 0.0), cloudName);
        visualizer->addPointCloud(pin_cloud, ColorHandlerT(cloud, 255.0, 0.0, 0.0), "pin");
        visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
        visualizer->initCameraParameters();
        visualizer->setBackgroundColor(0, 0, 0);
        visualizer->setSize(color.cols, color.rows);
        visualizer->setShowFPS(true);
        visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
        visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);


        while(running && ros::ok())
        {
            if(updateCloud && updateImage)
            {
                lock.lock();
                color = this->color_image;
                depth = this->depth_image;
                det_inf = this->detection_inf;
                updateCloud = false;
                updateImage = false;
                lock.unlock();

                //QHD_WIDTH_DZ += 1;
                //IR_HEIGHT_DZ += 1;

                cout << "QHD DZ:" <<QHD_WIDTH_DZ << endl;
                cout << "IR DZ:" << IR_HEIGHT_DZ << endl;

                createCloud(depth, color, box_cloud, pin_cloud, cloud, det_inf);
                //filter_cloud(cloud, filtered_cloud);
                visualizer->updatePointCloud(cloud, ColorHandlerT(cloud, 255.0, 255.0, 255.0) ,"Full cloud");
                visualizer->updatePointCloud(box_cloud, ColorHandlerT(cloud, 0.0, 255.0, 0.0), cloudName);
                visualizer->updatePointCloud(pin_cloud, ColorHandlerT(cloud, 255.0, 0.0, 0.0), "pin");

                for(int i = 0; i < det_inf.bounding_boxes.size(); i++)
                {
                    p1.x = det_inf.bounding_boxes[i].xmin; p1.y = det_inf.bounding_boxes[i].ymin;
                    p2.x = det_inf.bounding_boxes[i].xmax; p2.y = det_inf.bounding_boxes[i].ymax;
                    cv::rectangle(color, p1, p2, (255.0, 0.0, 0.0));
                    cv::putText(color, det_inf.bounding_boxes[i].Class, p1, cv::FONT_HERSHEY_PLAIN, 0.5, (255.0, 255.0, 255.0));

                }

                cv::imshow("ROI detector", color);
                //cv::imshow("depth", depth);

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
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const darknet_ros_msgs_eb::BoundingBoxes &det_inf) const
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

                    const float y = lookupY.at<float>(0, r);
                    const float *itX = lookupX.ptr<float>();

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
        //cout << "depth cols" << depth.cols << "\ndepth rows " << depth.rows << endl;



        for(int r = 0; r < depth.rows; ++r)
        {

            itP = &cloud->points[r * depth.cols];


            const float y = lookupY.at<float>(0, r);
            const float *itX = lookupX.ptr<float>();


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




    }

    void qhdTosd(int &x1, int &x2, int &y1, int &y2) const
    {
        x1 -= QHD_WIDTH_DZ; x2 -= QHD_WIDTH_DZ;

        double qhd_height = this->color_image.rows;
        double qhd_width = this->color_image.cols-2*QHD_WIDTH_DZ;
        double sd_height = this->depth_image.rows-2*IR_HEIGHT_DZ; //pga IR kamera er høyere enn hd kameraet
        double sd_width = this->depth_image.cols;

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
        //fx og fy er focal lengde, lengde/piksel. er begge like har man helt kvadratisk bilde.
        const float fx = 1.0f / cameraMatrixColor.at<double>(0,0);
        const float fy = 1.0f / cameraMatrixColor.at<double>(1,1);
        //Principal point cx og cy sier hvor på bilde-planet pinhole treffer.
        const float cx = cameraMatrixColor.at<double>(0,2);
        const float cy = cameraMatrixColor.at<double>(1,2);
        float *it;

        lookupY = cv::Mat(1, height, CV_32F);
        it = lookupY.ptr<float>();
        for(size_t r = 0; r < height; ++r, ++it)
        {
            *it = (r - cy) * fy;
        }

        lookupX = cv::Mat(1, width, CV_32F);
        it = lookupX.ptr<float>();
        for(size_t c = 0; c < width; ++c, ++it)
        {
            *it = (c - cx) * fx;
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

