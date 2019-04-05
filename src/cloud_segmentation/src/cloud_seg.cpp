
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


#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <ros/ros.h>
#include <ros/spinner.h>



#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>


#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
using namespace sensor_msgs;
using namespace std;

class CloudSegmentor
{
private:
    std::mutex lock;
    bool running, save, running_viewers;
    bool updateImage, updateCloud;
    size_t frame;
    darknet_ros_msgs::BoundingBoxes boxes;

    ros::NodeHandle nh;
    ros::Publisher pub;

    ros::AsyncSpinner spinner;

    cv::Mat color_image, depth_image;
    cv::Mat cameraMatrixColor, cameraMatrixDepth;
    cv::Mat lookupX, lookupY;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, box_cloud, pin_cloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal, aligned_boxEnd, filtered_cloud, segmented;
    pcl::PCDWriter writer;
    pcl::PLYReader reader;
    std::ostringstream oss;

    int QHD_WIDTH = 960;
    const int QHD_HEIGHT = 540;
    const int QHD_WIDTH_DZ = 80;//dødsone som ikke overlapper med depthmap på hver side
    const int IR_HEIGHT_DZ = 35;//Sone på topp og bunn på depthmap som ikke overlapper med hd


public:
    CloudSegmentor() :
            running(false), updateImage(false), updateCloud(false), nh("a"), spinner(0), running_viewers(true)
    {
        cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
        cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
    }

    ~CloudSegmentor()
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


        message_filters::Subscriber<Image> image_color_sub(nh, "/darknet_ros/detection_image", 1);
        message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bounding_boxes (nh, "/darknet_ros/bounding_boxes", 1);
        message_filters::Subscriber<Image> sd_depth_sub(nh, "/kinect2/sd/image_depth", 1);
        message_filters::Subscriber<CameraInfo> sd_cameraInfo_sub(nh, "/kinect2/sd/camera_info", 1);


        typedef message_filters::sync_policies::ApproximateTime<Image, Image, CameraInfo, darknet_ros_msgs::BoundingBoxes> ApproxSyncPolicy;
        message_filters::Synchronizer<ApproxSyncPolicy> *syncApprox;

        syncApprox = new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(5), image_color_sub, sd_depth_sub, sd_cameraInfo_sub, bounding_boxes);
        syncApprox->registerCallback(boost::bind(&CloudSegmentor::callback, this, _1, _2, _3, _4));


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

        box_cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pin_cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());

        box_cloud->height = depth_image.rows;
        box_cloud->width = depth_image.cols;
        box_cloud->is_dense = false;
        box_cloud->points.resize(box_cloud->height*box_cloud->width);

        createLookup(this->color_image.cols, this->color_image.rows);

        cloudViewer();
        //rgbViewer();

    }

    void callback(const sensor_msgs::Image::ConstPtr &img_color, const sensor_msgs::Image::ConstPtr &img_depth, const sensor_msgs::CameraInfo::ConstPtr info,
            const darknet_ros_msgs::BoundingBoxes::ConstPtr boxes)
    {
        cv::Mat color, depth;
        cout << "i callback" << endl;
        readCameraInfo(info, cameraMatrixColor);
        readCameraInfo(info, cameraMatrixDepth);
        readImage(img_depth, depth);
        readImage(img_color, color);

        if(color.type() == CV_16U)
        {
            cv::Mat tmp;
            color.convertTo(tmp, CV_8U, 0.02);
            cv::cvtColor(tmp, color, CV_GRAY2BGR);
        }



        lock.lock();
        this->boxes = *boxes;
        this->color_image = color;
        this->depth_image = depth;
        updateImage = true;
        updateCloud = true;
        lock.unlock();

    }

    void rgbViewer()
    {
        cv::Mat color, depth;

        lock.lock();
        color = this->color_image;
        depth = this->depth_image;
        updateImage = false;
        lock.unlock();

        cv::namedWindow("RGB Viewer");

        while(running && ros::ok())
        {
            if(updateImage)
            {
                lock.lock();
                color = this->color_image;
                depth = this->depth_image;
                updateImage = false;
                lock.unlock();

                cv::imshow("RGB Viewer", color);

            }

            if(save)
            {
                save = false;
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

    void cloudViewer()
    {
        darknet_ros_msgs::BoundingBoxes viewer_boxes;
        cv::Mat color, depth;

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud viewer"));
        const std::string cloudName = "rendered";

        lock.lock();
        color = this->color_image;
        depth = this->depth_image;
        viewer_boxes = this->boxes;
        updateCloud = false;
        lock.unlock();

        createCloud(depth, color, box_cloud, pin_cloud, viewer_boxes);
        visualizer->addPointCloud(box_cloud, cloudName);
        visualizer->addPointCloud(pin_cloud, "pin");
        visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
        visualizer->initCameraParameters();
        visualizer->setBackgroundColor(0, 0, 0);
        visualizer->setSize(color.cols, color.rows);
        visualizer->setShowFPS(true);
        visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
        visualizer->registerKeyboardCallback(&CloudSegmentor::keyboardEvent, *this);

        while(running && ros::ok())
        {
            if(updateCloud)
            {
                lock.lock();
                color = this->color_image;
                depth = this->depth_image;
                viewer_boxes = this->boxes;
                updateCloud = false;
                lock.unlock();


                createCloud(depth, color, box_cloud, pin_cloud, viewer_boxes);
                visualizer->updatePointCloud(box_cloud, cloudName);
                visualizer->updatePointCloud(pin_cloud, "pin");

            }
            if(save)
            {
                save = false;
                //saveCloudAndImages(pin_cloud, color, depth, depth);
            }
            //cv::waitKey(10);
            visualizer->spinOnce(10);
        }
        visualizer->close();
    }

    void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_box, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_pin, const darknet_ros_msgs::BoundingBoxes &boxes) const
    {
        const float badPoint = std::numeric_limits<float>::quiet_NaN();

        int r_min, r_max, c_min, c_max;

        cloud_box->clear();
        cloud_box->resize((size_t)depth.cols*depth.rows);
        cout << "QHD width" << color.cols << "\nQHD height: " << color.rows << endl;
        cloud_pin->clear();
        cloud_pin->resize((size_t)depth.cols*depth.rows);
        int width;
        pcl::PointXYZRGBA *itP;
        cv::Vec3b color_pixel;
        uint16_t depth_pixel;
        int x1, x2, y1, y2;

        for(int i = 0; i < boxes.bounding_boxes.size(); i++)
        {
            if(boxes.bounding_boxes[i].Class == "person")
            {
                //Reject proposals out of bounds with the IR depth image
                if(boxes.bounding_boxes[i].xmin < QHD_WIDTH_DZ || boxes.bounding_boxes[i].xmax > QHD_WIDTH-QHD_WIDTH_DZ)
                    continue;

                //Convert to from qhd coordinates to sd
                x1 = boxes.bounding_boxes[i].xmin; x2 = boxes.bounding_boxes[i].xmax;
                y1 = boxes.bounding_boxes[i].ymin; y2 = boxes.bounding_boxes[i].ymax;
                qhdTosd(x1, x2, y1, y2);

                width = x2 - x1;
                r_min = y1 - 1;
                r_max = y2 + 1;
                c_min = x1;
                c_max = x2 + 1;

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
                    if(boxes.bounding_boxes[i].Class == "person")
                        itP = &cloud_box->points[r * width];
                    else if(boxes.bounding_boxes[i].Class == "pin-end")
                        itP = &cloud_pin->points[r * width];

                    const float y = lookupY.at<float>(0, r);
                    const float *itX = lookupX.ptr<float>();

                    for(int i = 0; i < c_min; i++)
                        itX++;

                    for(size_t c = c_min; c < c_max; ++c, ++itP, /*++itC, ++itD,*/ ++itX)
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

    }

    void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const {
        double *itC = cameraMatrix.ptr<double>(0, 0);
        for (size_t i = 0; i < 9; ++i, ++itC) {
            *itC = cameraInfo->K[i];
        }
    }

    void readImage(const sensor_msgs::Image::ConstPtr img, cv::Mat &outImg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(img, img->encoding);
        cv_ptr->image.copyTo(outImg);
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

    void qhdTosd(int &x1, int &x2, int &y1, int &y2) const
    {
        cout << color_image.cols << endl;
        double qhd_height = depth_image.rows;
        double qhd_width = color_image.cols-2*IR_HEIGHT_DZ;
        double sd_height = depth_image.rows-2*IR_HEIGHT_DZ; //pga IR kamera er høyere enn hd kameraet
        double sd_width = depth_image.cols;

        double width_factor = sd_width/qhd_width;
        double height_factor = sd_height/qhd_height;

        x1 = (int)x1*width_factor;
        x2 = (int)x2*width_factor;
        y1 = (int)y1*height_factor;
        y2 = (int)y2*height_factor;

        cout << "width factor: " << width_factor << " x1: " << x1 << " x2: " << x2 << " y1: " << y1 << " y2: " << y2 << endl;
    }

};




int main(int argc, char** argv)
{
    ros::init(argc,argv, "hei");
    CloudSegmentor rr;
    rr.run();
    return 0;
}