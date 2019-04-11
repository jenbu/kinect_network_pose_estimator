//
// Created by erlendb on 28.03.19.
//




#ifndef PROJECT_POSE_ESTIMATOR_H
#define PROJECT_POSE_ESTIMATOR_H
#include <chrono>
#include <sstream>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

class PoseEstimator
{
private:
    typedef pcl::PointXYZ PointT;
    typedef pcl::FPFHSignature33 FeatureT;
    typedef pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, FeatureT> FeatureEstimationT;
    typedef pcl::PointCloud<FeatureT> FeatureCloudT;
    typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> ColorHandler;

    bool aligned;
    bool model_assigned;

    float leaf;
    std::ostringstream os;

    //pcl::visualization::PCLVisualizer::Ptr visualizer;
    pcl::PCDReader reader;
    pcl::PLYReader ply_reader;
    pcl::PassThrough<pcl::PointNormal> pass;
    pcl::NormalEstimation<pcl::PointNormal, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<pcl::PointNormal, pcl::Normal> seg;
    pcl::ExtractIndices<pcl::PointNormal> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree;
    pcl::VoxelGrid<pcl::PointNormal> vg;
    pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> nest;
    FeatureEstimationT fest;
    pcl::SampleConsensusPrerejective<pcl::PointNormal, pcl::PointNormal, FeatureT> align;

    pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered, cloud_filtered2, cloud_filtered_return;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, cloud_normals2;
    pcl::ModelCoefficients::Ptr coefficients_plane, coefficients_cylinder;
    pcl::PointIndices::Ptr inliers_plane, inliers_cylinder;
    pcl::PointCloud<pcl::PointNormal>::Ptr pipe_model;
    pcl::PointCloud<pcl::PointNormal>::Ptr segmented_pipe;


    FeatureCloudT::Ptr scene_features;
    FeatureCloudT::Ptr object_features;

    std::chrono::time_point<std::chrono::high_resolution_clock> start_time, now_time;

    //Translation and rotation values
    Eigen::Vector3f translation_vector;
    Eigen::Quaternionf rotation;
    float yaw, roll, pitch;
    float posX, posY, posZ;

//Functions
public:
    PoseEstimator();
    bool start(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr aligned_model);

    //Setters and getters
    bool setModel(pcl::PointCloud<pcl::PointNormal>::Ptr model);
    void getSegmented(pcl::PointCloud<pcl::PointNormal>::Ptr &seg_cloud);
    void getFiltered(pcl::PointCloud<pcl::PointNormal>::Ptr &filtered_cloud);


private:
    void voxel_filter();
    void extract_cylinder();
    void pose_estimate();
    void clearAll();
};



#endif //PROJECT_POSE_ESTIMATOR_H
