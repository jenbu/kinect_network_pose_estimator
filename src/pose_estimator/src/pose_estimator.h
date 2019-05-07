//
// Created by erlendb on 28.03.19.
//




#ifndef PROJECT_POSE_ESTIMATOR_H
#define PROJECT_POSE_ESTIMATOR_H
#include <chrono>
#include <sstream>
#include <vector>

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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>

class PoseEstimator
{
private:
    typedef pcl::PointXYZ PointT;
    typedef pcl::FPFHSignature33 FeatureT;
    typedef pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, FeatureT> FeatureEstimationT;
    typedef pcl::PointCloud<FeatureT> FeatureCloudT;
    typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandler;

    bool aligned;
    bool model_assigned;

    float leaf;
    std::ostringstream os;

    //pcl::visualization::PCLVisualizer::Ptr visualizer;
    pcl::PCDReader reader;
    pcl::PLYReader ply_reader;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest;
    FeatureEstimationT fest;
    pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, FeatureT> align;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, cloud_filtered2, cloud_filtered_return;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, cloud_normals2;
    pcl::ModelCoefficients::Ptr coefficients_plane, coefficients_cylinder;
    pcl::PointIndices::Ptr inliers_plane, inliers_cylinder;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pipe_model;
    pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_pipe;


    FeatureCloudT::Ptr scene_features;
    FeatureCloudT::Ptr object_features;

    std::chrono::time_point<std::chrono::high_resolution_clock> start_time, now_time;


    Eigen::Matrix4f transformation;
    //Translation and rotation values
    Eigen::Vector3f translation_vector;
    Eigen::Quaternionf rotation;
    float yaw, roll, pitch;
    float posX, posY, posZ;
    std::vector<float> pose;

//Functions
public:
    PoseEstimator();
    bool start(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_model,
            Eigen::Matrix4d &prealign_mat, Eigen::Matrix4d &alignment_mat);

    //Setters and getters
    bool setModel(pcl::PointCloud<pcl::PointXYZ>::Ptr model);
    void getSegmented(pcl::PointCloud<pcl::PointXYZ>::Ptr &seg_cloud);
    void getFiltered(pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud);
    void getPose(std::vector<float> &input_vec);


private:
    void voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void extract_cylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    void pose_estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &model,
            Eigen::Matrix4d &prealign_mat, Eigen::Matrix4d &alignment_mat);
    void clearAll();
};



#endif //PROJECT_POSE_ESTIMATOR_H
