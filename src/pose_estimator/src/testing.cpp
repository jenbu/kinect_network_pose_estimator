
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <chrono>
#include <pcl/console/time.h>   // TicToc
#include <fstream>
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
#include "seg_pose_estimator.h"


std::string path1 = "/home/erlendb/Pictures/4.19:13.32.170000box_cloud.pcd";
std::string path2 = "/home/erlendb/Pictures/4.19:13.32.170000pin_cloud.pcd";
std::string path3 = "/home/erlendb/Pictures/Masteroppgave_bilder/Bilder_poseTesting/pin_end/4.1:18.3.24139977279143939_cloud.pcd";
std::string path4 = "/home/erlendb/Pictures/Masteroppgave_bilder/Bilder_poseTesting/pin_end/4.1:18.3.55139977279143940_cloud.pcd";
std::string model_path = "/home/erlendb/Blender/box_end7_m.ply";
std::string model_path_pin = "/home/erlendb/Blender/pin_end1.ply";

void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}


int main(int argc, char** args)
{

    typedef pcl::FPFHSignature33 FeatureT;
    typedef pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::PointNormal, FeatureT> FeatureEstimationT;
    typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerT;
    typedef pcl::PointCloud<FeatureT> FeatureCloudT;
    pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr box_end(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::PointNormal>());
    pcl::PointCloud<pcl::PointNormal>::Ptr box_normals(new pcl::PointCloud<pcl::PointNormal>());
    //pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients());
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<pcl::PointNormal> extract_normals;
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices());
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices());
    FeatureCloudT::Ptr scene_features(new FeatureCloudT());
    FeatureCloudT::Ptr object_features(new FeatureCloudT());

    SegmentorPoseEstimator pose_est;

    FeatureEstimationT fest;

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::PointNormal> seg;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> nest;
    pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, FeatureT> align;
    Eigen::Matrix4d a = Eigen::Matrix4d::Identity(); Eigen::Matrix4d b = Eigen::Matrix4d::Identity();

    filtered->is_dense = false;
    filtered->points.resize(500*500);
    pcl::PCDReader reader;
    pcl::PLYReader ply_reader;

    ply_reader.read("/home/erlendb/Blender/box_end7_m.ply", *box_end);
    //reader.read("/home/erlendb/Pictures/4.22:16.12.31140725926139648box_cloud.pcd", *unfiltered);
    //reader.read("/home/erlendb/Pictures/Masteroppgave_bilder/Bilder_poseTesting/3.19:17.7.3718135632_cloud.pcd", *unfiltered);
    //reader.read("/home/erlendb/Pictures/5.7:18.31.337253516640box_cloud.pcd", *filtered);
    reader.read("/home/erlendb/Pictures/5.7:18.44.334294967295box_cloud.pcd", *filtered);
    //reader.read("/home/erlendb/Pictures/Masteroppgave_bilder/Data160219/Cloud/2.16:13.35.220079_cloud.pcd", *ir_lim);





    Eigen::Matrix4d transformation_matrix_box = Eigen::Matrix4d::Identity(); Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();


    double theta = -M_PI/2;
    transformation_matrix << 1,              0,           0, 0,
            0,     cos(theta),  sin(theta), 0,
            0,     -sin(theta), cos(theta), 0,
            0,              0,           0, 1;

    pcl::transformPointCloud(*box_end, *box_end, transformation_matrix);





    pcl::copyPointCloud(*filtered, *unfiltered);
    pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Visualizer"));
    visualizer->addPointCloud(unfiltered, ColorHandlerT(unfiltered, 255.0, 255.0, 255.0), "unfiltered");
    //visualizer->addPointCloud(box_end, ColorHandlerT(box_end, 255.0, 0.0, 255.0), "box");
    visualizer->spinOnce(2000);
    pose_est.setModel(box_end);
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time = std::chrono::high_resolution_clock::now();
    pose_est.start(filtered, box_end, a, b);
    std::chrono::time_point<std::chrono::high_resolution_clock> now_time = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now_time - start_time).count() / 1000.0;
    cout << "time lapsed: " << elapsed << endl;
    visualizer->addPointCloud(filtered, ColorHandlerT(filtered, 0.0, 0.0, 255.0), "ir");
    //visualizer->updatePointCloud(box_end, ColorHandlerT(box_end, 255.0, 0.0, 255.0), "box");


    //visualizer->setBackgroundColor(255.0, 255.0, 255.0);


    visualizer->addCoordinateSystem(0.5);
    visualizer->spin();

    /*


    for(int i = 0; i < 10; i++)
    {
        // Estimate normals for scene
        //pcl::console::print_highlight ("Estimating scene normals...\n");;
        nest.setRadiusSearch (0.01);
        nest.setInputCloud (filtered);
        nest.compute (*cloud_normals);
        nest.setInputCloud (box_end);
        nest.compute (*box_normals);

        // Estimate features
        //pcl::console::print_highlight ("Estimating features...\n");
        fest.setRadiusSearch (0.025);
        fest.setInputCloud (filtered);
        fest.setInputNormals (cloud_normals);
        fest.compute (*scene_features);
        fest.setInputCloud (box_end);
        fest.setInputNormals (box_normals);
        fest.compute (*object_features);

        align.setInputSource (box_end);
        align.setSourceFeatures (object_features);
        align.setInputTarget (filtered);
        align.setTargetFeatures (scene_features);
        align.setRANSACOutlierRejectionThreshold(0.1);
        align.setMaximumIterations (80000); // Number of RANSAC iterations
        align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
        align.setCorrespondenceRandomness (20); // Number of nearest features to use
        align.setSimilarityThreshold (0.75f); // Polygonal edge length similarity threshold
        align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
        align.setInlierFraction (0.45f); // Required inlier fraction for accepting a pose hypothesis

        pcl::console::print_highlight ("Ransac pose-estimating\n");
        align.align (*box_end);
        if (align.hasConverged ()) {
            // Print results
            printf("aligned!\n");

        }
        else {
            pcl::console::print_error("Alignment failed!\n");
        }
        visualizer->updatePointCloud(box_end, ColorHandlerT(box_end, 255.0, 0.0, 255.0), "box");
        visualizer->spinOnce(4000);
    }*/








    /*
    visualizer->addCoordinateSystem(2.0);
    visualizer->createViewPort(0, 0, 0.5, 1, v1);
    visualizer->setBackgroundColor(255.0, 255.0, 255.0, v1);
    visualizer->addPointCloud(unfiltered, ColorHandlerT(filtered, 0.0, 0.0, 255.0), "unfiltered", v1);
    visualizer->createViewPort(0.5, 0, 1, 1, v1);
    visualizer->setBackgroundColor(255.0, 255.0, 255.0, v1);
    visualizer->addPointCloud(filtered, ColorHandlerT(filtered, 0.0, 0.0, 255.0), "filtered", v1);
    */

    //visualizer->spin();



    return 0;
}