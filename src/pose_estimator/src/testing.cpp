
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
#include <chrono>
#include <pcl/console/time.h>   // TicToc


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
    typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerT;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);  // Original point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ>);  // Original point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_pin (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);  // Original point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_pin (new pcl::PointCloud<pcl::PointXYZ>);  // Original point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tr (new pcl::PointCloud<pcl::PointXYZ>);  // Transformed point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp (new pcl::PointCloud<pcl::PointXYZ>);  // ICP output point cloud

    int iterations = 200;
    pcl::PCDReader reader12;
    pcl::PLYReader ply_reader12;

    pcl::console::TicToc time;



    ply_reader12.read<pcl::PointXYZ> (model_path, *model);
    reader12.read<pcl::PointXYZ> (path1, *scene);
    ply_reader12.read<pcl::PointXYZ> (model_path_pin, *model_pin);
    reader12.read<pcl::PointXYZ> (path2, *scene_pin);

    //reader12.read(path1, *scene1);
    Eigen::Matrix4d transformation_matrix_box, transformation_matrix_pin;
    double sumX = 0, sumY = 0;
    double avgX, avgY;
    for(int i = 0; i < scene->points.size(); i++)
    {
        sumX += scene->points[i].x;
        sumY += scene->points[i].y;
    }
    avgX = sumX/scene->points.size();
    avgY = sumY/scene->points.size();
    double theta = -M_PI / 2;  // The angle of rotation in radians

    /*transformation_matrix << cos(theta), 0, -sin(theta), 0,
                             0         , 1, 0,           0,
                             sin(theta), 0, cos(theta) , 0,
                             0         , 0, 0,           1;
    */
    transformation_matrix_box << 1,              0,           0, (avgX),
                                 0,     cos(theta),  sin(theta), (avgY),
                                 0,     -sin(theta), cos(theta), 0.3,
                                 0,              0,           0, 1;

    sumX = 0; sumY = 0;
    for(int i = 0; i < scene_pin->points.size(); i++)
    {
        sumX += scene_pin->points[i].x;
        sumY += scene_pin->points[i].y;
    }
    avgX = sumX/scene_pin->points.size();
    avgY = sumY/scene_pin->points.size();

    theta = M_PI / 2;
    transformation_matrix_pin << 1,              0,           0, (avgX),
                                 0,     cos(theta),  sin(theta), (avgY),
                                 0,     -sin(theta), cos(theta), (0.55+0.3),
                                 0,              0,           0, 1;


    pcl::transformPointCloud(*model, *model, transformation_matrix_box);
    pcl::transformPointCloud(*model_pin, *model_pin, transformation_matrix_pin);


    pcl::visualization::PCLVisualizer viewer("ICP demo");
    viewer.addCoordinateSystem(2.0);
    viewer.addPointCloud(scene, ColorHandlerT(scene, 255.0, 255.0, 255.0), "scene");
    viewer.addPointCloud(model, ColorHandlerT(model, 255.0, 0.0, 255.0), "model");
    viewer.addPointCloud(scene_pin, ColorHandlerT(scene_pin, 255.0, 255.0, 255.0), "scene_pin");
    viewer.addPointCloud(model_pin, ColorHandlerT(model_pin, 255.0, 0.0, 255.0), "model_pin");
    viewer.spinOnce(1000);

    // The Iterative Closest Point algorithm
    time.tic ();
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations (iterations);
    icp.setInputSource (model);
    icp.setInputTarget (scene);
    icp.align (*model);
    //icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

    if (icp.hasConverged ())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix_box = icp.getFinalTransformation ().cast<double>();
        print4x4Matrix (transformation_matrix_box);
    }
    else
    {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
    }

    icp.setInputSource (model_pin);
    icp.setInputTarget (scene_pin);
    icp.align (*model_pin);

    if (icp.hasConverged ())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix_pin = icp.getFinalTransformation ().cast<double>();
        print4x4Matrix (transformation_matrix_pin);
    }
    else
    {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
    }


    viewer.updatePointCloud(scene, ColorHandlerT(scene, 255.0, 255.0, 255.0), "scene");
    viewer.updatePointCloud(model, ColorHandlerT(model, 255.0, 0.0, 255.0), "model");
    viewer.updatePointCloud(scene_pin, ColorHandlerT(scene_pin, 255.0, 255.0, 255.0), "scene_pin");
    viewer.updatePointCloud(model_pin, ColorHandlerT(model_pin, 255.0, 0.0, 255.0), "model_pin");

    viewer.spin();







    return 0;
}