
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>


std::string path1 = "/home/erlendb/Pictures/4.16:11.22.21140062047409400_cloud.pcd";
std::string path2 = "/home/erlendb/Pictures/Masteroppgave_bilder/Bilder_poseTesting/pin_end/4.1:18.0.43139977279143938_cloud.pcd";
std::string path3 = "/home/erlendb/Pictures/Masteroppgave_bilder/Bilder_poseTesting/pin_end/4.1:18.3.24139977279143939_cloud.pcd";
std::string path4 = "/home/erlendb/Pictures/Masteroppgave_bilder/Bilder_poseTesting/pin_end/4.1:18.3.55139977279143940_cloud.pcd";
std::string model_path = "/home/erlendb/Blender/pin_end_short.ply";

int main(int argc, char** args)
{
    typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> ColorHandlerT;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene1 (new  pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene1_filtered (new  pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointNormal>::Ptr scene2 (new  pcl::PointCloud<pcl::PointNormal> ());
    pcl::PointCloud<pcl::PointNormal>::Ptr scene3 (new  pcl::PointCloud<pcl::PointNormal> ());
    pcl::PointCloud<pcl::PointNormal>::Ptr scene4 (new  pcl::PointCloud<pcl::PointNormal> ());
    pcl::PointCloud<pcl::PointNormal>::Ptr segmented (new  pcl::PointCloud<pcl::PointNormal> ());
    pcl::PointCloud<pcl::PointNormal>::Ptr model (new  pcl::PointCloud<pcl::PointNormal> ());


    std::string examplePath = "/home/erlendb/Downloads/table_scene_lms400.pcd";
    pcl::PCDReader reader12;
    pcl::PLYReader ply_reader12;

    //reader12.read(path1, *scene1);


    // Replace the path below with the path where you saved your file
    reader12.read<pcl::PointXYZ> (path1, *scene1);

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *scene1 << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (scene1);
    sor.setMeanK (100);
    sor.setStddevMulThresh (1.0);
    sor.filter (*scene1_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *scene1_filtered << std::endl;

    //pcl::PCDWriter writer;
    //writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *scene1_filtered, false);

    //sor.setNegative (true);
    //sor.filter (*);


    pcl::visualization::PCLVisualizer viewer;

    viewer.addPointCloud(scene1_filtered, "scene");
    viewer.spin();







    return 0;
}