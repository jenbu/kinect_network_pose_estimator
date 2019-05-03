
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
#include <fstream>


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
    pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr ir_lim(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::PCDReader reader;
    reader.read("/home/erlendb/Pictures/4.22:16.12.31140725926139648box_cloud.pcd", *unfiltered);
    reader.read("/home/erlendb/Pictures/4.22:16.12.31140725926139648pin_cloud.pcd", *filtered);
    reader.read("/home/erlendb/Pictures/Masteroppgave_bilder/Data160219/Cloud/2.16:13.35.220079_cloud.pcd", *ir_lim);

    cout << ir_lim->points.size() << endl;

    //pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Visualizer"));
    int v1 = 1; double ymax = 1; double ymin = 0.0; double width = 1.0/2.0; int k = 0;

    //visualizer->setBackgroundColor(255.0, 255.0, 255.0);
    //visualizer->addPointCloud(ir_lim, ColorHandlerT(ir_lim, 0.0, 0.0, 255.0), "ir");

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

    ofstream myFile;
    myFile.open("example.txt");
    myFile << "Deviation X " << "Devation Y " << "Deviation Z\n";
    myFile << 0.032 << " " << 0.321 << " "<< 1.324 << "\n";
    myFile << 0.36 << " " << 0.121 <<  " "  << 0.924 << "\n";
    myFile.close();



    return 0;
}