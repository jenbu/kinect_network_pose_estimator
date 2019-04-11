
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>

#include "../pose_estimator.h"

std::string path1 = "/home/erlendb/Pictures/Masteroppgave_bilder/Bilder_poseTesting/pin_end/4.1:18.0.10139977279143937_cloud.pcd";
std::string path2 = "/home/erlendb/Pictures/Masteroppgave_bilder/Bilder_poseTesting/pin_end/4.1:18.0.43139977279143938_cloud.pcd";
std::string path3 = "/home/erlendb/Pictures/Masteroppgave_bilder/Bilder_poseTesting/pin_end/4.1:18.3.24139977279143939_cloud.pcd";
std::string path4 = "/home/erlendb/Pictures/Masteroppgave_bilder/Bilder_poseTesting/pin_end/4.1:18.3.55139977279143940_cloud.pcd";
std::string model_path = "/home/erlendb/Blender/pin_end_short.ply";

int main(int argc, char** args)
{
    typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> ColorHandlerT;
    pcl::PointCloud<pcl::PointNormal>::Ptr scene1 (new  pcl::PointCloud<pcl::PointNormal> ());
    pcl::PointCloud<pcl::PointNormal>::Ptr scene2 (new  pcl::PointCloud<pcl::PointNormal> ());
    pcl::PointCloud<pcl::PointNormal>::Ptr scene3 (new  pcl::PointCloud<pcl::PointNormal> ());
    pcl::PointCloud<pcl::PointNormal>::Ptr scene4 (new  pcl::PointCloud<pcl::PointNormal> ());
    pcl::PointCloud<pcl::PointNormal>::Ptr segmented (new  pcl::PointCloud<pcl::PointNormal> ());
    pcl::PointCloud<pcl::PointNormal>::Ptr model (new  pcl::PointCloud<pcl::PointNormal> ());

    pcl::PCDReader reader12;
    pcl::PLYReader ply_reader12;

    reader12.read(path1, *scene1);
    reader12.read(path2, *scene2);
    reader12.read(path3, *scene3);
    reader12.read(path4, *scene4);
    ply_reader12.read(model_path, *model);


    PoseEstimator test;
    test.setModel(model);
    pcl::visualization::PCLVisualizer viewer;

    for(int i = 0; i < 4; i++)
    {
        if(i == 0)
        {
            test.start(scene1, model);
            test.getFiltered(segmented);
            cout << "filtered_cloud size: " << segmented->points.size() << endl;
            viewer.addPointCloud(segmented, ColorHandlerT(segmented, 255.0, 0.0, 0.0), "scene3");
            //viewer.addPointCloud(scene3, ColorHandlerT(scene1, 255.0, 0.0, 0.0), "scene3");
            viewer.addPointCloud(model, ColorHandlerT(model, 255.0, 255.0, 0.0), "model");
        }
        else if(i==1)
        {
            viewer.removePointCloud("scene1");
            test.start(scene2, model);
            test.getFiltered(segmented);
            viewer.updatePointCloud(segmented, ColorHandlerT(segmented, 255.0, 0.0, 0.0), "scene3");
            //viewer.addPointCloud(scene2, ColorHandlerT(scene2, 255.0, 255.0, 0), "scene2");
            viewer.updatePointCloud(model, ColorHandlerT(model, 255.0, 255.0, 0.0), "model");
        }
        else if(i ==2)
        {
            //viewer.removePointCloud("scene2");
            test.start(scene3, model);
            test.getFiltered(segmented);
            viewer.updatePointCloud(segmented, ColorHandlerT(segmented, 255.0, 0.0, 0.0), "scene3");
            //viewer.addPointCloud(scene3, ColorHandlerT(scene3, 255.0, 255.0, 0), "scene3");
            viewer.updatePointCloud(model, ColorHandlerT(model, 255.0, 255.0, 0.0), "model");
        }
        else if(i ==3)
        {
            //viewer.removePointCloud("scene3");
            test.start(scene4, model);
            test.getFiltered(segmented);
            viewer.updatePointCloud(segmented, ColorHandlerT(segmented, 255.0, 0.0, 0.0), "scene3");
            //viewer.addPointCloud(scene4, ColorHandlerT(scene4, 255.0, 255.0, 0), "scene4");
            viewer.updatePointCloud(model, ColorHandlerT(model, 255.0, 255.0, 0.0), "model");
        }
        viewer.spinOnce(5000);
    }







    return 0;
}