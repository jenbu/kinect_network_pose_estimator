
#include "pose_estimator.h"




PoseEstimator::PoseEstimator() : yaw(0), roll(0), pitch(0), posX(0), posY(0), posZ(0), aligned(0)
{
    tree = pcl::search::KdTree<pcl::PointNormal>::Ptr(new pcl::search::KdTree<pcl::PointNormal>());
    //cloud = pcl::PointCloud<PointT>::Ptr();
    cloud_filtered = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
    cloud_filtered2 = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
    cloud_filtered_return = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>());
    cloud_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
    cloud_normals2 = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
    coefficients_plane =  pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());
    coefficients_cylinder = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());
    inliers_plane = pcl::PointIndices::Ptr(new pcl::PointIndices());
    inliers_cylinder = pcl::PointIndices::Ptr(new pcl::PointIndices());
    scene_features = FeatureCloudT::Ptr(new FeatureCloudT());
    object_features = FeatureCloudT::Ptr(new FeatureCloudT());
    leaf = 0.01f;
    pipe_model = pcl::PointCloud<pcl::PointNormal>::Ptr (new pcl::PointCloud<pcl::PointNormal> ());
    segmented_pipe = pcl::PointCloud<pcl::PointNormal>::Ptr (new pcl::PointCloud<pcl::PointNormal> ());
    input_cloud = pcl::PointCloud<pcl::PointNormal>::Ptr (new pcl::PointCloud<pcl::PointNormal> ());

}


bool PoseEstimator::start(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, pcl::PointCloud<pcl::PointNormal>::Ptr aligned_model)
{
    input_cloud = cloud;
    if(model_assigned)
    {
        voxel_filter();
        extract_cylinder();
        start_time = std::chrono::high_resolution_clock::now();
        pose_estimate();
        now_time = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now_time - start_time).count() / 1000.0;
        cout << "time lapsed: " << elapsed << endl;
        clearAll();
        if(aligned)
        {
            aligned_model = pipe_model;
            return true;
        }
        else{
            return false;
        }
    } else{
        cout << "a model is not assigned!" << endl;
    }

}



void PoseEstimator::clearAll()
{
    cloud_filtered.reset(new pcl::PointCloud<pcl::PointNormal>);
    cloud_filtered2.reset(new pcl::PointCloud<pcl::PointNormal>);
    cloud_normals.reset(new pcl::PointCloud<pcl::Normal>);
    cloud_normals2.reset(new pcl::PointCloud<pcl::Normal>);

    inliers_plane.reset(new pcl::PointIndices());
    inliers_cylinder.reset(new pcl::PointIndices());

}

void PoseEstimator::voxel_filter()
{
    pass.setInputCloud(input_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 3.0);
    pass.filter(*cloud_filtered);

    float leaf = 0.003f;
    vg.setInputCloud (cloud_filtered);
    vg.setLeafSize (leaf, leaf, leaf);
    vg.filter (*cloud_filtered);
    cloud_filtered_return = cloud_filtered;
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;

}

void PoseEstimator::extract_cylinder()
{
    //cout << "filtered_cloud" << cloud_filtered->points.size() << endl;
    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

     // Create the segmentation object for the planar model and set all the parameters
     seg.setOptimizeCoefficients (true);
     seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
     seg.setNormalDistanceWeight (0.1);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setMaxIterations (100);
     seg.setDistanceThreshold (0.03);
     seg.setInputCloud (cloud_filtered);
     seg.setInputNormals (cloud_normals);
     // Obtain the plane inliers and coefficients
     seg.segment (*inliers_plane, *coefficients_plane);
     //std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

     // Extract the planar inliers from the input cloud
     extract.setInputCloud (cloud_filtered);
     extract.setIndices (inliers_plane);
     extract.setNegative (false);
     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointNormal> ());
     extract.filter (*cloud_plane);

     // Remove the planar inliers, extract the rest
     extract.setNegative (true);
     extract.filter (*cloud_filtered2);
     extract_normals.setNegative (true);
     extract_normals.setInputCloud (cloud_normals);
     extract_normals.setIndices (inliers_plane);
     extract_normals.filter (*cloud_normals2);

     // Create the segmentation object for cylinder segmentation and set all the parameters
     seg.setOptimizeCoefficients (true);
     seg.setModelType (pcl::SACMODEL_CYLINDER);
     seg.setMethodType (pcl::SAC_RANSAC);
     seg.setNormalDistanceWeight (0.1);
     seg.setMaxIterations (10000);
     seg.setDistanceThreshold (0.05);
     seg.setRadiusLimits (0, 0.1);
     seg.setInputCloud (cloud_filtered2);
     seg.setInputNormals (cloud_normals2);

     // Obtain the cylinder inliers and coefficients
     seg.segment (*inliers_cylinder, *coefficients_cylinder);
     //std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

     // Write the cylinder inliers to disk
     extract.setInputCloud (cloud_filtered2);
     extract.setIndices (inliers_cylinder);
     extract.setNegative (false);
     extract.filter (*segmented_pipe);

}

void PoseEstimator::pose_estimate()
{

    // Downsample
    leaf = 0.02;
    //pcl::console::print_highlight ("Downsampling...\n");

    vg.setLeafSize (0.01, 0.01, 0.01);
    vg.setInputCloud (pipe_model);
    vg.filter (*pipe_model);
    vg.setLeafSize (leaf, leaf, leaf);
    vg.setInputCloud (segmented_pipe);
    vg.filter (*segmented_pipe);

    // Estimate normals for scene
    //pcl::console::print_highlight ("Estimating scene normals...\n");;
    nest.setRadiusSearch (0.01);
    nest.setInputCloud (segmented_pipe);
    nest.compute (*segmented_pipe);

    // Estimate features
    //pcl::console::print_highlight ("Estimating features...\n");
    fest.setRadiusSearch (0.025);
    fest.setInputCloud (segmented_pipe);
    fest.setInputNormals (segmented_pipe);
    fest.compute (*scene_features);
    fest.setInputCloud (pipe_model);
    fest.setInputNormals (pipe_model);
    fest.compute (*object_features);

    align.setInputSource (pipe_model);
    align.setSourceFeatures (object_features);
    align.setInputTarget (segmented_pipe);
    align.setTargetFeatures (scene_features);
    align.setRANSACOutlierRejectionThreshold(0.1);
    align.setMaximumIterations (50000); // Number of RANSAC iterations
    align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness (28); // Number of nearest features to use
    align.setSimilarityThreshold (0.75f); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
    align.setInlierFraction (0.45f); // Required inlier fraction for accepting a pose hypothesis

    align.align (*pipe_model);
    if (align.hasConverged ()) {
        aligned = true;
        // Print results
        printf("\n");
        Eigen::Matrix4f transformation = align.getFinalTransformation();
        pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1),
                                 transformation(0, 2));
        pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1),
                                 transformation(1, 2));
        pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1),
                                 transformation(2, 2));
        pcl::console::print_info("\n");
        pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3),
                                 transformation(2, 3));
        pcl::console::print_info("\n");
        pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), pipe_model->size());
        translation_vector << (transformation(0,3)+0.16/2), (transformation(1,3)-0.42/2), (transformation(2,3)+0.16/2);

        pitch = atan2f(-transformation(2, 0), sqrtf(powf(transformation(2, 1), 2)+powf(transformation(2,2), 2)));
        roll = atan2f(transformation(2,1), transformation(2,2));
        yaw = atan2f(transformation(1,0), transformation(0,0));
        posX = transformation(0,3);
        posY = transformation(1,3);
        posZ = transformation(2,3);

        os << "Relative position:\n"<< "x: " << posX << " Pitch: " << pitch << "\ny: " << posY << " Roll: " << roll << "\nz: " << posZ << " Yaw: " << yaw << endl;
        Eigen::Matrix3f matrix;
        matrix << transformation(0, 0), transformation(0, 1), transformation(0, 2),
                transformation(1, 0), transformation(1, 1), transformation(1, 2),
                transformation(2, 0), transformation(2, 1), transformation(2, 2);
        cout << "matrix:\n"<< matrix << endl;
        rotation = Eigen::Quaternionf(matrix);
    }
    else {
        aligned = false;
        pcl::console::print_error("Alignment failed!\n");
    }
}

bool PoseEstimator::setModel(pcl::PointCloud<pcl::PointNormal>::Ptr model)
{
    pipe_model = model;
    model_assigned = true;
}

void PoseEstimator::getSegmented(pcl::PointCloud<pcl::PointNormal>::Ptr &seg_cloud)
{
    seg_cloud = segmented_pipe;
}

void PoseEstimator::getFiltered(pcl::PointCloud<pcl::PointNormal>::Ptr &filtered_cloud)
{
    filtered_cloud = cloud_filtered_return;
}