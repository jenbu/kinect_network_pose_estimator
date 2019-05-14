
#include "pose_estimator.h"




PoseEstimator::PoseEstimator() : yaw(0), roll(0), pitch(0), posX(0), posY(0), posZ(0), aligned(0)
{
    //tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>());
    //cloud = pcl::PointCloud<PointT>::Ptr();
    cloud_filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_filtered2 = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_filtered_return = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
    cloud_normals2 = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>());
    coefficients_plane =  pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());
    coefficients_cylinder = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());
    inliers_plane = pcl::PointIndices::Ptr(new pcl::PointIndices());
    inliers_cylinder = pcl::PointIndices::Ptr(new pcl::PointIndices());
    scene_features = FeatureCloudT::Ptr(new FeatureCloudT());
    object_features = FeatureCloudT::Ptr(new FeatureCloudT());
    leaf = 0.01f;
    pipe_model = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ> ());
    segmented_pipe = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ> ());
    input_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ> ());

}


bool PoseEstimator::start(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_model,
                          Eigen::Matrix4d &prealign_mat, Eigen::Matrix4d &alignment_mat)
{
    //pcl::copyPointCloud(*input_cloud, *cloud_filtered);

    if(model_assigned)
    {
        voxel_filter(cloud);
        //RANSAC_segmentor(cloud);
        square_exclusion_segmentor(cloud);
        pose_estimate(cloud, aligned_model, prealign_mat, alignment_mat);
        clearAll();

        if(aligned)
        {
            //aligned_model = pipe_model;
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
    input_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_filtered2.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_normals.reset(new pcl::PointCloud<pcl::Normal>);
    cloud_normals2.reset(new pcl::PointCloud<pcl::Normal>);

    inliers_plane.reset(new pcl::PointIndices());
    inliers_cylinder.reset(new pcl::PointIndices());

}

void PoseEstimator::voxel_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    //pass.setInputCloud(input_cloud);
    //pass.setFilterFieldName("z");
    //pass.setFilterLimits(0, 3.0);
    //pass.filter(*cloud_filtered);


    /*float leaf = 0.008f;
    vg.setInputCloud (cloud);
    vg.setLeafSize (leaf, leaf, leaf);
    vg.filter (*cloud);*/
    //cloud_filtered_return = cloud_filtered;

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 2.0);
    pass.filter(*cloud);

}

void PoseEstimator::RANSAC_segmentor(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    //cout << "filtered_cloud" << cloud_filtered->points.size() << endl;
    // Estimate point normals

    ne.setSearchMethod (tree);
    ne.setKSearch (50);
    //ne.setInputCloud (cloud);
    //ne.compute (*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.03);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.02);

    sor.setMeanK (50);
    sor.setStddevMulThresh (0.2);
    sor.setInputCloud (cloud);
    sor.filter(*cloud);
    for(int i = 0; i < 1; i++)
    {
        ne.setInputCloud (cloud);
        ne.compute (*cloud_normals);

        seg.setInputCloud (cloud);
        seg.setInputNormals (cloud_normals);
        seg.segment (*inliers_plane, *coefficients_plane);

        // Extract the planar inliers from the input cloud
        extract.setInputCloud (cloud);
        extract.setIndices (inliers_plane);
        extract.setNegative (false);

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud);
        extract_normals.setNegative (true);
        extract_normals.setInputCloud (cloud_normals);
        extract_normals.setIndices (inliers_plane);
        extract_normals.filter(*cloud_normals2);

        sor.setInputCloud (cloud);
        sor.filter(*cloud);
    }

    //std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;






    ne.setInputCloud (cloud);
    ne.compute (*cloud_normals2);

    //segmented_pipe = cloud_filtered2;


    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (30000);
    seg.setDistanceThreshold (0.07);
    seg.setRadiusLimits (0.0, 0.1);
    seg.setInputCloud (cloud);
    seg.setInputNormals (cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);

    extract.setInputCloud (cloud);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    extract.filter (*cloud);


}

void PoseEstimator::square_exclusion_segmentor(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PassThrough<pcl::PointXYZ> pass_seg;
    pass_seg.setInputCloud(cloud);
    pass_seg.setFilterFieldName("z");
    pass_seg.setFilterLimits(0.45, 1.6);
    pass_seg.filter(*temp);
    double sumX, sumY;
    double avgX, avgY;

    sumX = 0.0; sumY = 0.0;
    if(temp->points.size() > 0)
    {
        for(int k = 0; k < temp->points.size(); k++)
        {
            sumX += temp->points[k].x;
            sumY += temp->points[k].y;
        }
        avgX = sumX/temp->points.size();
        avgY = sumY/temp->points.size();

        //cout << "avgX: " << avgX << " avgY: " << avgY << endl;

        pass_seg.setInputCloud(cloud);
        pass_seg.setFilterFieldName("z");
        pass_seg.setFilterLimits(0.02, 1.5);
        pass_seg.filter(*cloud);
        pass_seg.setInputCloud(cloud);
        pass_seg.setFilterFieldName("x");
        pass_seg.setFilterLimits((avgX-0.2), (avgX+0.2));
        pass_seg.filter(*cloud);
        pass_seg.setInputCloud(cloud);
        pass_seg.setFilterFieldName("y");
        pass_seg.setFilterLimits((avgY-0.2), (avgY+0.2));
        pass_seg.filter(*cloud);

        //Statistical Outlier Filter
        sor.setMeanK (50);
        sor.setStddevMulThresh (0.4);
        sor.setInputCloud (cloud);
        sor.filter(*cloud);
    }
    else
    {
        cout << "Class: pose_segmentor, Error: no point cloud to narrow!" << endl;
    }

}

void PoseEstimator::pose_estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &model,
        Eigen::Matrix4d &prealign_mat, Eigen::Matrix4d &alignment_mat)
{
    pcl::PointCloud<pcl::Normal>::Ptr pipe_normals(new pcl::PointCloud<pcl::Normal>());
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

    //Prealignment
    double scene_sumX = 0, scene_sumY = 0, scene_sumZ = 0, model_sumX = 0, model_sumY = 0, model_sumZ = 0;
    double scene_avgX, scene_avgY, scene_avgZ, model_avgX, model_avgY, model_avgZ;
    for(int i = 0; i < cloud->points.size(); i++)
    {
        scene_sumX += cloud->points[i].x;
        scene_sumY += cloud->points[i].y;
        scene_sumZ += cloud->points[i].z;
    }
    for(int i = 0; i < model->points.size(); i++)
    {
        model_sumX += model->points[i].x;
        model_sumY += model->points[i].y;
        model_sumZ += model->points[i].z;
    }
    scene_avgX = scene_sumX/cloud->points.size();
    scene_avgY = scene_sumY/cloud->points.size();
    scene_avgZ = scene_sumZ/cloud->points.size();
    model_avgX = model_sumX/model->points.size();
    model_avgY = model_sumY/model->points.size();
    model_avgZ = model_sumZ/model->points.size();

    //cout << model_avgX << " " << model_avgY << " " << model_avgZ << endl;
    //cout << scene_avgX<< " " << scene_avgY << " " << scene_avgZ << endl;

    if((scene_avgX-model_avgX) > 0.2 || (scene_avgX-model_avgX) < -0.2)
        transformation_matrix(0,3) = scene_avgX - model_avgX;
    else
        transformation_matrix(0,3) = 0.0;
    if((scene_avgY - model_avgY) > 0.1 || (scene_avgY - model_avgY) < -0.1)
        transformation_matrix(1,3) = scene_avgY - model_avgY;
    else
        transformation_matrix(1,3) = 0.0;
    if((scene_avgZ-model_avgZ) > 0.02 || (scene_avgZ-model_avgZ) < -0.02)
        transformation_matrix(2, 3) = scene_avgZ - model_avgZ;
    else
        transformation_matrix(2, 3) = 0;

    pcl::transformPointCloud(*model, *model, transformation_matrix);
    prealign_mat *= transformation_matrix;


      //RANSAC pose estimator
      /*
      // Estimate normals for scene
      //pcl::console::print_highlight ("Estimating scene normals...\n");;
      nest.setRadiusSearch (0.01);
      nest.setInputCloud (cloud);
      nest.compute (*cloud_normals);
      nest.setInputCloud (model);
      nest.compute (*pipe_normals);

      // Estimate features
      //pcl::console::print_highlight ("Estimating features...\n");
      fest.setRadiusSearch (0.025);
      fest.setInputCloud (cloud);
      fest.setInputNormals (cloud_normals);
      fest.compute (*scene_features);
      fest.setInputCloud (model);
      fest.setInputNormals (pipe_normals);
      feompute (*object_features);


      //RANSAC alignment
      align.setInputSource (model);
      align.setSourceFeatures (object_features);
      align.setInputTarget (cloud);
      align.setTargetFeatures (scene_features);
      align.setRANSACOutlierRejectionThreshold(0.1);
      align.setMaximumIterations (80000); // Number of RANSAC iterations
      align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
      align.setCorrespondenceRandomness (20); // Number of nearest features to use
      align.setSimilarityThreshold (0.75f); // Polygonal edge length similarity threshold
      align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
      align.setInlierFraction (0.45f); // Required inlier fraction for accepting a pose hypothesis

      pcl::console::print_highlight ("Ransac pose-estimating\n");
      align.align (*model);
      if (align.hasConverged ()) {
          aligned = true;
          // Print results
          printf("\n");
          transformation = align.getFinalTransformation();
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
          pose.empty();
          pose.push_back(posX); pose.push_back(posY); pose.push_back(posZ); pose.push_back(pitch); pose.push_back(roll); pose.push_back(yaw);

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
      */

    //ICP Alignment
    int iterations = 250;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations (iterations);
    icp.setEuclideanFitnessEpsilon(1.5); //Divergence criterion
    icp.setTransformationEpsilon(1e-18); //Convergence criterion
    //icp.setMaxCorrespondenceDistance(0.01);
    icp.setMaxCorrespondenceDistance(0.5);
    icp.setInputSource (model);
    icp.setInputTarget (cloud);


    int icp_iterator = 0;
    while(icp_iterator < 60)
    {

        icp.align (*model);

        if (icp.hasConverged ())
        {
            //std::cout << "\nICP for box-end has converged, score is " << icp.getFitnessScore () << std::endl;
            alignment_mat *= icp.getFinalTransformation ().cast<double>();
            //std::cout << "\nICP transformation " << iterations << std::endl;
            //icp_transformation_box *= icp.getFinalTransformation ().cast<double>();
        }
        else
        {
            PCL_ERROR ("\nICP has not converged.\n");
        }
        icp_iterator++;
    }
}

bool PoseEstimator::setModel(pcl::PointCloud<pcl::PointXYZ>::Ptr model)
{
    pipe_model = model;
    model_assigned = true;
}

void PoseEstimator::getSegmented(pcl::PointCloud<pcl::PointXYZ>::Ptr &seg_cloud)
{
    seg_cloud = segmented_pipe;
}

void PoseEstimator::getPose(std::vector<float> &input_vec)
{
    if(aligned)
    {
        for(int i = 0; i < pose.size(); i++)
        {
            input_vec.push_back(pose[i]);
        }
    }
    else
    {
        cout << "not aligned, cant get pose" << endl;
    }
}

void PoseEstimator::getFiltered(pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud)
{
    filtered_cloud = cloud_filtered_return;
}