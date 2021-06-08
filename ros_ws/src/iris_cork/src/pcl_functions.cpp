#include "pcl_functions.h"

// double test1, test2;
// int test3, test4;

void PCLFunctions::updateParams(iris_cork::PCLCorkConfig &config)
{
    // Filter Box params
    roller_height_value = config.roller_height_value;
    roller_height_angle = config.roller_height_angle;
    roller_width_value = config.roller_width_value;

    // Statistical outliers params
    meanK = config.mean_k;

    // Radius Search for multiple algorithms
    radius_search = config.radius_search;

    // Clustering params
    cec_params.leaf_size = config.leaf_size;
    cec_params.cluster_tolerance = config.cluster_tolerance;
    cec_params.min_cluster_size = config.min_cluster_size;
    cec_params.max_cluster_size = config.max_cluster_size;
    cec_params.normal_diff = config.normal_diff;
    cec_params.squared_dist = config.squared_dist;
    cec_params.curv = config.curvature;

    // test1 = config.test_param1;
    // test2 = config.test_param2;
    // test3 = config.test_param3;
    // test4 = config.test_param4;
}

void PCLFunctions::filterRollerBox(CloudPtr cloud_in, CloudPtr cloud_out)
{
    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(-roller_width_value, -1, 0, 1.0));
    boxFilter.setMax(Eigen::Vector4f(roller_width_value, 2, roller_height_value, 1.0));
    boxFilter.setRotation(Eigen::Vector3f(-roller_height_angle*M_PI/180, 0, 0));
    boxFilter.setKeepOrganized(true);
    boxFilter.setInputCloud(cloud_in);
    boxFilter.filter(*cloud_out);
}

void PCLFunctions::remove_outliers(CloudPtr cloud_in, CloudPtr cloud_out)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_in);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh (1.0);
    sor.setKeepOrganized(true);
    sor.filter (*cloud_out);
}

void PCLFunctions::cloud_smoothing(CloudPtr cloud_in, CloudPtr cloud_out)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    CloudNormal mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;
        
    mls.setComputeNormals (true);
    // Set parameters
    std::vector<int> test;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, test);
    mls.setInputCloud (cloud_in);
    mls.setPolynomialOrder (3);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);

    // Reconstruct
    mls.process (mls_points);
        
    // Alterar tipo da cloud para PointXYZRGB
    copyPointCloud(mls_points, *cloud_out);
}

void PCLFunctions::surface_normals(CloudPtr cloud_in, CloudPtr cloud_out)
{
    // Compute normals
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    ne.setInputCloud (cloud_in);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (radius_search);
    ne.compute (*cloud_normals);

    copyPointCloud(*cloud_in, *cloud_out);
        
    for (int i = 0; i < cloud_in->size(); i++)
    {
        cloud_out->points[i].r = abs(cloud_normals->points[i].normal[0]) * 255;
        cloud_out->points[i].g = abs(cloud_normals->points[i].normal[1]) * 255;
        cloud_out->points[i].b = abs(cloud_normals->points[i].normal[2]) * 255;
    }
}

void PCLFunctions::surface_curvatures(CloudPtr cloud_in, CloudPtr cloud_out)
{
    // Compute normals
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    ne.setInputCloud (cloud_in);
    ne.setSearchMethod (tree);
    ne.setRadiusSearch (radius_search);
    ne.compute (*cloud_normals);

    copyPointCloud(*cloud_in, *cloud_out);
        
    for (int i = 0; i < cloud_in->size(); i++)
    {
        cloud_out->points[i].r = cloud_out->points[i].g = cloud_out->points[i].b = 55 + abs(cloud_normals->points[i].curvature / 0.3) * 200;
    }
}

void PCLFunctions::cec_extraction(CloudPtr cloud_in, CloudPtr cloud_out, IdxClustersPtr clusters, IdxClustersPtr sclusters, IdxClustersPtr lclusters, CloudNormalPtr cloud_with_normals)
{
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud (cloud_in);
    vg.setLeafSize (cec_params.leaf_size, cec_params.leaf_size, cec_params.leaf_size);
    vg.setDownsampleAllData (true);
    // vg.setKeepOrganized(true);
    vg.filter (*cloud_out);
    // Voxel grids unorganizes the cloud. maybe cloud out needes to be a copy?
    
    cout << "after voxel grid" << cloud_out->isOrganized() << endl;

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

    pcl::copyPointCloud(*cloud_out, *cloud_with_normals);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
    ne.setInputCloud (cloud_out);
    ne.setSearchMethod (search_tree);
    ne.setRadiusSearch (radius_search);
    ne.compute (*cloud_with_normals);
    cout << "after normal estimation" << cloud_out->isOrganized() << endl;

    pcl::ConditionalEuclideanClustering<pcl::PointXYZRGBNormal> cec (true);
    cec.setInputCloud (cloud_with_normals);
    cec.setConditionFunction (&enforceNormals);
    cec.setClusterTolerance (cec_params.cluster_tolerance);
    cec.setMinClusterSize (cloud_with_normals->points.size () / cec_params.min_cluster_size);
    cec.setMaxClusterSize (cloud_with_normals->points.size () / cec_params.max_cluster_size);
    cec.segment (*clusters);
    cec.getRemovedClusters (sclusters, lclusters);  
      
}

bool PCLFunctions::enforceNormals (const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance)
{
    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap (), 
    point_b_normal = point_b.getNormalVector3fMap ();
    double enf_normal_diff = point_a_normal.dot(point_b_normal);
    
    // std::cout << "dist: "<< squared_distance << std::endl;
    if (squared_distance < cec_params.squared_dist)
    {
        if (enf_normal_diff >= cec_params.normal_diff)
        {
            return (true);  
        }
    }

    return (false);
}


void PCLFunctions::regionGrowingSegmentation(CloudPtr cloud_in, CloudPtr cloud_out, std::vector <pcl::PointIndices> &clusters){
   
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud_in);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize (50);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud_in);
    //reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (1 / 180.0 * M_PI);
    reg.setCurvatureThreshold (0.1);

    // std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

 
    cout << "cloud size: " << cloud_in->size() << endl;
 

    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    for(int i = 0; i < clusters.size(); i++){
        std::cout << "First cluster has " << clusters[i].indices.size () << " points." << std::endl;

    }

}

void PCLFunctions::superbodyClustering(CloudPtr cloud_in, CloudPtr cloud_out, std::vector <pcl::PointIndices> &clusters){

    float voxel_resolution = 0.008f;
    float seed_resolution = 0.1f;
    float color_importance = 0.2;
    float spatial_importance = 0.1;
    float normal_importance = 0.1;

    //Generate crystallizer
    pcl::SupervoxelClustering<pcl::PointXYZRGB> super(voxel_resolution, seed_resolution);
   
    //Enter point cloud and crystallization parameters
    super.setInputCloud(cloud_in);
    super.setColorImportance(color_importance);
    super.setSpatialImportance(spatial_importance);
    super.setNormalImportance(normal_importance);
    //Output the result of crystal segmentation: the result is a mapping table
    std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr > supervoxel_clusters;
    super.extract(supervoxel_clusters);
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);

    // //Generate LCCP splitter
    // pcl::LCCPSegmentation<pcl::PointXYZRGB> seg;
    // //Enter the result of superbody clustering
    // seg.setInputSupervoxels(supervoxel_clusters,supervoxel_adjacency);
    // //CC test beta value
    // seg.setConcavityToleranceThreshold(test4);
    // //K neighbors of CC effect
    // seg.setKFactor(0);
    // //
    // seg.setSmoothnessCheck(false, voxel_resolution, seed_resolution, 0.1);
    // //SC validation
    // seg.setSanityCheck(false);
    // //Minimum split size
    // seg.setMinSegmentSize(0);

    // seg.segment();


    pcl::CPCSegmentation<pcl::PointXYZRGB> seg;
    //Enter the result of superbody clustering
    seg.setInputSupervoxels(supervoxel_clusters,supervoxel_adjacency);
    //Set segmentation parameters
    // seg.setCutting (max_cuts = 20,
    //             cutting_min_segments = 0,
    //             cutting_min_score = 0.16,
    //             locally_constrained = true,
    //             directed_cutting = true,
    //             clean_cutting = false)；
    seg.setCutting();
    seg.setRANSACIterations(2);
    seg.segment();

    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud ();
    pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared ();
    seg.relabelCloud (*lccp_labeled_cloud);

    cout << "cluters: " << lccp_labeled_cloud->size() << endl;

    cout << "cluters: " << clusters.size() << endl;

    int max_label = lccp_labeled_cloud->points[0].label;

    for(int i = 0; i  < lccp_labeled_cloud->size(); i++){
        if (lccp_labeled_cloud->points[i].label > max_label){
            max_label = lccp_labeled_cloud->points[i].label;
        }
    }

    clusters.resize(max_label);

    for(int i = 0; i < lccp_labeled_cloud->size(); i++){
        cout << "label " << i << " :" << lccp_labeled_cloud->points[i].label << endl;
        clusters[lccp_labeled_cloud->points[i].label-1].indices.push_back(i);
    }

    
    // seg.relabelCloud(pcl::PointCloud<pcl::PointXYZL> &labeled_cloud_arg);

}

void PCLFunctions::iterativeBoundaryRemoval(CloudPtr cloud_in, CloudPtr cloud_out, int iterations){
    pcl::NormalEstimation<PointXYZRGB, Normal> ne;
	pcl::search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB> ());
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>);
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::Boundary> est;

	// CloudPtr process_cloud (new PointCloud<PointXYZRGB>);
    // copyPointCloud(*cloud_in, *process_cloud);

    std::cout << "proces cloud size: " << cloud_out->size() << std::endl;

    for(int i = 0; i < iterations; i++){

        ne.setInputCloud (cloud_out);
        ne.setSearchMethod (tree);
        ne.setRadiusSearch (1.5);
        ne.setViewPoint(0.0,0.0,0.0);
        ne.compute (*cloud_normals);  

        est.setInputCloud (cloud_out);
        est.setInputNormals (cloud_normals);
        // float radius = 0.02 + (i * 0.02);
        float radius = 0.1;
        std::cout << "radius: " << radius << std::endl;
        est.setRadiusSearch (radius);   // 2cm radius
        est.setSearchMethod (typename pcl::search::KdTree<PointXYZRGB>::Ptr (new pcl::search::KdTree<PointXYZRGB>));
        est.compute (boundaries);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        for(int i = 0; i < cloud_out->points.size(); i++)
        {
            if(boundaries[i].boundary_point < 1)
            {
                    inliers->indices.push_back(i);
            }
        }
        extract.setInputCloud(cloud_out);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_out);

        std::cout << "proces cloud size: " << cloud_out->size() << std::endl;
    
    }

}



BoundingBox PCLFunctions::computeCloudBoundingBox(CloudPtr cloud_in)
{
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud_in, pcaCentroid);

    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cloud_in, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
    ///    the signs are different and the box doesn't get correctly oriented in some cases.
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  
                                                                               
    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud_in, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZRGB minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    BoundingBox bb;
    bb.orientation = bboxQuaternion;
    bb.position = bboxTransform;
    bb.minPoint = minPoint;
    bb.maxPoint = maxPoint;
    // MaxPoint - MinPoint
    // x -> altura da caixa
    // y -> largura da caixa
    // z -> comprimento da caixa
    bb.centroid = pcaCentroid;

    return bb;
}

CloudPtr PCLFunctions::subtractCloud(CloudPtr cloud, pcl::PointIndices::Ptr indices)
{
    CloudPtr cloud_subtracted (new Cloud);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (indices);
    extract.setNegative (true);
    extract.filter (*cloud_subtracted);

    return cloud_subtracted;
}


void PCLFunctions::getNearestNeighbors(int K, Eigen::Vector4f searchPoint, CloudPtr cloud, std::vector<int>& points, std::vector<float>& dists)
{
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud (cloud);

    cout << searchPoint.x() << " " << searchPoint.y() << " " << searchPoint.z() << endl;

    pcl::PointXYZRGB startingPoint;
    startingPoint.x = searchPoint.x();
    startingPoint.y = searchPoint.y();
    startingPoint.z = searchPoint.z(); 

    kdtree.nearestKSearch (startingPoint, K, points, dists);
}

pcl::PCLImage PCLFunctions::extractImageFromCloud(CloudPtr cloud_in, bool paintNaNBlack)
{
    pcl::PCLImage image;
    pcl::io::PointCloudImageExtractorFromRGBField<pcl::PointXYZRGB> image_extractor;
    image_extractor.setPaintNaNsWithBlack(paintNaNBlack);
    bool success = image_extractor.extract(*cloud_in, image);
    return image;
}