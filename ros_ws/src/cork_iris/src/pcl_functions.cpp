#include "pcl_functions.h"

void PCLFunctions::updateParams(cork_iris::PCLCorkConfig &config)
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
}

void PCLFunctions::filterRollerBox(CloudPtr cloud_in, CloudPtr cloud_out)
{
    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(-roller_width_value, -1, 0, 1.0));
    boxFilter.setMax(Eigen::Vector4f(roller_width_value, 2, roller_height_value, 1.0));
    boxFilter.setRotation(Eigen::Vector3f(-roller_height_angle*M_PI/180, 0, 0));
    boxFilter.setInputCloud(cloud_in);
    boxFilter.filter(*cloud_out);
}

void PCLFunctions::remove_outliers(CloudPtr cloud_in, CloudPtr cloud_out)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_in);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh (1.0);
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
    mls.setPolynomialOrder (2);
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
    vg.filter (*cloud_out);
    
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

    pcl::copyPointCloud(*cloud_out, *cloud_with_normals);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
    ne.setInputCloud (cloud_out);
    ne.setSearchMethod (search_tree);
    ne.setRadiusSearch (radius_search);
    ne.compute (*cloud_with_normals);

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
    
    if (squared_distance < cec_params.squared_dist)
    {
        if (enf_normal_diff >= cec_params.normal_diff)
        {
            return (true);  
        }
    }

    return (false);
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
