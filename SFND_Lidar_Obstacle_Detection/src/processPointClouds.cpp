//
// Created by flying-dutchman on 05.05.20.
//

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Fill in the function to do voxel grid point reduction and region based filtering
    // Time segmentation process
    //auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr downsampled_cloud  (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr filtered_downsampled_cloud (new pcl::PointCloud<PointT> ());

    // voxel downsampling
    pcl::VoxelGrid<PointT> grid_filter;
    grid_filter.setInputCloud(cloud);
    grid_filter.setLeafSize(filterRes, filterRes, filterRes);
    grid_filter.filter(*downsampled_cloud);

    //std::cout<<"number of points in downsampled point cloud -:"<<downsampled_cloud->points.size()<<"\n";

    // region based filtering - removing all the points which lie outside the defined region
    pcl::CropBox<PointT> box_filter;
    box_filter.setInputCloud(downsampled_cloud);
    box_filter.setMin(minPoint);
    box_filter.setMax(maxPoint);
    box_filter.filter(*filtered_downsampled_cloud);

    //std::cout<<"number of points in region sampled point cloud -:"<<filtered_downsampled_cloud->points.size()<<"\n";

    /*auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;*/



    //removing the points from the roof of the car

    std::vector<int> roof_indices;

    //define a crop-box of apprx size that represents roof points
    pcl::CropBox<PointT> roof;
    box_filter.setInputCloud(filtered_downsampled_cloud);
    box_filter.setMin(Eigen::Vector4f(-1.5,  -1.7, -1, 1));
    box_filter.setMax(Eigen::Vector4f(2.6,  1.7, -0.4, 1));
    box_filter.filter(roof_indices);

    // iterate through roof_points and include them in inliers
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for (int pt : roof_indices)
    {
        inliers->indices.push_back(pt);
    }

    // setting up an extractor object to extract roof points from filtered_downsampled_cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(filtered_downsampled_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*filtered_downsampled_cloud);

    return filtered_downsampled_cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    //auto start_time = std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    //std::vector<float> v1;
    //std::vector<float> v2;
    std::vector<float> cross_prod;

    for (int i=0; i<maxIterations; i++)
    {
        cross_prod.clear();
        //using unordered_sets here as they can only contain unique elements hence we can prevent having 2 or more same
        //random number initialisations
        std::unordered_set<int> pt_ids;
        while(pt_ids.size() < 3)
        {
            pt_ids.insert(rand() % (cloud->points.size()));
        }

        float x1, y1, z1, x2, y2, z2, x3, y3, z3;

        //fetching and storing the points
        auto itr = pt_ids.begin();
        PointT pt1 = cloud->points[*itr];
        x1 = pt1.data[0]; y1 = pt1.data[1]; z1 = pt1.data[2];
        itr++;
        PointT pt2 = cloud->points[*itr];
        x2 = pt2.data[0]; y2 = pt2.data[1]; z2 = pt2.data[2];
        itr++;
        PointT pt3 = cloud->points[*itr];
        x3 = pt3.data[0]; y3 = pt3.data[1]; z3 = pt3.data[2];

        // fitting a plane
        // eq of plane Ax+By+Cz+D=0
        // x_comp + y_comp + z_comp + const_comp = 0

        //taking cross product of the vectors
        cross_prod.push_back((y2-y1)*(z3-z1) - (z2-z1)*(y3-y1));  //A
        cross_prod.push_back((z2-z1)*(x3-x1) - (x2-x1)*(z3-z1));  //B
        cross_prod.push_back((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1));  //C
        cross_prod.push_back(-(cross_prod[0]*x1+cross_prod[1]*y1+cross_prod[2]*z1));  //D

        // calculating distance from every point to the Plane

        for (int j=0; j<cloud->points.size(); j++)
        {
            if (pt_ids.count(j)>0)
                continue;

            PointT temp_pt = cloud->points[j];
            float dist_to_line = fabs((cross_prod[0]*temp_pt.data[0]) + (cross_prod[1]*temp_pt.data[1]) +
                                      cross_prod[2]*temp_pt.data[2] + cross_prod[3]) / sqrt(std::pow(cross_prod[0], 2)
                                                                                            + std::pow(cross_prod[1], 2) + std::pow(cross_prod[2], 2));

            if (dist_to_line<distanceTol)
            {
                pt_ids.insert(j);
            }
        }

        if (pt_ids.size()>inliersResult.size())
        {
            inliersResult =  pt_ids;
        }

    }

    //auto end_time = std::chrono::steady_clock::now();
    //auto time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
    //std::cout<<"RANSAC took "<<time_elapsed.count() << "ms" <<std::endl;


    // creating point clouds for inliers and outliers
    // outliers are the point for the obstacles as they do not fit the plane (plane is the road)
    // inliers are the points of the road (ground plane)
    typename pcl::PointCloud<PointT>::Ptr inliers (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr outliers (new pcl::PointCloud<PointT> ());

    if (!inliersResult.empty())
    {
        for (int index = 0; index< cloud->points.size(); index++)
        {
            const PointT point = cloud->points[index];

            if (inliersResult.count(index))  //count elements with a specific key (key is value of the ele)
            {
                inliers->points.push_back(point);
            }
            else
            {
                outliers->points.push_back(point);
            }
        }
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResults(outliers, inliers);
    return segResults;
}


template<typename PointT>
void ProcessPointClouds<PointT>::proximity_checker(std::vector<std::vector<float>> points, int id, KdTree *tree,
                                                   std::vector<bool> &processed_ids, std::vector<int> &sub_cluster,
                                                   float distanceTol, int minSize, int maxSize)
{
    processed_ids[id] = true;
    sub_cluster.push_back(id);

    std::vector<int>nearby_pts = tree->search(points[id], distanceTol);

    std::vector<int>::iterator nearby_it;

    for(nearby_it = nearby_pts.begin(); nearby_it != nearby_pts.end(); ++nearby_it)
    {
        if (processed_ids[*nearby_it]==false)
        {
            /*if (sub_cluster.size()>maxSize)
            {
                break;
            }*/
            proximity_checker(points, *nearby_it, tree, processed_ids, sub_cluster, distanceTol, minSize, maxSize);

        }

    }


}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol,
                                               typename pcl::PointCloud<PointT>::Ptr obstacle_cloud, int minSize, int maxSize)
{

    // Fill out this function to return list of indices for each cluster
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed_ids(points.size(), false);  //speed improvement -- make processed_ids vector of bool instead of int
    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloud_clusters;

    //int id = 0;
    for(int i=0; i<points.size(); ++i)
    {
        if (processed_ids[i]==true)
        {
            //id++;
            continue;
        }
        std::vector<int>sub_cluster;
        proximity_checker(points, i, tree, processed_ids, sub_cluster, distanceTol, minSize, maxSize);
        if ((sub_cluster.size()>=minSize) && (sub_cluster.size()<maxSize))
        {
            clusters.push_back(sub_cluster);
        }
        sub_cluster.clear();
    }

    // until this point cluster is a vec<vec<int>>, below written code transforms clusters to the to the point clouds

    std::vector<int>::iterator it;
    for(int i=0; i<clusters.size(); i++)
    {
        typename pcl::PointCloud<PointT>::Ptr pt_cloud_clusters (new pcl::PointCloud<PointT>);

        for(it = clusters[i].begin(); it!= clusters[i].end(); ++it)
        {
            pt_cloud_clusters->points.push_back(obstacle_cloud->points[*it]);
        }
        pt_cloud_clusters->width = pt_cloud_clusters->points.size();
        pt_cloud_clusters->height = 1;
        pt_cloud_clusters->is_dense = true;

        cloud_clusters.push_back(pt_cloud_clusters);
    }
    return cloud_clusters;

}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
