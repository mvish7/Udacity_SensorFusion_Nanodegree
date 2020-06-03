//
// Created by flying-dutchman on 05.05.20.
//

#ifndef LIDAR_OBSTACLE_DETECTION_PROCESSPOINTCLOUDS_H
#define LIDAR_OBSTACLE_DETECTION_PROCESSPOINTCLOUDS_H

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include "render/box.h"
#include "cluster/kdtree.h"

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    void proximity_checker(std::vector<std::vector<float>> points, int id, KdTree* tree, std::vector<bool>& processed_ids,
                           std::vector<int>& sub_cluster, float distanceTol, int minSize, int maxSize );

    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol,
                                                   typename pcl::PointCloud<PointT>::Ptr obstacle_cloud, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

};


#endif //LIDAR_OBSTACLE_DETECTION_PROCESSPOINTCLOUDS_H
