#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp"


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pt_cloud_processorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
//void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    /*auto* pt_cloud_processorI = new(ProcessPointClouds<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud = pt_cloud_processorI->loadPcd
            ("/home/flying-dutchman/CLionProjects/LIDAR_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000000.pcd");
    */

    //rendering the pt cloud
    //renderPointCloud(viewer, input_cloud, "city_block");


    //voxel grid and region based filtering
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>());

    float grid_resoln = 0.1;

    filtered_cloud = pt_cloud_processorI->FilterCloud(input_cloud, grid_resoln, Eigen::Vector4f (-15.0, -6.0, -2.0, 1),
                                                      Eigen::Vector4f ( 25.0, 7.0, 2.0, 1));
    renderPointCloud(viewer, filtered_cloud, "downsampled_filtered_cloud");


    // segmenting the point clouds in two -  road(ground plane) and obstacles
    int max_iterations = 100;
    float dist_threshold = 0.2;

    //using planeRANSAC function for segmentation instead of built it pcl segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segCloud =
            pt_cloud_processorI->RansacPlane(filtered_cloud, max_iterations, dist_threshold);

    // rendering the point clouds
    renderPointCloud(viewer,segCloud.first,"obstacleCloud",Color(1,0,0));
    renderPointCloud(viewer,segCloud.second,"groundplaneCloud",Color(0,1,0));


    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points_vec;

    for(int i=0; i<segCloud.first->size(); i++)
    {
        //fetching a point from obstacle cloud
        pcl::PointXYZI temp_pt = segCloud.first->points[i];

        // creating a  vector of X, Y, Z points as insert function requires a vector of floats as ip
        std::vector<float>temp_vec;
        temp_vec.push_back(temp_pt.data[0]);
        temp_vec.push_back(temp_pt.data[1]);
        temp_vec.push_back(temp_pt.data[2]);

        tree->insert(temp_vec, i);
        // euclidean cluster function takes point in the form of  vector <vector<floats>>
        points_vec.push_back(temp_vec);
        temp_vec.clear();
    }

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    float distanceTol = 0.4;
    int minSize = 30;
    int maxSize = 600;
    std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_clusters =
            pt_cloud_processorI->euclideanCluster(points_vec, tree, distanceTol, segCloud.first, minSize, maxSize);

    //
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering found " << cloud_clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

    int cluster_id = 0;
    std::vector<Color> color_set = {Color(1.0, 0.0, 0.0), Color(1.0, 1.0, 0.0), Color(0.0, 0.0, 1.0)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloud_clusters)
    {
        std::cout<<"cluster size";
        pt_cloud_processorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstacle_cloud"+std::to_string(cluster_id), color_set[cluster_id]);

        Box box = pt_cloud_processorI->BoundingBox(cluster);
        renderBox(viewer,box,cluster_id);
        ++cluster_id;
    }

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main() {
    {
        std::cout << "starting enviroment" << std::endl;

        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        CameraAngle setAngle = XY;
        initCamera(setAngle, viewer);

        //cityBlock(viewer);


        // streaming pcd as a gif
        ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
        std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1/");
        auto streamIterator = stream.begin();
        pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

        while (!viewer->wasStopped ())
        {
            // Clear viewer
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();

            // Load pcd and run obstacle detection process
            inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
            cityBlock(viewer, pointProcessorI, inputCloudI);

            streamIterator++;
            if(streamIterator == stream.end())
                streamIterator = stream.begin();
            viewer->spinOnce ();
        }

        /*
        while (!viewer->wasStopped())
        {
            viewer->spinOnce();
        }*/
    }
}
