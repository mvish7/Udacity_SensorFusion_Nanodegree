/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for(int i = -5; i < 5; i++)
    {
        double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
        double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
        pcl::PointXYZ point;
        point.x = i+scatter*rx;
        point.y = i+scatter*ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while(numOutliers--)
    {
        double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
        double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
        pcl::PointXYZ point;
        point.x = 5*rx;
        point.y = 5*ry;
        point.z = 0;

        cloud->points.push_back(point);

    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem (1.0);
    return viewer;
}
/* FUNCTION FOR PERFORMING RANSAC ON LINE
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    auto start_time = std::chrono::steady_clock::now();
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    for (int i=0; i<maxIterations; i++)
    {

        std::unordered_set<int> pt_ids;
        while(pt_ids.size() < 2)
        {
            pt_ids.insert(rand() % (cloud->points.size()));
        }

        auto itr = pt_ids.begin();
        pcl::PointXYZ pt1 = cloud->points[*itr];
        itr++;
        pcl::PointXYZ pt2 = cloud->points[*itr];

        // fitting a line
        // eq of line (y1−y2)x+(x2−x1)y+(x1∗y2−x2∗y1)=0 i.e.
        // x_comp + y_comp + const_comp = 0

        //pt contains [x, y, z] so pt.data[0] ...
        float x_comp = pt1.data[1] - pt2.data[1];
        float y_comp = pt2.data[0] - pt1.data[0];
        float const_comp = (pt1.data[0]*pt2.data[1]) - (pt2.data[0]*pt1.data[1]);

        // calculating distance from every point to the line

        for (int j=0; j<cloud->points.size(); j++)
        {
            if (pt_ids.count(j)>0)
                continue;

            pcl::PointXYZ temp_pt = cloud->points[j];
            float dist_to_line = fabs(x_comp*temp_pt.data[0] + y_comp*temp_pt.data[1] + const_comp) /
                    sqrt(std::pow(x_comp, 2) + std::pow(y_comp, 2));

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

    auto end_time = std::chrono::steady_clock::now();
    auto time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
    std::cout<<"RANSAC took "<<time_elapsed.count() << "ms" <<std::endl;

    return inliersResult;

}
*/

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    auto start_time = std::chrono::steady_clock::now();
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
        pcl::PointXYZ pt1 = cloud->points[*itr];
        x1 = pt1.data[0]; y1 = pt1.data[1]; z1 = pt1.data[2];
        itr++;
        pcl::PointXYZ pt2 = cloud->points[*itr];
        x2 = pt2.data[0]; y2 = pt2.data[1]; z2 = pt2.data[2];
        itr++;
        pcl::PointXYZ pt3 = cloud->points[*itr];
        x3 = pt3.data[0]; y3 = pt3.data[1]; z3 = pt3.data[2];

        //v1.push_back(x2-x1); v1.push_back(y2-y1); v1.push_back(z2-z1);
        //v2.push_back(x3-x1); v2.push_back(y3-y1); v2.push_back(z3-z1);

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

            pcl::PointXYZ temp_pt = cloud->points[j];
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

    auto end_time = std::chrono::steady_clock::now();
    auto time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time);
    std::cout<<"RANSAC took "<<time_elapsed.count() << "ms" <<std::endl;

    return inliersResult;

}


int main ()
{

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data - 2d data
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


    // Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        pcl::PointXYZ point = cloud->points[index];
        if(inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }


    // Render 2D point cloud with inliers and outliers
    if(inliers.size())
    {
        renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
        renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
    }
    else
    {
        renderPointCloud(viewer,cloud,"data");
    }

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }

}
