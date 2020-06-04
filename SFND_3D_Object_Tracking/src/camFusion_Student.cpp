
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, std::string img_id, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);
    /*string img_path = "/home/flying-dutchman/Pictures/";
    string img_name = "t_" + img_id + ".png";
    string img_filename = img_path.append(img_name);
    cv::imwrite(img_filename, topviewImg);*/

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // calculating mean of euclideam disance of each kpt match for outlier removal
    float median_euclidean_dist = 0;
    vector<float> euclidean_distances;
    for(auto & kptMatch : kptMatches)
    {
        euclidean_distances.push_back(kptMatch.distance);
    }

    std::sort(euclidean_distances.begin(), euclidean_distances.end());

    if(euclidean_distances.size() % 2 != 0 )
    {
        median_euclidean_dist = euclidean_distances[euclidean_distances.size() / 2];
    }
    else{
        median_euclidean_dist = (euclidean_distances[euclidean_distances.size() / 2] +
                                euclidean_distances[(euclidean_distances.size() / 2) + 1]) / 2;
    }

    float corr_match_thrsh = 100;

    cv::Point2f prev_kpt, curr_kpt;
    for(auto & kptMatch : kptMatches)
    {
        // if a kpt_match lies in the acceptable range of euclidean dist then
        if ((median_euclidean_dist - corr_match_thrsh < kptMatch.distance) && (median_euclidean_dist + corr_match_thrsh > kptMatch.distance))
        {
            prev_kpt = kptsPrev[kptMatch.queryIdx].pt;
            curr_kpt = kptsCurr[kptMatch.trainIdx].pt;

            if(boundingBox.roi.contains(prev_kpt) && boundingBox.roi.contains(curr_kpt))
            {
                boundingBox.kptMatches.push_back(kptMatch);
            }
        }
    }
    //cout << boundingBox.kptMatches.size() << endl;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // median distance ration calculation
    std::sort(distRatios.begin(), distRatios.end());
    double medianDistRatio;
    if(distRatios.size()%2 != 0)
    {
        int idx = (distRatios.size()+1)/2;
        medianDistRatio = distRatios[idx];

    }
    else
    {
        int idx = floor(distRatios.size()/2);
        //std::cout<<idx<<std::endl;
        medianDistRatio = ( distRatios[idx] + distRatios[idx+1]) / 2.0;

    }

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medianDistRatio);

}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // converting sensor frame rate to time between sensor frames
    double dT = 1/frameRate;

    double selected_x_val;
    vector<double> prev_x_pts;
    vector<double> curr_x_pts;

    for(auto it = lidarPointsPrev.begin(); it!= lidarPointsPrev.end(); ++it)
    {
        prev_x_pts.push_back(it->x);
    }

    for(auto it = lidarPointsCurr.begin(); it!= lidarPointsCurr.end(); ++it)
    {
        curr_x_pts.push_back(it->x);
    }

    std::sort(prev_x_pts.begin(), prev_x_pts.end());
    std::sort(curr_x_pts.begin(), curr_x_pts.end());


    /*cout<< "max value in x direction is " << *std::max_element(prev_x_pts.begin(), prev_x_pts.end()) << endl;
    cout<< "min value in x direction is " << *std::min_element(prev_x_pts.begin(), prev_x_pts.end()) << endl;
    cout<< "mean value in x direction is " << (std::accumulate(prev_x_pts.begin(), prev_x_pts.end(), *prev_x_pts.begin())) / prev_x_pts.size() << endl;*/

    vector<vector<double>> x_pts_vec;
    x_pts_vec.push_back(prev_x_pts);
    x_pts_vec.push_back(curr_x_pts);

    vector<double>selected_x_val_vec;

    // median calculation

    for (auto it = x_pts_vec.begin(); it != x_pts_vec.end(); ++it)
    {
        if(it->size() % 2 !=0)
        {

            selected_x_val = *(it->begin() + (it->size() + 1) / 2.0);
        }
        else
        {
            selected_x_val = (*(it->begin() + (it->size()) / 2.0) + *(it->begin() + (it->size()/2.0 + 1))) / 2.0;
        }
        selected_x_val_vec.push_back(selected_x_val);
    }

    // first val in selected_x_val_vec is median of x_val from prevFrame and second val in selected_x_val_vec is
    // median x_val from currFrame

    TTC = selected_x_val_vec[1] * dT / (selected_x_val_vec[0] - selected_x_val_vec[1]);

}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{

    int boxId_prev, boxId_curr;
    bool bothframes = false;

    std::map<int, vector<int>> temp_map;
    for (int i=0; i<currFrame.kptMatches.size(); i++)
    {
         int kpid_prev = currFrame.kptMatches[i].queryIdx;
         int kpid_curr = currFrame.kptMatches[i].trainIdx;

         cv::Point2f kpt_prev = prevFrame.keypoints[kpid_prev].pt;
         cv::Point2f kpt_curr = currFrame.keypoints[kpid_curr].pt;

        findBoundingBox(prevFrame.boundingBoxes, currFrame.boundingBoxes, kpt_prev, kpt_curr, boxId_prev, boxId_curr, bothframes);

        if(bothframes)
        {
            temp_map[boxId_prev].push_back(boxId_curr);
        }
    }
    vector<int> poss_matches;
    int count, max_count = 0;
    int most_freq_ele;
    for (auto &ele : temp_map)
    {
        count = 0;
        max_count = 0;
        poss_matches = ele.second;
        std::sort(poss_matches.begin(), poss_matches.end());
        for (int j = 0; j<poss_matches.size(); j++)
        {
            if(poss_matches[j]==poss_matches[j+1])
            {
                count += 1;
                if (count>max_count)
                {
                    max_count = count;
                    most_freq_ele = poss_matches[j];
                }
            }
            else{
                count = 0;
            }
        }
        bbBestMatches.insert( std::make_pair(ele.first, most_freq_ele));
    }
}

void findBoundingBox(std::vector<BoundingBox> &boxes_prev, std::vector<BoundingBox> &boxes_curr, cv::Point2f &kp_prev,
                     cv::Point2f &kp_curr, int &boxId_prev, int &boxId_curr, bool &bothframes)
{
    // function find if keypoint from prev and curr frame lies in any bounding boxes in respective frames,
    // if yes then return the id of respective bb and sets the flag bothframes
    bool in_prev = false; bool in_curr = false;
    vector<BoundingBox>::iterator it_prev;
    for (it_prev = boxes_prev.begin(); it_prev != boxes_prev.end(); ++it_prev)
    {
        if(it_prev->roi.contains(kp_prev))
        {
            in_prev = true;
            boxId_prev = it_prev->boxID;
            break;
        }
    }

    vector<BoundingBox>::iterator it_curr;
    for(it_curr = boxes_curr.begin(); it_curr != boxes_curr.end(); ++it_curr)
    {
        if(it_curr->roi.contains(kp_curr))
        {
            in_curr = true;
            boxId_curr = it_curr->boxID;
            break;
        }
    }

    if (in_prev && in_curr)
    {
        bothframes = true;
    }
}