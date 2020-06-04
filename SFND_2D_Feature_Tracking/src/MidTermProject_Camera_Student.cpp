/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time

    bool bVis = false;            // visualize results

    int num_keypoints, num_matches;
    float t_detector, t_descriptor;

    // Create an output filestream object
    std::ofstream result;

    //set it write and append mode
    result.open ("../results.csv", std::fstream::out |std::fstream::app);

    // Send the column header names to the created csv file
    result <<"imageIndex" << ","  << "detectorType" << ","<< "descriptorType" << ","<< "detectorTime" << ","<< "descriptorTime"<<
             ","<< "detectectedKeypoints" << ","<< "matchedKeypoints"<<","<<","<< "totalTime"<<","<< endl;

    //vector<string> detector_names = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "SIFT"};
    //vector<string> descriptor_names = {"BRISK", "BRIEF", "ORB", "FREAK", "SIFT"};

    vector<string> detector_names = {"AKAZE"};
    vector<string> descriptor_names = {"AKAZE"};
    vector<string>::iterator it1, it2;
    string detectorType, descriptorType;

    for (it1 = detector_names.begin(); it1!= detector_names.end(); ++it1)
    {
        if (*it1 == "SIFT")
        {
            descriptor_names.erase(descriptor_names.begin()+2);
        }
        detectorType = *it1;
        for (it2 = descriptor_names.begin(); it2!= descriptor_names.end(); ++it2)
        {
            vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
            descriptorType = *it2;


    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        if (dataBuffer.size()<2)
        {
            dataBuffer.push_back(frame);
        }
        else
        {
            dataBuffer.erase(dataBuffer.begin());
            dataBuffer.push_back(frame);
        }


        //// EOF STUDENT ASSIGNMENT
        //cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        // string detectorType = "ORB";   // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
        // string descriptorType = "SIFT"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, t_detector, false);
        }
        else if (detectorType.compare("HARRIS")== 0)
        {
            detKeypointsHarris(keypoints, imgGray, t_detector, false);
        }
        else
        {
            detKeypointsModern(keypoints, imgGray, detectorType, t_detector, false);
        }
        //// EOF STUDENT ASSIGNMENT
        // cout << "keypoint size is " << keypoints.size();

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        vector<cv::KeyPoint> car_keypoints;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            vector<cv::KeyPoint>::iterator kp_it;
            //for(auto kp : keypoints)
            for (kp_it = keypoints.begin(); kp_it!= keypoints.end(); ++kp_it)
            {
                if(vehicleRect.contains(kp_it->pt)) {
                    car_keypoints.push_back(*kp_it);
                }
            }
        }
        //// EOF STUDENT ASSIGNMENT
        //cout << "keypoint size is " << car_keypoints.size() << endl;
        num_keypoints = car_keypoints.size();
        keypoints = car_keypoints;

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            //if (detectorType.compare("SHITOMASI") == 0)
            // there is no response info, so keep the first 50 as they are sorted in descending quality order
            keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());

            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        //cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType, t_descriptor);
        /*if(descriptorType.compare("SIFT")==0)
        {
            descriptors.convertTo(descriptors, CV_8U);
        }*/
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        //cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            string descriptorTypestr ;
            if (descriptorType.compare("SIFT") == 0)
            {
                descriptorTypestr = "DES_HOG"; // DES_BINARY, DES_HOG

            }
            else{
                descriptorTypestr = "DES_BINARY"; // DES_BINARY,
            }

            string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorTypestr, matcherType, selectorType, num_matches);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            //cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            bVis = false;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images with detector " + detectorType;
                windowName.append(" descriptor ");
                windowName.append(descriptorType);
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                //cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }

        result <<imgIndex <<"," << detectorType << ","<< descriptorType << ","<< t_detector << ","<< t_descriptor<< ","
        << num_keypoints << ","<< num_matches<<","<<","<< t_detector + t_descriptor<<","<< endl;
        /*cout << "num of keypoints on car " << num_keypoints << endl;
        cout << "num of matched keypoints " << num_matches << endl;
        cout << "time taken by detector " << t_detector << endl;
        cout << "time taken by descriptor " << t_descriptor << endl;
        cout << "total time " << t_detector + t_descriptor << endl;
        cout << "---------------------------------------------------" << endl;*/

    } // eof loop over all images
    }
    }

    return 0;
}
