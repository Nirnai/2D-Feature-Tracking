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


void run(std::string detector, std::string descriptor)
{
    std::cout << "Detector: " << detector << std::endl;
    std::cout << "Descriptor: " << descriptor << std::endl;
    std::string dataPath = "../";
    std::string imgBasePath = dataPath + "images/";
    std::string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    std::string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    std::vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    dataBuffer.reserve(dataBufferSize); // optimizes vector usage
    bool bVis = false;            // visualize results
    float num_matches = 0;
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        std::ostringstream imgNumber;
        imgNumber << std::setfill('0') << std::setw(imgFillWidth) << imgStartIndex + imgIndex;
        std::string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
        DataFrame frame;
        frame.cameraImg = imgGray;
        // RingBuffer Implementation
        if(dataBuffer.size() >= dataBufferSize)
            dataBuffer.erase(dataBuffer.begin());
        dataBuffer.push_back(frame);


        /* DETECT IMAGE KEYPOINTS */
        std::vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        // string detectorType = "HARRIS";
        if (detector.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        else if(detector.compare("HARRIS") == 0)
        {
            detKeypointsHarris(keypoints, imgGray, false);
        }
        else
        {
            detKeypointsModern(keypoints, imgGray, detector);
        }

        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            cv::Mat mask = cv::Mat::zeros(imgGray.rows, imgGray.cols, CV_8U);
            mask(vehicleRect) = 1;
            cv::KeyPointsFilter::runByPixelsMask(keypoints, mask);
            float n = 0;
            for(auto& kp: keypoints)
            {
                n += kp.size;
            }

            // std::cout<<"Number Keypoints: " << keypoints.size() << std::endl;
            // std::cout<<"Neigh: " << n/keypoints.size() << std::endl;
            // std::cout<<"--------------------------------------" << std::endl;
        }
        (dataBuffer.end() - 1)->keypoints = keypoints;



        /* EXTRACT KEYPOINT DESCRIPTORS */
        cv::Mat descriptors;
        // string descriptorType = "BRISK"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptor);
        (dataBuffer.end() - 1)->descriptors = descriptors;


        /* MATCH KEYPOINT DESCRIPTORS */
        if (dataBuffer.size() > 1)
        {
            std::vector<cv::DMatch> matches;
            std::string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            std::string descriptorType;
            if (descriptor.compare("SIFT") == 0)
                descriptorType = "DES_HOG";
            else 
                descriptorType = "DES_BINARY";
            std::string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN
            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType);
            (dataBuffer.end() - 1)->kptMatches = matches;
            num_matches += matches.size();
        }
    }
    std::cout << "Average Number of Matches : " << num_matches/10 << std::endl;
    std::cout << "----------------------------------------------" << std::endl;
}


int main()
{   
    std::vector<std::string> detectors { "HARRIS", "SHITOMASI", "FAST", "BRISK", "ORB", "SIFT" };
    std::vector<std::string> descriptors { "FREAK", "BRIEF", "BRISK", "ORB", "SIFT" };
    // for (auto detector : detectors)
    // {
    //     // run(detector, "FREAK");
    //     for (auto descriptor: descriptors)
    //         run(detector, descriptor);
    // }
    run("AKAZE", "AKAZE");
    
}