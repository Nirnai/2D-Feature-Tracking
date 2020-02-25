# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 3.4
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.2
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.


## Writeup

### MP.1
By erasing the first element of a vector the other elements are automatically shifted left. This implementation allows to keep the vector as a container.
```c++
dataBuffer.reserve(dataBufferSize);
...
// RingBuffer Implementation
if(dataBuffer.size() >= dataBufferSize)
    dataBuffer.erase(dataBuffer.begin());
dataBuffer.push_back(frame);
```

### MP.2 
Keypoint Detection
``` c++
  cv::Ptr<cv::FeatureDetector> detector;
  if(detectorType.compare("FAST") == 0)
  {
      detector = cv::FastFeatureDetector::create();
  }
  else if(detectorType.compare("BRISK") == 0)
  {
      detector = cv::BRISK::create();
  }
  else if(detectorType.compare("ORB") == 0)
  {
      detector = cv::ORB::create();
  }
  else if(detectorType.compare("AKAZE") == 0)
  {
      detector = cv::AKAZE::create();
  }
  else if(detectorType.compare("SIFT") == 0)
  {
      detector = cv::xfeatures2d::SIFT::create();
  }
```
### MP.3 Keypoint Removal
Filtering all keypoints with a mask
```c++
// only keep keypoints on the preceding vehicle
bool bFocusOnVehicle = true;
cv::Rect vehicleRect(535, 180, 180, 150);
if (bFocusOnVehicle)
{
  cv::Mat mask = cv::Mat::zeros(imgGray.rows, imgGray.cols, CV_8U);
  mask(vehicleRect) = 1;
  cv::KeyPointsFilter::runByPixelsMask(keypoints, mask);
}
```


### MP.4 Descriptor
```c++
// select appropriate descriptor
cv::Ptr<cv::DescriptorExtractor> extractor;
if (descriptorType.compare("BRISK") == 0)
{

    int threshold = 30;        // FAST/AGAST detection threshold score.
    int octaves = 3;           // detection octaves (use 0 to do single scale)
    float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

    extractor = cv::BRISK::create(threshold, octaves, patternScale);
}
else if (descriptorType.compare("BRIEF") == 0)
{
    extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
}
else if (descriptorType.compare("ORB") == 0)
{
    extractor = cv::ORB::create();
}
else if (descriptorType.compare("FREAK") == 0)
{
    extractor = cv::xfeatures2d::FREAK::create();
}
else if (descriptorType.compare("AKAZE") == 0)
{
    extractor = cv::AKAZE::create();
}
else if (descriptorType.compare("SIFT") == 0)
{
    extractor = cv::xfeatures2d::SIFT::create();
}
// perform feature description
extractor->compute(img, keypoints, descriptors);
```


### MP.5 Matching
```c++
// configure matcher
bool crossCheck = false;
cv::Ptr<cv::DescriptorMatcher> matcher;

if (matcherType.compare("MAT_BF") == 0)
{
    int normType;
    if(descriptorType.compare("DES_BINARY") == 0)
        normType = cv::NORM_HAMMING;
    else
        normType = cv::NORM_L2;
    matcher = cv::BFMatcher::create(normType, crossCheck);
}
else if (matcherType.compare("MAT_FLANN") == 0)
{
    if(descSource.type() != CV_32F)
        descSource.convertTo(descSource, CV_32F);
    if(descRef.type() != CV_32F)
        descRef.convertTo(descRef, CV_32F);
    matcher = cv::FlannBasedMatcher::create();
}
```

### MP.6 Descriptor Distance Ratio

```c++
// k nearest neighbors (k=2)
std::vector<std::vector<cv::DMatch>> knn_matches;
matcher->knnMatch(descSource, descRef, knn_matches, 2);
for(auto& knn_match : knn_matches)
{
    if(knn_match[0].distance < 0.8 * knn_match[1].distance)
        matches.push_back(knn_match[0]);
}  
```

###  MP.7 Performance Evaluation 1

Number of points in a bounding box, the neighbourhood and the time for finding the keypoints can be seen in: `results/detectors.csv`

1. FAST
2. BRISK 
3. AKAZE 


###  MP.8 Performance Evaluation 2

The Number of Matches can be found for the different combinations in: `results/matches.csv`. This matches were avarages over the 10 frames. The x-axis shows the detector, and the y-axis the discriptor.

1. FAST - BRIEF
2. FAST - SIFT
3. FAST - ORB

It should be noted that AKAZE Descriptor only works with AKAZE Detector and the SIFT detector does not work with ORB Descriptors.   

###  MP.9 Performance Evaluation 3

The processing time of detectors can be found in the detector file and the timing of the best performing descriptors can be found in the descriptor file:
`results/detectors.csv`
`results/descriptors.csv`

1. FAST - BRIEF (Perform the best and are the fastest)
2. FAST - ORF (High performance and second fastest)
3. FAST - SIFT (High Perfromance but SIFT descriptor is slow)


