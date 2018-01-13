/*
 * RobustMatcher.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: eriba
 */

#include "../include/RobustMatcher.h"      // this class
#include <time.h>

#include "../include/Debug.h"

#include <opencv2/features2d/features2d.hpp>



/*====================         PUBLIC METHODS - DE-/CONSTRUCTORS        ====================*/

 /**
 * standard constructor creates detector_ and extractor_ for ORB features as well as
 * matcher_ with Flann based matcher using locality sensitive hashing.
 *
 * @param ratioTest : ratio by how much a feature match has to be different
 * from other feature matches to be considered a good match.
 * @param fast_match : if false, then only feature matches are used
 * that want to marry each other. If true, then this symmetry check is not done.
 * @param numKeyPoints : number of keypoints to detect
 *
 */
RobustMatcher::RobustMatcher(float ratioTest, bool fast_match, int numKeyPoints/*, int read_keypoints*/) :
  ratio_(ratioTest), fast_match(fast_match)//, read_keypoints(read_keypoints)
{
  cvIN_FCT;

  int minHessian = 400;

#ifdef SURFfeatures
  detector_ = cv::xfeatures2d::SURF::create(minHessian);
#else
  // ORB is the default feature
  detector_ = cv::ORB::create(numKeyPoints);    // set feature detector
#endif
  extractor_ = cv::ORB::create(numKeyPoints);   // set descriptor extractor

  // https://docs.opencv.org/3.0-alpha/modules/flann/doc/flann_fast_approximate_nearest_neighbor_search.html
  // FLANN : Fast Library for Approximate Nearest Neighbors
  // instantiate LSH index parameters
  // multi-probe LSH (by Multi-Probe LSH: Efficient Indexing for High-Dimensional Similarity Search)
  // locality sensitive hashing
  // TODO: changed from 6 to 10, has a stong effect!!!!
  cv::Ptr<cv::flann::IndexParams> indexParams = cv::makePtr<cv::flann::LshIndexParams>(/*6*/ 10/*20*/, /*12*/12, 2);
  // instantiate flann search parameters
  // changed from 50 to 32, do not know what this parameter is doing, but did not change a thing, 32 is default
  cv::Ptr<cv::flann::SearchParams> searchParams = cv::makePtr<cv::flann::SearchParams>(/*50*/50);

  // instantiate FlannBased matcher
  matcher_ = cv::makePtr<cv::FlannBasedMatcher>(indexParams, searchParams);

  // BruteFroce matcher with Norm Hamming is the default matcher
  //matcher_ = cv::makePtr<cv::BFMatcher>((int)cv::NORM_HAMMING, false);

  cvOUT_FCT;
}

/**
* standard destructor does nothing
*/
RobustMatcher::~RobustMatcher()
{
  // Auto-generated destructor stub
}



/*=============================         PUBLIC METHODS         =============================*/

/**
* Detects keypoints and extracts descriptors in the given frame and matches the extracted
* descriptors with the given descriptors last vector (descriptors.back()) (obtained from
* a previous frame). Matching descriptors are added to the
* good_matches vector. The found descriptors and keypoints are pushed_back at the
* end of the given descriptors and keypoints vectors.
*
* @param frame : the current frame of a camera
* @param descriptors : vector of descriptors. At the end of the vector (back()) must be a
* descriptors Mat containing descriptors of an old (the previous) frame.
* @param keypoints : the found keypoints in the given frame are pushed_back to the end of this
* vector.
* @param good_matches : the found good_matches between the new and previous descriptors are
* pushed_back at the end of this vector.
*
*/
void RobustMatcher::extractKeyPointsAndMatch(const cv::Mat &frame, std::vector<cv::Mat> &descriptors,
  std::vector<std::vector<cv::KeyPoint>> &keypoints, std::vector<std::vector<cv::DMatch>> &good_matches, 
  int counter)
{
  cvIN_FCT;

  cv::Mat descriptors_temp;
  std::vector<cv::DMatch> good_matches_temp;
  std::vector<cv::KeyPoint> keypoints_temp;

  if (fast_match)
  {
    descriptors_temp =
      fastRobustMatch(frame, good_matches_temp, keypoints_temp, descriptors.back(), counter);
  }
  else
  {
    descriptors_temp =
      robustMatch(frame, good_matches_temp, keypoints_temp, descriptors.back(), counter);
  }

  keypoints.push_back(keypoints_temp);
  descriptors.push_back(descriptors_temp);
  good_matches.push_back(good_matches_temp);

#ifndef FULLVERSION_D
  cvCTRACE << "found " << good_matches.back().size() <<
    " matching features in frame k-1 and k" << std::endl;
#endif

  cvOUT_FCT;
}

/**
* Detects keypoints and extracts descriptors in the given frame. The found descriptors
* and keypoints are pushed_back at the
* end of the given descriptors and keypoints vectors. For the very first frame call this method,
* for all subsequent frames call the method extractKeyPointsAndMatch() instead.
*
* @param frame : the current frame of a camera
* @param descriptors : the found descriptors in the given frame are pushed_back to the end of this
* vector.
* @param keypoints : the found keypoints in the given frame are pushed_back to the end of this
* vector.
*
*/
void RobustMatcher::extractKeyPointsAndDescriptors(const cv::Mat &frame, std::vector<cv::Mat> &descriptors,
  std::vector<std::vector<cv::KeyPoint>> &keypoints, int counter)
{
  cvIN_FCT;

  std::vector<cv::KeyPoint> keypoints_temp;
  cv::Mat descriptors_temp;

  // Detection of the ORB features
  computeKeyPoints(frame, keypoints_temp, counter);

  // Extraction of the ORB descriptors
  computeDescriptors(frame, keypoints_temp, descriptors_temp);
  // add features to vectors (k-2)
  keypoints.push_back(keypoints_temp);
  descriptors.push_back(descriptors_temp);

  cvOUT_FCT;
}


