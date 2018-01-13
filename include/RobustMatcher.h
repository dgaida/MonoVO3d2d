/*
 * RobustMatcher.h
 *
 *  Created on: Jun 4, 2014
 *      Author: eriba
 */

#ifndef ROBUSTMATCHER_H_
#define ROBUSTMATCHER_H_

#include <iostream>

//#include "opencv2/core.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "opencv2/xfeatures2d.hpp"        // SURF features

// if defined, then use SURF features, else use ORB features
#define SURFfeatures
//#undef SURFfeatures


 /**
 * ORB detector
 *
 * uniformly distributed features are implemented in computeKeyPoints
 *
 */
class RobustMatcher 
{
  /*=============================         PUBLIC METHODS         =============================*/

public:

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
  RobustMatcher(float ratioTest = 0.8f, bool fast_match= true, int numKeyPoints= 3000/*, 
    int read_keypoints= 0*/);
  
  /**
  * standard destructor does nothing
  */
  virtual ~RobustMatcher();



  /*=============================         PUBLIC METHODS - SET METHODS        =============================*/

public:

  // Set the feature detector
  //void setFeatureDetector(const cv::Ptr<cv::FeatureDetector>& detect) 
  //{  
  //  detector_ = detect; 
  //}

  //// Set the descriptor extractor
  //void setDescriptorExtractor(const cv::Ptr<cv::DescriptorExtractor>& desc) 
  //{ extractor_ = desc; }

  //// Set the matcher
  //void setDescriptorMatcher(const cv::Ptr<cv::DescriptorMatcher>& match) 
  //{  matcher_ = match; }



  /*=============================         PUBLIC METHODS         =============================*/

public:

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
  void extractKeyPointsAndMatch(const cv::Mat &frame, std::vector<cv::Mat> &descriptors,
    std::vector<std::vector<cv::KeyPoint>> &keypoints, 
    std::vector<std::vector<cv::DMatch>> &good_matches, int counter);

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
  void extractKeyPointsAndDescriptors(const cv::Mat &frame, std::vector<cv::Mat> &descriptors,
    std::vector<std::vector<cv::KeyPoint>> &keypoints, int counter);
  
  

  /*=============================         PRIVATE METHODS         =============================*/

private:

  // Match feature points using ratio and symmetry test
  cv::Mat robustMatch(const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
    std::vector<cv::KeyPoint>& keypoints_frame,
    const cv::Mat& descriptors_model, int counter);

  // Match feature points using ratio test
  cv::Mat fastRobustMatch(const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
    std::vector<cv::KeyPoint>& keypoints_frame,
    const cv::Mat& descriptors_model, int counter);

  // Compute the keypoints of an image
  void computeKeyPoints(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, int counter);

  // Compute the descriptors of an image given its keypoints
  void computeDescriptors(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
    cv::Mat& descriptors);

  // Clear matches for which NN ratio is > than threshold
  // return the number of removed points
  // (corresponding entries being cleared,
  // i.e. size will be 0)
  int ratioTest(std::vector<std::vector<cv::DMatch> > &matches) const;

  // Insert symmetrical matches in symMatches vector
  void symmetryTest(const std::vector<std::vector<cv::DMatch> >& matches1,
    const std::vector<std::vector<cv::DMatch> >& matches2,
    std::vector<cv::DMatch>& symMatches) const;

  void adaptiveNonMaximalSuppression(std::vector<cv::KeyPoint>& keypoints,
    const int numToKeep) const;



  /*=============================        PUBLIC VARIABLES        =============================*/


  /*===========================        PROTECTED VARIABLES        ===========================*/

protected:



  /*=============================       PRIVATE VARIABLES        =============================*/

private:

#ifdef SURFfeatures
  // pointer to the feature point detector object
  cv::Ptr<cv::xfeatures2d::SurfFeatureDetector> detector_;
#else
  // pointer to the feature point detector object
  cv::Ptr<cv::FeatureDetector> detector_;
#endif
  // pointer to the feature descriptor extractor object
  cv::Ptr<cv::DescriptorExtractor> extractor_;
  // pointer to the matcher object
  cv::Ptr<cv::DescriptorMatcher> matcher_;
  
  // max ratio between 1st and 2nd NN
  const float ratio_;

  // if false, then call robustMatch() which does a symmetry test as well. If true then
  // fastRobustMatch is called instead. 
  const bool fast_match;

  //const int numKeyPoints = 3000;// 2000;      // number of detected keypoints

};

#endif /* ROBUSTMATCHER_H_ */
