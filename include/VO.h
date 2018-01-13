//

#ifndef VO_H_
#define VO_H_

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

/**
* Visual Odometry
*
* https://docs.opencv.org/3.3.1/d9/d0c/group__calib3d.html
*
*/
class VO
{

  /*=============================         PUBLIC METHODS         =============================*/

public:

  /*====================         PUBLIC METHODS - DE-/CONSTRUCTORS        ====================*/

  /**
  * constructor does nothing
  *
  */
  VO();

  /**
  * standard destructor does nothing
  */
  virtual ~VO();



  /*=============================         PUBLIC METHODS         =============================*/

public:

  /**
  * find features that are present in frames k-2, k-1, and k. It goes through
  * good_matches: A descriptor which is used in frame k-2 as trainIdx and in frame k
  * as queryIndex also exists in frame k-1 as both. Those points exist in all three
  * frames and are returned in features for k-2, k-1 and k. The last entry in 
  * pnts3D_vec which contains 3D points in frame k-1 is reduced so that features at k-1
  * correspond to pnts3D_vec at time k-1. 
  *
  * @param features : vector of features. Contains at least features of frame k-2, k-1 and k.
  * the features of k-2, k-1 and k are changed by this function. Only those are returned
  * which exist in all three frames. 
  * @param keypoints : keypoints in frame k-2, k-1 and k
  * @param good_matches : good_matches between features of frame k-2 and k-1 as well as
  * k-1 and k. 
  * @param index_last_keyframe : TODO: at the moment as to be index of k-2
  * @param pnts3D_vec : Contains at least 3D points in frame k-1. After call contains as much
  * points as features at k-1. 
  *
  */
  static void findMatchingFeatures(std::vector<std::vector<cv::Point2f>> &features,
    const std::vector<std::vector<cv::KeyPoint>> &keypoints,
    const std::vector<std::vector<cv::DMatch>> &good_matches, int index_last_keyframe, 
    std::vector<std::vector<cv::Point3f>> &pnts3D_vec);

  /**
  * Deletes all features, good_matches and pnts3D_vec that are marked by mask with a 0.
  * All four inputs must have same length (good_matches and pnts3D_vec may be empty though, 
  * then they are not changed). Only the last three vectors in features and good_matches
  * are changed (this corresponds to k-2, k-1 and k). 
  *
  * @param features : vector of features over frames ..., k-2, k-1 and k
  * @param mask : mask gotten from epipolar constraint or recoverPose. 0 signalizes an outlier
  * and 1 an inlier. 
  * @param good_matches : matches of features for frames k-2, k-1 as well as k-1 and k as well as 
  * older frames. If empty then nothing is done with good_matches. 
  * @param pnts3D_vec : to features corresponding 3D points, although only for latest frame
  * thus for k-1 or k. 
  *
  */
  static void removeOutlierFeatures(std::vector<std::vector<cv::Point2f>> &features, 
    const cv::Mat &mask, std::vector<std::vector<cv::DMatch>> &good_matches, 
    std::vector<cv::Point3f> &pnts3D_vec);

  /**
  * Keeps all features and pnts3D_vec that are inside inliers_idx. Features of frames
  * k-2, k-1 and k are changed by this function as is the last entry in pnts3d_vec. Only
  * those features and 3d-points survive which are inside inliers_idx. 
  *
  * @param features : vector of features over frames ..., k-2, k-1 and k
  * @param pnts3D_vec : to features corresponding 3D points. 
  * @param inliers_idx : inliers returned by PnP method. 
  *
  */
  static void removePnPoutliers(std::vector<std::vector<cv::Point2f>> &features, 
    /*std::vector<*/std::vector<cv::Point3f>/*>*/ &pnts3D_vec, const cv::Mat &inliers_idx);

  /**
  * Calculates essential matrix as well as relative camera transformation using recoverPose
  * (only if calcPose == true). Thus it can be used to calculate relative camera movement
  * from 2D features or can be used to get features that satisfy the epipolar constraint 
  * (essential matrix) as at the end of the function features are removed which do not 
  * satisfy the epipolar constraint (calling removeOutlierFeatures). 
  *
  * @param index_last_keyframe : not used at the moment
  * @param features : vector of features must contain at least features for two different
  * frames. The last two vectors in features are used. they belong to the two most recent 
  * frames. 
  * @param cameraMatrix : camera matrix
  * @param R : relative rotation matrix calculated in recoverPose
  * @param t : relative translation vector calculated in recoverPose
  * @param calcPose : if true then calls recoverPose. If recoverPose keeps less than 200
  * inliers then only the outliers from the calculation of the essential matrix are deleted. 
  * @param good_matches : good_matches for passed features. They are reduced in 
  * removeOutlierFeatures as well to match in size with features. 
  * @param pnts3D_vec : corresponding 3D points for passed features. They are reduced in 
  * removeOutlierFeatures as well to match in size with features. 
  *
  */
  static int recoverPoseFromEssentialMat(int index_last_keyframe, 
    std::vector<std::vector<cv::Point2f>> &features,
    const cv::Mat &cameraMatrix, cv::Mat &R, cv::Mat &t, bool calcPose, 
    std::vector<std::vector<cv::DMatch>> &good_matches, 
    std::vector<cv::Point3f> &pnts3D_vec, float &inlierRatioPose);

  /**
  * Gets features for last two frames from good_matches. The keypoints of the features
  * must be in keypoints.back() and keypoints.at(keypoints.size() - 2) (the position before
  * back). 
  *
  * @param features : vector of features. the last two positions in the feature vector
  * are made new according to good_matches. If features vector is empty, then afterwards
  * it will have two elements (frames: k-2, k-1). 
  * @param keypoints : keypoints found in last two frames. 
  * @param good_matches : good matches between dscriptors in last two frames. number of 
  * keypoints must match number of good_matches. 
  *
  */
  static void getFeaturesFromGoodMatches(std::vector<std::vector<cv::Point2f>> &features, 
    const std::vector<std::vector<cv::KeyPoint>> &keypoints, 
    const std::vector<cv::DMatch> &good_matches);

  /**
  * Calculates the scale to scale the translation vector with. The scale is defined
  * as the ratio of the distance between 3D points in frame k-1 to the distance of the same
  * 3D points in frame k. This ratio is calculated for a bunch of points, then the median
  * distance is taken. Features from frame k-1 and k are triangulated in this function to
  * get 3D points of frame k. 
  *
  * @param features : features over all frames including k-1 and k
  * @param pnts3D_vec : 3D points of frame k-1
  * @param pose : camera pose at time k-1
  * @param cameraMatrix : camera matrix
  * 
  * @return : scale, being median distance ratio of 3D points in frame k-1 and k
  */
  static float calcScale(const std::vector<std::vector<cv::Point2f>> &features,
    const std::vector<cv::Point3f> &pnts3D_vec, const std::vector<cv::Mat> &pose,
    const cv::Mat &cameraMatrix);

  /**
  * Calculates the scale from ground truth camera positions to scale the translation 
  * vector with. The scale is defined as the ratio of the distance between ground truth
  * positions and estimated camera positions. The median scale is returned taken over all frames. 
  *
  * @param pose : camera pose at all times
  * @param GT_locs : ground truth camera locations
  *
  * @return : scale, being median distance ratio of ground truth positions over 
  * estimated camera positions
  */
  static float calcScaleFromGT(const std::vector<cv::Mat> &pose,
    const std::vector<cv::Point3f> &GT_locs);

  /**
  * Calls calcScale which calculates the scale to scale the translation vector with. 
  * This function then scales the current translation vector of pose.back() according to
  * the calculated scale. 
  * The scale is defined as the ratio of the distance 
  * between 3D points in frame k-1 to the distance of the same
  * 3D points in frame k. This ratio is calculated for a bunch of points, then the median
  * distance is taken. Features from frame k-1 and k are triangulated in this function to
  * get 3D points of frame k.
  *
  * @param features : features over all frames including k-1 and k
  * @param pnts3D_vec : 3D points of frame k-1
  * @param pose : camera pose at time k-1
  * @param cameraMatrix : camera matrix
  * @param scaleVector : vector of scales between 3D points of adjacent frames
  *
  */
  static float correctTranslationForScale(const std::vector<std::vector<cv::Point2f>> &features,
    const std::vector<cv::Point3f> &pnts3D_vec, std::vector<cv::Mat> &pose,
    const cv::Mat &cameraMatrix, std::vector<float> &scaleVector);

  /**
  * Calls calcScaleFromGT which calculates the scale from ground truth location data to 
  * scale the translation vector with.
  * This function then scales all translation vectors in pose according to
  * the calculated scale.
  * 
  * @param pose : camera poses at all times
  * @param GT_locs : ground truth locations
  * 
  */
  static float correctTranslationForScaleFromGT(std::vector<cv::Mat> &pose,
    const std::vector<cv::Point3f> &GT_locs);


  static void posePlausibilityCheck(std::vector<cv::Mat> &pose);

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  /// <summary> calcs median value of given vector
  /// https://stackoverflow.com/questions/1719070/what-is-the-right-approach-when-using-stl-container-for-median-calculation/1719155#1719155
  /// @todo: is wrong if vector has even number of elements
  /// </summary>
  ///
  /// <remarks> Daniel, 01.06.2017. </remarks>
  ///
  /// <param name="v">  The vector to calculate the median from, is changed by nth_element(). </param>
  /// 
  /// <returns> median value of given vector. </returns>
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  static float median(std::vector<float> &v);



};

#endif


