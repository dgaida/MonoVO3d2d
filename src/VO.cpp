// Implementation OK
// Documentation OK
// 13.01.2018, Daniel Gaida

#include "../include/VO.h"
#include "../include/PnPProblem.h"

#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include <numeric>

#include "../include/Debug.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>



/*====================         PUBLIC METHODS - DE-/CONSTRUCTORS        ====================*/

/**
* constructor does nothing
*
*/
VO::VO()
{
}


/**
* standard destructor does nothing
*/
VO::~VO()
{
}



/*=============================         PUBLIC METHODS         =============================*/

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
void VO::findMatchingFeatures(std::vector<std::vector<cv::Point2f>> &features,
  const std::vector<std::vector<cv::KeyPoint>> &keypoints,
  const std::vector<std::vector<cv::DMatch>> &good_matches, int index_last_keyframe,
  std::vector<std::vector<cv::Point3f>> &pnts3D_vec)
{
  cvIN_FCT;

  // TODO: this method does not work yet properly if index_last_keyframe is further away than k-2

  // make space for features at time k
  features.push_back(std::vector<cv::Point2f>());

  if (features.size() < 3)
    return;

  if (good_matches.size() < 2)
    return;

  if (keypoints.size() < 3)
    return;

  if (features.at(index_last_keyframe).size() != features.at(features.size() - 2).size())
    return;

 /* if (features.at(index_last_keyframe).size() >
    good_matches.at(index_last_keyframe).size())
    return;*/

  // clear features of k-1, because below we get new features from keypoints at k-1
  features.at(index_last_keyframe).clear();
  features.at(features.size() - 2).clear();

  std::vector<cv::Point3f> pnts3D_temp;

  unsigned int match_index;

  // for each match between frame k-2 and k-1
  // good_matches is sorted with queryIndex which comes from frame k-1
  for (unsigned int in_pos3D = 0; in_pos3D < good_matches.at(index_last_keyframe).size(); in_pos3D++)
  {
    // for each match between frame k-1 and k
    for (match_index = 0; match_index < good_matches.back().size(); ++match_index)
    {
      // if a point in frame k-1 is a match in k-2 and k-1 as well as k-1 and k, then we have this
      // point in all three frames and we take this
      // TODO: maybe we can make this faster, once we are behin with the first sorted value
      if (good_matches.at(index_last_keyframe)[in_pos3D].queryIdx ==
        good_matches.back()[match_index].trainIdx)
      {
        if (!pnts3D_vec.empty())
          // add 3d point at pos in_pos3D to a vector
          // 3d point must be sorted as is good_matches.at(index_last_keyframe)
          pnts3D_temp.push_back(pnts3D_vec.back().at(in_pos3D));
        
        break;
      }
    }
    // if we did not find the feature, then discard the feature
    if (match_index == good_matches.back().size())
      continue;

    // add features of frame k-1

    if (good_matches.at(index_last_keyframe)[in_pos3D].trainIdx >= 
      keypoints.at(index_last_keyframe).size())
      continue;

    if (good_matches.back()[match_index].trainIdx >= keypoints.at(keypoints.size() - 2).size())
      continue;

    if (good_matches.back()[match_index].queryIdx >= keypoints.back().size())
      continue;

    cv::Point2f point2d_scene = keypoints.at(index_last_keyframe)[
      good_matches.at(index_last_keyframe)[in_pos3D].trainIdx].pt; // 2D point from the scene
    features.at(index_last_keyframe).push_back(point2d_scene);         // add 2D point

    point2d_scene = keypoints.at(keypoints.size() - 2)[
      good_matches.back()[match_index].trainIdx].pt; // 2D point from the scene
    features.at(features.size() - 2).push_back(point2d_scene);         // add 2D point

    // add features of latest frame to the end of features
    point2d_scene = keypoints.back()[good_matches.back()[match_index].queryIdx].pt; // 2D point from the scene
    features.back().push_back(point2d_scene);         // add 2D point
  }

  if (!pnts3D_vec.empty())
    // copy the corresponding 3d points into vector
    pnts3D_vec.back() = pnts3D_temp;

  //for (int ifeat = 0; ifeat < features.size(); ifeat++)
    //std::cout << features.at(ifeat).size() << std::endl;

  cvOUT_FCT;
}

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
void VO::removeOutlierFeatures(std::vector<std::vector<cv::Point2f>> &features,
  const cv::Mat &mask, std::vector<std::vector<cv::DMatch>> &good_matches, 
  std::vector<cv::Point3f> &pnts3D_vec)
{
  cvIN_FCT;

  int imask = 0;
  int imask2 = 0;

  //std::cout << mask << std::endl;

  int n_features = features.at(features.size() - 1).size();

  while (imask < features.at(features.size() - 1).size())
  {
    if (mask.at<uchar>(imask2) == 0)
    {
      int start_idx = 0;
      // only delete features and good_matches from last 3 frames (k-2, k-1 and k)
      if (features.size() > 3)
        start_idx = features.size() - 3;

      if (features.at(start_idx).size() < features.back().size())
        start_idx++;

      for (int ifeat = start_idx; ifeat < features.size(); ifeat++)
      {
        //if (imask < features.at(ifeat).size())
        features.at(ifeat).erase(features.at(ifeat).begin() + imask);

        // there is always one less in good_matches than in features
        if (ifeat < good_matches.size())
          good_matches.at(ifeat).erase(good_matches.at(ifeat).begin() + imask);
      }

      if (!pnts3D_vec.empty())
        pnts3D_vec.erase(pnts3D_vec.begin() + imask);
    }
    else
      imask++;

    imask2++;
  }

#ifndef FULLVERSION_D
  cvCTRACE << "deleted " << n_features - features.at(features.size() - 1).size() <<
    " of " << n_features << " features." << std::endl;
#endif

  cvOUT_FCT;
}

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
void VO::removePnPoutliers(std::vector<std::vector<cv::Point2f>> &features,
  /*std::vector<*/std::vector<cv::Point3f>/*>*/ &pnts3D_vec, const cv::Mat &inliers_idx)
{
  std::vector<cv::Point3f> pnts3D_vec_temp;
  std::vector<cv::Point2f> features_temp_k;
  std::vector<cv::Point2f> features_temp_k_1;
  std::vector<cv::Point2f> features_temp_k_2;

  pnts3D_vec_temp.reserve(inliers_idx.rows);
  features_temp_k.reserve(inliers_idx.rows);
  features_temp_k_1.reserve(inliers_idx.rows);
  features_temp_k_2.reserve(inliers_idx.rows);

  // -- Step 4: Catch the inliers keypoints to draw
  for (int inliers_index = 0; inliers_index < inliers_idx.rows; ++inliers_index)
  {
    int n = inliers_idx.at<int>(inliers_index);         // i-inlier
    cv::Point2f point2d = features.back()[n]; // i-inlier point 2D
    features_temp_k.push_back(point2d);           // add i-inlier to list

    point2d = features.at(features.size() - 2)[n]; // i-inlier point 2D
    features_temp_k_1.push_back(point2d);           // add i-inlier to list

    point2d = features.at(features.size() - 3)[n]; // i-inlier point 2D
    features_temp_k_2.push_back(point2d);           // add i-inlier to list

    cv::Point3f point3d = pnts3D_vec[n]; // i-inlier point 2D
    pnts3D_vec_temp.push_back(point3d);           // add i-inlier to list
  }

  features.back() = features_temp_k;
  features[features.size() - 2] = features_temp_k_1;
  features[features.size() - 3] = features_temp_k_2;
  pnts3D_vec= pnts3D_vec_temp;

}

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
* @param inlierRatioPose : ratio of inliers for pose calculation are returned in this variable
*
* @return number of good points
*/
int VO::recoverPoseFromEssentialMat(int index_last_keyframe, 
  std::vector<std::vector<cv::Point2f>> &features,
  const cv::Mat &cameraMatrix, cv::Mat &R, cv::Mat &t, bool calcPose, 
  std::vector<std::vector<cv::DMatch>> &good_matches, 
  std::vector<cv::Point3f> &pnts3D_vec, float &inlierRatioPose)
{
  cvIN_FCT;

  cv::Mat E, mask;

  int sum_mask;
  
  // are going to save shuffled features of features.back() and the one before
  std::vector<cv::Point2f> feature_1;
  std::vector<cv::Point2f> feature_2;
  std::vector<int> indexes;     // shuffle indices

  int goodpnts = -1;
  // recoverPose has two overloads, if true, then recoverPose is called with a depth value
  // which for benchmark seems to be essential. For real world experiments however the given
  // depth value does not seem to work. if first call of recoverPose does not work well (too few inliers)
  // then algorithm automatically sets this parameter to false. 
  bool do_recoverWithDepth= true;

  // call calc essential matrix and recover pose 100 times. the reason is that essential matrix
  // is estimated from five points. as estimate can be totally wrong, we have to calculate essential
  // matrix plenty of times with randomly shuffled features to always select 5 different features.
  // once recoverPose has 80 % inliers break the loop
  for (int iiter = 0; iiter < 100; iiter++)
  {
    if (iiter == 99)
    {
#ifndef SPEED_OPTIM
      cvCTRACE << "iter == 99" << std::endl;
#endif
      //exit(-1);
      return -1;
    }
    // select 6 (I think only 5 are needed) points randomly from features

    indexes.clear();

    if (features.size() < 2)
    {
#ifndef SPEED_OPTIM
      cvCTRACE << "no features" << std::endl;
#endif
      return goodpnts;
    }
    
    indexes.reserve(features.back().size());
    for (int i = 0; i < features.back().size(); ++i)
      indexes.push_back(i);
    //std::iota(indexes.begin(), indexes.end(), 0);
    // obtain a time-based seed:
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::shuffle(indexes.begin(), indexes.end(), std::default_random_engine(seed));

    feature_1.clear();
    feature_2.clear();
    feature_1.reserve(features.back().size());
    feature_2.reserve(features.back().size());

    if (features.back().size() != features.at(features.size() - 2).size())
    {
#ifndef SPEED_OPTIM
      cvCTRACE << "features not same size" << std::endl;
#endif
      return -1;
    }

    for (int ip = 0; ip < features.back().size(); ip++)
    {
      feature_1.push_back(features.back().at(indexes.at(ip)));
      feature_2.push_back(features.at(features.size() - 2).at(indexes.at(ip)));
    }

    if (feature_1.empty() || feature_2.empty())
    {
#ifndef SPEED_OPTIM
      cvCTRACE << "no features in this or previous frame" << std::endl;
#endif
      return goodpnts;
    }

    // this is exactly the first step of Visual Odometry in Nister 03
    E = cv::findEssentialMat(feature_2, feature_1, cameraMatrix,
      /*cv::LMEDS*/cv::RANSAC, 0.999, 1.0/*pixel_acc*/, mask);
    
    //cvCTRACE << "essential Matrix: " << E << std::endl;

    sum_mask = 0;

    for (int imask2 = 0; imask2 < mask.size().height; imask2++)
      if (mask.at<uchar>(imask2) == 0)
        sum_mask++;

#ifndef FULLVERSION_D
    std::cout << "  findEssentialMat: number of outliers: " << sum_mask << " of " <<
      mask.size().height << std::endl;
#endif

    inlierRatioPose = (float)(mask.size().height - sum_mask) / (float)mask.size().height;

    // if mask from essential mat contains too few features, then calc essential matrix again
    if (inlierRatioPose < 0.3f)
      continue;


    if (mask.size().height == 0)
    {
#ifndef SPEED_OPTIM
      cvCTRACE << "mask is empty" << std::endl;
#endif
      return goodpnts;
    }

    if (E.cols != 3 || E.rows != 3)
    {
#ifndef SPEED_OPTIM
      cvCTRACE << "E is empty" << std::endl;
#endif
      return goodpnts;
    }

    // to call correctMatches does not change estimated pose in recoverPose at all
    //cv::Mat F = findFundamentalMat(feature_2, feature_1);

    /*if (F.cols == 0)
    {
      cvCTRACE << "F is empty" << std::endl;
      return;
    }*/

    //correctMatches(F, feature_2, feature_1, feature_2, feature_1);

    if (calcPose)
    {
      cv::Mat pnts3D_temp;

      // This function decomposes an essential matrix using decomposeEssentialMat() and 
      // then verifies possible pose hypotheses by doing cheirality check. The cheirality check 
      // basically means that the triangulated 3D points should have positive depth. 
      // Some details can be found in [Nister03].
      // An efficient solution to the five-point relative pose problem
        
      if (do_recoverWithDepth)
        goodpnts = cv::recoverPose(E, feature_2, feature_1, cameraMatrix, R, t,
          750.0f/*500.0f*/, mask, pnts3D_temp);/**/

      if (goodpnts < 5 || do_recoverWithDepth == false)
      {
        do_recoverWithDepth = false;
        goodpnts = cv::recoverPose(E, feature_2, feature_1, cameraMatrix, R, t, mask);

#ifndef SPEED_OPTIM
        cvCTRACE << "goodpnts: " << goodpnts << std::endl;
#endif
      }
        
      //cvCTRACE << "t: " << t << std::endl;
      //cvCTRACE << "R: " << R << std::endl;
#ifndef FULLVERSION_D
      cvCTRACE << "goodpnts: " << goodpnts << std::endl;
#endif
      //cvCTRACE << "pnts3D_temp: " << pnts3D_temp << std::endl;

      sum_mask = 0;

      for (int imask2 = 0; imask2 < mask.size().height; imask2++)
        if (mask.at<uchar>(imask2) == 0)
          sum_mask++;

#ifndef FULLVERSION_D
      std::cout << "  recoverPose: number of outliers: " << sum_mask << " of " <<
        mask.size().height << std::endl;
#endif
    }

    inlierRatioPose = (float)(mask.size().height - sum_mask) / (float)mask.size().height;

    // if mask from pose contains too few features, then discard it and use mask from Ess matrix
    // Mit einer absoluten Zahl funktioniert es bisher besser, als wie mit einem Verhältnis
    //if (mask.size().height - sum_mask < 200/*200*/)//230)
    if (inlierRatioPose > /*0.8f*/0.8f)
      break;

    if (inlierRatioPose > /*0.8f*/0.75f && iiter > 50)
    {
#ifndef SPEED_OPTIM
      cvCTRACE << "breaked early because of iteration: " << iiter << std::endl;
#endif
      break;
    }
    if (inlierRatioPose > /*0.8f*/0.7f && iiter > 70)
    {
#ifndef SPEED_OPTIM
      cvCTRACE << "breaked early because of iteration: " << iiter << std::endl;
#endif
      break;
    }
  }

  // good_matches have to be resorted according to indexes

  if (good_matches.size() > 0)
    if (good_matches.back().size() == features.back().size())
    {
      std::vector<cv::DMatch> good_match_last;
      good_match_last.reserve(good_matches.back().size());

      for (int ip = 0; ip < features.back().size(); ip++)
      {
        good_match_last.push_back(good_matches.back().at(indexes.at(ip)));
      }

      good_matches.back() = good_match_last;
    }

  features.back() = feature_1;
  features.at(features.size() - 2) = feature_2;
  
  // features at k-2 has to be resorted according to indexes
  if (features.size() >= 3)
  {
    std::vector<cv::Point2f> feature_3;
    feature_3.reserve(features.back().size());

    // TODO: instead of resorting features at k-2 have to resort features at last keyframe
    if (features.at(features.size() - 3).size() != features.back().size())
    {
#ifndef SPEED_OPTIM
      cvCTRACE << "feature size mismatch" << std::endl;
#endif
      return -1;
    }

    for (int ip = 0; ip < features.back().size(); ip++)
    {
      feature_3.push_back(features.at(features.size() - 3).at(indexes.at(ip)));
    }

    features.at(features.size() - 3) = feature_3;
  }

  // remove features that are marked as outliers by findEssentialMat and recoverPose
  removeOutlierFeatures(features, mask, good_matches, pnts3D_vec);

  return goodpnts;

  cvOUT_FCT;
}

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
void VO::getFeaturesFromGoodMatches(std::vector<std::vector<cv::Point2f>> &features,
  const std::vector<std::vector<cv::KeyPoint>> &keypoints, 
  const std::vector<cv::DMatch> &good_matches)
{
  // if called for the first time
  if (features.size() == 0)
  {
    // features for frame k-2 and k-1
    features.push_back(std::vector<cv::Point2f>());
    features.push_back(std::vector<cv::Point2f>());
  }
  else
  { // delete old features
    features.back().clear();
    features.at(features.size() - 2).clear();
  }

  features.back().reserve(good_matches.size());
  features.at(features.size() - 2).reserve(good_matches.size());

  for (unsigned int match_index = 0; match_index < good_matches.size(); ++match_index)
  {
    // add features of latest frame to the end of features (k-1)
    cv::Point2f point2d_scene = keypoints.back()[good_matches[match_index].queryIdx].pt; // 2D point from the scene
    features.back().push_back(point2d_scene);         // add 2D point

    // (k-2)
    point2d_scene = keypoints.at(keypoints.size() - 2)[good_matches[match_index].trainIdx].pt;      // features at k-2
    features.at(features.size() - 2).push_back(point2d_scene);         // add 2D point
  }

}

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
float VO::calcScale(const std::vector<std::vector<cv::Point2f>> &features,
  const std::vector<cv::Point3f> &pnts3D_vec, const std::vector<cv::Mat> &pose, 
  const cv::Mat &cameraMatrix)
{
  float scale = 0;
  //int good3dpoints;

  // TODO: evtl. besser den vektor als refrenz an triangulate funktion geben. evtl. schneller. mal testen
  std::vector<cv::Point3f> pnts3D_vec_temp;
  //cv::Mat pnts3D_temp;

  pnts3D_vec_temp = PnPProblem::triangulatePointsConvEuclidian(
    pose.at(pose.size() - 2), pose.back(), cameraMatrix,
    features.at(features.size() - 2), features.back()//,
    /*pnts3D_temp, good3dpoints*/);

  float dist_k, dist_k_1 = 0.0f;

  std::vector<float> scales;
  scales.reserve(pnts3D_vec.size() - 6);

  for (int ipoint = 0; ipoint < pnts3D_vec.size() - 6; ipoint++)
  {
    dist_k_1= sqrt( pow(pnts3D_vec.at(ipoint).x - pnts3D_vec.at(ipoint + 5).x, 2) +
      pow(pnts3D_vec.at(ipoint).y - pnts3D_vec.at(ipoint + 5).y, 2) +
      pow(pnts3D_vec.at(ipoint).z - pnts3D_vec.at(ipoint + 5).z, 2) );

    dist_k = sqrt(pow(pnts3D_vec_temp.at(ipoint).x - pnts3D_vec_temp.at(ipoint + 5).x, 2) +
      pow(pnts3D_vec_temp.at(ipoint).y - pnts3D_vec_temp.at(ipoint + 5).y, 2) +
      pow(pnts3D_vec_temp.at(ipoint).z - pnts3D_vec_temp.at(ipoint + 5).z, 2));

    scales.push_back(dist_k_1 / dist_k);
  }

  scale = median(scales);

  return scale;
}

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
float VO::calcScaleFromGT(const std::vector<cv::Mat> &pose,
  const std::vector<cv::Point3f> &GT_locs)
{
  std::vector<float> scales;
  scales.reserve(pose.size());

  for (int ipose = 0; ipose < pose.size(); ipose++)
  {

    cv::Mat t_est = PnPProblem::get_trans_part(pose.at(ipose));
    // GT_locs contains the very first location eye, 000, but pose does not, so have to add 1
    cv::Point3f t_GT = GT_locs.at(ipose+1);

    float magn_est = sqrt( pow(t_est.at<double>(0),2) + pow(t_est.at<double>(1),2) + 
                           pow(t_est.at<double>(2), 2) );
    float magn_GT = sqrt(pow(t_GT.x, 2) + pow(t_GT.y, 2) + pow(t_GT.z, 2));

    scales.push_back( magn_GT / magn_est );

  }

  float median_scale = median(scales);
  
  return median_scale;
}

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
float VO::correctTranslationForScaleFromGT(std::vector<cv::Mat> &pose,
  const std::vector<cv::Point3f> &GT_locs)
{

  float median_scale = calcScaleFromGT(pose, GT_locs);

  for (int ipose = 0; ipose < pose.size(); ipose++)
  {
    // change pose.back according to scale
    cv::Mat t = median_scale * PnPProblem::get_trans_part(pose.at(ipose));
    PnPProblem::set_t_vector(pose.at(ipose), t);
  }

  return median_scale;
}

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
float VO::median(std::vector<float> &v)
{
  // if only two values, then take mean value, because code below returns the 2nd value.
  if (v.size() == 2)
    return 1.0f / 2.0f * (v.at(0) + v.at(1));
  else if (v.size() == 4)
    return 1.0f / 4.0f * (v.at(0) + v.at(1) + v.at(2) + v.at(3));

  size_t n = v.size() / 2;
  // Rearranges the elements in the range [first,last), in such a way that the element 
  // at the nth position is the element that would be in that position in a sorted sequence
  std::nth_element(v.begin(), v.begin() + n, v.end());
  return v[n];
}

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
*
*/
float VO::correctTranslationForScale(const std::vector<std::vector<cv::Point2f>> &features,
  const std::vector<cv::Point3f> &pnts3D_vec, std::vector<cv::Mat> &pose,
  const cv::Mat &cameraMatrix, std::vector<float> &scaleVector)
{

  float scale = VO::calcScale(features, pnts3D_vec, pose, cameraMatrix);

  scaleVector.push_back(scale);

  float median_scale = median(scaleVector);

  //cvCTRACE << "median scale: " << median_scale << std::endl;

  // in VO 3D-2D I should not use scale, because PnP calculates the absolute camera 
  // transformation and not a relative transformation. Only use it in 2D-2D VO. 
  // TODO: change to true
  //if (pose.size() >= 3)//(false)//(scale < 2.0f) // TODO: I do not know
  {
    for (int ipose = 0; ipose < pose.size(); ipose++)
    {
      // change pose.back according to scale
      cv::Mat t = median_scale * PnPProblem::get_trans_part(pose.at(ipose));
      PnPProblem::set_t_vector(pose.at(ipose), t);
    }
  }

  return median_scale;
}

/**
* Checks whether the new calculated pose in pose.back() is similar to the previous one. If
* not then the last pose is replaced by the previous or a predicted pose out of the
* previous two poses. tranlation and rotation components of pose are checked and changed
* separately.
*
* @param pose : camera poses at all times
*
*/
void VO::posePlausibilityCheck(std::vector<cv::Mat> &pose)
{

  if (pose.size() > 10)
  {
    cv::Mat pose_k = pose.back();
    cv::Mat pose_k_1 = pose.at(pose.size() - 2);

    cv::Mat rvec_k, rvec_k_1;

    cv::Rodrigues(PnPProblem::get_rot_part(pose_k), rvec_k);
    cv::Rodrigues(PnPProblem::get_rot_part(pose_k_1), rvec_k_1);

    if (norm(rvec_k - rvec_k_1) > 2)
    {
      
#ifndef SPEED_OPTIM
      cvCTRACE << "kept old orientation: " << norm(rvec_k - rvec_k_1) << std::endl;
#endif
      
      PnPProblem::set_R_matrix(pose.back(), PnPProblem::get_rot_part(pose_k_1));

#ifndef SPEED_OPTIM
      cvCTRACE << "kept old orientation: " << pose.back() << std::endl;
#endif
    }
    if (norm(PnPProblem::get_trans_part(pose_k) - PnPProblem::get_trans_part(pose_k_1)) > 20)
    {
      // assume constant velocity. add to new position the difference of the previous ones
      // otherwise below at triangulation problem, that two camera poses are identical.

      cv::Mat t_k_1 = PnPProblem::get_trans_part(pose_k_1);
      cv::Mat t_k_2 = PnPProblem::get_trans_part(pose.at(pose.size() - 3));

      PnPProblem::set_t_vector(pose.back(), t_k_1 + 1.0f / 4.0f*(t_k_1 - t_k_2));

#ifndef SPEED_OPTIM
      cvCTRACE << "kept old position: " << pose.back() << std::endl;
#endif
    }
  }
  
}


