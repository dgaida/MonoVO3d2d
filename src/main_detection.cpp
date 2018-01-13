// C++
#include <iostream>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>
// PnP Tutorial
#include "../include/PnPProblem.h"           // solves the PnP problem
#include "../include/RobustMatcher.h"        // Matcher with ORB features
#include "../include/VO.h"                   // Visual Odometry methods
#include "../include/CamParams.h"            // Camera Params: Camera Matrix, distortion coeffs
#include "../include/FrameGrabber.h"         // grab frames from webcam or file
#include "../include/Utils.h"
#include "../include/Visualization.h"
#include "../include/IOslam.h"               // FileIO and CommandLine parser

#include "../include/Debug.h"

// TODOs
// extend with the idea of keyframes
// sparse bundle adjustment: http://users.ics.forth.gr/~lourakis/sba/

// see also: https://de.mathworks.com/help/vision/examples/monocular-visual-odometry.html

/**  GLOBAL VARIABLES  **/

using namespace cv;
using namespace std;

cv::Scalar red(0, 0, 255);
cv::Scalar blue(255, 0, 0);

// Robust Matcher parameters - can test different values here, but 0.6f is pretty good
// in Maltab default is 0.6f, see matchFeatures in Matlab
float ratioTest = 0.6f;// 0.7f;// 0.70f;          // ratio test

#ifdef SPEED_OPTIM
  bool fast_match = true;// true;       // fastRobustMatch() or robustMatch()
#else
// mit false läuft es aktuell nicht
  bool fast_match = true;// true;       // fastRobustMatch() or robustMatch()
#endif
#ifdef SPEED_OPTIM
  // TODO have to reduce number here
  int numKeyPoints = 2500;// 3000;// 3000;// 3000;              // number of keypoints to detect
#else
  int numKeyPoints = 2500;// 2500;//2000// 3000;              // number of keypoints to detect
#endif

// RANSAC parameters
#ifdef SPEED_OPTIM
  // TODO: have to reduce here
  int iterationsCount = 1000;// 1000;// 500;      // number of Ransac iterations.
#else
  int iterationsCount = 1000;// 1000;// 500;      // number of Ransac iterations.
#endif
//0.3 macht drehung um x-achse korrekt// 8;  // maximum allowed distance to consider it an inlier.
float reprojectionError = 0.8f;///*0.3f*/0.3;// 0.2;
double confidence = 0.9999;        // ransac successful confidence.

// threshold for std deviation of points3D
float th_std_points3D = 1.0f;

/**  Functions headers  **/
void help();

/**  Main program  **/
int main(int argc, char *argv[])
{
  // camera related stuff
  bool useRealCam = false;// false;

  /**
  * path to the image sequence when reading frames from file, including name of file
  */
  const char fullpath_imgs[170] = "D://Programmierung//SLAMMatlab//VO//NewTsukuba//image%04d.jpg";

#ifdef SPEED_OPTIM
  // true, then write orientation and translation to a file
  // TODO: in the end has to be false for speed optimization
  bool dumpResults2File = true/*false*/ && !useRealCam;
#else
  // true, then write orientation and translation to a file
  bool dumpResults2File = true/*true*/ && !useRealCam;
#endif
  // use ground truth data
  bool useGT = true && !useRealCam;// true;
  // cheat with ground truth locations, estimated locations are overwritten with ground truth
  bool useGTlocs = useGT && false;// false;
  // cheat with ground truth orientations, estimated rotations are overwritten with ground truth
  bool useGTorients = useGT && false;// false;
  // read features from file that were saved from Matlab 
  bool read_featuresFromFile = false && !useRealCam; // false
  // write features and triangulated 3d points to csv files
  bool write_features2file= false && !useRealCam; // false

  help();

  int cmdInt= IOslam::readCmdLineInterface(argc, argv, ratioTest, fast_match, numKeyPoints, 
    useRealCam, useGT);

  useGT = useGT && !useRealCam;

  cvCTRACE << "use groundtruth: " << useGT << std::endl;
  cvCTRACE << "use real camera: " << useRealCam << std::endl;

  if (cmdInt == 0)
    return cmdInt;

  FrameGrabber myframegrabber = FrameGrabber(useRealCam, fullpath_imgs);

  // create object which saves cameraMatrix and distortion coefficients
  CamParams myCamParams = CamParams();

  if (useRealCam)
  {
    // Intrinsic camera parameters: UVC WEBCAM
    myCamParams.setCameraMatrix(651.585f, 655.2391f, 331.7f, 238.1f);

    myCamParams.setDistCoeffs(4.6014673139661059e-002, -4.2368040144562297e-001,
      -5.5034476996566641e-003, 3.6349854162632724e-003, 1.5882434516416959e+000);
  }
  else
  {
    // Matlab example Tsukuba
    myCamParams.setCameraMatrix(615.0f, 615.0f, 320.0f, 240.0f);
    myCamParams.setDistCoeffs(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    myframegrabber.setImageOffset(1);
  }

  Mat cameraMatrix, distCoeffs;
  myCamParams.getCameraMatrix(cameraMatrix);
  myCamParams.getDistCoeffs(distCoeffs);

  //

  PnPProblem pnp_detection(cameraMatrix);      // used for PnPRansac
   
  RobustMatcher rmatcher(ratioTest, fast_match, numKeyPoints);   // instantiate RobustMatcher

  // to visualize current frame as well as trajectory
  Visualization myvisu;

  std::vector<cv::Point3f> GT_locs;

  // as I wnat to plot the ground truth always if I do not use a camera, load the file
  if (!useRealCam/*useGT*/)
    IOslam::readPoint3f_FromFile(GT_locs, "D:/Programmierung/SLAMMatlab/VO/groundtruth_locs.csv");

  std::vector<cv::Mat> GT_orients;

  if (useGTorients)
    IOslam::readDouble3x3Mat_FromFile(GT_orients, "D:/Programmierung/SLAMMatlab/VO/groundtruth_orients.csv");

  int index_last_keyframe = 0;      // saves counter value at last keyframe

  Mat frame, frame_vis;

  // descriptors at time k-2, k-1 and k
  std::vector<cv::Mat> descriptors;
  // keypoints at time k-2, k-1 and k
  std::vector<std::vector<cv::KeyPoint>> keypoints;

  // good matches between keypoints from frame k-2 and k-1; k-1 and k
  // this is only a 2dim vector
  std::vector<vector<DMatch>> good_matches;       
  
  // positions of matching features at time k-2, k-1 and k
  std::vector<vector<Point2f>> features; 
  
  // pose of camera at frame k-2, k-1, k in world coordinates
  // that's the estimated orientation and position of the camera
  std::vector<Mat> pose;     
  // keyframe pose of camera at frame k-2, k-1, k in world coordinates
  std::vector<Mat> keyframes;     

  // triangulated 3D points in euclidian coordinates from frames k-2, k-1 in world coordinates
  std::vector<vector<Point3f>> pnts3D_vec; 

  // contains all scale values calculated from distances between 3D points from adjacent frames
  // the location of all frames is multiplied with the same median scale
  std::vector<float> scaleVector;
  float median_scale = 1.0f;

#ifndef SPEED_OPTIM
  std::vector<float> inliersPnPratios;
  std::vector<int> inliersPnP;
  std::vector<float> inliersRatioPose;
  std::vector<float> reproErrPnP;
#endif

  // temporary - used 
  cv::Mat pose_temp;
  vector<Point3f> pnts3D_vec_temp;
  
  // initialize orientation and position to 0,0,0
  
  Mat R = Mat::eye(cv::Size(3, 3), CV_64FC1);
  Mat t = Mat::zeros(cv::Size(1, 3), CV_64FC1);

  // start of algorithm 3d to 2d VO

  // 1)
  // 1.1) capture frame k-2

  std::cout << "1.1) capture frame k-2" << std::endl;

  // set keyframe
  keyframes.push_back(PnPProblem::get_P_matrix/*_float*/(R, t));

#ifndef SPEED_OPTIM
  std::cout << "  initial pose in WCS (k-2): " << keyframes.back() << std::endl;
#endif

  if (dumpResults2File)
  {
    remove("estimated_locs.txt");
    remove("estimated_orients.txt");

    IOslam::writeMatToFile(t, "estimated_locs.txt");
    IOslam::writeMatToFile(R, "estimated_orients.txt");
  }

  myframegrabber.grabNextUndistortedFrame(frame, cameraMatrix, distCoeffs);

  index_last_keyframe = myframegrabber.getCounter();

  //cv::Mat frame_k_2 = frame.clone();

  imshow("Real-time frame", frame);
  if (useRealCam)
    waitKey(1000);   // when using a real camera then wait a while
  else
    waitKey(1);

  // 1.2) extract features ...
#ifndef SPEED_OPTIM
  std::cout << "1.2) extract features in frame k-2" << std::endl;
#endif
  rmatcher.extractKeyPointsAndDescriptors(frame, descriptors, keypoints, myframegrabber.getCounter());

  // 1.1) capture frame k-1
  std::cout << "1.1) capture frame k-1" << std::endl;
  myframegrabber.grabNextUndistortedFrame(frame, cameraMatrix, distCoeffs);
  
  // 1.2) extract features and match features between frame k-2 and k-1
#ifndef SPEED_OPTIM
  std::cout << "1.2) extract features and match features between frame k-2 and k-1" << std::endl;
#endif

  rmatcher.extractKeyPointsAndMatch(frame, descriptors, keypoints, good_matches, myframegrabber.getCounter());

  VO::getFeaturesFromGoodMatches(features, keypoints, good_matches.back());


  // read features from Matlab

  /*if (read_featuresFromFile)
  {
    IOslam::readPoint2f_FromFile(features.at(features.size() - 2), "D:/Programmierung/SLAMMatlab/VO/features_1_2.csv");
    IOslam::readPoint2f_FromFile(features.back(), "D:/Programmierung/SLAMMatlab/VO/features_1_1.csv");
  }*/


  draw2DPoints(frame, features.back(), red);
  imshow("Real-time frame", frame);
  waitKey(1);

  // 1.3) triangulate features from frame k-2 and k-1 to get 3D points at time k-1
#ifndef SPEED_OPTIM
  std::cout << "1.3) triangulate features from frame k-2 and k-1" << std::endl;
#endif

  float inlierRatioPose;

  VO::recoverPoseFromEssentialMat(index_last_keyframe, features, cameraMatrix, 
    R, t, true, good_matches, pnts3D_vec_temp, inlierRatioPose); // just use an empty vector as last argument

#ifndef SPEED_OPTIM
  inliersRatioPose.push_back(inlierRatioPose);
#endif

  // TODO: I think this transformation is not logical. should be R.t()
  // get translation vector in world coordinates, therefore have to inverse t
  // Diese berechnung ist korrekt, verglichen mit matlab
  pose_temp = PnPProblem::multiplyTransformations(keyframes.back(), 
    PnPProblem::get_P_matrix(R, -R.t()*t));

  if (useGTlocs)
  {
    t.at<double>(0) = GT_locs.at(myframegrabber.getCounter()).x;
    t.at<double>(1) = GT_locs.at(myframegrabber.getCounter()).y;
    t.at<double>(2) = GT_locs.at(myframegrabber.getCounter()).z;

    PnPProblem::set_t_vector(pose_temp, t);
  }

  if (useGTorients)
  {
    GT_orients.at(myframegrabber.getCounter()).copyTo(R);

    PnPProblem::set_R_matrix(pose_temp, R);
  }

  // pose of camera at time k-1
  pose.push_back(pose_temp);

#ifndef FULLVERSION_D
  std::cout << "  recovered pose before rescale: " << pose.back() << std::endl;
#endif

  if (useGT)
  {
    median_scale= VO::correctTranslationForScaleFromGT(pose, GT_locs);
  }
  
#ifndef SPEED_OPTIM
  std::cout << "  recovered pose: " << pose.back() << std::endl;
#endif

  //exit(0);

  if (dumpResults2File)
  {
    IOslam::writeMatToFile(PnPProblem::get_trans_part(pose.back()), "estimated_locs.txt");
    IOslam::writeMatToFile(PnPProblem::get_rot_part(pose.back()), "estimated_orients.txt");
  }

  // Triangulates the 3d position of 2d correspondences between several images
  //pnts3D_vec_temp = PnPProblem::triangulatePointsConvEuclidian(
  //  keyframes.back(), pose.back(), cameraMatrix,
  //  features.at(index_last_keyframe), features.back()/*,
  //  pnts3D_temp, good3dpoints*/);

  // 3D points at time k
  //pnts3D_vec.push_back(pnts3D_vec_temp);
  
#ifdef SPEED_OPTIM
  std::cout << "Starting timer now. Running speed optimized, therefore no debug messages anymore." << std::endl;
#endif

  // TODO: what if too many features are deleted, then repeat calculating findEssentialMat

  // start the clock
  myframegrabber.startTimer();

  // 2) 
  // 2.1) capture new frame
  while(/*cap.read(frame) &&*/ (char)waitKey(30) != 27) // capture frame until ESC is pressed
  {
#ifndef SPEED_OPTIM
    std::cout << "2.1) capture new frame k= " << myframegrabber.getCounter() << std::endl;
#endif
    myframegrabber.grabNextUndistortedFrame(frame, cameraMatrix, distCoeffs);
    
    Mat inliers_idx;

    frame_vis = frame.clone();    // refresh visualisation frame

    // 2.2) Extract features and match with previous frame k-1
#ifndef FULLVERSION_D
    std::cout << "2.2) Extract features and match with previous frame k-1" << std::endl;
#endif

    rmatcher.extractKeyPointsAndMatch(frame, descriptors, keypoints, good_matches, myframegrabber.getCounter());

    // find features that are present in k-2, k-1, and k
    VO::findMatchingFeatures(features, keypoints, good_matches, index_last_keyframe, 
      /*pnts3D_vec*/std::vector<std::vector<cv::Point3f>>());
    
    // write features of last 3 frames to csv files
    if (write_features2file)
    {
      char string1[255];
      sprintf(string1, "myfeatures_%i_1.csv", myframegrabber.getCounter());
      IOslam::write2DPointVecToFile(features.at(features.size() - 2), string1);
      sprintf(string1, "myfeatures_%i_2.csv", myframegrabber.getCounter());
      IOslam::write2DPointVecToFile(features.at(features.size() - 3), string1);
      sprintf(string1, "myfeatures_%i.csv", myframegrabber.getCounter());
      IOslam::write2DPointVecToFile(features.back(), string1);
    }

    // read features from Matlab

    if (read_featuresFromFile)
    {
      char string1[255];
      sprintf(string1, "D:/Programmierung/SLAMMatlab/VO/features_%i_2.csv", myframegrabber.getCounter());

      //std::cout << string1 << std::endl;

      IOslam::readPoint2f_FromFile(features.at(features.size() - 3), string1);

      sprintf(string1, "D:/Programmierung/SLAMMatlab/VO/features_%i_1.csv", myframegrabber.getCounter());
      IOslam::readPoint2f_FromFile(features.at(features.size() - 2), string1);

      sprintf(string1, "D:/Programmierung/SLAMMatlab/VO/features_%i.csv", myframegrabber.getCounter());
      IOslam::readPoint2f_FromFile(features.back(), string1);
    }

    //VO::getFeaturesFromGoodMatches(features, keypoints, good_matches.back());

    // pass an empty dummy vector to recoverPose, as from here on, good_matches and features
    // do not have to be have the same amount of entries anymore
    vector<vector<DMatch>> good_matches_temp;

    // have to pass 3d vector as well
    // called to delete features that are not inliers of essential matrix calculation
    // I am not sure, but maybe the pose recoverPose calculates has a lot of outliers, because
    // it is a relative transformation. Only works at first time, because of eye and 0 position
    int goodpnts= 
      VO::recoverPoseFromEssentialMat(index_last_keyframe, features, cameraMatrix, 
      R, t, true/*false*/, good_matches_temp, /*pnts3D_vec.back()*/std::vector<cv::Point3f>(), inlierRatioPose);
    
    if (goodpnts < 5)
    {
#ifndef SPEED_OPTIM
      cvCTRACE << "continuing, too few good points" << std::endl;
#endif
      // TODO - temporarily
      index_last_keyframe++;
      continue;
    }

#ifndef SPEED_OPTIM
    inliersRatioPose.push_back(inlierRatioPose);
#endif

    /*if (read_featuresFromFile)
    {
      char string1[255];
      sprintf(string1, "D:/Programmierung/SLAMMatlab/VO/features_%i_2.csv", myframegrabber.getCounter());

      //std::cout << string1 << std::endl;

      IOslam::readPoint2f_FromFile(features.at(features.size() - 3), string1);

      sprintf(string1, "D:/Programmierung/SLAMMatlab/VO/features_%i_1.csv", myframegrabber.getCounter());
      IOslam::readPoint2f_FromFile(features.at(features.size() - 2), string1);

      sprintf(string1, "D:/Programmierung/SLAMMatlab/VO/features_%i.csv", myframegrabber.getCounter());
      IOslam::readPoint2f_FromFile(features.back(), string1);
    }*/

    // Draw outliers
    draw2DPoints(frame_vis, features.back(), red);


//#ifndef SPEED_OPTIM
    std::vector<float> dist_feat;
    dist_feat.reserve(features.back().size());

    for (int ipoint = 0; ipoint < features.back().size(); ipoint++)
    {
      Point2f mypoint_prev;
      mypoint_prev.x = features.at(index_last_keyframe)[ipoint].x;
      mypoint_prev.y = features.at(index_last_keyframe)[ipoint].y;

      Point2f mypoint_curr;
      mypoint_curr.x = features.back()[ipoint].x;
      mypoint_curr.y = features.back()[ipoint].y;

      dist_feat.push_back(sqrt(pow(mypoint_prev.x - mypoint_curr.x, 2) +
        pow(mypoint_prev.y - mypoint_curr.y, 2)));
    }
//#endif

    //std::cout << VO::median(dist_feat) << std::endl;

    // None matches, then RANSAC crashes
    if (
//#ifndef SPEED_OPTIM      
     (VO::median(dist_feat) > 1/*9*//*0*/) &&
//#endif
      (features.back().size() > 5)/* && good3dpoints > 5*/) 
    {
      // 2.3) compute camera pose (PnP) from 3D-to-2D matches
#ifndef FULLVERSION_D
      std::cout << "2.3) compute camera pose (PnP) from 3D-to-2D matches" << std::endl;
#endif

      if (pose.size() >= 2)
        pose_temp = pose.at(pose.size() - 2);
      else
        pose_temp = keyframes.back();

      // Triangulates the 3d position of 2d correspondences between several images
      pnts3D_vec_temp = PnPProblem::triangulatePointsConvEuclidian(
        pose_temp, pose.back(), cameraMatrix,
        features.at(features.size() - 3), features.at(features.size() - 2));

      //for (int ip = 0; ip < pnts3D_vec_temp.size(); ip++)
        //std::cout << pnts3D_vec_temp.at(ip) << std::endl;

      // 3D points at time k
      pnts3D_vec.push_back(pnts3D_vec_temp);

      // write pnts3D_vec_temp to csv file
      if (write_features2file)
      {
        char string1[255];
        sprintf(string1, "3dpoints_%i.csv", myframegrabber.getCounter());

        IOslam::write3DPointVecToFile(pnts3D_vec_temp, string1);
      }

      //exit(-1);

      // correct scale of 3D points

      /*if (useGT && !pnts3D_vec.empty())
      {
        for (int ipoint = 0; ipoint < pnts3D_vec.back().size(); ipoint++)
        {
          pnts3D_vec.back().at(ipoint) = median_scale * pnts3D_vec.back().at(ipoint);
        }
        if (pnts3D_vec.size() >= 2)
          for (int ipoint = 0; ipoint < pnts3D_vec.at(pnts3D_vec.size() - 2).size(); ipoint++)
          {
            pnts3D_vec.at(pnts3D_vec.size() - 2).at(ipoint) =
              median_scale * pnts3D_vec.at(pnts3D_vec.size() - 2).at(ipoint);
          }
      }*/



      float inliers_ratio;

      // Matlab does the same
      if (myframegrabber.getCounter() > 15)
        iterationsCount = 5000;

      t = PnPProblem::get_trans_part(pose.back());
      R = PnPProblem::get_rot_part(pose.back());
      cv::Mat rvec;
      cv::Rodrigues(R, rvec);

      // Estimate the pose using RANSAC approach
      // 3D points from k-1 and features from frame k
      pnp_detection.estimatePoseRANSAC(pnts3D_vec.back(), features.back()/*at(features.size() - 2)*//*back()*/,
        /*pnpMethod,*/ inliers_idx,
        iterationsCount, reprojectionError, confidence, distCoeffs, inliers_ratio, rvec, t);
      
#ifndef SPEED_OPTIM
      inliersPnPratios.push_back(inliers_ratio);
      inliersPnP.push_back(inliers_idx.rows);
      reproErrPnP.push_back(reproErr);
#endif

      //

      if (useGTlocs)
      {
        t.at<double>(0) = GT_locs.at(myframegrabber.getCounter()).x;
        t.at<double>(1) = GT_locs.at(myframegrabber.getCounter()).y;
        t.at<double>(2) = GT_locs.at(myframegrabber.getCounter()).z;

        PnPProblem::set_t_vector(pnp_detection.get_P_matrix(), t);
      }

      if (useGTorients)
      {
        GT_orients.at(myframegrabber.getCounter()).copyTo(R);

        PnPProblem::set_R_matrix(pnp_detection.get_P_matrix(), R);
      }

      // add estimated pose to vector, pose at frame k
      pose.push_back(pnp_detection.get_P_matrix());

      //std::cout << "  estimated camera pose: " << pose.back() << std::endl;

      //VO::removePnPoutliers(features, pnts3D_vec.back(), inliers_idx);

      // estimate scale of translation

      //if (myframegrabber.getCounter() <= 15)
      {
        if (useGT)
        {
          median_scale = VO::correctTranslationForScaleFromGT(pose, GT_locs);
          scaleVector.push_back(median_scale);
        }
        else
          median_scale = VO::correctTranslationForScale(features, pnts3D_vec.back(),
            pose, cameraMatrix, scaleVector);
      }

      //std::cout << "  estimated camera pose: " << pose.back() << std::endl;
#ifndef FULLVERSION_D
      cvCTRACE << "  n inliers: " << inliers_idx.rows << std::endl;
#endif

      // test new pose for plausability

      VO::posePlausibilityCheck(pose);


      if (dumpResults2File)
      {
        IOslam::writeMatToFile(PnPProblem::get_trans_part(pose.back()), "estimated_locs.txt");
        IOslam::writeMatToFile(PnPProblem::get_rot_part(pose.back()), "estimated_orients.txt");
      }

      
      // Draw inliers points 2D
      draw2DPoints(frame_vis, features.back(), blue);

//#ifndef FULLVERSION_D
      if(!useRealCam)
        myvisu.displayTrajectory(pose.back(), pnts3D_vec.back(), GT_locs.at(myframegrabber.getCounter()));
      else
        myvisu.displayTrajectory(pose.back(), pnts3D_vec.back(), cv::Point3f(0,0,0));
//#endif
      
      // 2.4) triangulate all new feature matches between frames k and k-1
#ifndef FULLVERSION_D
      std::cout << "2.4) triangulate all feature matches between frames k-1 and k" << std::endl;
#endif

      // TODO - hier sollte ich alle features nutzen, welche in frame k-1 and k gut passen
      // ohne k-2 zu berücksichtigen
//      VO::getFeaturesFromGoodMatches(features, keypoints, good_matches.back());
//
//      // TODO: maybe check inliers in essential matrix
//      VO::recoverPoseFromEssentialMat(index_last_keyframe, features, cameraMatrix,
//        R, t, false/*false*/, good_matches, std::vector<cv::Point3f>(), inlierRatioPose);
//
//#ifndef SPEED_OPTIM
//      inliersRatioPose.push_back(inlierRatioPose);
//#endif
//
//      // Triangulates the 3d position of 2d correspondences between several images
//      pnts3D_vec_temp = PnPProblem::triangulatePointsConvEuclidian(
//        pose.at(pose.size() - 2), pose.back(), cameraMatrix,
//        features.at(features.size() - 2), features.back()/*,
//        pnts3D_temp, good3dpoints*/);

      //std::cout << "  good3dpoints: " << good3dpoints << std::endl;

      // 3D points at time k
      //pnts3D_vec.push_back(pnts3D_vec_temp);
      //pnts3D_hom.push_back(pnts3D_temp);

      //std::cout << pnts3D_hom.back() << std::endl;

      // it can happen, that triangulatePointsConvEuclidian deletes features, in case z < 0

      /*if (features.back().size() < 20)
      {

      }*/

#ifndef FULLVERSION_D
      std::cout << //"Number of features in frame k: " << features.back().size() <<
        "  Number of 3D points in frame k: " << pnts3D_vec_temp.size() <<
        //"; number of good 3D points in frame k: " << good3dpoints <<
        std::endl;
#endif

      /*std::cout << "Number of features in last keyframe: " <<
        features.at(index_last_keyframe).size() <<
        "; number of features in frame k: " << features.back().size() <<
        "; number of 3D points in frame k-1: " << pnts3D_vec.back().size() <<
        std::endl;*/



      // 2.5) check until the average uncertainty of the 3d points decreases below a threshold

      if (100 < th_std_points3D)
      {
        //

        Mat pose_last = keyframes.back();

        // TODO: das funkt nicht, da matrizen nicht 4x4
        keyframes.push_back( pose_last * pose.back() );

        // delete old stuff from vector

        //  have to adapt index, depending on how much old stuff I deleted
        index_last_keyframe = myframegrabber.getCounter();
      }

    } // if 
    else
    {
#ifndef SPEED_OPTIM
      cvCTRACE << "too few changes between frames" << std::endl;
#endif
    }

    // TODO - temporarily
    index_last_keyframe++;

    // if size of features and the others exceeds some size
    // delete old stuff
    while (features.size() >= 4)
    {
      // macht probleme wenn ich am beginn was lösche
      features.erase(features.begin());
      descriptors.erase(descriptors.begin());
      keypoints.erase(keypoints.begin());
      good_matches.erase(good_matches.begin());

      // because we delete one k from the beginning
      index_last_keyframe--;
    }


    // FRAME RATE

    myvisu.displayFrame(frame_vis, myframegrabber.getFPS(), 
      inliers_idx, (int)good_matches.back().size());


    if (!useRealCam && myframegrabber.getCounter() >= 149/*100*//*100*//*50*/)
      break;

  }

  
  /*for (int ip = 0; ip < scaleVector.size(); ip++)
  {
    std::cout << scaleVector.at(ip) << std::endl;
  }*/

  /*for (int ip = 0; ip < inliersPnPratios.size(); ip++)
  {
    std::cout << ip << ": " << inliersPnPratios.at(ip) << ", " << inliersPnP.at(ip) 
      << ", " << reproErrPnP.at(ip) << std::endl;
  }*/

  /*for (int ip = 0; ip < inliersRatioPose.size(); ip++)
  {
    std::cout << inliersRatioPose.at(ip) << std::endl;
  }*//**/

  std::cout << "Press a key to exit the application." << std::endl;
  cv::waitKey(0);

  cout << "GOODBYE ..." << endl;

}

/**********************************************************************************************************/
void help()
{
cout
<< "--------------------------------------------------------------------------"   << endl
<< "This program does Monocolar 2D-3D Visual Odometry. You can choose to "
<< "use a recorded video or the webcam."                                          << endl
<< "Usage:"                                                                       << endl
<< "visSLAM.exe -help"                                                            << endl
<< "Good call: visSLAM.exe -groundtruth=false"                                    << endl
<< "Keys:"                                                                        << endl
<< "'esc' - to quit."                                                             << endl
<< "--------------------------------------------------------------------------"   << endl
<< endl;
}

