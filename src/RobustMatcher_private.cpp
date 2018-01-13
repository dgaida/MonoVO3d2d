/*
 * RobustMatcher.cpp
 *
 *  Created on: Jun 4, 2014
 *      Author: eriba
 */

#include "../include/RobustMatcher.h"            // this class
#include <time.h>

#include "../include/IOslam.h"

#include "../include/Debug.h"

#include <opencv2/features2d/features2d.hpp>


// might be that Matlab does a symmetric match, not sure, see: helperDetectAndMatchFeatures
// 'Unique', true: does avoid duplicate matches, but I do not know whether it makes a complete 
// two sided comparison or just then when thee is a duplicate. 
cv::Mat RobustMatcher::robustMatch(const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
  std::vector<cv::KeyPoint>& keypoints_frame, const cv::Mat& descriptors_model, int counter)
{

  // 1a. Detection of the ORB features
  this->computeKeyPoints(frame, keypoints_frame, counter);

  // 1b. Extraction of the ORB descriptors
  cv::Mat descriptors_frame;
  this->computeDescriptors(frame, keypoints_frame, descriptors_frame);

  // 2. Match the two image descriptors
  std::vector<std::vector<cv::DMatch> > matches12, matches21;

  // 2a. From image 1 to image 2
  matcher_->knnMatch(descriptors_frame, descriptors_model, matches12, 2); // return 2 nearest neighbours

  // 2b. From image 2 to image 1
  matcher_->knnMatch(descriptors_model, descriptors_frame, matches21, 2); // return 2 nearest neighbours

  // 3. Remove matches for which NN ratio is > than threshold
  // clean image 1 -> image 2 matches
  ratioTest(matches12);
  // clean image 2 -> image 1 matches
  ratioTest(matches21);

  // 4. Remove non-symmetrical matches
  symmetryTest(matches12, matches21, good_matches);

  return descriptors_frame;
}

cv::Mat RobustMatcher::fastRobustMatch(const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
  std::vector<cv::KeyPoint>& keypoints_frame,
  const cv::Mat& descriptors_model, int counter)
{
  good_matches.clear();

  // 1a. Detection of the ORB features
  this->computeKeyPoints(frame, keypoints_frame, counter);

  // 1b. Extraction of the ORB descriptors
  cv::Mat descriptors_frame;
  this->computeDescriptors(frame, keypoints_frame, descriptors_frame);

  // 2. Match the two image descriptors
  std::vector<std::vector<cv::DMatch> > matches;
  // find 2 best nearest neighbour for descriptors_model (trainining features)
  matcher_->knnMatch(descriptors_frame, descriptors_model, matches, 2);

  // 3. Remove matches for which NN (nearest neighbour) ratio is > than threshold
  int remfeats= ratioTest(matches);

#ifndef FULLVERSION_D
  cvCTRACE << "removed features in ratiotest: " << remfeats << std::endl;
#endif

  // 4. Fill good matches container
  for (std::vector<std::vector<cv::DMatch> >::iterator
    matchIterator = matches.begin(); matchIterator != matches.end(); ++matchIterator)
  {
    if (!matchIterator->empty()) good_matches.push_back((*matchIterator)[0]);
  }

  return descriptors_frame;
}



void RobustMatcher::computeKeyPoints(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, int counter)
{
  std::vector<cv::KeyPoint> keypoints_uniform;
  //cv::Mat img_slice;

  detector_->detect(image, keypoints);

  //
  // Sort by response
  //
  std::sort(keypoints.begin(), keypoints.end(),
    [&](const cv::KeyPoint& lhs, const cv::KeyPoint& rhs)
  {
    return lhs.response > rhs.response;
  });

  int n_uniform_keys = 350/*250*/;

  int num_slices_w = image.size().width / 100;      // slices in width direction
  int num_slices_h = image.size().height / 100;      // slices in vertical direction

  int step_w = image.size().width / (num_slices_w);
  int step_h = image.size().height / (num_slices_h);

  int n_keys_per_slice = n_uniform_keys / (num_slices_w*num_slices_h);

  for (int islice_w = 0; islice_w < num_slices_w; islice_w++)
  {
    int start_idx_w = islice_w*step_w + 1;
    int end_idx_w= (islice_w + 1)*step_w;

    for (int islice_h = 0; islice_h < num_slices_h; islice_h++)
    {
      int start_idx_h = islice_h*step_h + 1;
      int end_idx_h = (islice_h + 1)*step_h;

      int num_keys_inserted = 0;

      for (int ikeypoint = 0; ikeypoint < keypoints.size() && num_keys_inserted < n_keys_per_slice; ikeypoint++)
      {
        cv::Point2f myloc = keypoints.at(ikeypoint).pt;

        if (myloc.x >= start_idx_w && myloc.x < end_idx_w && 
            myloc.y >= start_idx_h && myloc.y < end_idx_h)
        {
          // TODO - test, make features non-invariant to rotation
          // macht das Matlab beispiel besser, ob das aber generell besser ist fraglich, 
          // da dann feature nicht invariant gegenüber rotation ist
          keypoints.at(ikeypoint).angle = CV_PI / 2;

          keypoints_uniform.push_back(keypoints.at(ikeypoint));
          num_keys_inserted++;
        }
      }
    }
  }

  keypoints_uniform.swap(keypoints);

#ifndef FULLVERSION_D 
  cvCTRACE << "num keypoints: " << keypoints.size() << std::endl;
#endif

#ifndef FULLVERSION_D 
  char string1[255];
  sprintf(string1, "keypoints_%i.csv", counter);

  remove(string1);

  for (int ip = 0; ip < keypoints.size(); ip++)
    IOslam::write2DPointToFile(keypoints.at(ip).pt, string1);
#endif

  // für 1000 funktioniert es im zusammenspiel mit faktor 1.5 unten ganz unten
  // für manch andere werte bspw. 1500 und 1.5 funkt es gar nicht
  adaptiveNonMaximalSuppression(keypoints, 250/*150*//*1000*/);

#ifndef FULLVERSION_D 
  sprintf(string1, "keypoints_suppr_%i.csv", counter);

  remove(string1);

  for (int ip= 0; ip < keypoints.size(); ip++)
    IOslam::write2DPointToFile(keypoints.at(ip).pt, string1);
#endif
}

void RobustMatcher::computeDescriptors(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, 
  cv::Mat& descriptors)
{
  extractor_->compute(image, keypoints, descriptors);
}

// http://answers.opencv.org/question/93317/orb-keypoints-distribution-over-an-image/
void RobustMatcher::adaptiveNonMaximalSuppression(std::vector<cv::KeyPoint>& keypoints,
  const int numToKeep) const
{
  if (keypoints.size() < numToKeep) { return; }

  //
  // Sort by response
  //
  std::sort(keypoints.begin(), keypoints.end(),
    [&](const cv::KeyPoint& lhs, const cv::KeyPoint& rhs)
  {
    return lhs.response > rhs.response;
  });

  std::vector<cv::KeyPoint> anmsPts;

  std::vector<double> radii;
  radii.resize(keypoints.size());
  std::vector<double> radiiSorted;
  radiiSorted.resize(keypoints.size());

  // Wert von 1.49 oder 1.51 gibt ganz andere ergebnisse
  const float robustCoeff = 1.01f;// 1.11; // see paper

  for (int i = 0; i < keypoints.size(); ++i)
  {
    const float response = keypoints[i].response * robustCoeff;
    double radius = std::numeric_limits<double>::max();
    for (int j = 0; j < i && keypoints[j].response > response; ++j)
    {
      radius = std::min(radius, cv::norm(keypoints[i].pt - keypoints[j].pt));
    }
    radii[i] = radius;
    radiiSorted[i] = radius;
  }

  std::sort(radiiSorted.begin(), radiiSorted.end(),
    [&](const double& lhs, const double& rhs)
  {
    return lhs > rhs;
  });

  const double decisionRadius = radiiSorted[numToKeep];

#ifndef FULLVERSION_D
  cvCTRACE << "decision radius: " << decisionRadius << std::endl;
#endif // !FULLVERSION_D

  for (int i = 0; i < radii.size(); ++i)
  {
    if (radii[i] >= decisionRadius)
    {
      anmsPts.push_back(keypoints[i]);
    }
  }

  anmsPts.swap(keypoints);
}

int RobustMatcher::ratioTest(std::vector<std::vector<cv::DMatch> > &matches) const
{
  int removed = 0;
  // for all matches
  for ( std::vector<std::vector<cv::DMatch> >::iterator
        matchIterator= matches.begin(); matchIterator!= matches.end(); ++matchIterator)
  {
    // if 2 NN has been identified
    if (matchIterator->size() > 1)
    {
      // check distance ratio
      if ((*matchIterator)[0].distance / (*matchIterator)[1].distance > ratio_)
      {
        matchIterator->clear(); // remove match
        removed++;
      }
    }
    else
    { // does not have 2 neighbours
      matchIterator->clear(); // remove match
      removed++;
    }
  }
  return removed;
}

void RobustMatcher::symmetryTest( const std::vector<std::vector<cv::DMatch> >& matches1,
                     const std::vector<std::vector<cv::DMatch> >& matches2,
                     std::vector<cv::DMatch>& symMatches ) const
{

  // for all matches image 1 -> image 2
   for (std::vector<std::vector<cv::DMatch> >::const_iterator
       matchIterator1 = matches1.begin(); matchIterator1 != matches1.end(); ++matchIterator1)
   {

      // ignore deleted matches
      if (matchIterator1->empty() || matchIterator1->size() < 2)
         continue;

      // for all matches image 2 -> image 1
      for (std::vector<std::vector<cv::DMatch> >::const_iterator
          matchIterator2 = matches2.begin(); matchIterator2 != matches2.end(); ++matchIterator2)
      {
        // ignore deleted matches
        if (matchIterator2->empty() || matchIterator2->size() < 2)
           continue;

        // Match symmetry test
        if ((*matchIterator1)[0].queryIdx ==
            (*matchIterator2)[0].trainIdx &&
            (*matchIterator2)[0].queryIdx ==
            (*matchIterator1)[0].trainIdx)
        {
            // add symmetrical match
            symMatches.push_back(
              cv::DMatch((*matchIterator1)[0].queryIdx,
                         (*matchIterator1)[0].trainIdx,
                         (*matchIterator1)[0].distance));
            break; // next match in image 1 -> image 2
        }
      }
   }

}


