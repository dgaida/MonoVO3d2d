// Implementation OK - could insert some measures to prevent functions failing/crashing 
// and return true/false if files could not be read
// Documentation OK - could be improved as well
// 11.01.2018, Daniel Gaida

#include "../include/IOslam.h"           // this class

// C++
#include <iostream>
#include <fstream>
#include <sstream>

#include "../include/Debug.h"        // for cvCTRACE



/*=============================         PUBLIC METHODS         =============================*/

/**
* standard constructor does nothing
*/
IOslam::IOslam()
{
}

/**
* standard destructor does nothing
*/
IOslam::~IOslam()
{
}



/*=====================    PUBLIC METHODS - STATIC METHODS    =======================*/

/**
* read parameters from command line interface and return them in ratioTest and fast_match
*
* @param argc : number of arguments passed to the application
* @param argv : arguments passed to the application
* @param ratioTest : A parameter you can set: ratio by how much a feature match has to be a better match
* compared with the 2nd best feature matche to be considered a good match. Used inside RobustMatcher.
* @param fast_match : A parameter you can set: if false, then only feature matches are used
* that want to marry each other. If true, then this symmetry check is not done. Used inside RobustMatcher.
* @param numKeyPoints : number of keypoints to detect
* @param useRealCam : if true, then use a real camera, if false, then grab frames from file
* @param useGT : if true then use ground truth location to estimate scale. If false, then estimate
* scale via distance ratio of 3d points in consecutive frames. If useRealCam= true, then useGT is internally
* set to false.
*
* @return : 0 if application was called with "help", else returns 1
*/
int IOslam::readCmdLineInterface(int argc, char *argv[], float &ratioTest, bool &fast_match,
  int &numKeyPoints, bool &useRealCam, bool &useGT)
{

  const cv::String keys =
    "{help h        |      | print this message                   }"
    "{keypoints k   |2500  | number of keypoints to detect        }"
    "{ratio r       |0.6   | threshold for ratio test             }"
    "{fast f        |true  | use of robust fast match             }"
    "{realcam c     |false | true to use real camera, else use Tsukuba (or some other) benchmark}"
    "{groundtruth g |true  | use ground truth to estimate scale   }"
    ;
  
  cv::CommandLineParser parser(argc, argv, keys);

  if (parser.has("help"))
  {
    parser.printMessage();
    return 0;
  }
  else
  {
    numKeyPoints = parser.has("keypoints") ? parser.get<int>("keypoints") : numKeyPoints;
    ratioTest = parser.has("ratio") ? parser.get<float>("ratio") : ratioTest;
    fast_match = parser.has("fast") ? parser.get<bool>("fast") : fast_match;
    useRealCam = parser.has("realcam") ? parser.get<bool>("realcam") : useRealCam;
    useGT = parser.has("groundtruth") ? parser.get<bool>("groundtruth") : useGT;
  }

  return 1;
}

/**
* read ground truth location data from a csv file with the given filename and return it in
* GT_locs. Can of course also be other 3d data. The format of the file must be:
*
* x,y,z values
* x,y,z values
* ...
*
* @param GT_locs : vector with ground truth 3D points
* @param filename : filename including path to the csv file containing the ground truth
* locations. Each line contains one position. The 3 coordinates are separated with a ",".
*
*/
void IOslam::readPoint3f_FromFile(std::vector<cv::Point3f> &GT_locs, const char* filename)
{
  std::vector<std::vector<float>> numbers;

  readFloatVec_FromFile(numbers, filename); // get numbers as a vector of vectors

  cv::Point3f mypnt3d;

  GT_locs.clear();
  GT_locs.reserve(numbers.size());

  // save vector of vectors as a vector of Point3f
  for (int ii= 0; ii < numbers.size(); ii++)
  {
    mypnt3d.x = numbers.at(ii).at(0);
    mypnt3d.y = numbers.at(ii).at(1);
    mypnt3d.z = numbers.at(ii).at(2);

    GT_locs.push_back(mypnt3d);
  }
}

/**
* read 2dimensional float data from a csv file with the given filename and return it in
* imagePoints. The data has to look like:
*
* x,y
* x,y
* ...
*
* @param imagePoints : vector with 2d float vectors
* @param filename : filename including path to the csv file containing the
* data. Each line contains one position. The 2 coordinates are separated with a ",".
*
*/
void IOslam::readPoint2f_FromFile(std::vector<cv::Point2f> &imagePoints, const char* filename)
{
  std::vector<std::vector<float>> numbers;

  readFloatVec_FromFile(numbers, filename);     // get numbers as a vector of vectors

  cv::Point2f mypnt2d;

  imagePoints.clear();
  imagePoints.reserve(numbers.size());

  // save vector of vectors as a vector of Point2f
  for (int ii = 0; ii < numbers.size(); ii++)
  {
    mypnt2d.x = numbers.at(ii).at(0);
    mypnt2d.y = numbers.at(ii).at(1);
    
    imagePoints.push_back(mypnt2d);
  }
}

/**
* read a bunch of float vectors from a csv file with the given filename and return it in
* numbers. The file content should look like (must have same number of floats in each row):
*
* 1,2,3,...
* 1,2,3,...
* ...
*
* @param numbers : vector with vector of floats
* @param filename : filename including path to the csv file containing the ground truth
* locations. Each line contains one position. The 3 coordinates are separated with a ",".
*
* @return true, if file could be read, else false
*/
bool IOslam::readFloatVec_FromFile(std::vector<std::vector<float>> &numbers, const char* filename)
{
  std::ifstream fin;
  fin.open(filename, std::ios_base::in);    // open file for reading

  if (!fin)
  {
    cvCTRACE << "File Not Opened" << std::endl;
    return false;
  }

  while (fin.good()) // go through each line
  {
    std::string s;
    // get the complete line
    if (!getline(fin, s)) break;

    std::istringstream ss(s);
    std::vector<float> record;

    while (ss)
    {
      std::string s;
      // split the line on commas
      if (!getline(ss, s, ',')) break;

      std::istringstream ss(s);
      float result;
      ss >> result;     // convert the string containing a number to a float

      record.push_back(result);
    }

    numbers.push_back(record);
  }

  fin.close();

  return true;
}

/**
* read 3x3 matrix data from a csv file with the given filename and return it in
* numbers.
*
* @param numbers : vector with 3x3 matrices
* @param filename : filename including path to the csv file containing the 3x3
* matrices. Each 3 lines contains one matrix. The 3 coordinates are separated with a ",".
*
* @return true, if file was read successfully, else false
*/
bool IOslam::readDouble3x3Mat_FromFile(std::vector<cv::Mat> &numbers, const char* filename)
{
  std::ifstream fin;
  fin.open(filename, std::ios_base::in);

  if (!fin)
  {
    cvCTRACE << "File Not Opened" << std::endl;
    return false;
  }

  while (fin.good())
  {
    cv::Mat Rmat = cv::Mat::zeros(3, 3, CV_64FC1);

    for (int iline = 0; iline < 3; iline++)   // running over 3 rows
    {
      std::string s;
      // get the complete line
      if (!getline(fin, s)) break;

      std::istringstream ss(s);

      int icol = 0;   // column index

      while (ss)
      {
        std::string s;
        // split the line on commas
        if (!getline(ss, s, ',')) break;

        std::istringstream ss(s);
        double result;
        ss >> result;

        Rmat.at<double>(iline, icol)= result;

        icol++;
      }
    }

    numbers.push_back(Rmat);
  }

  fin.close();

  return true;
}

/**
* write estimated position and orientation (or whatever is saved in the Mat m) inside
* the given filename. The data will be appended if something else already exists in the file.
*
* @param m : matrix with estimated position or orientation, or something else with double values.
* @param filename : filename including path to the txt file in which m will be saved.
* Each double is separated with a ",".
*
*/
void IOslam::writeMatToFile(const cv::Mat& m, const char* filename)
{
  std::ofstream fout;
  fout.open(filename, std::ios_base::app);

  if (!fout)
  {
    std::cout << "File Not Opened" << std::endl;  
    return;
  }

  // for each row
  for (int i = 0; i<m.rows; i++)
  {
    // write all columns except the last, separated with a ,
    for (int j = 0; j<m.cols - 1; j++)
    {
      fout << m.at<double>(i, j) << ",";
    }
    // write the last column
    fout << m.at<double>(i, m.cols - 1);
    // a column vector is saved in a transposed form
    if (m.cols == 1 && i < m.rows - 1)
      fout << ",";

    // a new line separates two rows, in case we do not have a column vector
    if (m.cols > 1 || i == m.rows - 1)
      fout << std::endl;
  }

  fout.close();
}

/**
* write vector of 3d points to 
* the given filename. The data will be appended if something else already exists in the file.
*
* @param vec : vector of 3D points with double/float values.
* @param filename : filename including path to the txt file in which vec will be saved.
* Each double is separated with a ",".
*
*/
void IOslam::write3DPointVecToFile(const std::vector<cv::Point3f>& vec, const char* filename)
{
  std::ofstream fout(filename);
  //fout.open(filename, std::ios_base::app);

  if (!fout)
  {
    std::cout << "File Not Opened" << std::endl;
    return;
  }

  // for each row
  for (int i = 0; i < vec.size(); i++)
  {
    fout << vec.at(i).x << "," << vec.at(i).y << "," << vec.at(i).z << std::endl;
  }

  fout.close();
}

/**
* write vector of 2d points to
* the given filename. The data will be appended if something else already exists in the file.
*
* @param vec : vector of 2D points with double/float values.
* @param filename : filename including path to the txt file in which vec will be saved.
* Each double is separated with a ",".
*
*/
void IOslam::write2DPointVecToFile(const std::vector<cv::Point2f>& vec, const char* filename)
{
  std::ofstream fout(filename);
  //fout.open(filename, std::ios_base::app);

  if (!fout)
  {
    std::cout << "File Not Opened" << std::endl;
    return;
  }

  // for each row
  for (int i = 0; i < vec.size(); i++)
  {
    fout << vec.at(i).x << "," << vec.at(i).y << std::endl;
  }

  fout.close();
}

/**
* write components of v inside
* the given filename. The data will be appended if something else already exists in the file.
*
* @param v : vector double values.
* @param filename : filename including path to the txt file in which m will be saved.
* Each double is separated with a ",".
*
*/
void IOslam::write2DPointToFile(const cv::Point2f& v, const char* filename)
{
  std::ofstream fout;
  fout.open(filename, std::ios_base::app);

  if (!fout)
  {
    std::cout << "File Not Opened" << std::endl;
    return;
  }

  fout << v.x << "," << v.y << std::endl;

  fout.close();
}



