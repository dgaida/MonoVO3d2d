//

#ifndef IOSLAM_H_
#define IOSLAM_H_

#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>

/**
* contains static methods to read the command line interface, to write a Mat file and to read
* a csv file. 
*/
class IOslam
{
  /*=============================         PUBLIC METHODS         =============================*/

public:

  /*====================         PUBLIC METHODS - DE-/CONSTRUCTORS        ====================*/

  /**
  * standard constructor does nothing
  */
  IOslam();
  
  /**
  * standard destructor does nothing
  */
  virtual ~IOslam();



  /*=====================    PUBLIC METHODS - STATIC METHODS    =======================*/

public:

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
  static int readCmdLineInterface(int argc, char *argv[], float &ratioTest, bool &fast_match, 
    int &numKeyPoints, bool &useRealCam, bool &useGT);

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
  static void readPoint3f_FromFile(std::vector<cv::Point3f> &GT_locs, const char* filename);

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
  static void readPoint2f_FromFile(std::vector<cv::Point2f> &imagePoints, const char* filename);

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
  static bool readFloatVec_FromFile(std::vector<std::vector<float>> &numbers, const char* filename);

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
  static bool readDouble3x3Mat_FromFile(std::vector<cv::Mat> &numbers, const char* filename);

  /**
  * write estimated position and orientation (or whatever is saved in the Mat m) inside 
  * the given filename. The data will be appended if something else already exists in the file. 
  *
  * @param m : matrix with estimated position or orientation, or something else with double values. 
  * @param filename : filename including path to the txt file in which m will be saved. 
  * Each double is separated with a ",".
  *
  */
  static void writeMatToFile(const cv::Mat& m, const char* filename);

  /**
  * write vector of 3d points to
  * the given filename. The data will be appended if something else already exists in the file.
  *
  * @param vec : vector of 3D points with double/float values.
  * @param filename : filename including path to the txt file in which vec will be saved.
  * Each double is separated with a ",".
  *
  */
  static void write3DPointVecToFile(const std::vector<cv::Point3f>& vec, const char* filename);

  /**
  * write vector of 2d points to
  * the given filename. The data will be appended if something else already exists in the file.
  *
  * @param vec : vector of 2D points with double/float values.
  * @param filename : filename including path to the txt file in which vec will be saved.
  * Each double is separated with a ",".
  *
  */
  static void write2DPointVecToFile(const std::vector<cv::Point2f>& vec, const char* filename);

  /**
  * write components of v inside
  * the given filename. The data will be appended if something else already exists in the file.
  *
  * @param v : vector double values.
  * @param filename : filename including path to the txt file in which m will be saved.
  * Each double is separated with a ",".
  *
  */
  static void write2DPointToFile(const cv::Point2f& v, const char* filename);



};

#endif


