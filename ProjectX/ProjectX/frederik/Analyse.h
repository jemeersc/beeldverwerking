#if !defined ANALYSE
#define ANALYSE
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
using namespace std;
//using namespace cv;
class Analyse{
public:
	void readallpics(cv::String directory);
	void processimage(cv::Mat& image);
	cv::Mat findEdgeLines(cv::Mat image);
	cv::Mat findEdges(cv::Mat image);
	cv::Mat findRoadMarkings(cv::Mat image);
	cv::Mat findLines(cv::Mat contours, cv::Mat image);
	cv::Mat noiseFilter(cv::Mat image);
};



#endif