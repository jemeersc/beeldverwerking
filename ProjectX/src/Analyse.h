#if !defined ANALYSE
#define ANALYSE
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
using namespace std;
using namespace cv;
class Analyse{
public:
	void readallpics(String directory);
	void processimage(Mat& image);
	Mat findEdgeLines(Mat image);
	Mat findEdges(Mat image);
	Mat findRoadMarkings(Mat image);
	Mat findLines(Mat contours,Mat image);
	Mat noiseFilter(Mat image);
};



#endif