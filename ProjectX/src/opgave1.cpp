#include <cstdio>
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

using namespace cv;
using namespace std;

int main(int argc, char **argv){
	printf("Lege template voor labo beeldverwerking: %s\n",argv[0]);

	Mat img = imread("clouds.png");
	namedWindow("Wolkjes");
	imshow("Wolkjes", img);
	waitKey(10000);

	cout << "Press a key to exit...";
	cin.get();
	return 0;
}
