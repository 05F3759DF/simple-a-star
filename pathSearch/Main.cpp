#include <iostream>
#include "Map.h"
#include "Planner.h"
using namespace std;
#pragma comment(lib, "opencv_core2412d.lib")
#pragma comment(lib, "opencv_highgui2412d.lib")
#pragma comment(lib, "opencv_imgproc2412d.lib")
#pragma comment(lib, "opencv_contrib2412d.lib")

int main() {
	Map map = Map("0.jpg");
	Planner planner(map);
	planner.setStartPoint(60, 40, 0);
	planner.setEndPoint(210, 30, 270);

	cout << (planner.getStartPoint()).toString() << endl;
	vector<Point<int>> path = planner.getPath();
	cv::namedWindow("MAP", 0);
	cv::Mat m;
	map.getMap().convertTo(m, CV_8U);
	cv::cvtColor(m, m, CV_GRAY2RGB);
	cout << (m.type() == CV_8UC3)<< endl;
	for (vector<Point<int>>::iterator iter = path.begin(); iter != path.end(); ++iter) {
		cout << iter->toString() << endl;
		m.at<cv::Vec3b>(iter->i, iter->j) = { 0, 0, 255 };
	}
	cv::imshow("MAP", m);
	cv::waitKey();
	cv::destroyAllWindows();
	cout << "quiting..." << endl;
	return 0;
}
 