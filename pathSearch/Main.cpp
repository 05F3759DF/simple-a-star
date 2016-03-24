#include <iostream>
#include "Map.h"
#include "Planner.h"
using namespace std;
#pragma comment(lib, "opencv_core2412d.lib")
#pragma comment(lib, "opencv_highgui2412d.lib")
#pragma comment(lib, "opencv_imgproc2412d.lib")
#pragma comment(lib, "opencv_contrib2412d.lib")

int main() {
	Map rawMap = Map("a.jpg");
	Planner planner(rawMap);
	planner.setStartPoint(125, 60, 0);
	planner.setEndPoint(515, 450, 0);

	cout << (planner.getStartPoint()).toString() << endl;
	vector<Point<int>> path = planner.getPath();
	cv::namedWindow("MAP");
	cv::namedWindow("GRIDMAP");
	cv::Mat m, n = planner.map.getMap();
	rawMap.getMap().convertTo(m, CV_8U);
	n.convertTo(n, CV_8U);
	cv::cvtColor(m, m, CV_GRAY2RGB);
	cv::cvtColor(n, n, CV_GRAY2RGB);
	int height = rawMap.getHeight();
	int width = rawMap.getWidth();
	vector<Point<int>>::iterator iter0 = path.begin();
	for (vector<Point<int>>::iterator iter1 = iter0 + 1; iter1 != path.end(); ++iter1) {
		cout << iter1->toString() << endl;
		m.at<cv::Vec3b>(iter1->i, iter1->j) = { 0, 0, 255 };
		cv::line(m, cv::Point2i(iter0->j, iter0->i),
			cv::Point2i(iter1->j, iter1->i), { 0, 0, 255 }, 1);
		iter0 = iter1;
	}
	cv::imshow("MAP", m);
	cv::imshow("GRIDMAP", n);
	cv::waitKey();
	cv::destroyAllWindows();
	cout << "quiting..." << endl;
	return 0;
}
 