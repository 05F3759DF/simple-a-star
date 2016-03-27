#include <iostream>
#include <ctime>
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
	
	time_t now_time;
	now_time = time(NULL);
	cout << now_time << endl;

	vector<Point<int>> path = planner.getPath();

	now_time = time(NULL);
	cout << now_time << endl;

	cv::namedWindow("BFS");
	cv::Mat bfs;
	planner.fH2.getMap().convertTo(bfs, CV_8U);
	cv::cvtColor(bfs, bfs, CV_GRAY2RGB);
	for (int i = 0; i < bfs.rows; i++) {
		for (int j = 0; j < bfs.cols; j++) {
			if (planner.fH2.getPoint(i, j) > 200) {
				bfs.at<cv::Vec3b>(i, j) = { 255, 255, 100 };
			}
			else if (planner.fH2.getPoint(i, j) > 100) {
				bfs.at<cv::Vec3b>(i, j) = { 255, 255, 150 };
			}
			else if (planner.fH2.getPoint(i, j) > 50) {
				bfs.at<cv::Vec3b>(i, j) = { 255, 255, 200 };
			}
			else if (planner.fH2.getPoint(i, j) > 0){
				bfs.at<cv::Vec3b>(i, j) = { 255, 255, 255 };
			}
		}
	}
	cv::imshow("BFS", bfs);

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
		//cout << iter1->toString() << endl;
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
 