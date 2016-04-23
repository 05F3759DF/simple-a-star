#include <iostream>
#include <ctime>
#include "Map.h"
#include "Planner.h"

#include "Planning\PathGeneration\kelly_PG.h"
using namespace std;
#ifdef _DEBUG
	#pragma comment(lib, "opencv_core2412d.lib")
	#pragma comment(lib, "opencv_highgui2412d.lib")
	#pragma comment(lib, "opencv_imgproc2412d.lib")
	#pragma comment(lib, "opencv_contrib2412d.lib")
#else
	#pragma comment(lib, "opencv_core2412.lib")
	#pragma comment(lib, "opencv_highgui2412.lib")
	#pragma comment(lib, "opencv_imgproc2412.lib")
	#pragma comment(lib, "opencv_contrib2412.lib")
#endif // DEBUG



int main() {
	Map rawMap = Map("1.png");
	Planner planner(rawMap);
	cv::Mat vono = planner.getVoronoiDiagram();

	planner.setStartPoint(25, 25, 0);
	planner.setEndPoint(210, 240, 0);

	

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
	cv::normalize(bfs, bfs, 255, 0, cv::NORM_MINMAX);
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
		cout << iter1->toString() << endl;
		cv::line(m, cv::Point2i(iter0->j, iter0->i),
			cv::Point2i(iter1->j, iter1->i), { 0, 0, 255 }, 1);
		iter0 = iter1;
	}
	cv::imshow("MAP", m);
	cv::imshow("GRIDMAP", n);
 	cv::namedWindow("Vonoroi", 1);
	vono.convertTo(vono , CV_8U);
	cv::normalize(vono, vono, 255, 0, cv::NORM_MINMAX);
	cv::imshow("Vonoroi", vono);
	cv::waitKey();
	cv::destroyAllWindows();
	cout << "quiting..." << endl;
	return 0;
}