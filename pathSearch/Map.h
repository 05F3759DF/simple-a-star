#ifndef __MAP_H__
#define __MAP_H__

#include <string>
#include <opencv2/opencv.hpp>
#include "Point.h"
class Map {
public:
	Map();
	Map(int _width, int _height, int type = CV_64F);
	Map(std::string _fielname);
	cv::Mat getMap();
	cv::Mat getShowMap();
	int getWidth();
	int getHeight();
	double getPoint(int _i, int _j);
	double getPoint(Point<int> point);
	void setPoint(int _i, int _j, double x);
	void setPoint(Point<int> point, double x);
	bool isAvaliable(Point<int> point);
private:
	int width, height;
	cv::Mat map;
};

#endif