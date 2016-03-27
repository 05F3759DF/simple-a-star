#ifndef __POINT_H__
#define __POINT_H__
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
template <class T>
class Point: public cv::Point_<T> {
public:
	T i, j;
	int theta;
	double value;
	Point();
	Point(T _i, T _j);
	Point(T _i, T _j, int _theta);
	Point(cv::Point_<T> point);
	Point(cv::Point_<T> point, int _theta);
	bool operator <(const Point<T>& other) const;
	bool operator >(const Point<T>& other) const;
	bool operator ==(const Point<T>& other) const;
	bool operator !=(const Point<T>& other) const;
	std::string toString() const;
	double distance(Point<T>& other);
};
#include "Point.tpp"

#endif