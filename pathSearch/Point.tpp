#ifndef __POINT_TPP__
#define __POINT_TPP__
#include "Point.h" 

template <class T>
Point<T>::Point() : cv::Point_<T>() {

}

template <class T>
Point<T>::Point(T _i, T _j) : cv::Point_<T>(_i, _j) {
	i = _i;
	j = _j;
}

template <class T>
Point<T>::Point(T _i, T _j, int _theta) : cv::Point_<T>(_i, _j) {
	i = _i;
	j = _j;
	theta = _theta;
}

template <class T>
Point<T>::Point(cv::Point_<T> point) : cv::Point_<T>(point) {
	i = point.x;
	j = point.y;
}

template <class T>
Point<T>::Point(cv::Point_<T> point, int _theta) : cv::Point_<T>(point) {
	i = point.x;
	j = point.y;	
	theta = _theta;
}

template <class T>
bool Point<T>::operator<(const Point<T>& other) const {
	if (i < other.i) {
		return true;
	}
	else if (i == other.i && j < other.j) {
		return true;
	}
	else if (i == other.i && j == other.j && theta < other.theta) {
		return true;
	}
	return false;
}

template <class T>
bool Point<T>::operator >(const Point<T>& other) const {
	if (i > other.i) {
		return true;
	}
	else if (i == other.i && j > other.j) {
		return true;
	}
	else if (i == other.i && j == other.j && theta > other.theta) {
		return true;
	}
	return false;
}

template <class T>
bool Point<T>::operator ==(const Point<T>& other) const {
	return i == other.i && j == other.j && theta == other.theta;
}

template <class T>
bool Point<T>::operator !=(const Point<T>& other) const {
	return i != other.i || j != other.j || theta != other.theta;
}

template <class T>
std::string Point<T>::toString() const {
	return "[" + std::to_string(i) + ", " + std::to_string(j) + ", " + std::to_string(theta) + "]";
}

#endif