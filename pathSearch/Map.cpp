#include "Map.h"

Map::Map() {

}

Map::Map(int _height, int _width, int type) {
	width = _width;
	height = _height;
	map = cv::Mat::zeros(_height, _width, type);
}

Map::Map(std::string _filename) {
	map = cv::imread(_filename, CV_LOAD_IMAGE_GRAYSCALE);
	height = map.rows;
	width = map.cols;
}

cv::Mat Map::getMap() {
	return map;
}

cv::Mat Map::getShowMap() {
	cv::Mat mapn;
	cv::normalize(map, mapn, 255, 0, cv::NORM_MINMAX);
	mapn.convertTo(mapn, CV_8U);
	return mapn;
}

int Map::getWidth() {
	return width;
}

int Map::getHeight() {
	return height;
}

double Map::getPoint(int _i, int _j) {
	if (map.type() == CV_64F) {
		return ((double*)(map.data))[_i * width + _j];
	} else if (map.type() == CV_32F) {
		return ((float*)(map.data))[_i * width + _j];
	} else if (map.type() == CV_8U) {
		return ((uchar*)(map.data))[_i * width + _j];
	}
	else if (map.type() == CV_32S) {
		return ((int*)(map.data))[_i * width + _j];
	}
}

double Map::getPoint(Point<int> point) {
	return getPoint(point.i, point.j);
}

void Map::setPoint(int _i, int _j, double x) {
	if (map.type() == CV_64F) {
		((double*)(map.data))[_i * width + _j] = x;
	} else if (map.type() == CV_32F) {
		((float*)(map.data))[_i * width + _j] = x;
	} else if (map.type() == CV_8U) {
		((uchar*)(map.data))[_i * width + _j] = x;
	}
	else if (map.type() == CV_32S) {
		((int*)(map.data))[_i * width + _j] = x;
	}
}

void Map::setPoint(Point<int> point, double x) {
	setPoint(point.x, point.y, x);
}

bool Map::isAvaliable(Point<int> point) {
	if (point.i < 0 || point.j < 0 || point.i >= height || point.j >= width) {
		return false;
	}
	if (getPoint(point) < 128) {
		return false;
	}
	return true;
}