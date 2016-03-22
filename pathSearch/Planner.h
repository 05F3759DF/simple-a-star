#ifndef __PLANNER_H__
#define __PLANNER_H__

#include <vector>
#include <set>
#include <map>
#include <algorithm>
#include <cmath>
#include "Map.h"
#include "Point.h"

const int deltaNum = 3;

class Planner {
public:
	Planner(Map _map);
	std::vector<Point<int>> getPath();
	Point<int> getStartPoint();
	Point<int> getEndPoint();
	void setStartPoint(Point<int> _startPoint);
	void setEndPoint(Point<int> _endPoint);
	void setStartPoint(int _x, int _y, int _theta);
	void setEndPoint(int _x, int _y, int _theta);
public:
	Map map, fH, fG;
	Point<int> startPoint, endPoint;
	std::set<Point<int>> openList, closeList;
	void computeFunctionH();
	double computeFunctionF(Point<int> point);
	Point<int> findMinValue(std::set<Point<int>> l);
	std::map<Point<int>, Point<int>> prefix;
	Point<int> transform(Point<int> currentPosition, int index);
};

#endif