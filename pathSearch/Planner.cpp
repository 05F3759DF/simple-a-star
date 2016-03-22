#include "Planner.h"

Planner::Planner(Map _map) {
	map = _map;
	fH = Map(_map.getHeight(), _map.getWidth());
	fG = Map(_map.getHeight(), _map.getWidth());
}

std::vector<Point<int>> Planner::getPath() {
	using namespace std;
	int deltaTheta[deltaNum] = { 90, 0, -90 };
	int t = 0;
	while (!openList.empty()) {
		Point<int> currentPoint = findMinValue(openList);
		//cout << currentPoint << endl;
		set<Point<int>>::iterator iter = openList.find(currentPoint);
		openList.erase(iter);
		closeList.insert(currentPoint);
		if (currentPoint == endPoint) {
			break;
		}
		for (int i = 0; i < deltaNum; i++) {
			int newI = currentPoint.i;
			int newJ = currentPoint.j;
			int newCost = deltaTheta[i] == 0 ? 1 : 2;
			if (currentPoint.theta == 0) {
				newJ += 1;
			}
			else if (currentPoint.theta == 90) {
				newI -= 1;
			}
			else if (currentPoint.theta == 180) {
				newJ -= 1;
			}
			else if (currentPoint.theta == 270) {
				newI += 1;
			}
			int newTheta = currentPoint.theta + deltaTheta[i];
			if (newTheta >= 360) {
				newTheta -= 360;
			}
			else if (newTheta < 0) {
				newTheta += 360;
			}
			Point<int> newPoint(newI, newJ, newTheta);
			if (!map.isAvaliable(newPoint)) {
				continue;
			}
			set<Point<int>>::iterator closeIter = closeList.find(newPoint);
			if (closeIter == closeList.end()) {
				set<Point<int>>::iterator openIter = openList.find(newPoint);
				double newG = fG.getPoint(currentPoint) + newCost;
				if (openIter == openList.end()) {
					openList.insert(newPoint);
					fG.setPoint(newPoint, newG);
					prefix[newPoint] = currentPoint;
					//cout << newPoint << ' ' << currentPoint << endl;
				}
				else if (newG < fG.getPoint(newPoint)) {
					fG.setPoint(newPoint, newG);
					prefix[newPoint] = currentPoint;
					//cout << newPoint << ' ' << currentPoint << endl;
				}
			}
		}
	}
	std::vector<Point<int>> path;
	if (fG.getPoint(endPoint) > 0) {
		cout << fG.getPoint(endPoint) << endl;
		Point<int> lastPoint = endPoint;
		while (lastPoint != startPoint) {
			path.push_back(lastPoint);
			lastPoint = prefix[lastPoint];
		}
		path.push_back(startPoint);
	}
	std::reverse(path.begin(), path.end());
	return path;
}

Point<int> Planner::getStartPoint() {
	return startPoint;
}

Point<int> Planner::getEndPoint() {
	return endPoint;
}
 
void Planner::setStartPoint(Point<int> _startPoint) {
	startPoint = _startPoint;
	openList.clear();
	prefix.clear();
	openList.insert(startPoint);
}

void Planner::setEndPoint(Point<int> _endPoint) {
	endPoint = _endPoint;
	computeFunctionH();
}

void Planner::setStartPoint(int _x, int _y, int _theta) {
	setStartPoint(Point<int>(_x, _y, _theta));
}

void Planner::setEndPoint(int _x, int _y, int _theta) {
	setEndPoint(Point<int>(_x, _y, _theta));
}

void Planner::computeFunctionH() {
	int height = map.getHeight();
	int width = map.getWidth();
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			fH.setPoint(i, j, abs(endPoint.i - i) + abs(endPoint.j - j));
		}
	}
}

double Planner::computeFunctionF(Point<int> point) {
	return fH.getPoint(point) + fG.getPoint(point);
}

Point<int> Planner::findMinValue(std::set<Point<int>> l) {
	double minValue = 0x7fffffff;
	Point<int> minPoint;
	for (std::set<Point<int>>::iterator iter = l.begin(); iter != l.end(); ++iter) {
		double f = computeFunctionF(*iter);
		if (f <= minValue) {
			minValue = f;
			minPoint = *iter;
		}
	}
	return minPoint;
}

Point<int> Planner::transform(Point<int> currentPoint, int index) {
	Point<int> transformArray[deltaNum] = { { 0, 1, 0 }, { -1, 1, -90 }, { 1, 1, 90 } };
	return transformArray[index];
}