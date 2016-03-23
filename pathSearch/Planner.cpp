#include "Planner.h"

Planner::Planner(Map _map) {
	map = _map;
}

std::vector<Point<int>> Planner::getPath() {
	int t = 0;
	while (!openList.empty()) {
		Point<int> currentPoint = findMinValue(openList);
		if (currentPoint == endPoint) {
			break;
		}
		std::set<Point<int>>::iterator iter = openList.find(currentPoint);
		openList.erase(iter);
		closeList.insert(currentPoint);
		//std::cout << 'C' << currentPoint.toString() << std::endl;
		for (int i = 0; i < deltaNum; i++) {
			double currentG = fG[currentPoint.theta].getPoint(currentPoint);
			double newCost = 1;
			Point<int> newPoint = transform(currentPoint, i);
			if (newPoint.theta != currentPoint.theta) {
				newCost += 30;
			}
			//std::cout << 'N' << newPoint.toString() << std::endl;

			if (!map.isAvaliable(newPoint)) {
				continue;
			}
			std::set<Point<int>>::iterator closeIter = closeList.find(newPoint);
			if (closeIter == closeList.end()) {
				std::set<Point<int>>::iterator openIter = openList.find(newPoint);
				double newG = currentG + newCost;
				if (openIter == openList.end()) {
					openList.insert(newPoint);
					fG[newPoint.theta].setPoint(newPoint, newG);
					prefix[newPoint] = currentPoint;
					if (fG[endPoint.theta].getPoint(endPoint) > 0)
						std::cout << newPoint.toString() << ' ' << currentPoint.toString() << std::endl;
				}
				else if (newG < fG[newPoint.theta].getPoint(newPoint)) {
					fG[newPoint.theta].setPoint(newPoint, newG);
					prefix[newPoint] = currentPoint;
					if (fG[endPoint.theta].getPoint(endPoint) > 0)
						std::cout << newPoint.toString() << ' ' << currentPoint.toString() << std::endl;
				}
			}
		}
		t++;
		std::cout << t << std::endl;
	}
	std::vector<Point<int>> path;
	if (!openList.empty()) {
		std::cout << fG[endPoint.theta].getPoint(endPoint) << std::endl;
		Point<int> lastPoint = endPoint;
		while (lastPoint != startPoint) {
			//std::cout << lastPoint.toString() << std::endl;
			path.push_back(lastPoint);
			lastPoint = prefix[lastPoint];
		}
		path.push_back(startPoint);
	}
	else {
		std::cout << "no solution" << std::endl;
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
	for (int theta = 0; theta < 360; theta++) {
		fG[theta] = Map(height, width);
		fH[theta] = Map(height, width);
	}
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			for (int theta = 0; theta < 360; theta++) {
				fH[theta].setPoint(i, j, abs(endPoint.i - i) + abs(endPoint.j - j));
			}
		}
	}
}

double Planner::computeFunctionF(Point<int> point) {
	return fG[point.theta].getPoint(point) + fH[point.theta].getPoint(point);
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
	int transformArray[deltaNum][2] = { { 1, 0 }, { 0, -90 }, { 0, 90 } };
	Point<int> newPoint;
	//std::cout << transformArray[index][0] << ' ' << transformArray[index][1] << std::endl;
	newPoint.theta = currentPoint.theta + transformArray[index][1];
	newPoint.i = currentPoint.i - round(transformArray[index][0] * sin(newPoint.theta / 180.0 * M_PI));
	newPoint.j = currentPoint.j + round(transformArray[index][0] * cos(newPoint.theta / 180.0 * M_PI));
	if (newPoint.theta < 0) newPoint.theta += 360;
	if (newPoint.theta >= 360) newPoint.theta -= 360;
	return newPoint;
}