#include "Planner.h"

Planner::Planner(Map _rawMap) {
	rawMap = _rawMap;
	int width = rawMap.getWidth();
	int height = rawMap.getHeight();
	int miniWidth = width / scale + 1;
	int miniHeight = height / scale + 1;
	map = Map(miniHeight, miniWidth, CV_8U);

	for (int i = 0; i < miniWidth; i++) {
		for (int j = 0; j < miniHeight; j++) {
			int flag = 0;
			for (int di = 0; di < scale; di++) {
				for (int dj = 0; dj < scale; dj++) {
					if (i * scale + di < width && j * scale + dj < height) {
						if (rawMap.getPoint(j * scale + dj, i * scale + di) < 127) {
							flag = 1;
							break;
						}
					}
				}
				if (flag == 1) {
					break;
				}
			}
			if (flag == 0) {
				map.setPoint(j, i, 255);
			}
			else {
				map.setPoint(j, i, 0);
			}
 		}
	}
	mirror = new Point<int>**[miniHeight];
	for (int i = 0; i < miniHeight; i++) {
		mirror[i] = new Point<int>*[miniWidth];
	}
	for (int i = 0; i < miniHeight; i++) {
		for (int j = 0; j < miniWidth; j++) {
			mirror[i][j] = new Point<int>[360];
		}
	}
}
Planner::~Planner() {
	int miniHeight = map.getHeight();
	int miniWidth = map.getWidth();
	for (int i = 0; i < miniHeight; i++) {
		for (int j = 0; j < miniWidth; j++)
			delete []mirror[i][j];
	}	
	for (int i = 0; i < miniHeight; i++)
		delete []mirror[i];
	delete []mirror;
}
std::vector<Point<int>> Planner::getPath() {
	std::cout << miniStartPoint.toString() << "->" << miniEndPoint.toString() << std::endl;
	int t = 0;
	Point<int> pausePoint = miniEndPoint;
	while (!openList.empty()) {
		Point<int> currentPoint = findMinValue(openList);
		if (currentPoint == miniEndPoint) {
			break;
		}
		trajec_state start = { mirror[currentPoint.i][currentPoint.j][currentPoint.theta].i, mirror[currentPoint.i][currentPoint.j][currentPoint.theta].j, mirror[currentPoint.i][currentPoint.j][currentPoint.theta].theta / 180.0 * 3.14 };
		trajec_state end = { endPoint.i, endPoint.j, endPoint.theta / 180.0 * 3.14 };
		planning.Path_Generating(start, end);
		if (planning.state_data.size() > 0) {
			pausePoint = currentPoint;
			break;
		}
		double dist = dividedMap.getPoint(currentPoint);
		std::set<Point<int>>::iterator iter = openList.find(currentPoint);
		openList.erase(iter);
		closeList.insert(currentPoint);
		//std::cout << 'C' << currentPoint.toString() << std::endl;
		//std::cout << 'R' << mirror[currentPoint.i][currentPoint.j][currentPoint.theta].toString() << std::endl;
		for (int i = 0; i < deltaNum; i++) {
			double currentG = fG[currentPoint.theta].getPoint(currentPoint);
			Point<int> target = transform(mirror[currentPoint.i][currentPoint.j][currentPoint.theta], i);


			Point<int> newPoint = convertToMinimap(target);
			if (!map.isAvaliable(newPoint)) {
				continue;
			}
			double length = currentPoint.distance(newPoint);
			double newCost = length;
			if (currentPoint.theta != target.theta) {
				newCost += length * 1.5;
			}		
		
			if (dividedMap.getPoint(newPoint) < dist) {
				newCost += length * 3;
			}
			else {
				newCost -= length * 0.5;
			}
			std::cout << 'N' << newPoint.toString() << ' ' << target.toString() << std::endl;

			std::set<Point<int>>::iterator closeIter = closeList.find(newPoint);
			if (closeIter == closeList.end()) {
				std::set<Point<int>>::iterator openIter = openList.find(newPoint);
				double newG = currentG + newCost;
				if (openIter == openList.end()) {
					openList.insert(newPoint);
					fG[newPoint.theta].setPoint(newPoint, newG);
					prefix[newPoint] = currentPoint;
					mirror[newPoint.i][newPoint.j][newPoint.theta] = target;
				}
				else if (newG < fG[newPoint.theta].getPoint(newPoint)) {
					fG[newPoint.theta].setPoint(newPoint, newG);
					prefix[newPoint] = currentPoint;
					mirror[newPoint.i][newPoint.j][newPoint.theta] = target;
				}
			}
		}
		t++;
		//
	}
	std::cout << t << " nodes extended" << std::endl;
	std::vector<Point<int>> path;
	if (!openList.empty()) {
		//std::cout << fG[endPoint.theta].getPoint(miniEndPoint) << std::endl;
		Point<int> lastPoint = pausePoint;
		while (lastPoint != pausePoint) {
			//std::cout << lastPoint.toString() << std::endl;
			path.push_back(mirror[lastPoint.i][lastPoint.j][lastPoint.theta]);
			lastPoint = prefix[lastPoint];
		}
		int dataSize = planning.state_data.size();
		for (int i = dataSize - 1; i >= 0; i--) {
			path.push_back(Point<int>(planning.state_data[i].x, planning.state_data[i].y, planning.state_data[i].theta / 3.14 * 180));
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
	miniStartPoint = convertToMinimap(startPoint);
	openList.clear();
	prefix.clear();
	openList.insert(miniStartPoint);
	mirror[miniStartPoint.i][miniStartPoint.j][miniStartPoint.theta] = startPoint;
}

void Planner::setEndPoint(Point<int> _endPoint) {
	endPoint = _endPoint;
	miniEndPoint = convertToMinimap(endPoint);
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
				fH[theta].setPoint(i, j, miniEndPoint.distance(Point<int>(i, j)));
			}
		}
	}
	breadthFirstSearch();
}

double Planner::computeFunctionF(Point<int> point) {
	//return fG[point.theta].getPoint(point) + fH[point.theta].getPoint(point);
	//std::cout << fG[point.theta].getPoint(point) << ' ' << fH2.getPoint(mirror[point.i][point.j][point.theta]) << std::endl;
	return fG[point.theta].getPoint(point) + fH2.getPoint(mirror[point.i][point.j][point.theta]);
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
	double transformArray[deltaNum][2] = { { 1.7, 0 }, { 2, -30 }, { 2, 30 } };
	Point<int> newPoint;
	//std::cout << transformArray[index][0] << ' ' << transformArray[index][1] << std::endl;
	newPoint.theta = currentPoint.theta + transformArray[index][1];
	newPoint.i = currentPoint.i - round(transformArray[index][0] * sin(newPoint.theta / 180.0 * M_PI));
	newPoint.j = currentPoint.j + round(transformArray[index][0] * cos(newPoint.theta / 180.0 * M_PI));
	if (newPoint.theta < 0) newPoint.theta += 360;
	if (newPoint.theta >= 360) newPoint.theta -= 360;
	return newPoint;
}

Point<int> Planner::convertToMinimap(Point<int> _point) {
	return Point<int>(ceil(_point.i / scale * 1.0), ceil(_point.j / scale * 1.0), _point.theta);
}

void Planner::breadthFirstSearch() {
	const int di[8] = { -1, -1, -1,  0, 0,  1, 1, 1 };
	const int dj[8] = { -1,  0,  1, -1, 1, -1, 0, 1 };
	std::list<Point<int>> queue;
	queue.push_back(endPoint);
	fH2 = Map(rawMap.getHeight(), rawMap.getWidth());
	Map mark(rawMap.getHeight(), rawMap.getWidth(), CV_8UC1);
	mark.setPoint(endPoint, 1);
	while (!queue.empty()) {
		Point<int> current = *queue.begin();
		double currentCost = fH2.getPoint(current);
		queue.pop_front();
		for (int k = 0; k < 8; k++) {
			int newI = current.i + di[k];
			int newJ = current.j + dj[k];
			if (rawMap.isAvaliable(Point<int>(newI, newJ)) && mark.getPoint(newI, newJ) < 1) {
				queue.push_back(Point<int>(newI, newJ));
				mark.setPoint(newI, newJ, 1);
				double dist = dividedMap.getPoint(newI, newJ);
				if (dist > 5) {
					fH2.setPoint(newI, newJ, currentCost + 1);
				}
				else {
					fH2.setPoint(newI, newJ, currentCost + 2);
				}
			}
		}
	}
}

cv::Mat Planner::getVoronoiDiagram() {
	std::list<Point<int>> queue;
	int width = map.getWidth();
	int height = map.getHeight(); 
	dividedMap = Map(height, width, CV_32F);
	dividedColorMap = Map(height, width, CV_32S);
	const int di[8] = { -1, -1, -1, 0, 0, 1, 1, 1 };
	const int dj[8] = { -1, 0, 1, -1, 1, -1, 0, 1 };
	int areaNum = 0;
	std::cout << height << ' ' << width << std::endl;

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int state = map.getPoint(i, j);
			if (state < 128) {
				if (dividedColorMap.getPoint(i, j) == 0) {
					areaNum++;
					divideArea(i, j, areaNum);
					/*std::cout << dividedColorMap.getMap() << std::endl;
					std::cout << std::endl;*/
				}
			}
			else {
				dividedColorMap.setPoint(i, j, 0);
			}
		}
	}
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int state = map.getPoint(i, j);
			if (state < 128) {
				Point<int> p = Point<int>(i, j);
				p.value = 0;
				queue.push_back(p);
				dividedMap.setPoint(i, j, 0);
			}
			else {
				dividedMap.setPoint(i, j, -1);
			}
		}
	}
	while (!queue.empty()) {
		Point<int> current = *queue.begin();
		//std::cout << dividedColorMap.getMap() << std::endl;
		int color = dividedColorMap.getPoint(current.i, current.j);
		//std::cout << current.toString() << ' ' <<  color << std::endl;
		queue.pop_front();
		for (int k = 0; k < 8; k++) {
			int newI = current.i + di[k];
			int newJ = current.j + dj[k];
			if (newI >= 0 && newJ >= 0 && newI < height && newJ < width) {
				//std::cout << dividedMap.getPoint(newI, newJ) << std::endl;
				if (dividedMap.getPoint(newI, newJ) == -1) {
					dividedMap.setPoint(newI, newJ, current.value + 1);
					dividedColorMap.setPoint(newI, newJ, color);
					Point<int> p(newI, newJ);
					p.value = current.value + 1;
					queue.push_back(p);
				}
			}
		}
	}
	cv::namedWindow("color", 1);
	cv::Mat m = dividedColorMap.getMap();
	m.convertTo(m, CV_8U);
	cv::normalize(m, m, 255, 0, cv::NORM_MINMAX);
	cv::imshow("color", m);
	return dividedMap.getMap();
}

void Planner::divideArea(int i, int j, int num) {
	std::list<Point<int>> queue;
	const int di[8] = { -1, -1, -1, 0, 0, 1, 1, 1 };
	const int dj[8] = { -1, 0, 1, -1, 1, -1, 0, 1 };
	int width = map.getWidth();
	int height = map.getHeight();
	queue.push_back(Point<int>(i, j));
	while (!queue.empty()) {
		Point<int> current = *queue.begin();
		queue.pop_front();
		for (int k = 0; k < 8; k++) {
			int newI = current.i + di[k];
			int newJ = current.j + dj[k];
			if (newI >= 0 && newJ >= 0 && newI < height && newJ < width) {
				if (dividedColorMap.getPoint(newI, newJ) != num && map.getPoint(newI, newJ) < 128) {
					dividedColorMap.setPoint(newI, newJ, num);
					queue.push_back(Point<int>(newI, newJ));
				}
			}
		}
	}
}