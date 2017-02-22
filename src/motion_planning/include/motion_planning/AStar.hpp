#pragma once
#include <algorithm>
#include <functional>
#include <vector>
#include <cmath>
#include <iostream>
#include <map>
#include <queue>
#include <stdio.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
//Checks if compiling on windows (mostly for testing) to include the header
#ifdef _WIN32 
#include <Windows.h>
#endif

using namespace std;

/*
Author: Andrew Petersen

Description: Simple AStar path planning implementation
using STL priority queue and hash maps. Also
provides functions to display grid data to standard
output (color functionality WINDOWS only).

Example Usage:
int main(){

//Creates the path planner
AStar pplan(data, width, height);

//Sets the starting point
pplan.setStart(38, 38);

//Sets the end point
pplan.setEnd(1, 1);

//Displays the data as a grid to standar output
pplan.displayData();

//Computes the path, stores it in a vector of Nodes
vector<AStar::Node> path = pplan.compute();

//Dislays the path
pplan.displayPath();

//NOTE:	The structure path now contains the list of points. Easy to iterate over
and store within a different data structure
}

Where:	"data" = 1d char array in row major format
"width" = width of the grid (# of cols)
"height" = height of the grid (# of rows)

*/

namespace motion_planning {


class AStar {

private:

	/*
	These can be changed based on the format of the grid data
	If the grid data just contain 0s and 1s, you could change it,
	for example, to INVALID_CHAR = 1; VALID_CHAR = 0
	*/

	const char INVALID_CHAR = '1';
	const char VALID_CHAR = '0';
	const char PATH_CHAR = '*';

	const int NUM_DIRECTIONS = 4;
	const int drVals[8] = { 1,-1, 0, 0, 1, 1, -1, -1 };
	const int dcVals[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };

#ifdef _WIN32
	HANDLE hConsole;
#endif

public:
	//Node class used for grid cells
	class Node {
	public:
		int row = 0, col = 0;
		double f = 0, g = 0, h = 0;
		Node* parent = NULL;
		double dx, dy;
		int id = -1;

		void setF(Node* goal) {
			dy = abs(goal->row - row);
			dx = abs(goal->col - col);
			//cross = abs(dx*dy2 - dx2*dy);

			h = dx + dy;
			f = g + h;
		}
		void setID(const int& width) {
			id = row*width + col;
		}
	};

private:

	Node* start;
	Node* end;
	int m_width, m_height;
	char* m_data;
	vector<Node> m_computedPath;
	ros::NodeHandle& nh;
	ros::Subscriber subMap;
	ros::Publisher pathPub;

public:
	AStar(ros::NodeHandle nh_);
	void MapCallBack(nav_msgs::OccupancyGrid map);
	//char* data, int width, int height
	void setStart(int startRow, int startCol);
	void setEnd(int endRow, int endCol);

	//Computes the path
	vector<Node> compute();

	//Returns the last computed path
	vector<Node> getLastComputedPath();

	//Display functions for debugging and visualization
	void displayData();
	void displayPath();

};


}