#pragma once

#include <algorithm>
#include <functional>
#include <vector>
#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Path.h>
#include <map>
#include <queue>
#include <stdio.h>
#include <motion_planning/AStar.hpp>
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
namespace motion_planning{



//using namespace std;

//#define FILENAME "obsGrid.txt"

char* gridData;
int width, height;
AStar::AStar(ros::NodeHandle nh_) : nh(nh_) {
	ROS_INFO("AStar node started");

	
	
	pathPub = nh.advertise<nav_msgs::Path>("/path_plan", 1);
	
	subMap = nh.subscribe<nav_msgs::OccupancyGrid>("/Built_Map", 1, &AStar::MapCallBack, this);

}

void AStar::MapCallBack(nav_msgs::OccupancyGrid map){

	start = new Node[1];
	end = new Node[1];
	// location_msg = *(ros::topic::waitForMessage<geometry_msgs::PointStamped>("START TOPIC",ros::Duration(5)));
	// startx =location_msg.position.x/res_of_cell;
	// starty = location_msg.position.y/res_of_cell;
	// AStar::setStart(startx,starty);
	AStar::setStart(1,1);
	
	AStar::setEnd(3,3);
	for(int j= 0; j < map.data.size(); j++){

			m_data[j] = map.data[j];
	}


	m_width = map.info.width;
	m_height = map.info.height;

	vector<AStar::Node> path;

	path = AStar::compute();
	std::vector<geometry_msgs::PoseStamped> ListCell;
	geometry_msgs::PoseStamped cellIter;
	for(int k = 0; k < path.size(); k++){
		cellIter.header.seq = k;
		cellIter.pose.position.x = path[k].row;
		cellIter.pose.position.y = path[k].col;

		ListCell.push_back(cellIter);
	}
	nav_msgs::Path PlanPath;
	PlanPath.poses = ListCell;
	PlanPath.header.frame_id = "/base_link";
	pathPub.publish(PlanPath);
}
struct CompF {
	bool operator()(const AStar::Node* lhs, const AStar::Node* rhs) {
		return lhs->f > rhs->f; //Used to create min heap
	}
};

struct CompLoc {
	bool operator()(const AStar::Node* lhs, const AStar::Node* rhs) {
		return lhs->id < rhs->id; //Used for tracking cell f values
	}
};


void AStar::setStart(int startRow, int startCol) {
	
	start->row = startRow;
	start->col = startCol;

}
void AStar::setEnd(int endRow, int endCol) {
	end->row = endRow;
	end->col = endCol;
}

//Computes the path
vector<AStar::Node> AStar::compute() {
	vector<Node*> closed_list;
	priority_queue<Node*, vector<Node*>, CompF> open_list;

	//Used to mark f values @ locations
	map<Node*, double, CompLoc> open_map;
	map<Node*, double, CompLoc> closed_map;

	bool goalFound = false;

	//Initializes list with starting node
	open_list.push(start);
	open_map.emplace(start, start->f);


	Node* currentNode;
	Node* s; //successor node pointer

	while (open_list.size() > 0 && goalFound == false) {
		currentNode = open_list.top();
		open_list.pop();

		//Removes marked entry in open_map, as it will be added to the closed one
		if (open_map.find(currentNode) != open_map.end())
			open_map.erase(currentNode);

		//Generate successors
		for (int i = 0; i < NUM_DIRECTIONS; i++) {
			if (currentNode->row + drVals[i] < m_height &&
				currentNode->row + drVals[i] >= 0 &&
				currentNode->col + dcVals[i] < m_width &&
				currentNode->col + dcVals[i] >= 0 &&
				m_data[m_width*(currentNode->row + drVals[i]) + currentNode->col + dcVals[i]] != INVALID_CHAR) {

				//creates new node, computes cost, heuristic, etc.
				s = new Node[1];
				s->row = currentNode->row + drVals[i];
				s->col = currentNode->col + dcVals[i];
				s->g = currentNode->g + 1;
				s->setF(end);
				s->parent = currentNode;
				s->setID(m_width);

				//Chekcs if expanded node is goal,if so breaks
				if (s->row == end->row && s->col == end->col) {
					closed_list.push_back(s);
					std::cout << "Goal Found" << std::endl;
					goalFound = true;
				}

				//Checks maps to verify new node has less cost than existing node @ location
				if (closed_map.find(s) != closed_map.end()) {
					if (closed_map[s] <= s->f) {
						delete s;
						continue;
					}
				}
				if (open_map.find(s) != open_map.end()) {
					if (open_map[s] <= s->f) {
						delete s;
						continue;
					}
					else
						open_map[s] = s->f;
				}
				else
					open_map.emplace(s, s->f);

				//adds valid successor to open list
				open_list.push(s);
			}
			
			if (goalFound) break;
		}

		if (goalFound) break;


		//Pushed current node onto closed list, updates closed_map fi applicable
		closed_list.push_back(currentNode);
		if (closed_map.find(currentNode) == closed_map.end()) {
			closed_map.emplace(currentNode, currentNode->f);
		}
		else {
			if (currentNode->f < closed_map[currentNode])
				closed_map[currentNode] = currentNode->f;
		}

	}
	

	if (goalFound) {
		//Reconstructs path, stores in vector, and returns it
		m_computedPath.clear();
		m_computedPath.push_back(*closed_list.back());
		Node p = *closed_list.back();
		while (p.parent != NULL) {
			m_computedPath.push_back(p);
			p = *p.parent;
		}
		m_computedPath.push_back(p);
	}
	else {
		cout << "Path could not be computed..." << endl;
	}

	//Cleanup memory in closed list
	for (Node* n : closed_list)
		delete n;

	return m_computedPath;
}

//Returns the last computed path
vector<AStar::Node> AStar::getLastComputedPath() {
	return m_computedPath;
}


//Display functions for debugging and visualization
void AStar::displayData() {
	cout << endl;

	for (int i = 0; i < m_height; i++) {
#ifdef _WIN32
		SetConsoleTextAttribute(hConsole, 15);
#endif
		cout << "|";
		for (int j = 0; j < m_width; j++) {
			if (start->row == i && start->col == j) {
#ifdef _WIN32
				SetConsoleTextAttribute(hConsole, 9);
#endif
				cout << 'S' << "|";
			}
			else if (end->row == i && end->col == j) {
#ifdef _WIN32
				SetConsoleTextAttribute(hConsole, 9);
#endif
				cout << 'G' << "|";
			}
			else if (m_data[m_width*i + j] == PATH_CHAR) {
#ifdef _WIN32
				SetConsoleTextAttribute(hConsole, 10);
#endif
				cout << m_data[m_width*i + j] << "|";
			}
			else if (m_data[m_width*i + j] == INVALID_CHAR) {
#ifdef _WIN32
				SetConsoleTextAttribute(hConsole, 12);
#endif
				cout << m_data[m_width*i + j] << "|";
			}
			else {
#ifdef _WIN32
				SetConsoleTextAttribute(hConsole, 15);
#endif
				cout << m_data[m_width*i + j] << "|";
			}
		}
		cout << endl;
	}
	cout << endl;
}


void AStar::displayPath() {
	if (m_computedPath.empty()) {
		cout << endl << "No path to display. Recompute path" << endl;
	}
	else {
		for (Node n : m_computedPath) {
			m_data[m_width*n.row + n.col] = PATH_CHAR;
		}
		displayData();
		for (Node n : m_computedPath) {
			m_data[m_width*n.row + n.col] = VALID_CHAR;
		}
	}
}




}



