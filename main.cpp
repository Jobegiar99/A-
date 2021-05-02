#include <iostream>
#include <vector>
#include <math.h>
#include <chrono>
#include <thread>
#include <map>
#include<stdlib.h>

using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, second

using namespace std;

struct Point{
	float f;
	float pathCost;
	float heuristic;
	float xPos;
	float yPos;
	Point(float pCost, float hCost, float x, float y): pathCost(pCost), heuristic(hCost), f ( pCost - hCost), xPos(x), yPos(y) {};
};

struct Path{
	Point point;
	Path* previous = nullptr;
	Path(Point p): point(p){};
};

map<Point,Point> path;

Path* obtainSmallestFValuePoint(vector<Path*>& open){
	Path* currentBest = open[0];
	int bestIndex = 0;
	for(int i = 1; i < open.size(); i++ ){

		if ( open[i]->point.f < currentBest->point.f ){

			currentBest = open[i];
			bestIndex = i;

		}
	}
	open.erase(open.begin() + bestIndex);
	return currentBest;
}

bool checkIfExistsOpen(vector<Path*>& open, Point possibleSuccesor){
	for(int i = 0; i < open.size() ; i++ ){
		if ( open[i]->point.xPos == possibleSuccesor.xPos and open[i]->point.yPos == possibleSuccesor.yPos){
			if( possibleSuccesor.f < open[i]->point.f){

				open[i]->point = possibleSuccesor;
				
			}
			return true;
		}
	}
	return false;
}

bool checkIfExistsClosed( vector<Path*> closed, Point possibleSuccesor){
	for(int i = 0; i < closed.size() ; i++ ){
		if ( closed[i]->point.xPos == possibleSuccesor.xPos and closed[i]->point.yPos == possibleSuccesor.yPos){
			return true;
		}
	}
	return false;
}

//Using Manhattan Distance as the agent can only move up, down, right or left.
float getDistance( float nextX, float x, float nextY, float y){

	float xValue = abs( nextX - x);

	float yValue = abs( nextY - y);

	float distance = xValue + yValue;

	return distance;

}

//Creates the possible movements and add them to their corresponding path
Path* obtainSucessors(vector<vector<string>> matrix, vector<Path*>& open, vector<Path*> closed, Path* bestOption, Point goal){

	vector<pair<int,int>> points{
		                 make_pair(0,-1),
		make_pair(-1, 0),                make_pair(1, 0)
		                ,make_pair(0, 1)
	};

	bool foundGoal = false;

	for(int i = 0; i < 8; i++ ){

		float nextX = bestOption->point.xPos + points[i].first;
 		float nextY = bestOption->point.yPos + points[i].second;
		
		if(nextX >= 0 and nextX < matrix.size() and nextY >= 0 and nextY < matrix[0].size()){
			if(matrix[nextX][nextY] != "□"){

				if (nextX == goal.xPos and nextY == goal.yPos){
					foundGoal = true;
				}

				
				float nextG = getDistance( nextX, 0, nextY, 0);
				
				float nextH = getDistance( nextY, goal.xPos, nextY, goal.yPos);

				Point succesor = Point( nextG, nextH, nextX, nextY );

				if(! checkIfExistsOpen(open, succesor) and !checkIfExistsClosed(closed,succesor)){

					Path* nextPoint = new Path(succesor);

					nextPoint -> previous = bestOption;

					open.push_back(nextPoint);
					
				}
			}
		}
	}
	return (foundGoal) ? new Path(goal) : bestOption;
}

void drawPath(Path* bestOption, vector<vector<string>> matrix){
	Path* p = bestOption;
	while ( p ){

		matrix[ p->point.xPos ] [p -> point.yPos ] = "x";
		p = p -> previous;
	}
	for(int i = 0 ; i < matrix.size(); i++ ){
		for(int j = 0 ; j < matrix[0].size(); j++ ){
			cout<<matrix[i][j]<<" ";
		}
		cout<<endl;
	}
	cout<<"-------------------"<<endl;
	auto const start_time = std::chrono::steady_clock::now();
	auto const wait_time = std::chrono::milliseconds{300};
	auto next_time = start_time + wait_time; // regularly updated time point
	std::this_thread::sleep_until(next_time);
    next_time += wait_time; // increment absolute time
	system ("clear");
}

Path* AStar(vector<vector<string>> matrix, Point goal, Point start){
	
	vector<Path*> open{ new Path(start)};
	vector<Path*> closed;
	//path[start] = start;
	while ( ! open.empty() ){
		
		Path* bestOption = obtainSmallestFValuePoint(open);
		Path* possibleGoal = obtainSucessors(matrix, open, closed, bestOption, goal );
		drawPath(bestOption,matrix);
		
		if ( possibleGoal -> point.xPos == goal.xPos and possibleGoal -> point.yPos == goal.yPos){
			
			possibleGoal -> previous = bestOption;
			return possibleGoal;
			
		}
		closed.push_back(bestOption);
	}
	return nullptr;
}


Path* reverse(Path* path){

	Path* p, *q, *r;

	p = nullptr;
	q = path;
	r = path -> previous;

	while(q){
		q -> previous = p;
		p = q;
		q = r;
		if ( r ){
			r = r -> previous;
		}
	}
	return p;
}



int main() {

	vector<vector<string>> matrix{
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "},
		{"□","□","□","□"," ","□","□"," ","□","□","□","□"," ","□","□"," "},
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "},
		{" ","□","□"," ","□","□","□"," "," ","□","□"," ","□","□","□"," "},
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "},
		{"□","□"," "," "," ","□"," ","□","□","□"," "," "," ","□"," ","□"},
		{" "," "," ","□"," ","□"," ","□"," "," "," ","□"," ","□"," ","□"},
		{" ","□"," ","□"," ","□"," ","□"," ","□"," ","□"," ","□"," ","□"},
		{"□","□","□","□"," ","□"," "," ","□","□","□","□"," ","□"," "," "},
		{" "," "," "," "," ","□","□"," "," "," "," "," "," ","□","□"," "},
		{" ","□","□","□","□","□"," "," "," ","□","□","□","□","□"," "," "},
		{" ","□"," ","□"," ","□","□"," "," ","□"," ","□"," ","□","□"," "},
		{" ","□"," ","□"," ","□"," "," "," ","□"," ","□"," ","□"," "," "},
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "},
		{"□","□","□","□"," ","□","□"," ","□","□","□","□"," ","□","□"," "},
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "},
		{" ","□","□"," ","□","□","□"," "," ","□","□"," ","□","□","□"," "},
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "},
		{"□","□"," "," "," ","□"," ","□","□","□"," "," "," ","□"," ","□"},
		{" "," "," ","□"," ","□"," ","□"," "," "," ","□"," ","□"," ","□"},
		{" ","□"," ","□"," ","□"," ","□"," ","□"," ","□"," ","□"," ","□"},
		{"□","□","□","□"," ","□"," "," ","□","□","□","□"," ","□"," "," "},
		{" "," "," "," "," ","□","□"," "," "," "," "," "," ","□","□"," "},
		{" ","□","□","□","□","□"," "," "," ","□","□","□","□","□"," "," "},
		{" ","□"," ","□"," ","□","□"," "," ","□"," ","□"," ","□","□"," "},
		{" ","□"," ","□"," ","□"," "," "," ","□"," ","□"," ","□"," "," "},
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "},
		{"□","□","□","□"," ","□","□"," ","□","□","□","□"," ","□","□"," "},
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "},
		{" ","□","□"," ","□","□","□"," "," ","□","□"," ","□","□","□"," "},
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "},
		{"□","□"," "," "," ","□"," ","□","□","□"," "," "," ","□"," ","□"},
		{" "," "," ","□"," ","□"," ","□"," "," "," ","□"," ","□"," ","□"},
		{" ","□"," ","□"," ","□"," ","□"," ","□"," ","□"," ","□"," ","□"},
		{"□","□","□","□"," ","□"," "," ","□","□","□","□"," ","□"," "," "},
		{" "," "," "," "," ","□","□"," "," "," "," "," "," ","□","□"," "},
		{" ","□","□","□","□","□"," "," "," ","□","□","□","□","□"," "," "},
		{" ","□"," ","□"," ","□","□"," "," ","□"," ","□"," ","□","□"," "},
		{" ","□"," ","□"," ","□"," "," "," ","□"," ","□"," ","□"," "," "},
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","GOAL"},
	};

	vector<vector<string>> copy{
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "},
		{"□","□","□","□"," ","□","□"," ","□","□","□","□"," ","□","□"," "},
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "},
		{" ","□","□"," ","□","□","□"," "," ","□","□"," ","□","□","□"," "},
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "},
		{"□","□"," "," "," ","□"," ","□","□","□"," "," "," ","□"," ","□"},
		{" "," "," ","□"," ","□"," ","□"," "," "," ","□"," ","□"," ","□"},
		{" ","□"," ","□"," ","□"," ","□"," ","□"," ","□"," ","□"," ","□"},
		{"□","□","□","□"," ","□"," "," ","□","□","□","□"," ","□"," "," "},
		{" "," "," "," "," ","□","□"," "," "," "," "," "," ","□","□"," "},
		{" ","□","□","□","□","□"," "," "," ","□","□","□","□","□"," "," "},
		{" ","□"," ","□"," ","□","□"," "," ","□"," ","□"," ","□","□"," "},
		{" ","□"," ","□"," ","□"," "," "," ","□"," ","□"," ","□"," "," "},
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "},
		{"□","□","□","□"," ","□","□"," ","□","□","□","□"," ","□","□"," "},
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "},
		{" ","□","□"," ","□","□","□"," "," ","□","□"," ","□","□","□"," "},
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "},
		{"□","□"," "," "," ","□"," ","□","□","□"," "," "," ","□"," ","□"},
		{" "," "," ","□"," ","□"," ","□"," "," "," ","□"," ","□"," ","□"},
		{" ","□"," ","□"," ","□"," ","□"," ","□"," ","□"," ","□"," ","□"},
		{"□","□","□","□"," ","□"," "," ","□","□","□","□"," ","□"," "," "},
		{" "," "," "," "," ","□","□"," "," "," "," "," "," ","□","□"," "},
		{" ","□","□","□","□","□"," "," "," ","□","□","□","□","□"," "," "},
		{" ","□"," ","□"," ","□","□"," "," ","□"," ","□"," ","□","□"," "},
		{" ","□"," ","□"," ","□"," "," "," ","□"," ","□"," ","□"," "," "},
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "},
		{"□","□","□","□"," ","□","□"," ","□","□","□","□"," ","□","□"," "},
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "},
		{" ","□","□"," ","□","□","□"," "," ","□","□"," ","□","□","□"," "},
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," "," "},
		{"□","□"," "," "," ","□"," ","□","□","□"," "," "," ","□"," ","□"},
		{" "," "," ","□"," ","□"," ","□"," "," "," ","□"," ","□"," ","□"},
		{" ","□"," ","□"," ","□"," ","□"," ","□"," ","□"," ","□"," ","□"},
		{"□","□","□","□"," ","□"," "," ","□","□","□","□"," ","□"," "," "},
		{" "," "," "," "," ","□","□"," "," "," "," "," "," ","□","□"," "},
		{" ","□","□","□","□","□"," "," "," ","□","□","□","□","□"," "," "},
		{" ","□"," ","□"," ","□","□"," "," ","□"," ","□"," ","□","□"," "},
		{" ","□"," ","□"," ","□"," "," "," ","□"," ","□"," ","□"," "," "},
		{" "," "," "," "," "," "," "," "," "," "," "," "," "," "," ","GOAL"},
	};

	Point start = Point(0,0,0,0);
   	Point goal = Point(0,0,copy.size()-1,copy[0].size()-1);
	
    Path* path = reverse(AStar(matrix, goal, start));

	cout<<"------------BEST PATH-----------"<<endl;
	while( path ){

	   Point current = path -> point;
	   cout<<current.xPos<<" "<<current.yPos<<endl;
	   copy[current.xPos][current.yPos] = "x";
	   for(int row = 0; row < copy.size(); row++){
		   for(int column = 0; column < copy[0].size(); column++){
			   cout<< copy[row][column] <<" ";
		   }
		   cout<<endl;
	   }
	   	cout<<"---------------"<<endl;
	    sleep_until(system_clock::now() + seconds(1));
		path = path -> previous;
	}
   
}