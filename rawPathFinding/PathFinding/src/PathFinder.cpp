//
//  PathFinder.cpp
//  PathFinding
//
//  Created by 赵常凯 on 16/12/19.
//  Copyright © 2016年 Roby. All rights reserved.
//

#include "PathFinder.hpp"
#include <map>
#include <queue>

using namespace std;
/**
 ===============================
    A* path finding implements
 ===============================
 **/

typedef tuple<int,int> Location;
namespace std {
    template <>
    struct hash<tuple<int,int> > {
        inline size_t operator()(const tuple<int,int>& location) const {
            int x, y;
            tie(x, y) = location;
            return x * 1812433253 + y;
        }
    };
}


//inline int heuristic(Point2i a, Point2i b) {
//    return abs(a.x - b.x) + abs(a.y - b.y);
//}
inline double heuristic(Location a, Location b) {
    int x1, y1, x2, y2;
    tie(x1, y1) = a;
    tie(x2, y2) = b;
    return abs(x1 - x2) + abs(y1 - y2);
}


template<typename T, typename priority_t>
struct PriorityQueue {
    typedef pair<priority_t, T> PQElement;
    priority_queue<PQElement, vector<PQElement>,std::greater<PQElement>> elements;
    
    inline bool empty() const { return elements.empty(); }
    
    inline void put(T item, priority_t priority) {
        elements.emplace(priority, item);
    }
    
    inline T get() {
        T best_item = elements.top().second;
        elements.pop();
        return best_item;
    }
};

std::basic_iostream<char>::basic_ostream& operator<<(std::basic_iostream<char>::basic_ostream& out, tuple<int,int> loc) {
    int x, y;
    tie (x, y) = loc;
    out << '(' << x << ',' << y << ')';
    return out;
}

void a_star_search(const MapObj& graph,Point2i _start,Point2i _goal,vector<Point2i>& _path){
    Location start(_start.x,_start.y);
    Location goal(_goal.x,_goal.y);
//    std::cout<<_start.x<<","<<_start.y<<"  "<<_goal.x<<","<<_goal.y<<std::endl;
//    std::cout<<start<<"  "<<goal<<std::endl;
    //typedef typename Graph::Location Location;
    PriorityQueue<Location, long> frontier;
    frontier.put(start, 0);
    map<Location,Location> came_from;
    map<Location,long> cost_so_far;
    came_from[start] = start;
    cost_so_far[start] = 0;

    while (!frontier.empty()) {
        auto current = frontier.get();
        
        if (current == goal) {
            break;
        }
        
        for (auto _next : graph.neighbors(Point2i(get<0>(current),get<1>(current)))) {
            Location next(_next.x,_next.y);
            //std::cout<<next<<std::endl;
            //temp.at<uchar>(_next.y,_next.x) = 128;
            long new_cost = cost_so_far[current]; //+ graph.cost(current, next);
            if (!cost_so_far.count(next) || new_cost < cost_so_far[next]) {
                cost_so_far[next] = new_cost;
                long priority = new_cost + (long)heuristic(next, goal);
                frontier.put(next, priority);
                came_from[next] = current;
            }
        }
    }

    //vector<Location> path;
    Location current = goal;
    _path.push_back(Point2i(get<0>(current),get<1>(current)));
    while (current != start) {
        int x1,x2,y1,y2;
        Location next = came_from[current];
        tie (x1,y1) = current;
        tie (x2,y2) = next;
        if (abs(x1-x2) > 1 || abs(y1 - y2)>1) {
            printf("No path found!");
            break;
        }
        current = next;
        _path.push_back(Point2i(get<0>(current),get<1>(current)));
    }
    //_path.push_back(Point2i(get<0>(start),get<1>(start))); // optional
    std::reverse(_path.begin(), _path.end());
    
}


/**
 ====================================
    end of A* path finding
 ===================================
 **/
DIR getdir(Point2i origin,Point2i goal){
    if ((goal.x - origin.x) > 0 && goal.y == origin.y) {
        return DIR::RIGTH;
    }else if((goal.x - origin.x) < 0 && goal.y == origin.y){
        return DIR::LEFT;
    }else if((goal.y - origin.y) > 0 && goal.x == origin.x){
        return DIR::DOWN;
    }else if((goal.y - origin.y) < 0 && goal.x == origin.x){
        return DIR::UP;
    }else if((goal.x - origin.x) > 0 && (goal.y - origin.y) > 0){
        return DIR::RD;
    }else if((goal.x - origin.x) > 0 && (goal.y - origin.y) < 0){
        return DIR::RU;
    }else if((goal.x - origin.x) < 0 && (goal.y - origin.y) > 0){
        return DIR::LD;
    }else if((goal.x - origin.x) < 0 && (goal.y - origin.y) < 0){
        return DIR::LU;
    }else{
        return DIR::CENTTER;
    }
}
bool PathFinder::linkable(Point2i start,Point2i end){
    DIR dir = getdir(start, end);
    int x_start = (dir == DIR::RIGTH || dir == DIR::RU || dir == DIR::RD) ? start.x + 1 : ((dir == DIR::LEFT || dir == DIR::LU || dir == DIR::LD) ? end.x + 1 : start.x);
    
    int x_end = (dir == DIR::RIGTH || dir == DIR::RU || dir == DIR::RD) ? end.x : start.x;
    for (int x = x_start; x < x_end; x++) {
        if (dir == DIR::UP || dir == DIR::DOWN) {
            int y_start = (dir == DIR::UP) ? end.y + 1 : start.y + 1;
            int y_end = (dir == DIR::UP) ? start.y : end.y;
            for (int y = y_start; y < y_end;  y++) {
                if (this->map.expandedData.at<uchar>(y,x)) {
                    return false;
                }
            }
        }else{
            int y = (dir == DIR::RIGTH) ? start.y : (int)round((float)(end.y - start.y)*(float)(x - start.x)/(float)(end.x - start.x) + (float)start.y);
            if (this->map.expandedData.at<uchar>(y,x)) {
                return false;
            }
        }

    }
    return true;
}

DIR PathFinder::getmovedir(Point2i point){
    int offset = this->offset;
    bool rightup = this->map.expandedData.at<uchar>(point.y-offset,point.x+offset) ? true : false;
    bool rightdown = this->map.expandedData.at<uchar>(point.y+offset,point.x+offset) ? true : false;
    bool leftup = this->map.expandedData.at<uchar>(point.y-offset,point.x-offset) ? true : false;
    bool leftdown = this->map.expandedData.at<uchar>(point.y+offset,point.x-offset) ? true : false;

    if (rightup && rightdown && !leftup && !leftdown) {
        return DIR::LEFT;
    }
    
    if (rightup && !rightdown && !leftup && !leftdown) {
        return DIR::LD;
    }
    
    if (!rightup && rightdown && !leftup && !leftdown) {
        return DIR::LU;
    }
    if (!rightup && !rightdown && leftup && leftdown) {
        return DIR::RIGTH;
    }
    if (rightup && !rightdown && leftup && !leftdown) {
        return DIR::DOWN;
    }
    if (!rightup && rightdown && !leftup && leftdown) {
        return DIR::UP;
    }
    if (!rightup && !rightdown && !leftup && leftdown) {
        return DIR::RU;
    }
    if (!rightup && !rightdown && leftup && !leftdown) {
        return DIR::RD;
    }
    if (rightup && rightdown && leftup && !leftdown) {
        return DIR::LD;
    }
    
    if (rightup && rightdown && !leftup && leftdown) {
        return DIR::LU;
    }
    if (!rightup && rightdown && leftup && leftdown) {
        return DIR::RU;
    }
    if (rightup && !rightdown && leftup && leftdown) {
        return DIR::RD;
    }
    return DIR::CENTTER;
}
void PathFinder::Movepoint(Point2i &p, DIR dir){
    switch (dir) {
        case DIR::LD:
            p = Point2i(p.x-this->offset,p.y+this->offset);
            break;
        case DIR::LU:
            p = Point2i(p.x-this->offset,p.y-this->offset);
            break;
        case DIR::RD:
            p = Point2i(p.x+this->offset,p.y+this->offset);
            break;
        case DIR::RU:
            p = Point2i(p.x+this->offset,p.y-this->offset);
            break;
        case DIR::RIGTH:
            p = Point2i(p.x+this->offset,p.y);
            break;
        case DIR::LEFT:
            p = Point2i(p.x-this->offset,p.y);
            break;
        case DIR::UP:
            p = Point2i(p.x,p.y-this->offset);
            break;
        case DIR::DOWN:
            p = Point2i(p.x,p.y+this->offset);
            break;
        default:
            break;
    }
}

void PathFinder::combinePath(vector<Point2i> &path){
    if (path.size() <= 2) {
        return;
    }
    assert(path.size() > 2);
    Point2i start = path[0];
    vector<Point2i> temp = path;
    path.clear();
    path.push_back(start);
    int id = 2;
    for (; id < temp.size();id++) {
        if (linkable(start, temp[id])) {
            //path.push_back(temp[id]);
            //start = temp[id-2];
        }else{
            path.push_back(temp[id-1]);
            start = temp[id-1];
        }
    }
    path.push_back(temp[id-1]);
}
float PathFinder::distance(Point2i start,Point2i end){
    return floor(sqrt((start.x - end.x)*(start.x - end.x)+(start.y-end.y)*(start.y-end.y)));
}
void PathFinder::adjustPoints(vector<Point2i> &path){
    //adjust point
    for (int ind=1; ind < path.size()-1; ind++) {
        Point2i tempP = path[ind];
        DIR dir = getmovedir(tempP);
        Movepoint(tempP, dir);
        if (linkable(path[ind-1], tempP) && linkable(tempP, path[ind+1]) && !this->map.expandedData.at<uchar>(tempP.y,tempP.x)) {
            Movepoint(path[ind], dir);
        }
    }
    combinePath(path);
}

void PathFinder::refinePath(vector<Point2i> _path,vector<Point2i>& refinedPath){
    assert(_path.size()>0);
    refinedPath.push_back(_path[0]);
    DIR direction = getdir(_path[0], _path[1]);
    int i = 2;
    while (i < _path.size()) {
        DIR tempdir = getdir(_path[i-1], _path[i]);
        if (tempdir != direction) {
            refinedPath.push_back(_path[i]);
            direction = tempdir;
        }
        i++;
    }
    refinedPath.push_back(_path[i-1]);
    
    
    //
    combinePath(refinedPath);
    //combinePath(refinedPath);
    while(true){
        unsigned long oldcount = refinedPath.size();
        adjustPoints(refinedPath);
        unsigned long newcount = refinedPath.size();
        if (newcount == oldcount) {
            break;
        }
    }
    

}
Point2i PathFinder::mm2pixl(Point2f p){
    int x = (int)roundf(p.x/this->map.presicion);
    int y = (int)roundf(p.y/this->map.presicion);
    return Point2i(x,y);
}

void PathFinder::init(const cv::string& mapimgPath,float mindis,float pres,float Mapwidht,float Mapheight,bool BlackWall,float _robotRadius,float expandfac){
    this->offset = 5;
    this->robotRadius = _robotRadius;
    this->RobotRadiux = (int)ceilf(_robotRadius/pres);
    this->mindis = mindis;
    if (this->map.data.empty()) {
        this->map.MapGenerate(mapimgPath,pres,Mapwidht,Mapheight,BlackWall,expandfac,this->RobotRadiux);
    }    
}


PathFinder::PathFinder(const cv::string& mapimgPath,float mindis,float pres,float Mapwidht,float Mapheight,bool BlackWall,float robotRadius){
    init(mapimgPath,mindis,pres,Mapwidht,Mapheight,BlackWall,robotRadius,0);
}
PathFinder::PathFinder(const cv::string& mapimgPath,float mindis,float pres,float Mapwidht,float Mapheight,bool BlackWall,float robotRadius,float expandfactor){
    init(mapimgPath,mindis,pres,Mapwidht,Mapheight,BlackWall,robotRadius,expandfactor);
}
void PathFinder::singlepathPlan(Point2f start, Point2f goal, vector<Point2f> &path){
    vector<Point2i> _path;
    
    
    Point2i  _start = mm2pixl(start);
    Point2i _goal = mm2pixl(goal);
    if (this->map.inbound(_start)&&this->map.inbound(_goal)&&!this->map.againstwall(_start, true)&&!this->map.againstwall(_goal, true)) {

        a_star_search(this->map, _start,_goal,_path);//, came_from, cost_so_far);
        
        
        //===============
        //   DEBUG - Begin
        //===============
        cv::Mat temp;
        this->map.data.copyTo(temp);
        circle(temp, _start, 45, Scalar(128),5);
        circle(temp, _goal, 45, Scalar(128),5);
        for (int i = 1; i<_path.size(); i++) {
            temp.at<uchar>(_path[i].y,_path[i].x) = 180;
        }
        
        vector<Point2i> refinedpath;
        //===============
        //   DEBUG - End
        //===============
        refinePath(_path,refinedpath);
        //===============
        //   DEBUG - Begin
        //===============
        for (int i = 1; i<refinedpath.size(); i++) {
            circle(temp, refinedpath[i], 6, Scalar(128),1);
            line(temp, refinedpath[i-1], refinedpath[i], Scalar(128),1);

        }
        imshow("res",temp);
        //===============
        //   DEBUG - End
        //===============
        path = refinedpath;

    }else{
        Mat temp;
        this->map.data.copyTo(temp);
        string name = "test";
        circle(temp, _start, 5, Scalar(128),3);
        circle(temp, _goal, 5, Scalar(128),3);

        
        imshow(name, temp);
        printf("Start or goal point is not valid!");
    }
    

}

void PathFinder::multipathPlan(vector<Point2f> spots, vector<Point2f> &path){

}