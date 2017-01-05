//
//  PathFinder.hpp
//  PathFinding
//
//  Created by 赵常凯 on 16/12/19.
//  Copyright © 2016年 Roby. All rights reserved.
//

#ifndef PathFinder_hpp
#define PathFinder_hpp

#include <stdio.h>
#include "MapObj.hpp"
using namespace cv;
enum DIR{CENTTER = 0,RIGTH=1,LEFT=2,UP=3,DOWN=4,LU=5,LD=6,RU=7,RD=8};

class PathFinder{
public:
    float robotRadius;    //robot assume to be a circle with radius mm
    int RobotRadiux; //in pix
    MapObj map;
    float mindis;
    int offset;
    //constructor
    PathFinder(const cv::string& mapimgPath,float mindis,float pres,float Mapwidht,float Mapheight,bool BlackWall,float robotRadius);
    PathFinder(const cv::string& mapimgPath,float mindis,float pres,float Mapwidht,float Mapheight,bool BlackWall,float robotRadius,float expandfactor);
    
    //single path plan 
    void singlepathPlan(Point2f start,Point2f goal,vector<Point2f> &path);
    
    
    // multi path plan start from first spot in spots end at last one in spots, so spots count >= 2
    void multipathPlan(vector<Point2f> spots,vector<Point2f> &path);
    
private:
    Point2i mm2pixl(Point2f p);
    void init(const cv::string& mapimgPath,float mindis,float pres,float Mapwidht,float Mapheight,bool BlackWall,float _robotRadius,float expandfac);
    void refinePath(vector<Point2i> _path,vector<Point2i>& refinedPath);
    bool linkable(Point2i start,Point2i end);
    void combinePath(vector<Point2i> &path);
    float distance(Point2i start,Point2i end);
    DIR getmovedir(Point2i point);
    void Movepoint(Point2i &p,DIR dir);
    void adjustPoints(vector<Point2i> &path);
};
#endif /* PathFinder_hpp */
