//
//  main.cpp
//  PathFinding
//
//  Created by 赵常凯 on 16/12/19.
//  Copyright © 2016年 Roby. All rights reserved.
//

#include <iostream>
#include "PathFinder.hpp"
#include <opencv2/opencv.hpp>
using namespace cv;
int main(int argc, const char * argv[]) {
    
    //load Map
    //if not have Map load one

    
    //get path from pathfinder
    PathFinder pf("/Development/PathFinding/rawmap/map.png",300,5,8000,4000,true,250);
    vector<Point2f> path;
    pf.singlepathPlan(Point2f(94*5,94*5), Point2f(1500*5,94*5), path);
    
    //chart path show result

    
    waitKey(0);
    return 0;
}
