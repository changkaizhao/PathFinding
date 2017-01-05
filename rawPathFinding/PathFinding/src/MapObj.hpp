//
//  MapObj.hpp
//  PathFinding
//
//  Created by 赵常凯 on 16/12/19.
//  Copyright © 2016年 Roby. All rights reserved.
//

#ifndef MapObj_hpp
#define MapObj_hpp

#include <stdio.h>
#include <opencv2/opencv.hpp>
class MapObj{
    
public:
    
    
    /**
     Map coordinates (mm)  
     
     +---------------> x
     |(0,0)
     |
     |
     |
     |
     V y
     
     
     **/
    float MaprealWidth;   //Map width unit is mm
    float MaprealHeight;   //Map height unit is mm
    
    float presicion;      //a pix in Mapimage represent real length in mm
    

    cv::Mat data;  //bin map
    cv::Mat expandedData;
    float expandedfactor;
    
    
    void MapGenerate(const cv::string& mapimgPath,float pres,float Mapwidht,float Mapheight,bool BlackWall,float expandedfac,int robotridux);
    void MapLoader(){};
    cv::vector<cv::Point2i> neighbors(cv::Point2i id) const;
    bool inbound(cv::Point2i p) const;
    bool againstwall(cv::Point2i p,bool expanded) const;
};
#endif /* MapObj_hpp */
