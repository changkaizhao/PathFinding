//
//  MapObj.cpp
//  PathFinding
//
//  Created by 赵常凯 on 16/12/19.
//  Copyright © 2016年 Roby. All rights reserved.
//

#include "MapObj.hpp"

cv::Mat expandmap(cv::Mat map,float expandedfac,int robotradiux){
    //edge detect
    cv::Mat res;
    map.copyTo(res);
    cv::Mat edge;
    cv::Canny(map, edge, 125, 350);
    //cv::imshow("edge",edge);
    
    int brushradius = (int)ceilf((float)robotradiux*(1+expandedfac));
    for (int r = 0 ; r < res.rows; r++) {
        for (int c = 0; c < res.cols; c++) {
            if (edge.at<uchar>(r,c) > 128) {
                //on edge
                //brush
                int w_cs = (c - brushradius) < 0 ? 0 : (c - brushradius);
                int w_ce = (c + brushradius) > (res.cols - 1) ? (res.cols - 1) : (c + brushradius);
                int w_rs = (r - brushradius) < 0 ? 0 : (r - brushradius);
                int w_re = (r + brushradius) > (res.rows - 1) ? (res.rows - 1) : (r + brushradius);
                for (int wr = w_rs; wr <= w_re; wr++) {
                    for (int wc = w_cs; wc <= w_ce; wc++) {
                        res.at<uchar>(wr,wc) = 255;
                    }
                }

            }
        }
    }
    //cv::imshow("expanded",res);
    //cv::imwrite("/Development/PathFinding/rawmap/expandedmap.png", res);
    return res;
}

void MapObj::MapGenerate(const cv::string &mapimgPath,float pres,float Mapwidht,float Mapheight,bool BlackWall,float expandedfac,int robotridux){
    cv::Mat imagemap = cv::imread(mapimgPath,0);
    this->MaprealWidth = Mapwidht;
    this->MaprealHeight = Mapheight;
    this->presicion = pres;
    unsigned int width = (unsigned int)ceilf(Mapwidht/pres);
    unsigned int height = (unsigned int)ceilf(Mapheight/pres);
    
    cv::Mat binmap;
    cv::resize(imagemap, binmap, cv::Size(width,height));
    cv::threshold(binmap, binmap, 128, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
//    cv::imshow("binmap", binmap);
    
    if (BlackWall) {
        for(int r = 0;r<binmap.rows;r++){
            for(int c = 0 ; c < binmap.cols;c++){
                binmap.at<uchar>(r,c) = binmap.at<uchar>(r,c) > 128 ? 0 : 255;
            }
        }
    }

    this->data = binmap;
    
    //expande map
    this->expandedData = expandmap(binmap, expandedfac, robotridux);
}
bool MapObj::inbound(cv::Point2i p) const{
    return p.x>=0 && p.x<data.cols && p.y>=0 && p.y<data.rows;
}
bool MapObj::againstwall(cv::Point2i p,bool expanded) const{
    return expanded ? expandedData.at<uchar>(p.y,p.x) : data.at<uchar>(p.y,p.x);
}
cv::vector<cv::Point2i> MapObj::neighbors(cv::Point2i id) const {
        //int x, y, dx, dy;
        //tie (x, y) = id;
    cv::vector<cv::Point2i> results;
    
    /**
            +---+---+---+
            | 1 | 2 | 3 |
            +---+---+---+
            | 4 | c | 5 |
            +---+---+---+
            | 6 | 7 | 8 |
            +---+---+---+
     
            return the 8 elements around center element
     **/
    for (int r = id.y - 1; r <= id.y + 1; r++) {
        for (int c = id.x - 1; c <= id.x + 1; c++) {
            if (r == id.y && c == id.x) {
                continue;
            }else{
                cv::Point2i p = cv::Point2i(c,r);
                if (inbound(p) && !againstwall(p, true)) {
                    results.push_back(p);
                }
            }

        }
    }


        return results;
    }
