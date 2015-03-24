#ifndef MAP_H
#define MAP_H

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <sn_geometry/sn_geometry.h>
#include <nav_msgs/OccupancyGrid.h>

namespace sn{

class Map
{
public:
    Map():
        resolution(0.05),
        width(1000),
        height(1000)
    {
        grid = cv::Mat::zeros(width, height, CV_8UC1) + 128;
        blocked_ = cv::Mat::zeros(width, height, CV_8UC1);
        origin.x = -height/2*resolution;
        origin.y = -width/2*resolution;
        origin.theta = 0.0;
    }

    void load(const std::string& filename);

    cv::Point toMap(double x, double y) const;
    cv::Point toMap(const point_t& p) const;
    point_t toMapFrame(const point_t& p)const;

    point_t toWorld(int px, int py) const;
    point_t toWorld(const cv::Point& p) const;

    bool isValid(const cv::Point& p) const;
    bool isValid(const point_t& p) const{
      return isValid(toMap(p));
    }

    uchar at(const cv::Point& p) const{
        if(!isValid(p))
            return 128;
        return grid.at<uchar>(p);
    }

    uchar& at(const cv::Point& p){
        return grid.at<uchar>(p);
    }

    double at(const point_t& point) const;

    void computeDistSup(int threshold=128){
        cv::distanceTransform(grid>=threshold, dist_grid, CV_DIST_L2, 3);
    }

    void computeDistEq(int threshold=128){
        cv::distanceTransform(grid==threshold, dist_grid, CV_DIST_L2, 3);
    }

    void computeDistDiff(int threshold=128){
        cv::distanceTransform(grid!=threshold, dist_grid, CV_DIST_L2, 3);
    }


    void computeDistInf(int threshold=128){
        cv::distanceTransform(grid<=threshold, dist_grid, CV_DIST_L2, 3);
    }

    float dist(const point_t& point) const;

    void rezise(int left, int right, int top, int bottom);

    void computeRoi();

    void block(const cv::Point& p){
      blocked_.at<uchar>(p) = (uchar)255;
    }

    void unblock(const cv::Point& p){
      blocked_.at<uchar>(p) = (uchar)0;
    }

    void unblockall(){
      blocked_ = cv::Mat::zeros(width, height, CV_8UC1);
    }

    void blockall(){
      blocked_ = cv::Mat::zeros(width, height, CV_8UC1)+255;
    }

    bool blocked(const cv::Point& p){
      return blocked_.at<uchar>(p) > 0;
    }

    cv::Rect getRoi()const{
      return roi;
    }

    int getHeight()const{
      return height;
    }

    int getWidth()const{
      return width;
    }

    double getResolution()const{
      return resolution;
    }

    cv::Mat getGrid()const{
      return grid;
    }

    friend void rosToMap(nav_msgs::OccupancyGrid& og, Map &map);
    friend void rosToMap(nav_msgs::OccupancyGrid::ConstPtr og, Map &map);
    friend void mapToRos(const Map &map, nav_msgs::OccupancyGrid& og);

private:
    cv::Point minPoint();

    cv::Point maxPoint();


    float dist(const cv::Point& p) const{
        return dist_grid.at<float>(p);
    }



private:
    double resolution;
    pose_t origin;
    int width;
    int height;
    cv::Mat grid;
    cv::Mat blocked_;
    cv::Mat dist_grid;
    cv::Rect roi;
};

}

#endif // MAP_H
