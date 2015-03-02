#include <sn_mapper/map.h>

namespace sn{

void Map::load(const std::string& filename){
    resolution = 0.05;
    //    origin = point_t(-100.0, -100.0);
    roi = cv::Rect(1700, 1700, 800, 800);
    grid = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE)(roi);
    cv::distanceTransform(grid, dist_grid, CV_DIST_L2, 3);
}

cv::Point Map::toMap(double px, double py) const{
    double x = (px-origin.x)/resolution;
    double y = grid.rows - (py-origin.y)/resolution;
    return cv::Point(round(x), round(y));
}
cv::Point Map::toMap(const point_t &p) const{
    return toMap(p.x, p.y);
}

point_t Map::toMapFrame(const point_t &p) const
{
  double x = (p.x-origin.x)/resolution;
  double y = grid.rows - (p.y-origin.y)/resolution;
  return create(x, y);
}

point_t Map::toWorld(int px, int py) const{
    double x = px*resolution+origin.x;
    double y = (grid.rows - py)*resolution + origin.y;
    return create(x, y);
}
point_t Map::toWorld(const cv::Point& p) const{
    return toWorld(p.x, p.y);
}


bool Map::isValid(const cv::Point &pt) const{
    if( pt.x < 0 ||
            pt.y < 0 ||
            pt.x >= grid.cols ||
            pt.y >= grid.rows)
        return false;
    return true;
}

double Map::at(const point_t &point) const
{
  point_t p = toMapFrame(point);

  //bilinear interpolation
  cv::Point p11(std::floor(p.x), std::floor(p.y));
  cv::Point p12(std::floor(p.x), std::ceil(p.y));
  cv::Point p21(std::ceil(p.x), std::floor(p.y));
  cv::Point p22(std::ceil(p.x), std::ceil(p.y));

  double dist = 0.0;
  dist += at(p11)*(p22.x - p.x)*(p22.y - p.y);
  dist += at(p21)*(p.x - p11.x)*(p22.y - p.y);
  dist += at(p12)*(p22.x - p.x)*(p.y - p11.y);
  dist += at(p22)*(p.x - p11.x)*(p.y - p11.y);
  // dist/=1/(x2-x1)(y2-y1) == 1 not necessary
  return dist;
}

float Map::dist(const point_t &point) const
{
  point_t p = toMapFrame(point);

  //bilinear interpolation
  cv::Point p11(std::floor(p.x), std::floor(p.y));
  cv::Point p12(std::floor(p.x), std::ceil(p.y));
  cv::Point p21(std::ceil(p.x), std::floor(p.y));
  cv::Point p22(std::ceil(p.x), std::ceil(p.y));

  double dist = 0.0;
  dist += dist_grid.at<float>(p11)*(p22.x - p.x)*(p22.y - p.y);
  dist += dist_grid.at<float>(p21)*(p.x - p11.x)*(p22.y - p.y);
  dist += dist_grid.at<float>(p12)*(p22.x - p.x)*(p.y - p11.y);
  dist += dist_grid.at<float>(p22)*(p.x - p11.x)*(p.y - p11.y);
  // dist/=1/(x2-x1)(y2-y1) == 1 not necessary
  return dist;
}

void Map::rezise(int left, int right, int top, int bottom){
    cv::Mat tmp = grid.clone();

    height += top + bottom;
    width += left + right;

    grid = cv::Mat::zeros(height, width, grid.type()) + 128;

    origin.x -= left*resolution;
    origin.y -= bottom*resolution;

    for(int j=top, l=0; j<grid.rows && l<tmp.rows; ++j, ++l)
        for(int i=left, k=0; i<grid.cols && k<tmp.cols; ++i, ++k)
            grid.at<uchar>(j, i) = tmp.at<uchar>(l, k);

}

void Map::computeRoi()
{
  roi.x = 0;
  roi.y = 0;
  roi.width = width;
  roi.height = height;

  cv::Point minp = minPoint();
  cv::Point maxp = maxPoint();
//  std::cout << minp << maxp  <<std::endl;
  roi.x = minp.x;
  roi.y = minp.y;
  roi.width = maxp.x - minp.x;
  roi.height = maxp.y - minp.y;
}

cv::Point Map::minPoint()
{
  int mini = grid.cols;
  int minj = grid.rows;
  for(int j=0; j<grid.rows; ++j)
      for(int i=0; i<grid.cols; ++i)
      {
        if(grid.at<uchar>(j, i) != 128){
          mini = std::min(mini, i);
          minj = std::min(minj, j);
        }
      }
  return cv::Point(minj, mini);
}

cv::Point Map::maxPoint()
{
  int maxi = 0;
  int maxj = 0;
  for(int j=0; j<grid.rows; ++j)
      for(int i=0; i<grid.cols; ++i)
      {
        if(grid.at<uchar>(j, i) != 128){
          maxi = std::max(maxi, i);
          maxj = std::max(maxj, j);
        }
      }
  return cv::Point(maxj, maxi);
}

}
