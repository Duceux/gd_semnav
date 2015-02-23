#ifndef BOWTOOLS_H
#define BOWTOOLS_H

#include <fstream>
#include <string>
//#include <opencv2/highgui/highgui.hpp>
#include <cv.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/core/mat.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include "nonfree/nonfree.hpp"
#include <opencv2/nonfree/features2d.hpp>
//#include <opencv2/opencv.hpp>
#include <vector>

bool readLibSvmData(char * filename, cv::Mat & data);
void writeLibSvmData( char * filename, cv::Mat & data ) ;


class opencvBOW
{
protected:
  cv::Ptr<cv::FeatureDetector > detector ;
 cv::Ptr<cv::DescriptorMatcher > matcher ;
 cv::Ptr<cv::DescriptorExtractor > extractor ;
 cv::Ptr<cv::BOWImgDescriptorExtractor> bow ;
 
 std::map<std::string, int > nameLookup;
 int nrObjectClasses ;
 cv::Mat vocabulary ;
 
 std::vector < std::map< int, float > > ilt ; // map key is object index, associated value is voting strength/nr of occurrences

public:
 opencvBOW() ;
 ~opencvBOW() ; 
 void setVocabulary(cv::Mat & voc ) {this->vocabulary = voc; this->bow->setVocabulary (voc);}
 bool loadVocabulary (char * s) { if (readLibSvmData(s, this->vocabulary)) {this->setVocabulary(vocabulary); return true;} else return false;}
 void compute(cv::Mat  & img, cv::Mat & result, const cv::Mat & mask=cv::Mat());
 void compute(cv::Mat  & img, std::vector <cv::KeyPoint> & keypoints,  cv::Mat & result);
 
 void resetObjectList();
 void addObjectSlot(std::string & name, std::string subclass);
 void addExample (std::string & name, cv::Mat img);
 void classifyFeatureVector(const cv::Mat & fv, std::vector<float> & scores);
 void classifyImg(const cv::Mat  & img, cv::Mat & scores);
 void compute_feature(cv::Mat & img, cv::Mat & result, const cv::Mat & mask=cv::Mat());
} ;


class opencvBOWForSmallImgs : public opencvBOW
{
  public:
  opencvBOWForSmallImgs() ;
  ~opencvBOWForSmallImgs() ;
  void outputilt(char * s);
  void initIlt (){ ilt.resize(this->vocabulary.rows); }
  void loadIlt (char * s );
  void normalizeIlt (std::vector<int> count);
  void addExample_hsv (std::string & name, cv::Mat img);
  void compute_hsv (cv::Mat  & img, cv::Mat & result, const cv::Mat & mask=cv::Mat());
  void compute_hsv_feature (cv::Mat  & img, cv::Mat & result, const cv::Mat & mask=cv::Mat());
  void compute_hsvlc (cv::Mat  & img, cv::Mat & result, const cv::Mat & mask=cv::Mat());
  void compute_tbgr_feature (cv::Mat  & img, cv::Mat & result, const cv::Mat & mask=cv::Mat());
  void compute_hovers_feature (cv::Mat  & img, cv::Mat & result, const cv::Mat & mask=cv::Mat());
  void classifyFeatureVector_d(cv::Mat & fv, std::vector<float> & scores);
  void classifyFeatureVector_d2(cv::Mat & fv, std::vector<float> & scores);
  void classifyFeatureVector_d3(cv::Mat & fv, std::vector<float> & distances);
  int getvocabrows ();
  void createVoca(cv::Mat features, int vocaSize, std::string vocaFileName);
  int hbins;
} ;



#endif
