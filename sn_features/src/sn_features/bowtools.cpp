#include "sn_features/bowtools.h"
#include <iostream>
#include <math.h>
// need to find a way to reverse normalization during training!

using namespace std ;
using namespace cv;

bool readLibSvmData(char * filename, cv::Mat & data)
{
  string s;
  ifstream infile; infile.open (filename) ;
  if (infile.is_open())
  {
    getline(infile,s);
    int nrFields=0;
    for (int i=0; i<s.size(); i++)
      if (s[i] == ':')
        nrFields++ ;
    int nrLines=0;
    while (! infile.eof() )
    {
      getline(infile,s) ;
      nrLines++ ;
    }

    infile.close() ;
    cout << filename << " has: lines: " << nrLines << ", fields: " << nrFields << endl;
    data.create (nrLines,nrFields, CV_32FC1);

    infile.open (filename) ;
    int index=0 ;
    while ( infile.eof()==false)
    {
      getline(infile,s);
      const char * cstr = s.c_str() ;
      int countFields=0;
      float fl; int dummy ;
      for (int k=0; k<s.size(); k++)
        if (s[k] == ':')
        {
          s[k] = ' ';
          sscanf(cstr+k,"%f",  &fl) ;
          data.at<float>(index,countFields) = fl ;
          //cout << data.at<float>(index,countFields) << " ";
          countFields++;
        } //cout << endl;
      if ((index++%100)  == 0) printf("%d\n",index) ;
    }
    infile.close() ;
  }
  else
  {
    std::cout << "Could not open dictionary file: " << filename << std::endl;
    return false;
  }

  return true;
}


void writeLibSvmData( char * filename, cv::Mat & data ) 
{
  ofstream f;
  f.open(filename) ;
  
  for (int i=0; i<data.rows; i++)
  {
    f << "+1 ";
    for (int j=0; j<data.cols; j++)
    {
      f << j << ":" << data.at<float>(i,j) << " ";
    }
    f << endl;
  }
  f.close() ;
}


void opencvBOW::resetObjectList()
{
  this->ilt.clear() ;
  nrObjectClasses = 0 ;
}


void opencvBOW::addObjectSlot(std::string & name, std::string subclass)
{
  this->nameLookup[name] = ++(this->nrObjectClasses) ;
  //cout << this->nameLookup[name] << endl;
}


void opencvBOW::addExample (std::string & name, cv::Mat img)
{
  cv::Mat descr;

  this->compute(img,descr) ;//compute the feature vecteur
  int objIndex=this->nameLookup[name] ;
  cout << "class name found: "<<objIndex<<endl;
  
  for (int wordIndex=0; wordIndex < this->vocabulary.rows ; wordIndex++)
  {
    //cout << wordIndex << endl;
    float wordWeight = descr.at<float> (0,wordIndex);
    if (wordWeight != 0)
    {
      std::map<int,float>::iterator it;
      it = this->ilt[wordIndex].find(objIndex) ;
      if (it == ilt[wordIndex].end() )//if there is no such a object
        this->ilt[wordIndex] [objIndex] = wordWeight ;
      else
        this->ilt[wordIndex] [objIndex] += wordWeight ;
    }
    //cout <<wordIndex<<" "<< wordWeight << endl;
  }
}


void opencvBOW::classifyFeatureVector(const cv::Mat & fv, vector <float> & scores)//enter the fv of a image, compute the score for each object
{
  scores.assign(this->nrObjectClasses,0.0) ;
  for (int wordIndex=0; wordIndex < this->vocabulary.rows ; wordIndex++)//look up all the words
  {
    float wordWeight = fv.at<float> (0,wordIndex);
    for (std::map<int,float>::iterator it=this->ilt[wordIndex].begin(); it != this->ilt[wordIndex].end(); it++)
    {
      scores[it->first] += ((it->second)*(wordWeight)) ;//maybe miss the wordWeight
      //cout << it->first << ":"<< it->second << " ";
    }
    //cout << endl;
  }
}





opencvBOW::~opencvBOW()
{
  //delete this->matcher ;
  //delete this->detector ;
  //delete this->extractor ;
}



opencvBOW::opencvBOW()
{
  this->detector =  new SiftFeatureDetector ();
  this->matcher = new BFMatcher() ;

  this->extractor = new cv::SiftDescriptorExtractor () ;
  this->bow = new BOWImgDescriptorExtractor(extractor,matcher) ;
}

void opencvBOW::compute(cv::Mat  & img, cv::Mat & result, const cv::Mat & mask)
{
  vector<KeyPoint> keypoints ;
/*  int noctaves = 0;
  noctaves = (img.rows > img.cols)?img.cols:img.rows;
  noctaves = (log((double)noctaves)/log(2.0)) - 2;
  cout << noctaves <<endl;
  this->detector = new SiftFeatureDetector (SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
                                             SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD(),
                                             noctaves,3,SIFT::CommonParams::DEFAULT_FIRST_OCTAVE,
                                             SIFT::CommonParams::FIRST_ANGLE) ;
  this->extractor = new cv::SiftDescriptorExtractor (3.0,true,true,noctaves,3,-1, 0 ) ;
  this->bow =  new BOWImgDescriptorExtractor(extractor,matcher) ;
*/
  this->detector->detect(img,keypoints, mask);
  this->bow->compute(img, keypoints, result);

  //this->bow->delete_obj();
  //this->extractor->delete_obj();
  //this->detector->delete_obj();
}


void opencvBOW::compute(cv::Mat  & img, std::vector <cv::KeyPoint> & keypoints, cv::Mat & result)
{
/*  int noctaves = 0;
  noctaves = (img.rows > img.cols)?img.cols:img.rows;
mmmmmmmmmm  noctaves = (log((double)noctaves)/log(2.0)) - 2;
  this->extractor = new cv::SiftDescriptorExtractor (3.0,true,true,noctaves,3,-1, 0 ) ;
  this->bow =  new BOWImgDescriptorExtractor(extractor,matcher) ;
*/
  this->bow->compute(img, keypoints, result);
}


void opencvBOW::compute_feature(cv::Mat & img, cv::Mat & result, const cv::Mat & mask)
{
  vector <cv::KeyPoint> siftKPs ;
  this->detector->detect(img, siftKPs, mask) ;
  this->extractor->compute(img,siftKPs, result) ;
}

opencvBOWForSmallImgs::opencvBOWForSmallImgs():
    hbins(16)
{
//  cv::initModule_nonfree();
  this->detector = new SiftFeatureDetector () ;
  this->matcher = new BFMatcher() ;

  this->extractor = new cv::SiftDescriptorExtractor () ;
  this->bow = new BOWImgDescriptorExtractor(extractor,matcher) ;
}


opencvBOWForSmallImgs::~opencvBOWForSmallImgs()
{
  //delete this->matcher ;
  //delete this->detector ;
  //delete this->extractor ;
}

void opencvBOWForSmallImgs::outputilt( char * s )
{
  ofstream outfile;
  outfile.open(s);
  for (int wordIndex=0; wordIndex < this->vocabulary.rows ; wordIndex++)//look up all the words
  {
    for (std::map<int,float>::iterator it=this->ilt[wordIndex].begin(); it != this->ilt[wordIndex].end(); it++)
    {
      outfile << it-> second << " ";
    }
    outfile << endl;
  }
  outfile.close();
}

void opencvBOWForSmallImgs::loadIlt (char * s )
{
  ifstream infile;
  infile.open(s);
  cout<<"file opened"<<endl;
  int wordIndex = 0;
  int i = 0;
  while (wordIndex != this->vocabulary.rows)
  {
    cout << wordIndex<< ": ";
    for (i = 0; i< this->nrObjectClasses; i++)
    {
      cout << i <<": ";
      infile >> this->ilt[wordIndex][i];
      cout << this->ilt[wordIndex][i] <<" ";
    }
    cout << endl;
    wordIndex ++;
   }
  infile.close();
}

void opencvBOWForSmallImgs::normalizeIlt (std::vector<int> count)
{
  std::vector <float> sum (count.size(),0);
  //std::vector <float> min (count.size(),0);
  std::vector <float> thd (count.size(),0);
  std::vector <float> mean (this->vocabulary.rows ,0);

  for (int wordIndex=0; wordIndex < this->vocabulary.rows ; wordIndex++)//look up all the words
  {
    for (std::map<int,float>::iterator it=this->ilt[wordIndex].begin(); it != this->ilt[wordIndex].end(); it++)
    {
      //it->second = it->second*1000 / count[it->first-1];
      sum[it->first-1] += (it->second)*(it->second);
      //if (min[it->first-1] > it->second) min[it->first-1] = it->second;
      //thd[it->first-1] += it->second;
      // cout << count[it->first-1] <<" ";
      //mean[wordIndex] += it->second;
    }
    //mean[wordIndex] = mean[wordIndex]/this->nrObjectClasses;
  }
//---------------------------------------------------------------------------------------------
  //cout << count[0] << " " << count [1] << endl;
/*  for (std::map<int,float>::iterator it=this->ilt[0].begin(); it != this->ilt[0].end(); it++)
  {
   //thd[it->first-1] = (max[it->first-1]- min[it->first-1])*0.6 + min[it->first-1];
    thd[it->first-1] = thd[it->first-1] /this->vocabulary.rows;
  }*/
//---------------------------------------------------------------------------------------------
  for (int wordIndex=0; wordIndex < this->vocabulary.rows ; wordIndex++)//look up all the words
  {
    for (std::map<int,float>::iterator it=this->ilt[wordIndex].begin(); it != this->ilt[wordIndex].end(); it++)
    {
      it->second = it->second / sqrt(sum[it->first-1]);
    }
  }
//---------------------------------------------------------------------------------------------
/*  for (int wordIndex=0; wordIndex < this->vocabulary.rows ; wordIndex++)
  {
    for (std::map<int,float>::iterator it=this->ilt[wordIndex].begin(); it != this->ilt[wordIndex].end(); it++)
    {
      it->second = it->second / max[it->first-1];
    }
  }*/
}

void opencvBOWForSmallImgs::addExample_hsv (std::string & name, cv::Mat img)
{
  Mat descr,temp;
  Mat input, hsv;
  MatND hist;
  float sum = 0;

//  this->compute(img,descr) ;//compute the feature vecteur
  int objIndex=this->nameLookup[name] ;
  cout << "class name found: "<<objIndex<<endl;

  cout << img.cols<<" "<<img.rows<<endl;
  descr.create (1,this->hbins, CV_32FC1);
  temp.create (1,this->hbins, CV_32FC1);
  int index= 0;
  int wsize = 40;
  if ( (img.rows<=40)||(img.cols<=40) )
  {
    wsize = (img.rows > img.cols)?img.cols:img.rows;
    wsize --;
  }

  for (int i= 0; (i+1)*wsize < img.rows; i++)
  {
    for (int j = 0; (j+1)*wsize < img.cols; j++)
    {
      //cout << i <<", "<<j<<endl;
      Rect rect( j*wsize, i*wsize, wsize, wsize );

      input = img(rect);
      cvtColor(input,hsv,CV_BGR2HSV);
//      cout << "convertion completed"<<endl;
      int hbins = this->hbins;//, sbins = 8;
      int histSize[] = {hbins};
      float hranges[] = { 0, 180 };
      const float* ranges[] = { hranges };
      int channels[] = {0};

      calcHist( &hsv, 1, channels, Mat(),
                hist, 1, histSize, ranges,
                true,
                false );

      sum = 0;
      for (int i = 0; i< hbins; i++)
      {
        sum += hist.at<float>( i,0 );
      }

      for (int i = 0; i< hbins; i++)
      {
        hist.at<float>( i,0 ) = hist.at<float>( i,0 ) / sum;
        //cout << hist.at<float>( i,0 )<<endl;
        if (index == 0)
        {
          descr.at<float> (0,i) = hist.at<float>( i,0 );
        }
        else
        {
          temp.at<float> (0,i) = hist.at<float>( i,0 );
        }
      }
      if (index != 0)
      {
        descr.push_back(temp);
      }
      index ++;
    }
  }

  std::vector<DMatch> matches;
  this->matcher->match( descr, this->vocabulary, matches );
  std::vector<float> results( this->vocabulary.rows, 0 );
  for (int i=0; i < descr.rows ; i++)
  {
    results[matches[i].trainIdx] += exp( -10.0*matches[i].distance );
  }

  for (int wordIndex=0; wordIndex < this->vocabulary.rows ; wordIndex++)
  {
    float wordWeight = results[wordIndex];
    if (wordWeight != 0)
    {
      std::map<int,float>::iterator it;
      it = this->ilt[wordIndex].find(objIndex) ;
      if (it == ilt[wordIndex].end() )//if there is no such a object
        this->ilt[wordIndex] [objIndex] = wordWeight ;
      else
        this->ilt[wordIndex] [objIndex] += wordWeight ;
    }
    //cout <<wordIndex<<" "<< wordWeight << endl;
  }
}

void opencvBOWForSmallImgs::compute_hsv (cv::Mat  & img, cv::Mat & result, const cv::Mat & mask)
{
  cv::Mat descr;
  compute_hsv_feature(img, descr, mask);

  std::vector<DMatch> matches;
  this->matcher->match( descr, this->vocabulary, matches );
  Mat results=Mat::zeros(1,this->vocabulary.rows,CV_32FC1);
  for (int i=0; i < descr.rows ; i++)
  {
//    cout << matches[i].trainIdx << " ";
    results.at<float>(0,matches[i].trainIdx) += exp( -10.0*matches[i].distance );
    //cout << "distance: " << matches[i].distance << endl;
  }
 //cout << descr << endl;
  result = results;
}

//void opencvBOWForSmallImgs::compute_hsvlc (cv::Mat  & img, cv::Mat & result, const cv::Mat & mask)
//{
//  Mat hist;
//  compute_hsvlc_feature(img, hist, mask);
//  std::vector<DMatch> matches;
//  this->matcher->match( hist, this->vocabulary, matches );
//  Mat results=Mat::zeros(1,this->vocabulary.rows,CV_32FC1);
//  for (int i=0; i < hist.rows ; i++)
//  {
//    results.at<float>(0,matches[i].trainIdx) += exp( -10.0*matches[i].distance );
//    //cout << "distance: " << matches[i].distance << endl;
//  }
//  cout << hist << endl;
//  result = results;
//}
//
void opencvBOWForSmallImgs::compute_tbgr_feature(cv::Mat  & img, cv::Mat & result, const cv::Mat & mask)
{
  int img_size = (img.rows)*(img.cols);
  float mask_valid_cnt = 0;
  int sum[3] = {0, 0, 0};
  int j = 0;
  for (int i = 0; i < img_size*3; i+=3)
  {
    if (mask.at<uchar>(j))
    {
      sum[0] += img.at<uchar>(i);
      sum[1] += img.at<uchar>(i+1);
      sum[2] += img.at<uchar>(i+2);
      mask_valid_cnt++;
    }
//    mask_valid_cnt += static_cast<float>(1 & mask.at<uchar>(j));
    j++;
  }
  float mean[3] = {static_cast<float>(sum[0])/mask_valid_cnt, static_cast<float>(sum[1])/mask_valid_cnt, static_cast<float>(sum[2])/mask_valid_cnt};

  float accum[3] = {0., 0., 0.};
  j = 0;
  for (int i = 0; i < img_size*3; i+=3)
  {
    if (mask.at<uchar>(j))
    {
      accum[0] += (static_cast<float>(img.at<uchar>(i)) - mean[0]) * (static_cast<float>(img.at<uchar>(i)) - mean[0]);
      accum[1] += (static_cast<float>(img.at<uchar>(i+1)) - mean[1]) * (static_cast<float>(img.at<uchar>(i+1)) - mean[1]);
      accum[2] += (static_cast<float>(img.at<uchar>(i+2)) - mean[2]) * (static_cast<float>(img.at<uchar>(i+2)) - mean[2]);
    }
    j++;
  }
  double stdev[3] = {sqrt(accum[0] / (mask_valid_cnt-1.)), sqrt(accum[1] / (mask_valid_cnt-1)), sqrt(accum[2] / (mask_valid_cnt-1))};

  cv::Mat norm_img;
  img.convertTo(norm_img, CV_32FC3);
  for (int i = 0; i < img_size*3; i+=3)
  {
    norm_img.at<float>(i) = (norm_img.at<float>(i)-mean[0]) / stdev[0];
    norm_img.at<float>(i+1) = (norm_img.at<float>(i+1)-mean[1]) / stdev[1];
    norm_img.at<float>(i+2) = (norm_img.at<float>(i+2)-mean[2]) / stdev[2];
  }

  result = cv::Mat(1, this->hbins*3, CV_32FC1);
  int k = 0;
  for (int i = 0; i < 3; ++i)
  {
    cv::Mat tmp_result;
    int histSize[] = {this->hbins};
    float hranges[] = { -2., 2.};
    const float* ranges[] = { hranges };
    int channels[] = {i};
    calcHist( &norm_img, 1, channels, mask,
              tmp_result, 1, histSize, ranges,
              true,
              false );

    for (int j = 0; j < this->hbins; j++)
    {
      result.at<float>(0,k) = tmp_result.at<float>(j) / mask_valid_cnt;
      k++;
    }
  }
//  for (int i=0; i < this->hbins ; i++)
//  {
//    for (int j=0; j < this->hbins ; j++)
//    {
//      std::cout << setw(4) << setfill('0') << result.at<float>(i) << " ";
//    }
//    std::cout << std::endl;
//  }
//  std::cout << std::endl;

//  for (int i = 0; i< this->hbins; i++)
//  {
//    result.at<float>( i,0 ) = (result.at<float>( i,0 )-mean[0]) / stdev[0];
//    result.at<float>( i,1 ) = (result.at<float>( i,1 )-mean[1]) / stdev[1];
//    result.at<float>( i,2 ) = (result.at<float>( i,2 )-mean[2]) / stdev[2];
//  }
//
//  for (int i = 0; i< this->hbins; i++)
//  {
//      std::cout << setw(8) << setfill('0') << result.at<float>(i,0) << " ";
//  }
//  std::cout << std::endl;
//  Mat hsv;
//  cvtColor(img,hsv,CV_BGR2HSV);
//  int histSize[] = {this->hbins};
//  float hranges[] = { 0, 180 };
//  const float* ranges[] = { hranges };
//  int channels[] = {0};
//  calcHist( &hsv, 1, channels, mask,
//            result, 1, histSize, ranges,
//            true,
//            false );
//  float sum = 0;
//  for (int i = 0; i< this->hbins; i++)
//  {
//    sum += result.at<float>( i,0 );
//  }
//  for (int i = 0; i< this->hbins; i++)
//  {
//    result.at<float>( i,0 ) = result.at<float>( i,0 ) / sum;
//  }

//  for (int i=0; i < result.rows ; i++)
//  {
//      std::cout << setw(8) << setfill('0') << result.at<float>(i,0) << " ";
//  }
//    std::cout << std::endl;
}


void opencvBOWForSmallImgs::compute_hovers_feature (cv::Mat  & img, cv::Mat & result, const cv::Mat & mask)
{
  Mat hsv;
  cvtColor(img,hsv,CV_BGR2HSV);
  int img_size = (hsv.rows)*(hsv.cols);

  result = cv::Mat(1, this->hbins, CV_32FC1, 0.);
  float sum = 0;
  for (int i=0; i < img_size*3; i+=3)
  {
    int bin = static_cast<float>(hsv.at<uchar>(i)) / 180. * static_cast<float>(this->hbins);
    result.at<float>(bin) += static_cast<float>(hsv.at<uchar>(i+1));
    sum += static_cast<float>(hsv.at<uchar>(i+1));
  }
  for (int i = 0; i< this->hbins; i++)
  {
    result.at<float>(i) = result.at<float>(i) / sum;
  }
}

void opencvBOWForSmallImgs::compute_hsv_feature (cv::Mat  & img, cv::Mat & result, const cv::Mat & mask)
{
  Mat descr,temp;
  Mat input, hsv, imask;
  MatND hist;
  float sum = 0;

//  cout << img.cols<<" "<<img.rows<<endl;
  descr.create (1,this->hbins, CV_32FC1);
  temp.create (1,this->hbins, CV_32FC1);
  int index= 0;
  int wsize = 40;
  if ( (img.rows<=40)||(img.cols<=40) )
  {
    wsize = (img.rows > img.cols)?img.cols:img.rows;
    wsize --;
  }

  for (int i= 0; (i+1)*wsize < img.rows; i++)
  {
    for (int j = 0; (j+1)*wsize < img.cols; j++)
    {
      //cout << i <<", "<<j<<endl;
      Rect rect( j*wsize, i*wsize, wsize, wsize );

      input = img(rect);
      if (mask.data)
        imask = mask(rect);
      else
        imask = cv::Mat();
      cvtColor(input,hsv,CV_BGR2HSV);
      int histSize[] = {this->hbins};
      float hranges[] = { 0, 180 };
      const float* ranges[] = { hranges };
      int channels[] = {0};

      calcHist( &hsv, 1, channels, imask,
                hist, 1, histSize, ranges,
                true,
                false );
      sum = 0;
      for (int i = 0; i< this->hbins; i++)
      {
        sum += hist.at<float>( i,0 );
      }

      for (int i = 0; i< this->hbins; i++)
      {
        hist.at<float>( i,0 ) = hist.at<float>( i,0 ) / sum;
        if (index == 0)
        {
          descr.at<float> (0,i) = hist.at<float>( i,0 );
         }
        else
        {
          temp.at<float> (0,i) = hist.at<float>( i,0 );
        }
      }
      if (index != 0)
      {
        descr.push_back(temp);
      }
      index ++;
    }
  }
  result = descr;

//
//  for (int i=0; i < descr.rows ; i++)
//  {
//    for (int j=0; j < descr.cols ; j++)
//    {
//      std::cout << fixed << setw(8) << setfill('0') << descr.at<float>(i,j) << " ";
//    }
//    std::cout << std::endl;
//  }
}

void opencvBOWForSmallImgs::classifyFeatureVector_d(cv::Mat & fv, vector <float> & scores)//enter the fv of$
{
  scores.assign(this->nrObjectClasses,0.0) ;
  vector <float> distances;
  distances.assign(this->nrObjectClasses,0.0) ;
  float sum = 0;
  int min = 0;

  for (int wordIndex=0; wordIndex < this->vocabulary.rows ; wordIndex++)//look up all the words
  {
    sum += fv.at<float> (0,wordIndex);
  }
  for (int wordIndex=0; wordIndex < this->vocabulary.rows ; wordIndex++)//look up all the words
  {
    fv.at<float> (0,wordIndex) /= sum;
  }

  for (int wordIndex=0; wordIndex < this->vocabulary.rows ; wordIndex++)//look up all the words
  {
    float wordWeight = fv.at<float> (0,wordIndex);
    for (std::map<int,float>::iterator it=this->ilt[wordIndex].begin(); it != this->ilt[wordIndex].end(); it++)
    {
      distances[it->first] += ((it->second)-(wordWeight))*((it->second)-(wordWeight));//maybe miss the wordWeight
      //cout << it->first << ":"<< it->second << " ";
    }
    //cout << endl;
  }

  for (int i=0;i < this->nrObjectClasses; i++)
  {
    if (distances[i] < distances[min]) min = i;
  }
  scores[min] = 1;
}

void opencvBOWForSmallImgs::classifyFeatureVector_d2(cv::Mat & fv, vector <float> & scores)//enter the fv of$
{
  scores.assign(this->nrObjectClasses,0.0) ;
  vector <float> distances;
  distances.assign(this->nrObjectClasses,0.0) ;
  float sum = 0;
  int min = 0;

  for (int wordIndex=0; wordIndex < this->vocabulary.rows ; wordIndex++)//look up all the words
  {
    sum += (fv.at<float> (0,wordIndex))*(fv.at<float> (0,wordIndex));
  }
  for (int wordIndex=0; wordIndex < this->vocabulary.rows ; wordIndex++)//look up all the words
  {
    fv.at<float> (0,wordIndex) /= sqrt(sum);
  }

  for (int wordIndex=0; wordIndex < this->vocabulary.rows ; wordIndex++)//look up all the words
  {
    float wordWeight = fv.at<float> (0,wordIndex);
    for (std::map<int,float>::iterator it=this->ilt[wordIndex].begin(); it != this->ilt[wordIndex].end(); it++)
    {
      distances[it->first] += ((it->second)-(wordWeight))*((it->second)-(wordWeight));//maybe miss the wordWeight
      //cout << it->first << ":"<< it->second << " ";
    }
    //cout << endl;
  }

  for (int i=0;i < this->nrObjectClasses; i++)
  {
    if (distances[i] < distances[min]) min = i;
  }
  scores[min] = 1;
}

void opencvBOWForSmallImgs::classifyFeatureVector_d3(cv::Mat & fv, vector <float> & distances)
{
  distances.assign(this->nrObjectClasses,0.0) ;
//  vector <float> distances;
//  distances.assign(this->nrObjectClasses,0.0) ;
  float sum = 0;
  int min = 0;

  for (int wordIndex=0; wordIndex < this->vocabulary.rows ; wordIndex++)
  {
    sum += (fv.at<float> (0,wordIndex))*(fv.at<float> (0,wordIndex));
  }
  for (int wordIndex=0; wordIndex < this->vocabulary.rows ; wordIndex++)
  {
    fv.at<float> (0,wordIndex) /= sqrt(sum);
  }

  for (int wordIndex=0; wordIndex < this->vocabulary.rows ; wordIndex++)
  {
    float wordWeight = fv.at<float> (0,wordIndex);
    for (std::map<int,float>::iterator it=this->ilt[wordIndex].begin(); it != this->ilt[wordIndex].end(); it++)
    {
      distances[it->first] += ((it->second)-(wordWeight))*((it->second)-(wordWeight));
      //cout << it->first << ":"<< it->second << " ";
    }
  }

/*  for (int i=0;i < this->nrObjectClasses; i++)
  {
    if (distances[i] < distances[min]) min = i;
  }
  scores[min] = 1;*/
}

int opencvBOWForSmallImgs::getvocabrows ()
{
  return this->vocabulary.rows;
}

void opencvBOWForSmallImgs::createVoca(cv::Mat features, int vocaSize, std::string vocaFileName)
{
  cv::BOWKMeansTrainer bowtrainer( vocaSize );
  bowtrainer.add( features );
//  std::cout << "Creating vocabulary" << std::endl;
  cv::Mat sift_vocabulary = bowtrainer.cluster();
  this->setVocabulary(sift_vocabulary);
  writeLibSvmData(const_cast<char*>(vocaFileName.c_str()), sift_vocabulary);
//  std::cout << "Done creating vocabulary" << std::endl;
}
