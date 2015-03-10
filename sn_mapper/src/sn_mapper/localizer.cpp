#include <sn_mapper/localizer.h>

#include <cmath>
#include <omp.h>

#define LOST_SCORE 3
#define LOST_DIST 2

namespace sn{



void Localizer::init(const Map &map){
  mMap = map;
  mMap.computeRoi();
  mInit = true;
  mStdth = 2.0;
  mTh = 5;
  mHasHint = true;
  mIsLost = false;
}

int Localizer::process(vector_pts_t &points){

  if(!mInit){
    std::cerr << "Localizer not initiliazed\n";
    return NO_UPDATE;
  }

  scoreParticle(mBestP, points);

  double prev_score = DMIN;
  int nb = 1000;
  ros::Time tic = ros::Time::now();
  while(ros::Time::now()-tic < ros::Duration(0.04))
  {
    prev_score = mBestP.score;

    mParticles.clear();
    mParticles.reserve(nb*10);
#pragma omp parallel for
    for(unsigned int i=0; i<nb; i++){
      generateParticles(1, 0.2, mBestP);
    }

#pragma omp parallel for
    for(unsigned int i=0; i<mParticles.size(); i++){
      scoreParticle(mParticles[i], points);
      if(mParticles[i].score <=  mBestP.score)
        mBestP = mParticles[i];
    }
  }
  return UPDATE;
}

void Localizer::generateUniformParticles(int number){

  mParticles.reserve(mParticles.size() + number);
  int count=0;
  while(count<number){
    int dy = mRandom.uniform(mMap.roi.x, mMap.roi.x + mMap.roi.width);
    int dx = mRandom.uniform(mMap.roi.y, mMap.roi.y + mMap.roi.height);

    cv::Point pp(dx, dy);
    if(!mMap.isValid(pp))
      continue;
    /*
    if(mMap.at(pp) <= 128)
      continue;
*/
    double dtheta = mRandom.uniform(-M_PI, M_PI);
    point_t p = mMap.toWorld(pp);
    pose_t pose = create_pose(p.x, p.y, dtheta);
    Particle newp;
    newp.set(pose);
    mParticles.push_back(newp);
    ++count;
  }
}

void Localizer::generateParticles(int number, float strength, const Particle& p){

  mParticles.reserve(mParticles.size() + number);
  int count=0;
  while(count<number){
    float dx = mRandom.normal(0, strength);
    float dy = mRandom.normal(0, strength);
    float dz = mRandom.normal(0, 0.75*strength);
    Particle newp;
    newp.set(oplus(p, create_pose(dx, dy, dz)));
    point_t newpt = create(newp.x, newp.y);
    if(!mMap.isValid(newpt)){
      continue;
    }
    if(mMap.at(newpt) <= 127){
      continue;
    }
    mParticles.push_back(newp);
    ++count;
  }

}

void Localizer::scoreParticle(Particle& pl, const vector_pts_t &points){
  pl.fast_score = pl.score = DMAX;

  point_t p = create(pl.x, pl.y);
  if(!mMap.isValid(p)){
    pl.fast_score = pl.score = DMAX;
    return;
  }
  if(mMap.at(p) <= 127){
    pl.fast_score = pl.score = DMAX;
    return;
  }
  double score = 0.0;
  int count = 0;
  for(uint it = 0; it<points.size(); it+=4, ++count){
    point_t projection = project(pl, points[it]);
    if(mMap.isValid(projection)){
      score += mMap.dist(projection);
    }
    else{
      score += mMap.height*mMap.resolution;
    }
    if(score > 1.5*mBestP.fast_score)
      break;
  }
  pl.fast_score = score;
  if(pl.fast_score > 1.5*mBestP.fast_score)
    return;

  double mean = score/count;

  count = 0;
  score = 0.0;
#pragma omp parallel for
  for(uint it = 0; it<points.size(); ++it){
    point_t projection = project(pl, points[it]);
    if(mMap.isValid(projection) && mMap.dist(projection) < mean*2.0){
      score += mMap.dist(projection);
      count++;
    }
  }
  pl.score = score/count;

}

}
