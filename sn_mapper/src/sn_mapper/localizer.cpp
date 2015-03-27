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
}

int Localizer::process(vector_pts_t &points){

  if(!mInit){
    std::cerr << "Localizer not initiliazed\n";
    return NO_UPDATE;
  }

  mScoreFPtr->score(mMap, mBestP, points, DMAX);

  auto tentative = mBestP;
  tentative.set(mPFilter.predict());
  mScoreFPtr->score(mMap, tentative, points, DMAX);
  if(tentative.score < mBestP.score)
    mBestP = tentative;

#pragma omp parallel for
    for(unsigned int i=0; i<mParticles.size(); i++){
      mCandidates[i].set(oplus(mBestP, mParticles[i]));
      mScoreFPtr->score(mMap, mCandidates[i], points, mBestP.fast_score);
      if(mScoreFPtr->better_than(mCandidates[i], mBestP))
        mBestP = mCandidates[i];
    }

  mPFilter.update(mBestP);

  return UPDATE;
}

void MeanDistScore::score(Map const& map, Particle &model, const vector_pts_t &points, double best_score_yet)
{
  model.fast_score = model.score = DMAX;

  double score = 0.0;
  int count = 0;
  for(uint it = 0; it<points.size(); it+=4, ++count){
    point_t projection = oplus(model, points[it]);
    if(map.isValid(projection)){
      score += map.dist(projection);
    }
    else{
      score += map.getHeight()*map.getResolution();
    }
    if(score > 1.5*best_score_yet)
      break;
  }
  model.fast_score = score;
  if(model.fast_score > 1.5*best_score_yet)
    return;

  double mean = score/count;

  count = 0;
  score = 0.0;
#pragma omp parallel for
  for(uint it = 0; it<points.size(); ++it){
    point_t projection = oplus(model, points[it]);
    if(map.isValid(projection) && map.dist(projection) < mean*factor_filter_){
      score += map.dist(projection);
      count++;
    }
  }
  model.score = score/count;
}

bool MeanDistScore::better_than(const Particle &cand, const Particle &ref)
{
  return cand.score < ref.score;
}

void NbOfFitted::score(const Map &map, Particle &model, const vector_pts_t &points, double best_score_yet)
{
  model.fast_score = model.score = 0;
#pragma omp parallel for
  for(uint it = 0; it<points.size(); ++it){
    point_t projection = oplus(model, points[it]);
    if(map.isValid(projection) && map.dist(projection) < 2.0){
      model.fast_score += 2.0-map.dist(projection);
      model.score = model.fast_score / it;
    }
  }
}

bool NbOfFitted::better_than(const Particle &cand, const Particle &ref)
{
  return cand.fast_score > ref.fast_score;
}

}
