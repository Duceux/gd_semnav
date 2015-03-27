#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <sn_geometry/sn_geometry.h>
#include <sn_mapper/map.h>
#include <sn_mapper/conversions.h>
#include <sn_mapper/randomgenerator.h>

namespace sn{

constexpr double DMAX = std::numeric_limits<double>::max();
constexpr double DMIN = std::numeric_limits<double>::min();

struct Particle: public pose_t{
public:
    Particle():
        pose_t(),
        score(DMAX),
        fast_score(DMAX)
    {}
    double score;
    double fast_score;
    void set(pose_t o){
        x = o.x;
        y = o.y;
        theta = o.theta;
    }
};

inline std::ostream& operator << (std::ostream& out, const Particle& p){
    out << "(" << p.x << ", " << p.y << ", "
        << p.theta << ", " << p.fast_score
        << ", " << p.score <<")";
    return out;
}

enum Status{
    NO_UPDATE,
    UPDATE,
    LOST,
    NOT_MOVING
};


class PositionFilter{
public:

  pose_t predict(){
    return oplus(current_, ominus(last_, current_));
  }

  void update(pose_t const& measurement){
    last_ = current_;
    current_ = measurement;
  }

  void set_initial(pose_t const& p0){
    current_ = p0;
    last_ = p0;
  }

private:
  pose_t current_;
  pose_t last_;
};

class BaseScore{
public:
  virtual ~BaseScore(){}
  virtual void score(Map const& map, Particle& model, vector_pts_t const& sensor, double best_score_yet) = 0;
  virtual bool better_than(Particle const& cand, Particle const& ref) = 0;
};

class MeanDistScore: public BaseScore{
public:
  MeanDistScore(double factor = 1.0):factor_filter_(factor){}
  virtual void score(Map const& map, Particle& model, vector_pts_t const& sensor, double best_score_yet);
  virtual bool better_than(Particle const& cand, Particle const& ref);
private:
  double factor_filter_;
};

class NbOfFitted: public BaseScore{
public:
  virtual void score(Map const& map, Particle& model, vector_pts_t const& sensor, double best_score_yet);
  virtual bool better_than(Particle const& cand, Particle const& ref);
};


class Localizer{
public:
    Localizer():
      mInit(false),
      mScoreFPtr(std::make_shared<MeanDistScore>(2.0))
    {
      mParticles.clear();
      for(double dx = -0.01; dx<=0.01; dx+=0.002)
        for(double dy = -0.01; dy<=0.01; dy+=0.002)
          for(double dz = -0.01; dz<=0.01; dz+=0.002){
            Particle newp;
            newp.set(create_pose(dx, dy, dz));
            mParticles.push_back(newp);
          }
      for(double dx = -0.25; dx<=0.25; dx+=0.05)
        for(double dy = -0.25; dy<=0.25; dy+=0.05)
          for(double dz = -0.25; dz<=0.25; dz+=0.05){
            Particle newp;
            newp.set(create_pose(dx, dy, dz));
            mParticles.push_back(newp);
          }
      mCandidates.resize(mParticles.size());

    }

    void init(const Map &map);

    int process(vector_pts_t& points);

    pose_t getPosition(){return mBestP;}
    void setPosition(const pose_t& pos){
        mBestP.x = pos.x;
        mBestP.y = pos.y;
        mBestP.theta = pos.theta;
    }

    double getScore(){return mBestP.score;}
    double getFastScore(){return mBestP.fast_score;}

    const std::vector< Particle >& getParticles()const{return mCandidates;}

private:
    Map mMap;
    Particle mBestP;
    std::vector< Particle > mPath;
    bool mInit;
    RandomGenerator mRandom;
    std::vector< Particle > mParticles;
    std::vector< Particle > mCandidates;

    PositionFilter mPFilter;
    std::shared_ptr<BaseScore> mScoreFPtr;
};

}

#endif
