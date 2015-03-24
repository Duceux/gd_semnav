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
    LOST
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

class Localizer{
public:
    Localizer(){
        mInit = false;
    }

    void init(const Map &map);

    int process(vector_pts_t& points);

    inline point_t project(const pose_t& ref, const point_t& point){
        return oplus(ref, point);
    }

    void generateUniformParticles(int number);

    void generateParticles(int number, float strength, const Particle &p);

    void scoreParticle(Particle& pl, const vector_pts_t &points);
    void fullScoreParticle(Particle& pl, const vector_pts_t &points);

    pose_t getPosition(){return mBestP;}
    void setPosition(const pose_t& pos){
        mBestP.x = pos.x;
        mBestP.y = pos.y;
        mBestP.theta = pos.theta;
    }

    double getScore(){return mBestP.score;}
    double getFastScore(){return mBestP.fast_score;}

    const std::vector< Particle >& getParticles()const{return mParticles;}

private:
    Map mMap;
    Particle mBestP;
    std::vector< Particle > mPath;
    bool mInit;
    RandomGenerator mRandom;
    std::vector< Particle > mParticles;
    PositionFilter mPFilter;
};


}

#endif
