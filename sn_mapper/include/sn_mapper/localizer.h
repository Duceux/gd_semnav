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

class Localizer{
public:
    Localizer(){
        mInit = false;
        mHasHint = false;
    }

    void init(const Map &map);

    int process(vector_pts_t& points);

    inline point_t project(const pose_t& ref, const point_t& point){
        return oplus(ref, point);
    }

    void generateUniformParticles(int number);

    void generateParticles(int number, float strength, const Particle &p);

    void scoreParticle(Particle& pl, const vector_pts_t &points);

    void scoreParticle2(Particle& pl, const vector_pts_t &points);


    pose_t getPosition(){return mBestP;}
    void setPosition(const pose_t& pos){
        mBestP.x = pos.x;
        mBestP.y = pos.y;
        mBestP.theta = pos.theta;
    }

    double getScore(){return mBestP.score;}
    double getFastScore(){return mBestP.fast_score;}

    void setvector_pts_tPose(const pose_t& trans){
        vector_pts_tPose_ = trans;
    }
    pose_t getvector_pts_tPose(){return vector_pts_tPose_;}

    Particle predict(){
        if(mPath.size()<2)
            return mBestP;
        Particle prediction;
        prediction.set(oplus(mPath[mPath.size()-1],
              ominus(mPath[mPath.size()-2] ,mPath[mPath.size()-1])));
        return prediction;
    }

    std::vector< Particle > mParticles;


private:

    Map mMap;

    Particle mBestP;
    Particle mLastP;

    std::vector< Particle > mPath;

    std::vector< Particle > mSeeds;

    vector_pts_t mvector_pts_t;

    bool mInit;

    RandomGenerator mRandom;

    float mStdth;
    double mTh;

    bool mHasHint;
    bool mIsLost;

    pose_t vector_pts_tPose_;
};

}

#endif
