#include <ros/ros.h>
#include <sn_msgs/DetectionArray.h>
#include <eigen3/Eigen/Core>
#include <visualization_msgs/Marker.h>
#include <sn_msgs/TrackerArray.h>
#include <sn_geometry/sn_geometry.h>
#include <rosbag/bag.h>
#include <std_srvs/Empty.h>

struct Tracking{
  ros::Publisher mDebugPub;
  ros::Publisher mTrackerPub;
  ros::Publisher mTrackerPub2;
  ros::Subscriber mDetectSub;

  ros::ServiceServer mDumpService;

  double mThreshold;
  double mTimeThreshold;
  sn_msgs::TrackerArray mTrackers;
  std::map<ros::Time, std::array<float, 3>> mColors;

  void detection_callback(const sn_msgs::DetectionArrayConstPtr &ptr);
  void track();
  sn_msgs::Tracker create_tracker(const sn_msgs::Detection& det);

  template<typename Iterator>
  Iterator find_closest_in_time(Iterator begin, Iterator end, ros::Time time);

  bool save(const std::string& filename);

  bool dump(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& rep);

};

bool Tracking::dump(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &rep){
  while(mTrackers.trackers.size() > 0){
    mTrackerPub2.publish(mTrackers.trackers.back());
    mTrackers.trackers.erase(mTrackers.trackers.end());
  }
  return true;
}

bool Tracking::save(const std::string &filename){
  rosbag::Bag bag;
  try{
    bag.open(filename, rosbag::bagmode::Write);
    for(auto tk: mTrackers.trackers){
      bag.write("/tracking/saved_tracker", ros::Time::now(), tk);
    }
    bag.close();
  }catch(const std::exception& e){
    ROS_ERROR("%s", e.what());
    bag.close();
    return false;
  }
  ROS_INFO("Saved %lu graphs to %s", mTrackers.trackers.size(), filename.c_str());
  return true;
}

sn_msgs::Tracker Tracking::create_tracker(const sn_msgs::Detection& det){
  sn_msgs::Tracker res;
  res.detections.push_back(det);
  res.ended = false;
  auto real_now = ros::WallTime::now();
  res.uid = ros::Time(real_now.sec, real_now.nsec);
  res.header = det.header;
  return res;
}

template<typename Iterator>
Iterator Tracking::find_closest_in_time(Iterator begin, Iterator end, ros::Time time)
{

  // find detection in time in the track
  // because they are ordorred in time start from end
  // if distance augment I can stop looking
  auto det_min = begin;
  double time_dist = std::abs((det_min->header.stamp-time).toSec());
  for(auto it=begin; it!=end; ++it){
    double dist = std::abs((it->header.stamp-time).toSec());
    if(dist <= time_dist){
      det_min = it;
      time_dist = dist;
    }
    else
      break;
  }
  return det_min;
}

void Tracking::detection_callback(const sn_msgs::DetectionArrayConstPtr &ptr){

  mTrackers.header = ptr->header;
  for(auto& tck: mTrackers.trackers)
    tck.ended = true;

  for(const sn_msgs::Detection& det: ptr->detections){

    // find closest tracker
    double mindist = std::numeric_limits<double>::max();
    sn_msgs::Tracker* arg_min = NULL;
    bool found = false;

    for(sn_msgs::Tracker& tck: mTrackers.trackers){

      auto det_min = find_closest_in_time(tck.detections.rbegin(),
                                          tck.detections.rend(),
                                          det.header.stamp);

      sn::point_t center1 = det_min->bbox.center;
      sn::point_t center2 = det_min->bbox.center;
      center1.z = center2.z = 0.0;
      double distance = sn::l2_norm(center1-center2);
      if(distance < mindist && distance<mThreshold){
        arg_min=&tck;
        mindist = distance;
        found = true;
      }
    }

    if(!found){
      // new tracker     
      mTrackers.trackers.push_back(create_tracker(det));
    }else{
      auto det_min = find_closest_in_time(arg_min->detections.rbegin(),
                                          arg_min->detections.rend(),
                                          det.header.stamp);
      arg_min->detections.insert(det_min.base(), det);
      arg_min->ended = false;
      arg_min->header = ptr->header;
    }
  }

  mTrackerPub.publish(mTrackers);

  // erase those that haven´t been updated
  for(uint i=0; i<mTrackers.trackers.size(); ++i){
    ros::Duration duration = mTrackers.header.stamp - mTrackers.trackers[i].header.stamp;
    if(duration.toSec() > mTimeThreshold && mTrackers.trackers[i].ended){
      mTrackerPub2.publish(mTrackers.trackers[i]);
      mTrackers.trackers.erase(mTrackers.trackers.begin()+i);
      --i;
    }
  }

  if(mDebugPub.getNumSubscribers() == 0)
    return;

  for(auto& tck: mTrackers.trackers){
    auto& det = tck.detections.back();
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    auto center = det.bbox.center;
    marker.pose.position.x = center.x;
    marker.pose.position.y = center.y;
    marker.pose.position.z = center.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    auto label = tck.uid;
    if(mColors.count( label ) == 0){
      static std::default_random_engine generator;
      static std::uniform_real_distribution<float> distribution(0.f,1.f);
      static auto random = std::bind ( distribution, generator );
      mColors[label][0] = random();
      mColors[label][1] = random();
      mColors[label][2] = random();
    }
    marker.scale.x = mThreshold;
    marker.scale.y = mThreshold;
    marker.scale.z = 0.01;
    marker.color.r = mColors[label][0];
    marker.color.g = mColors[label][1];
    marker.color.b = mColors[label][2];
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(mTimeThreshold);
    marker.header.frame_id = ptr->header.frame_id;
    marker.ns = "cylinder";
    marker.id =  label.toNSec();
    mDebugPub.publish(marker);
  }
  for(auto& tck: mTrackers.trackers){
    auto& det = tck.detections.back();
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    auto center = det.bbox.center;
    marker.pose.position.x = center.x;
    marker.pose.position.y = center.y;
    marker.pose.position.z = center.z;
    auto label = tck.uid;
    marker.scale.z = 0.4;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1.0;
    std::stringstream str;
    str << label;
    marker.text = str.str();
    marker.lifetime = ros::Duration(mTimeThreshold);
    marker.header.frame_id = ptr->header.frame_id;
    marker.ns = "text";
    marker.id = label.toNSec();
    mDebugPub.publish(marker);
  }
  /*
    for(auto& tck: mTrackers.trackers){
        auto& det = tck.detections.back();
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        auto center = det.bbox.center;
        sn::point_t robot = sn::create(det.robot.x, det.robot.y, center.z);
        sn::point_t arrow = center-robot;
        arrow = arrow*sn::l2_norm(arrow)*1.1;
        marker.points.push_back(robot);
        marker.points.push_back(arrow+robot);
        auto label = tck.uid;
        marker.scale.x = 0.01;
        marker.scale.y = 0.0;
        marker.scale.z = 0.0;
        marker.color.r = mColors[label][0];
        marker.color.g = mColors[label][1];
        marker.color.b = mColors[label][2];
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        marker.header.frame_id = ptr->header.frame_id;
        marker.ns = "arrow";
        marker.id = ros::Time::now().toNSec()+label.toNSec();
        mDebugPub.publish(marker);
    }
*/
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "tracking");
  ros::NodeHandle handle(std::string("~"));

  Tracking tracking;
  std::string input;
  ros::param::param<std::string>("~input", input,
                                 "/detections");
  tracking.mDetectSub = handle.subscribe(input,
                                         1,
                                         &Tracking::detection_callback,
                                         &tracking
                                         );

  tracking.mTrackerPub = handle.advertise<sn_msgs::TrackerArray>("trackers", 1);
  tracking.mTrackerPub2 = handle.advertise<sn_msgs::Tracker>("ended_tracker", 5);
  tracking.mDebugPub = handle.advertise<visualization_msgs::Marker>("markers", 1);

  tracking.mDumpService = handle.advertiseService("dump", &Tracking::dump, &tracking);

  ROS_INFO("Listening to: %s", input.c_str());
  ros::param::param<double>("~threshold", tracking.mThreshold, 0.35);
  std::cout << "Tracking: " << tracking.mThreshold << std::endl;
  ros::param::param<double>("~time_threshold", tracking.mTimeThreshold, 2.0);
  std::cout << "Tracking: " << tracking.mTimeThreshold << std::endl;

  ros::spin();

  return 0;
}

