#ifndef IO_H
#define IO_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>

namespace fs = boost::filesystem;

namespace sn{
namespace tools{

template<typename MsgPtr>
void load(const fs::path& path, std::vector<MsgPtr>& trackers, std::string topic)
{
  std::cout << "opening: " << path << std::endl;
  rosbag::Bag bag;
  boost::smatch match;
  boost::regex e ("([^0-9]+)");
  try{
    bag.open(path.string(), rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string(topic));

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for(auto m: view){
      MsgPtr g = m.instantiate<typename MsgPtr::element_type>();
      if (g != NULL){
        boost::regex_search(path.stem().string(),match,e);
        g->name = match[0];
        std::cout << g->name << std::endl;
        trackers.push_back(g);
      }
    }
    bag.close();
  }
  catch(const std::exception& e){
    ROS_ERROR("%s", e.what());
  }
  ROS_INFO("nb sequences loaded: %lu ", trackers.size());

}

}//sn
}//tools

#endif // IO_H
