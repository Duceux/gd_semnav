#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sn_msgs/Tracker.h>
#include <opencv2/opencv.hpp>
#include <sn_geometry/sn_geometry.h>
#include <sn_features/laser_descriptor.h>
#include <sn_features/polar_histogram.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <geometry_msgs/PoseArray.h>
#include <boost/filesystem.hpp>
#include <sn_msgs/DescriptorSequence.h>

namespace fs = boost::filesystem;

struct compare_func{ bool operator()(sn_msgs::TrackerPtr const& l, sn_msgs::TrackerPtr const& r){return l->uid<r->uid;} };
typedef std::set<sn_msgs::TrackerPtr, compare_func> TrackersSet;

typedef sn_msgs::DescriptorSequence Seq;
typedef sn_msgs::Descriptor Des;
typedef std::vector<Seq> VSeq;

void load(const std::string& filename, TrackersSet& trackers)
{
    std::cout << "opening: " << filename << std::endl;
    rosbag::Bag bag;
    try{
        bag.open(filename, rosbag::bagmode::Read);
        std::vector<std::string> topics;
        topics.push_back(std::string("/tracking/ended_tracker"));

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        for(auto m: view){
            sn_msgs::TrackerPtr g = m.instantiate<sn_msgs::Tracker>();
            if (g != NULL){
                trackers.insert(g);
                if(g->name.size() != 0)
                    (*trackers.find(g))->name = g->name;
            }
        }
        bag.close();
    }
    catch(const std::exception& e){
        ROS_ERROR("%s", e.what());
    }
    ROS_INFO("nb trackers loaded: %lu ", trackers.size());

}

void save(const std::string& filename, VSeq const& trackers)
{
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Write);
    long unsigned int total = 0;
    for(auto tck: trackers){
        try{
            bag.write("sequence", ros::Time::now(), tck);
            total++;
        }catch(const std::exception& e){
            ROS_ERROR("%s", e.what());
            bag.close();
            return;
        }
    }
    bag.close();

    ROS_INFO("Saved %lu sequences to %s", total, filename.c_str());
}


int main( int argc, char** argv )
{
    ros::init(argc, argv, "tool");
    ros::NodeHandle handle(std::string("~"));


    TrackersSet trackers;
    load("/home/robotic/Desktop/datasets/kdb_trackers/all.bag", trackers);


    sn::TriangleLaserExtractor extractor;
    extractor.sampling_resolution = 0.001;
    extractor.smoothing_factor = 50;
    extractor.downsampling_factor = 50;
    extractor.theta_bin_size = M_PI*2.0/12.0;
    extractor.rho_bin_size = 0.1;

    VSeq vseq;
    for(sn_msgs::TrackerPtr tkr: trackers){
        std::cout << "doing: " << tkr->uid << std::endl;
        Seq seq;
        seq.uid = tkr->uid;
        seq.header = tkr->header;
        seq.name = tkr->name;

        //cloud
        pcl::PointCloud<pcl::PointXYZRGB> ptcld2;
        for(auto det: tkr->detections){
            //laser
            for(sn::point_t p: det.points){
                pcl::PointXYZRGB tmp;
                tmp.x = p.x;
                tmp.y = p.y;
                tmp.z = p.z;
                ptcld2.push_back(tmp);
            }
            //kinect
            if(det.cloud.data.size() > 0){
                pcl::PointCloud<pcl::PointXYZRGB> tmp;
                pcl::fromROSMsg(det.cloud, tmp);
                ptcld2.insert(ptcld2.end(), tmp.begin(), tmp.end());
            }
        }
        pcl::toROSMsg(ptcld2, seq.cloud);
        seq.cloud.header = tkr->header;

        //descriptors
        for(sn_msgs::Detection det: tkr->detections){
            if(det.points.size() > 0){
                Des des = extractor(det);
                seq.descriptors.push_back(des);
            }


        }
        vseq.push_back(seq);
        std::cout << "added: " << seq.name << " " << seq.descriptors.size() << std::endl;

    }


    save("/home/robotic/Desktop/datasets/kdb_sequences/all.bag", vseq);
    return 0;
}
