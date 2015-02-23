#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sn_msgs/DescriptorSequence.h>
#include <sn_dictionary/dico.h>
#include <unordered_map>
#include <sn_features/histogram_distance.h>
#include <unordered_set>
#include <sn_models/bag_of_word.h>
#include <sn_tools/confusion_matrix.h>

typedef sn_msgs::DescriptorSequence Sequence;
typedef sn_msgs::Descriptor Descriptor;
typedef std::vector<Sequence::Ptr> VSeq;

void load(const std::string& filename, VSeq& trackers)
{
    std::cout << "opening: " << filename << std::endl;
    rosbag::Bag bag;
    try{
        bag.open(filename, rosbag::bagmode::Read);
        std::vector<std::string> topics;
        topics.push_back(std::string("sequence"));

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        for(auto m: view){
            Sequence::Ptr g = m.instantiate<sn_msgs::DescriptorSequence>();
            if (g != NULL){
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

typedef std::unordered_set<sn::Word> BinaryBagOfWord;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "tool");
    ros::NodeHandle handle(std::string("~"));


    VSeq trackers;
    load("/home/robotic/Desktop/datasets/sequences/all.bag", trackers);

    sn::Dictionary<sn::FastGetter> dicos;
    dicos.set("laser", 0.2, sn::Distance(sn::symmetric_chi2_distance));
    std::vector<Descriptor> descriptors;
    descriptors.reserve(100000);
    for(auto& it: trackers)
        for(auto& des: it->descriptors){
            descriptors.push_back(des);
        }
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(descriptors.begin(), descriptors.end(), g);
    if(descriptors.size() > 10000)
        descriptors.resize(10000);
    for(auto des: descriptors){
        dicos.get(des);
    }
    std::cout << descriptors.size() << std::endl;
    std::cout << dicos.size("laser") << std::endl;

    std::vector<sn::BagOfWord::Ptr> truth;
    std::map<std::string, int> labels;
    std::vector<sn::BagOfWord::Ptr> test;
    std::vector<sn::BagOfWord::Ptr> all;
    for(sn_msgs::DescriptorSequencePtr seq_ptr: trackers){
        auto bow = sn::BagOfWord::create(*seq_ptr, dicos);
        if(labels[bow->name] < 2){
            truth.push_back(bow);
            labels[bow->name]++;
        }
        else
            test.push_back(bow);
        all.push_back(bow);
    }

    for(auto bow: truth){
        std::cout << bow->name << std::endl;
        std::cout << bow->bag.size() << std::endl;
    }
    for(auto bow: test){
        std::cout << bow->name << std::endl;
        std::cout << bow->bag.size() << std::endl;
    }
    auto matrix = sn::confusion_matrix(truth, test, &sn::intersection);
    std::cout << sn::get_precision(matrix) << std::endl;
    sn::print(matrix);

    matrix = sn::confusion_matrix(truth, test, &sn::binary_intersection);
    std::cout << sn::get_precision(matrix) << std::endl;
    sn::print(matrix);



    return 0;
}
