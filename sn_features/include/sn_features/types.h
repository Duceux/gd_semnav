#ifndef TYPES_H
#define TYPES_H

#include <sn_msgs/DescriptorArray.h>
#include <sn_msgs/DetectionArray.h>

namespace sn {

typedef std::vector<double> feature_t;
typedef sn_msgs::Descriptor descriptor_t;
typedef sn_msgs::Detection detection_t;
typedef sn_msgs::DescriptorArray descriptor_array_t;
typedef sn_msgs::DetectionArray detection_array_t;

}

#endif // TYPES_H
