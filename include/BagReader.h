#pragma once
#ifndef BAG_READER_H
#define BAG_READER_H

/* rosbag */
#include <rosbag/bag.h>
#include <rosbag/view.h>

/* nodelet */
#include <nodelet/nodelet.h>

/* input and output files */
#include <iostream>
#include <fstream>

/* time */
#include <time.h>

/* ros msgs */
#include <nav_msgs/Odometry.h>

/* flocking custom msgs */
#include <flocking/ModeStamped.h>
#include <flocking/Neighbors.h>

/* MRS packages */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/attitude_converter.h>

/* MRS msgs */
#include <mrs_msgs/Float64Stamped.h>

namespace bag_reader {

class BagReader : public nodelet::Nodelet {
  
  public:
    void onInit();

  private:
    std::vector<std::string> _uav_names_;
    unsigned num_uavs_;
    double _time_step_;

    std::string _path_bags_;
    std::vector<std::string> _bag_names_;

    std::string _path_output_;

    std::vector<ros::Time> time_start_;

    std::vector<std::vector<float>> positions_x_;
    std::vector<std::vector<float>> positions_y_;
    std::vector<std::vector<float>> headings_;
    std::vector<std::vector<ros::Time>> odom_stamps_;

    std::vector<std::vector<float>> virtual_headings_;
    std::vector<std::vector<ros::Time>> virtual_headings_stamps_;

    std::vector<std::vector<std::string>> ranges_;
    std::vector<std::vector<std::string>> bearings_;
    std::vector<std::vector<ros::Time>> neighbors_stamps_;
    
    std::ofstream out_global_headings_;
    std::ofstream out_virtual_headings_;
    std::vector<std::ofstream> out_odometry_;
    std::vector<std::ofstream> out_neighbors_range_;
    std::vector<std::ofstream> out_neighbors_bearing_;

    void readBags();

    void processReadings();

    std::string convertRosTime2String(ros::Time T);
};

} // namespace bag_reader

#endif