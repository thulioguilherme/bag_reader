#include <BagReader.h>
#include <pluginlib/class_list_macros.h>

namespace bag_reader
{

/* onInit() */
void BagReader::onInit() {
  
  ros::NodeHandle nh("~");

  mrs_lib::ParamLoader param_loader(nh, "BagReader");

  param_loader.loadParam("uav_names", _uav_names_);
  param_loader.loadParam("path_bags", _path_bags_);
  param_loader.loadParam("bag_names", _bag_names_);
  param_loader.loadParam("path_output", _path_output_);
  param_loader.loadParam("time_step", _time_step_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[BagReader]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  num_uavs_ = _uav_names_.size();

  if (num_uavs_ != _bag_names_.size()) {
  	ROS_ERROR("[BagReader]: number of UAVs different from number of bags!");
  	ros::shutdown();
  }

  positions_x_.resize(num_uavs_);
  positions_y_.resize(num_uavs_);
  headings_.resize(num_uavs_);
  odom_stamps_.resize(num_uavs_);

  virtual_headings_.resize(num_uavs_);
  virtual_headings_stamps_.resize(num_uavs_);

  ranges_.resize(num_uavs_);
  bearings_.resize(num_uavs_);
  neighbors_stamps_.resize(num_uavs_);

  BagReader::readBags();

  ros::spin();

  ROS_INFO_ONCE("[BagReader]: initialized");
}

void BagReader::readBags() {

  for (unsigned i = 0; i < num_uavs_; i++) {
    std::string bag_with_path = _path_bags_ + "/" + _bag_names_[i];

    std::string topic_change_mode     = "/" + _uav_names_[i] + "/flocking/mode_changed";
    std::string topic_virtual_heading = "/" + _uav_names_[i] + "/flocking/virtual_heading";
    std::string topic_neighbors       = "/" + _uav_names_[i] + "/sensor_neighbor/neighbors";
    std::string topic_odometry        = "/" + _uav_names_[i] + "/odometry/odom_main";

    ROS_INFO("[BagReader]: Reading bag %s", bag_with_path.c_str());
    
    try{
      rosbag::Bag bag;
      bag.open(bag_with_path);

      bool has_began = false;

      for (rosbag::MessageInstance const m: rosbag::View(bag)) {

        if(m.getTopic() == topic_change_mode) {
          flocking::ModeStamped::ConstPtr msg_mode = m.instantiate<flocking::ModeStamped>();

          if (msg_mode != NULL) {
            if (msg_mode->mode == "Swarming") {
              has_began = true;
              time_start_.push_back(m.getTime());
            } else if (msg_mode->mode == "No mode") {
              has_began = false;
            }

            std::string change_time = BagReader::convertRosTime2String(m.getTime());
            ROS_INFO("[BagReader]: The %s changed to %s at %s", _uav_names_[i].c_str(), msg_mode->mode.c_str(), change_time.c_str());
            //std::cout << "The "+ _uav_names_[i] + " changed to "+ msg_mode->mode +" at "+ change_time + " \n"; 
          }
        } else if(m.getTopic() == topic_neighbors) {
          flocking::Neighbors::ConstPtr msg_neighbors = m.instantiate<flocking::Neighbors>();

          std::string line_range = "";
          std::string line_bearing = "";
          if (msg_neighbors != NULL && has_began) {
            for (unsigned j = 0; j < msg_neighbors->num_neighbors; j++) {
              line_range = line_range + std::to_string(msg_neighbors->range[j]);
              line_bearing = line_bearing + std::to_string(msg_neighbors->bearing[j]);

              if (j + 1 < msg_neighbors->num_neighbors) {
                line_range += ",";
                line_bearing += ",";
              }
            }

            ranges_[i].push_back(line_range);
            bearings_[i].push_back(line_bearing);
            neighbors_stamps_[i].push_back(m.getTime());

          }
        } else if (m.getTopic() == topic_virtual_heading) { 
          mrs_msgs::Float64Stamped::ConstPtr msg_virtual_heading = m.instantiate<mrs_msgs::Float64Stamped>();

          if (msg_virtual_heading != NULL && has_began) {
            virtual_headings_[i].push_back(msg_virtual_heading->value);
            virtual_headings_stamps_[i].push_back(m.getTime());
          }
        } else if (m.getTopic() == topic_odometry) { 
          nav_msgs::Odometry::ConstPtr msg_odometry = m.instantiate<nav_msgs::Odometry>();

          if (msg_odometry != NULL && has_began) {
            positions_x_[i].push_back(msg_odometry->pose.pose.position.x);
            positions_y_[i].push_back(msg_odometry->pose.pose.position.y);
            headings_[i].push_back(mrs_lib::AttitudeConverter(msg_odometry->pose.pose.orientation).getHeading());

            odom_stamps_[i].push_back(m.getTime());
          }
        }

      }

      /* close bag */
      bag.close();

    } catch (ros::Exception &e) {
      ROS_ERROR("[BagReader]: %s", e.what());
    }
  }

  ROS_INFO("[BagReader]: Finished the reading of the bags. Starting the processing of the readings");

  BagReader::processReadings();
}

void BagReader::processReadings(){
  ros::Time time_begin = time_start_[0];
  for (unsigned i = 1; i < num_uavs_; i++) {
    if (time_begin.toSec() > time_start_[i].toSec()) {
      time_begin = time_start_[i];
    }
  }

  out_global_headings_.open(_path_output_ + "/global_headings.csv");
  out_virtual_headings_.open(_path_output_ + "/virtual_headings.csv");

  /* open files */
  out_odometry_.resize(num_uavs_);
  out_neighbors_range_.resize(num_uavs_);
  out_neighbors_bearing_.resize(num_uavs_);

  for (unsigned i = 0; i < num_uavs_; i++){
    out_odometry_[i].open(_path_output_ +  "/" + _uav_names_[i] + "_odometry.csv");
    out_neighbors_range_[i].open(_path_output_ +  "/" +  _uav_names_[i] + "_neighbors_range.csv");
    out_neighbors_bearing_[i].open(_path_output_ +  "/" +  _uav_names_[i] + "_neighbors_bearing.csv");
  }

  /* write odometry and global heading files */
  std::vector<unsigned> counter_odom(num_uavs_, 0);
  bool finished = false;
  ros::Time time_begin_cp = time_begin;

  ROS_INFO("[BagReader]: Creating odometry and global heading files");

  while (!finished) {
    for (unsigned i = 0; i < num_uavs_; i++) {
      while (counter_odom[i] < odom_stamps_[i].size() && odom_stamps_[i][counter_odom[i] + 1].toSec() < time_begin_cp.toSec()) {
        counter_odom[i] += 1;
      }

      if (counter_odom[i] >= odom_stamps_[i].size()) {
        finished = true;
        break;
      } 
    }

    if (finished) {
      break;
    }

    for (unsigned i = 0; i < num_uavs_; i++) {
      out_odometry_[i] << positions_x_[i][counter_odom[i]] << "," << positions_y_[i][counter_odom[i]] << "," << headings_[i][counter_odom[i]] << "\n";
      out_global_headings_ << headings_[i][counter_odom[i]];

      if (i + 1 < num_uavs_) {
        out_global_headings_ << ",";
      }
    }

    out_global_headings_ << "\n";

    time_begin_cp += ros::Duration(_time_step_);
  }


  /* write neighbors files */
  std::vector<unsigned> counter_neighbors(num_uavs_, 0);
  finished = false;
  time_begin_cp = time_begin;

  ROS_INFO("[BagReader]: Creating neighbors files");

  while (!finished) {
    for (unsigned i = 0; i < num_uavs_; i++) {
      while (counter_neighbors[i] < neighbors_stamps_[i].size() && neighbors_stamps_[i][counter_neighbors[i] + 1].toSec() < time_begin_cp.toSec()) {
        counter_neighbors[i] += 1;
      }

      if (counter_neighbors[i] >= neighbors_stamps_[i].size()) {
        finished = true;
        break;
      } 
    }

    if (finished) {
      break;
    }

    for (unsigned i = 0; i < num_uavs_; i++) {
      out_neighbors_range_[i] << ranges_[i][counter_neighbors[i]] << "\n";
      out_neighbors_bearing_[i] << bearings_[i][counter_neighbors[i]] << "\n";
    }

    time_begin_cp += ros::Duration(_time_step_);
  }

  /* write virtual heading files */
  std::vector<unsigned> counter_virtual_heading(num_uavs_, 0);
  finished = false;
  time_begin_cp = time_begin;

  ROS_INFO("[BagReader]: Creating virtual heading files");

  while (!finished) {
    for (unsigned i = 0; i < num_uavs_; i++) {
      while (counter_virtual_heading[i] < virtual_headings_stamps_[i].size() && virtual_headings_stamps_[i][counter_virtual_heading[i] + 1].toSec() < time_begin_cp.toSec()) {
        counter_virtual_heading[i] += 1;
      }

      if (counter_virtual_heading[i] >= virtual_headings_stamps_[i].size()) {
        finished = true;
        break;
      } 
    }

    if (finished) {
      break;
    }

    for (unsigned i = 0; i < num_uavs_; i++) {
      out_virtual_headings_ << virtual_headings_[i][counter_virtual_heading[i]];

      if (i + 1 < num_uavs_) {
        out_virtual_headings_ << ",";
      }
    }

    out_virtual_headings_ << "\n";

    time_begin_cp += ros::Duration(_time_step_);
  }

  /* close files */
  out_global_headings_.close();
  out_virtual_headings_.close();

  for (unsigned i = 0; i < num_uavs_; i++){
    out_odometry_[i].close();
    out_neighbors_range_[i].close();
    out_neighbors_bearing_[i].close();
  }

  ROS_INFO("[BagReader]: Finished the processing. Check the files in the %s folder", _path_output_.c_str());

  ros::shutdown();
}

std::string BagReader::convertRosTime2String(ros::Time T) {
  const int output_size = 100;
  char output[output_size];

  std::time_t raw_time = static_cast<time_t>(T.sec);
  struct tm* timeinfo = localtime(&raw_time);
  std::strftime(output, output_size, "%T", timeinfo);

  return std::string(output);
}

}

PLUGINLIB_EXPORT_CLASS(bag_reader::BagReader, nodelet::Nodelet);