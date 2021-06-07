/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Eigen>
#include <mutex>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/subscribe_handler.h>

#include <compton_localization/compton_localizationConfig.h>

#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <compton_localization/Swarm.h>

#include <nav_msgs/Odometry.h>

#include <std_srvs/Trigger.h>

#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>

//}

namespace compton_localization
{

/* ComptonLocalization //{ */
class ComptonLocalization : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized = false;
  std::string     uav_name_;

  ros::Publisher publisher_swarm_control;

  mrs_lib::ServiceClientHandler<mrs_msgs::ReferenceStampedSrv> sc_reference_;

  double validateHeading(const double heading_in);

  ros::Subscriber subscriber_pose;
  void            callbackPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  bool            got_pose = false;
  std::mutex      mutex_radiation_pose;

  geometry_msgs::PoseWithCovarianceStamped radiation_pose;

  ros::Subscriber subscriber_optimizer;
  void            callbackOptimizer(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  bool            got_optimizer = false;
  std::mutex      mutex_optimizer;

  ros::ServiceServer service_server_search;

  void               callbackOdometry(const nav_msgs::OdometryConstPtr &msg);
  std::mutex         mutex_odometry;
  nav_msgs::Odometry odometry;
  ros::Subscriber    subscriber_odometry;
  bool               got_odometry = false;
  double             odometry_heading;
  double             odometry_roll;
  double             odometry_pitch;

  geometry_msgs::PoseWithCovarianceStamped optimizer;

  ros::Subscriber subscriber_swarm_control;
  void            callbackSwarmControl(const compton_localization::SwarmConstPtr &msg);
  bool            got_swarm = false;

  std::mutex                               mutex_swarm_uavs;
  std::vector<compton_localization::Swarm> swarm_uavs_list;
  std::map<std::string, int>               swarm_uavs_map;

  ros::Timer main_timer;
  int        main_timer_rate_;
  void       mainTimer(const ros::TimerEvent &event);

  ros::Timer swarm_timer;
  int        swarm_timer_rate;
  void       swarmTimer(const ros::TimerEvent &event);

  mrs_msgs::Reference generateTrackingReference(void);
  mrs_msgs::Reference generateSearchingReference(void);

  double searching_x, searching_y;

  bool callbackSearch([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  // --------------------------------------------------------------
  // |                     dynamic reconfigure                    |
  // --------------------------------------------------------------

  double     tracking_radius_, tracking_height_;
  double     searching_radius_, searching_height_, searching_heading_rate_;
  std::mutex mutex_params;

  boost::recursive_mutex                                   config_mutex_;
  typedef compton_localization::compton_localizationConfig Config;
  typedef dynamic_reconfigure::Server<Config>              ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>                     reconfigure_server_;
  void                                                     drs_callback(compton_localization::compton_localizationConfig &config, uint32_t level);
  compton_localization::compton_localizationConfig         drs_compton_localization;

  void       dynamicReconfigureCallback(compton_localization::compton_localizationConfig &config, uint32_t level);
  std::mutex mutex_drs;
};
//}

/* inInit() //{ */

void ComptonLocalization::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO("[ComptonLocalization]: initializing");

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "ComptonLocalization");

  param_loader.loadParam("uav_name", uav_name_);

  param_loader.loadParam("main_timer_rate", main_timer_rate_);
  param_loader.loadParam("swarm_timer_rate", swarm_timer_rate);

  param_loader.loadParam("tracking/radius", tracking_radius_);
  param_loader.loadParam("tracking/height", tracking_height_);

  param_loader.loadParam("searching/initial_position/x", searching_x);
  param_loader.loadParam("searching/initial_position/y", searching_y);
  param_loader.loadParam("searching/radius", searching_radius_);
  param_loader.loadParam("searching/height", searching_height_);
  param_loader.loadParam("searching/heading_rate", searching_heading_rate_);

  // | ----------------------- subscribers ---------------------- |

  subscriber_pose          = nh_.subscribe("pose_in", 1, &ComptonLocalization::callbackPose, this, ros::TransportHints().tcpNoDelay());
  subscriber_optimizer     = nh_.subscribe("optimizer_in", 1, &ComptonLocalization::callbackOptimizer, this, ros::TransportHints().tcpNoDelay());
  subscriber_odometry      = nh_.subscribe("odom_in", 1, &ComptonLocalization::callbackOdometry, this, ros::TransportHints().tcpNoDelay());
  subscriber_swarm_control = nh_.subscribe("swarm_in", 1, &ComptonLocalization::callbackSwarmControl, this, ros::TransportHints().tcpNoDelay());

  // | ----------------------- publishers ----------------------- |

  publisher_swarm_control = nh_.advertise<compton_localization::Swarm>("swarm_out", 1);

  // | --------------------- service servers -------------------- |

  service_server_search = nh_.advertiseService("search_in", &ComptonLocalization::callbackSearch, this);

  // | --------------------- service clients -------------------- |

  sc_reference_ = mrs_lib::ServiceClientHandler<mrs_msgs::ReferenceStampedSrv>(nh_, "reference_out");

  // | ------------------------- timers ------------------------- |

  main_timer  = nh_.createTimer(ros::Rate(main_timer_rate_), &ComptonLocalization::mainTimer, this);
  swarm_timer = nh_.createTimer(ros::Rate(swarm_timer_rate), &ComptonLocalization::swarmTimer, this);

  // | ------------------- dynamic reconfigure ------------------ |

  drs_compton_localization.tracking_radius = tracking_radius_;
  drs_compton_localization.tracking_height = tracking_height_;

  drs_compton_localization.searching_radius = searching_radius_;
  drs_compton_localization.searching_height = searching_height_;

  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh_));
  reconfigure_server_->updateConfig(drs_compton_localization);
  ReconfigureServer::CallbackType f = boost::bind(&ComptonLocalization::dynamicReconfigureCallback, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ComptonLocalization]: Could not load all parameters!");
    ros::shutdown();
  }

  is_initialized = true;

  ROS_INFO("[ComptonLocalization]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackPose() //{ */

void ComptonLocalization::callbackPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {

  if (!is_initialized)
    return;

  ROS_INFO_ONCE("[ComptonLocalization]: getting pose");

  std::scoped_lock lock(mutex_radiation_pose);

  got_pose = true;

  radiation_pose = *msg;
}

//}

/* callbackOdometry() //{ */

void ComptonLocalization::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized)
    return;

  ROS_INFO_ONCE("[ComptonLocalization]: getting odometry");

  std::scoped_lock lock(mutex_odometry);

  got_odometry = true;

  odometry = *msg;

  // calculate the euler angles
  tf::Quaternion quaternion_odometry;
  quaternionMsgToTF(odometry.pose.pose.orientation, quaternion_odometry);
  tf::Matrix3x3 m(quaternion_odometry);
  m.getRPY(odometry_roll, odometry_pitch, odometry_heading);
}

//}

/* callbackSwarmControl() //{ */

void ComptonLocalization::callbackSwarmControl(const compton_localization::SwarmConstPtr &msg) {

  if (!is_initialized)
    return;

  if (msg->uav_name == uav_name_) {
    return;
  }

  std::scoped_lock lock(mutex_swarm_uavs);

  ROS_INFO_ONCE("[ComptonLocalization]: getting other uavs");

  // get the idx of the uav_name
  auto uav_in_map = swarm_uavs_map.find(msg->uav_name);

  if (uav_in_map == swarm_uavs_map.end()) {

    ROS_INFO("[ComptonLocalization]: registering new uav: %s", msg->uav_name.c_str());

    swarm_uavs_list.push_back(*msg);
    swarm_uavs_map.insert(std::pair(msg->uav_name, swarm_uavs_list.size() - 1));

  } else {

    // update the uav in the list
    swarm_uavs_list.at(uav_in_map->second) = *msg;
  }
}

//}

/* callbackOptimizer() //{ */

void ComptonLocalization::callbackOptimizer(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {

  if (!is_initialized) {
    return;
  }

  std::scoped_lock lock(mutex_optimizer);

  got_optimizer = true;

  optimizer = *msg;
}

//}

/* dynamicReconfigureCallback() //{ */

void ComptonLocalization::dynamicReconfigureCallback(compton_localization::compton_localizationConfig &config, [[maybe_unused]] uint32_t level) {

  std::scoped_lock lock(mutex_params);

  tracking_radius_ = config.tracking_radius;
  tracking_height_ = config.tracking_height;

  searching_radius_ = config.searching_radius;
  searching_height_ = config.searching_height;

  ROS_INFO("[ComptonLocalization]: drs updated params");
}

//}

/* callbackSearch() //{ */

bool ComptonLocalization::callbackSearch([[maybe_unused]] std_srvs::Trigger::Request &req, [[maybe_unused]] std_srvs::Trigger::Response &res) {

  got_pose = false;

  ROS_INFO("[ComptonLocalization]: switching back to search");

  res.message = "";
  res.success = true;

  return true;
}

//}

// --------------------------------------------------------------
// |                       custom routines                      |
// --------------------------------------------------------------

/* validateHeading() //{ */

double ComptonLocalization::validateHeading(const double heading_in) {

  double heading_out = heading_in;

  if (!std::isfinite(heading_out)) {

    heading_out = 0;
    ROS_ERROR("[validateheadingSetpoint]: Desired heading is not finite number!");
  }

  if (fabs(heading_out) > 1000) {

    ROS_WARN("[validateheadingSetpoint]: Desired heading is > 1000");
  }
  // if desired heading_out is grater then 2*PI mod it
  if (fabs(heading_out) > 2 * M_PI) {
    heading_out = fmod(heading_out, 2 * M_PI);
  }

  // move it to its place
  if (heading_out > M_PI) {
    heading_out -= 2 * M_PI;
  } else if (heading_out < -M_PI) {
    heading_out += 2 * M_PI;
  }

  return heading_out;
}

//}

/* generateTrackingReference() //{ */

mrs_msgs::Reference ComptonLocalization::generateTrackingReference(void) {

  // get current angle
  double current_angle =
      atan2(odometry.pose.pose.position.y - radiation_pose.pose.pose.position.y, odometry.pose.pose.position.x - radiation_pose.pose.pose.position.x);

  // calculate the angle bias
  double closest_dist = 2 * M_PI;
  double angle_bias   = 0;

  {
    std::scoped_lock lock(mutex_swarm_uavs);

    for (std::vector<compton_localization::Swarm>::iterator it = swarm_uavs_list.begin(); it != swarm_uavs_list.end(); it++) {

      double dist = fabs(validateHeading(it->orbit_angle) - validateHeading(current_angle));

      if (dist < closest_dist) {

        closest_dist = dist;
        angle_bias   = (it->orbit_angle - current_angle > 0) ? -0.10 : 0.10;
      }
    }
  }

  ROS_INFO_THROTTLE(1.0, "[ComptonLocalization]: angle_bias: %2.2f", angle_bias);

  current_angle += angle_bias;

  std::scoped_lock lock(mutex_radiation_pose, mutex_odometry);

  // create the trajectory
  mrs_msgs::Reference new_reference;

  new_reference.position.x = radiation_pose.pose.pose.position.x + tracking_radius_ * cos(current_angle + 0.7);
  new_reference.position.y = radiation_pose.pose.pose.position.y + tracking_radius_ * sin(current_angle + 0.7);
  new_reference.position.z = tracking_height_;
  new_reference.heading = atan2(radiation_pose.pose.pose.position.y - new_reference.position.y, radiation_pose.pose.pose.position.x - new_reference.position.x);

  ROS_INFO_THROTTLE(1.0, "[ComptonLocalization]: current angle: %.2f", current_angle);

  // update the searching coordinates
  searching_x = radiation_pose.pose.pose.position.x;
  searching_y = radiation_pose.pose.pose.position.y;

  return new_reference;
}

//}

/* generateSearchingReference() //{ */

mrs_msgs::Reference ComptonLocalization::generateSearchingReference(void) {

  // get current angle
  double current_angle = atan2(odometry.pose.pose.position.y - searching_y, odometry.pose.pose.position.x - searching_x);

  // calculate the angle bias
  double closest_dist = 2 * M_PI;
  double angle_bias   = 0;

  {
    std::scoped_lock lock(mutex_swarm_uavs);

    for (std::vector<compton_localization::Swarm>::iterator it = swarm_uavs_list.begin(); it != swarm_uavs_list.end(); it++) {

      double dist = fabs(validateHeading(it->orbit_angle) - validateHeading(current_angle));

      if (dist < closest_dist) {

        closest_dist = dist;
        angle_bias   = (it->orbit_angle - current_angle > 0) ? -0.10 : 0.10;
      }
    }
  }

  current_angle += angle_bias;

  ROS_INFO_THROTTLE(1.0, "[ComptonLocalization]: angle_bias: %2.2f", angle_bias);

  std::scoped_lock lock(mutex_odometry);

  // create the trajectory
  mrs_msgs::Reference new_reference;

  double current_heading = odometry_heading;

  mrs_msgs::Reference new_point;

  current_heading += searching_heading_rate_ / 5.0;

  new_reference.position.x = searching_x + searching_radius_ * cos(current_angle + 0.7);
  new_reference.position.y = searching_y + searching_radius_ * sin(current_angle + 0.7);
  new_reference.position.z = searching_height_;
  /* new_reference.reference.heading    = current_heading; */
  new_point.heading = atan2(new_reference.position.y - searching_y, new_reference.position.x - searching_x);

  return new_reference;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* mainTimer() //{ */

void ComptonLocalization::mainTimer([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized) {
    return;
  }

  if (!got_odometry) {
    return;
  }

  mrs_msgs::ReferenceStampedSrv new_reference_srv;

  new_reference_srv.request.header.stamp    = ros::Time::now();
  new_reference_srv.request.header.frame_id = "stable_origin";

  if (got_pose) {

    ROS_INFO_THROTTLE(1.0, "[ComptonLocalization]: generating tracking reference");

    new_reference_srv.request.reference = generateTrackingReference();

  } else {

    ROS_INFO_THROTTLE(1.0, "[ComptonLocalization]: generating searching reference");

    new_reference_srv.request.reference = generateSearchingReference();
  }

  ROS_INFO("[ComptonLocalization]: reference: [%.2f, %.2f, %.2f]", new_reference_srv.request.reference.position.x,
           new_reference_srv.request.reference.position.y, new_reference_srv.request.reference.position.z);

  bool success = sc_reference_.call(new_reference_srv);

  if (!success) {
    ROS_ERROR("[ComptonLocalization]: could not call reference service");
  } else {
    if (!new_reference_srv.response.success) {
      ROS_ERROR("[ComptonLocalization]: service call for reference failed: '%s'", new_reference_srv.response.message.c_str());
    }
  }
}

//}

/* swarmTimer() //{ */

void ComptonLocalization::swarmTimer([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized) {
    return;
  }

  compton_localization::Swarm swarm_out;

  swarm_out.orbit_angle =
      atan2(odometry.pose.pose.position.y - radiation_pose.pose.pose.position.y, odometry.pose.pose.position.x - radiation_pose.pose.pose.position.x);
  swarm_out.header.stamp = ros::Time::now();
  swarm_out.uav_name     = uav_name_;

  try {
    publisher_swarm_control.publish(swarm_out);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", publisher_swarm_control.getTopic().c_str());
  }
}

//}

}  // namespace compton_localization

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(compton_localization::ComptonLocalization, nodelet::Nodelet)
