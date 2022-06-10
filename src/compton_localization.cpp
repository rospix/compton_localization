/* includes //{ */

#include "ros/init.h"
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Eigen>
#include <mutex>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/attitude_converter.h>

#include <compton_localization/compton_localizationConfig.h>

#include <mrs_msgs/ReferenceStamped.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <compton_localization/Swarm.h>

#include <nav_msgs/Odometry.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>

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
  ros::NodeHandle          nh_;
  bool                     is_initialized_ = false;
  std::string              _uav_name_;
  std::vector<std::string> _uav_names_;

  std::string _swarm_topic_name_;

  // | ----------------------- publishers ----------------------- |

  ros::Publisher publisher_swarm_control_;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<mrs_msgs::ReferenceStampedSrv> sc_reference_;

  // | ----------------------- subscribers ---------------------- |

  std::atomic<bool> active_ = false;

  double validateHeading(const double heading_in);

  ros::Subscriber subscriber_pose;
  void            callbackPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  bool            got_pose_ = false;
  std::mutex      mutex_radiation_pose_;

  geometry_msgs::PoseWithCovarianceStamped radiation_pose;

  ros::Subscriber subscriber_optimizer;
  void            callbackOptimizer(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
  bool            got_optimizer = false;
  std::mutex      mutex_optimizer;

  ros::ServiceServer service_server_activate_;

  void               callbackOdometry(const nav_msgs::OdometryConstPtr &msg);
  std::mutex         mutex_odometry_;
  nav_msgs::Odometry odometry_;
  ros::Subscriber    subscriber_odometry;
  bool               got_odometry_ = false;
  double             heading_;

  geometry_msgs::PoseWithCovarianceStamped optimizer;

  std::vector<mrs_lib::SubscribeHandler<compton_localization::Swarm>> sh_swarm_control_;

  void callbackSwarmControl(mrs_lib::SubscribeHandler<compton_localization::Swarm> &sh_ptr);

  bool got_swarm = false;

  std::mutex                               mutex_swarm_uavs;
  std::vector<compton_localization::Swarm> swarm_uavs_list;
  std::map<std::string, int>               swarm_uavs_map;

  ros::Timer main_timer;
  double     _main_timer_rate_;
  void       timerMain(const ros::TimerEvent &event);

  ros::Timer swarm_timer;
  double     _swarm_timer_rate_;
  void       timerSwarming(const ros::TimerEvent &event);

  // | ------------------------ routines ------------------------ |

  mrs_msgs::Reference generateTrackingReference(void);

  bool callbackActivate([[maybe_unused]] std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  // | ------------------- dynamic reconfigure ------------------ |

  boost::recursive_mutex                                   mutex_drs_;
  typedef compton_localization::compton_localizationConfig DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t>         Drs_t;
  boost::shared_ptr<Drs_t>                                 drs_;
  void                                                     callbackDrs(compton_localization::compton_localizationConfig &config, uint32_t level);
  DrsConfig_t                                              params_;
  std::mutex                                               mutex_params_;

  void dynamicReconfigureCallback(compton_localization::compton_localizationConfig &config, uint32_t level);
};

//}

/* inInit() //{ */

void ComptonLocalization::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO("[ComptonLocalization]: initializing");

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "ComptonLocalization");

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("uav_names", _uav_names_);

  param_loader.loadParam("swarm_topic", _swarm_topic_name_);

  param_loader.loadParam("main_timer_rate", _main_timer_rate_);
  param_loader.loadParam("swarm_timer_rate", _swarm_timer_rate_);

  param_loader.loadParam("tracking/radius", params_.tracking_radius);
  param_loader.loadParam("tracking/height", params_.tracking_height);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ComptonLocalization]: Could not load all parameters!");
    ros::requestShutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  subscriber_pose      = nh_.subscribe("pose_in", 1, &ComptonLocalization::callbackPose, this, ros::TransportHints().tcpNoDelay());
  subscriber_optimizer = nh_.subscribe("optimizer_in", 1, &ComptonLocalization::callbackOptimizer, this, ros::TransportHints().tcpNoDelay());
  subscriber_odometry  = nh_.subscribe("odom_in", 1, &ComptonLocalization::callbackOdometry, this, ros::TransportHints().tcpNoDelay());

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "ComptonLocalization";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  /* subscriber_swarm_control = nh_.subscribe("swarm_in", 1, &ComptonLocalization::callbackSwarmControl, this, ros::TransportHints().tcpNoDelay()); */

  for (int i = 0; i < int(_uav_names_.size()); i++) {

    std::string topic_name = std::string("/") + _uav_names_[i] + std::string("/") + _swarm_topic_name_;

    ROS_INFO("[MpcTracker]: subscribing to %s", topic_name.c_str());

    sh_swarm_control_.push_back(mrs_lib::SubscribeHandler<compton_localization::Swarm>(shopts, topic_name, &ComptonLocalization::callbackSwarmControl, this));
  }

  // | ----------------------- publishers ----------------------- |

  publisher_swarm_control_ = nh_.advertise<compton_localization::Swarm>("swarm_out", 1);

  // | --------------------- service servers -------------------- |

  service_server_activate_ = nh_.advertiseService("activate_in", &ComptonLocalization::callbackActivate, this);

  // | --------------------- service clients -------------------- |

  sc_reference_ = mrs_lib::ServiceClientHandler<mrs_msgs::ReferenceStampedSrv>(nh_, "reference_out");

  // | ------------------------- timers ------------------------- |

  main_timer  = nh_.createTimer(ros::Rate(_main_timer_rate_), &ComptonLocalization::timerMain, this);
  swarm_timer = nh_.createTimer(ros::Rate(_swarm_timer_rate_), &ComptonLocalization::timerSwarming, this);

  // | ------------------- dynamic reconfigure ------------------ |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(params_);
  Drs_t::CallbackType f = boost::bind(&ComptonLocalization::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  ROS_INFO("[ComptonLocalization]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackPose() //{ */

void ComptonLocalization::callbackPose(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {

  if (!is_initialized_)
    return;

  ROS_INFO_ONCE("[ComptonLocalization]: getting pose");

  std::scoped_lock lock(mutex_radiation_pose_);

  got_pose_ = true;

  radiation_pose = *msg;
}

//}

/* callbackOdometry() //{ */

void ComptonLocalization::callbackOdometry(const nav_msgs::OdometryConstPtr &msg) {

  if (!is_initialized_)
    return;

  ROS_INFO_ONCE("[ComptonLocalization]: getting odometry");

  std::scoped_lock lock(mutex_odometry_);

  got_odometry_ = true;

  odometry_ = *msg;
}

//}

/* callbackSwarmControl() //{ */

void ComptonLocalization::callbackSwarmControl(mrs_lib::SubscribeHandler<compton_localization::Swarm> &sh_ptr) {

  if (!is_initialized_)
    return;

  auto msg = sh_ptr.getMsg();

  if (msg->uav_name == _uav_name_) {
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

  if (!is_initialized_) {
    return;
  }

  std::scoped_lock lock(mutex_optimizer);

  got_optimizer = true;

  optimizer = *msg;
}

//}

/* dynamicReconfigureCallback() //{ */

void ComptonLocalization::callbackDrs(compton_localization::compton_localizationConfig &config, [[maybe_unused]] uint32_t level) {

  std::scoped_lock lock(mutex_params_);

  params_ = config;

  ROS_INFO("[ComptonLocalization]: drs updated params");
}

//}

/* callbackActivate() //{ */

bool ComptonLocalization::callbackActivate([[maybe_unused]] std_srvs::SetBool::Request &req, [[maybe_unused]] std_srvs::SetBool::Response &res) {

  active_ = req.data;

  std::stringstream ss;
  ss << std::string(active_ ? "activating" : "de-activating");

  ROS_INFO("[ComptonLocalization]: %s", ss.str().c_str());

  res.message = ss.str();
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
      atan2(odometry_.pose.pose.position.y - radiation_pose.pose.pose.position.y, odometry_.pose.pose.position.x - radiation_pose.pose.pose.position.x);

  // calculate the angle bias
  double closest_dist = 2 * M_PI;
  double angle_bias   = 0;

  {
    std::scoped_lock lock(mutex_swarm_uavs);

    for (std::vector<compton_localization::Swarm>::iterator it = swarm_uavs_list.begin(); it != swarm_uavs_list.end(); it++) {

      double dist = fabs(validateHeading(it->orbit_angle) - validateHeading(current_angle));

      if (dist < closest_dist) {

        closest_dist = dist;
        angle_bias   = (it->orbit_angle - current_angle > 0) ? -0.50 : 0.50;
      }
    }
  }

  ROS_INFO_THROTTLE(1.0, "[ComptonLocalization]: angle_bias: %2.2f", angle_bias);

  current_angle += angle_bias;

  std::scoped_lock lock(mutex_radiation_pose_, mutex_odometry_);

  // create the trajectory
  mrs_msgs::Reference new_reference;

  new_reference.position.x = radiation_pose.pose.pose.position.x + params_.tracking_radius * cos(current_angle + 1.0);
  new_reference.position.y = radiation_pose.pose.pose.position.y + params_.tracking_radius * sin(current_angle + 1.0);
  new_reference.position.z = params_.tracking_height;
  new_reference.heading =
      atan2(radiation_pose.pose.pose.position.y - new_reference.position.y, radiation_pose.pose.pose.position.x - new_reference.position.x) - 1.57;

  ROS_INFO_THROTTLE(1.0, "[ComptonLocalization]: current angle: %.2f", current_angle);

  return new_reference;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* mainTimer() //{ */

void ComptonLocalization::timerMain([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  if (!got_odometry_) {
    return;
  }

  if (!active_) {
    ROS_INFO_THROTTLE(1.0, "[ComptonLocalization]: not active");
    return;
  }

  if (!got_pose_) {
    return;
  }

  mrs_msgs::ReferenceStampedSrv new_reference_srv;

  new_reference_srv.request.header.stamp    = ros::Time::now();
  new_reference_srv.request.header.frame_id = "stable_origin";

  ROS_INFO_THROTTLE(1.0, "[ComptonLocalization]: generating tracking reference");

  new_reference_srv.request.reference = generateTrackingReference();

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

void ComptonLocalization::timerSwarming([[maybe_unused]] const ros::TimerEvent &event) {

  if (!is_initialized_) {
    return;
  }

  compton_localization::Swarm swarm_out;

  swarm_out.orbit_angle =
      atan2(odometry_.pose.pose.position.y - radiation_pose.pose.pose.position.y, odometry_.pose.pose.position.x - radiation_pose.pose.pose.position.x);
  swarm_out.header.stamp = ros::Time::now();
  swarm_out.uav_name     = _uav_name_;

  try {
    publisher_swarm_control_.publish(swarm_out);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", publisher_swarm_control_.getTopic().c_str());
  }
}

//}

}  // namespace compton_localization

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(compton_localization::ComptonLocalization, nodelet::Nodelet)
