/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

//}

namespace mrs_actionlib_interface
{

/* class MrsActionlibInterface //{ */

class MrsActionlibInterface : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  /* flags */
  std::atomic<bool> is_initialized_ = false;

  /* ros parameters */
  std::string _uav_name_;

  /* mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_; */

  /* void callbackControlManagerDiag(mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>& sh); */

  // | --------------------- timer callbacks -------------------- |

  void           callbackMainTimer(const ros::TimerEvent& te);
  ros::Timer     main_timer_;

  // | ---------------- service server callbacks ---------------- |

  /* bool               callbackStartWaypointFollowing(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res); */
  /* ros::ServiceServer srv_server_start_waypoints_following_; */

  // | --------------------- service clients -------------------- |

  ros::ServiceClient srv_client_land_;
  bool               _land_end_;

};

//}

/* onInit() //{ */

void MrsActionlibInterface::onInit() {

  /* obtain node handle */
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */
  /* mrs_lib::ParamLoader param_loader(nh, "MrsActionlibInterface"); */

  /* param_loader.loadParam("uav_name", _uav_name_); */

  // | ------------------ initialize subscribers ----------------- |

  /* mrs_lib::SubscribeHandlerOptions shopts; */
  /* shopts.nh                 = nh; */
  /* shopts.node_name          = "MrsActionlibInterface"; */
  /* shopts.no_message_timeout = ros::Duration(1.0); */
  /* shopts.threadsafe         = true; */
  /* shopts.autostart          = true; */
  /* shopts.queue_size         = 10; */
  /* shopts.transport_hints    = ros::TransportHints().tcpNoDelay(); */

  /* sh_odometry_             = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odom_uav_in"); */
  /* sh_control_manager_diag_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in", */

  /* subscribe ground truth only in simulation, where it is available */
  /* if (_simulation_) { */
  /*   sh_ground_truth_ = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "odom_gt_in"); */
  /* } */

  // | ------------------ initialize publishers ----------------- |

  /* pub_dist_to_waypoint_ = nh.advertise<mrs_msgs::Float64Stamped>("dist_to_waypoint_out", 1); */

  // | -------------------- initialize timers ------------------- |

  main_timer_ = nh.createTimer(ros::Rate(10), &MrsActionlibInterface::callbackMainTimer, this);


  // | --------------- initialize service servers --------------- |

  /* srv_server_start_waypoints_following_ = nh.advertiseService("start_waypoints_following_in", &MrsActionlibInterface::callbackStartWaypointFollowing, this); */

  // | --------------- initialize service clients --------------- |

  /* srv_client_land_ = nh.serviceClient<std_srvs::Trigger>("land_out"); */

  ROS_INFO_ONCE("[MrsActionlibInterface]: initialized");

  is_initialized_ = true;
}

//}

// | ---------------------- msg callbacks --------------------- |

/* callbackControlManagerDiag() //{ */

/* void MrsActionlibInterface::callbackControlManagerDiag(mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>& sh) { */

/*   /1* do not continue if the nodelet is not initialized *1/ */
/*   if (!is_initialized_) */
/*     return; */

/*   mrs_msgs::ControlManagerDiagnosticsConstPtr diagnostics = sh.getMsg(); */

/*   ROS_INFO_ONCE("[MrsActionlibInterface]: Received first control manager diagnostics msg"); */

/*   if (have_goal_ && !diagnostics->tracker_status.have_goal) { */

/*     ROS_INFO("[MrsActionlibInterface]: Waypoint reached."); */
/*     have_goal_ = false; */

/*     /1* start idling at the reached waypoint *1/ */
/*     is_idling_ = true; */

/*     ros::NodeHandle nh("~"); */
/*     timer_idling_ = nh.createTimer(ros::Duration(_waypoint_idle_time_), &MrsActionlibInterface::callbackTimerIdling, this, */
/*                                    true);  // the last boolean argument makes the timer run only once */

/*     ROS_INFO("[MrsActionlibInterface]: Idling for %.2f seconds.", _waypoint_idle_time_); */
/*   } */
/* } */

//}

// | --------------------- timer callbacks -------------------- |

/* callbackTimerIdling() //{ */

void MrsActionlibInterface::callbackMainTimer([[maybe_unused]] const ros::TimerEvent& te) {

  ROS_INFO("[MrsActionlibInterface]: main timer loop");
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackStartWaypointFollowing() */

/* bool MrsActionlibInterface::callbackStartWaypointFollowing([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) { */

/*   if (!is_initialized_) { */

/*     res.success = false; */
/*     res.message = "Waypoint flier not initialized!"; */
/*     ROS_WARN("[MrsActionlibInterface]: Cannot start waypoint following, nodelet is not initialized."); */
/*     return true; */
/*   } */

/*   if (waypoints_loaded_) { */

/*     timer_publisher_reference_.start(); */

/*     ROS_INFO("[MrsActionlibInterface]: Starting waypoint following."); */

/*     res.success = true; */
/*     res.message = "Starting waypoint following."; */

/*   } else { */

/*     ROS_WARN("[MrsActionlibInterface]: Cannot start waypoint following, waypoints are not set."); */
/*     res.success = false; */
/*     res.message = "Waypoints not set."; */
/*   } */

/*   return true; */
/* } */

//}

}  // namespace mrs_actionlib_interface

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrs_actionlib_interface::MrsActionlibInterface, nodelet::Nodelet);
