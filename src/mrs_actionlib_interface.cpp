/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>

#include <mrs_actionlib_interface/gotoAction.h>
#include <actionlib/server/simple_action_server.h>

/* class MrsActionlibInterface //{ */

class MrsActionlibInterface {

public:
  MrsActionlibInterface();

private:
  ros::NodeHandle nh_;

  std::string       _uav_name_;
  std::atomic<bool> is_initialized_ = false;

  // | --------------------- actionlib stuff -------------------- |
  //

  typedef actionlib::SimpleActionServer<mrs_actionlib_interface::gotoAction> GotoServer;
  void                                                                       actionCallbackGoto();
  void                                                                       actionCallbackPreemptGoto();
  std::unique_ptr<GotoServer>                                                goto_server_ptr_;
  mrs_actionlib_interface::gotoGoal                                          action_server_goto_goal_;
  mrs_actionlib_interface::gotoFeedback                                      action_server_goto_feedback_;
  mrs_actionlib_interface::gotoResult                                        action_server_goto_result_;

  // | --------------------- timer callbacks -------------------- |

  void       callbackMainTimer(const ros::TimerEvent& te);
  ros::Timer main_timer_;

  // | ---------------- service server callbacks ---------------- |

  /* bool               callbackStartWaypointFollowing(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res); */
  /* ros::ServiceServer srv_server_start_waypoints_following_; */

  // | --------------------- service clients -------------------- |

  ros::ServiceClient srv_client_land_;
  bool               _land_end_;
};

//}

/* onInit() //{ */

MrsActionlibInterface::MrsActionlibInterface() {

  /* obtain node handle */
  nh_ = ros::NodeHandle("~");

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  ros::Subscriber scan_subscriber_;

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

  main_timer_ = nh_.createTimer(ros::Rate(10), &MrsActionlibInterface::callbackMainTimer, this);


  // | --------------- initialize service servers --------------- |

  /* srv_server_start_waypoints_following_ = nh.advertiseService("start_waypoints_following_in", &MrsActionlibInterface::callbackStartWaypointFollowing, this);
   */

  // | --------------- initialize service clients --------------- |

  goto_server_ptr_ = std::make_unique<GotoServer>(nh_, ros::this_node::getName(), false);
  goto_server_ptr_->registerGoalCallback(boost::bind(&MrsActionlibInterface::actionCallbackGoto, this));
  goto_server_ptr_->registerPreemptCallback(boost::bind(&MrsActionlibInterface::actionCallbackPreemptGoto, this));
  goto_server_ptr_->start();
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


/*  actionCallbackGoto()//{ */

void MrsActionlibInterface::actionCallbackGoto() {

  boost::shared_ptr<const mrs_actionlib_interface::gotoGoal> temp = goto_server_ptr_->acceptNewGoal();

  action_server_goto_goal_ = *temp;

  ROS_INFO("[MrsActionlibInterface]: got a new goal from the action server");

  /* as->setSucceeded(); */
}

//}

/*  actionCallbackGoto()//{ */

void MrsActionlibInterface::actionCallbackPreemptGoto() {

  if (goto_server_ptr_->isActive()) {

    ROS_INFO_ONCE("[MrsActionlibInterface]: preempted");

    // abort the mission
    goto_server_ptr_->setPreempted();
  }
}

//}
// | --------------------- timer callbacks -------------------- |

/* callbackMainTimer() //{ */

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

int main(int argc, char** argv) {
  ros::init(argc, argv, "mrs_actionlib_interface");
  MrsActionlibInterface interface;
  while (ros::ok()) {
    ros::spin();
    return 0;
  }
}
