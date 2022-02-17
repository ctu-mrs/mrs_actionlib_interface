/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>

#include <mrs_actionlib_interface/commandAction.h>
#include <actionlib/server/simple_action_server.h>

#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

/* class MrsActionlibInterface //{ */

class MrsActionlibInterface {

public:
  MrsActionlibInterface();

private:
  ros::NodeHandle nh_;

  std::string       _uav_name_;
  std::atomic<bool> is_initialized_ = false;

  bool have_all_data_ = false;

  bool have_new_goal_ = false;

  bool landoff_in_progress_ = false;
  bool goto_in_progress_    = false;

  ros::Time arming_time_;

  typedef enum
  {
    LANDED,
    TAKEOFF_LANDING,
    IDLE_FLYING,
    GOTO,
    UNKNOWN
  } uav_state;

  const std::vector<std::string> uav_state_str{"LANDED", "TAKEOFF_LANDING", "IDLE_FLYING", "GOTO", "UNKNOWN"};

  uav_state state_ = UNKNOWN;

  // | --------------------- actionlib stuff -------------------- |
  //

  typedef actionlib::SimpleActionServer<mrs_actionlib_interface::commandAction> CommandServer;
  void                                                                          actionCallbackGoal();
  void                                                                          actionCallbackPreempt();
  std::unique_ptr<CommandServer>                                                command_server_ptr_;

  static const mrs_actionlib_interface::commandGoal _ACTION_SERVER_GOAL_;
  mrs_actionlib_interface::commandGoal              action_server_goal_;
  mrs_actionlib_interface::commandFeedback          action_server_feedback_;
  mrs_actionlib_interface::commandResult            action_server_result_;

  // | --------------------- subscribers -------------------- |

  ros::Subscriber control_manager_diag_subscriber_;
  ros::Subscriber mavros_state_subscriber_;

  void                                controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg);
  mrs_msgs::ControlManagerDiagnostics control_manager_diag_;
  bool                                got_control_manager_diag_ = false;

  void               mavrosStateCallback(const mavros_msgs::StateConstPtr& msg);
  mavros_msgs::State mavros_state_;
  bool               got_mavros_state_ = true;


  // | --------------------- timer callbacks -------------------- |

  void callbackMainTimer(const ros::TimerEvent& te);
  void callbackFeedbackTimer(const ros::TimerEvent& te);

  ros::Timer main_timer_;
  ros::Timer feedback_timer_;

  // | --------------------- service clients -------------------- |

  ros::ServiceClient srv_client_mavros_arm_;
  ros::ServiceClient srv_client_mavros_set_mode_;

  // | --------------------- misc routines -------------------- |

  void changeState(uav_state new_state);
  bool takeoff();
};

//}

/* MrsActionlibInterface() //{ */

MrsActionlibInterface::MrsActionlibInterface() {

  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  // | --------------------- subscribers -------------------- |

  control_manager_diag_subscriber_ =
      nh_.subscribe("control_manager_diag_in", 1, &MrsActionlibInterface::controlManagerDiagCallback, this, ros::TransportHints().tcpNoDelay());

  mavros_state_subscriber_ = nh_.subscribe("mavros_state_in", 1, &MrsActionlibInterface::mavrosStateCallback, this, ros::TransportHints().tcpNoDelay());

  // | --------------------- timer callbacks -------------------- |

  main_timer_ = nh_.createTimer(ros::Rate(1), &MrsActionlibInterface::callbackMainTimer, this);
  /* main_timer_ = nh_.createTimer(ros::Rate(10), &MrsActionlibInterface::callbackFeedbackTimer, this); */

  srv_client_mavros_arm_      = nh_.serviceClient<mavros_msgs::CommandBool>("mavros_arm_out");
  srv_client_mavros_set_mode_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros_set_mode_out");

  command_server_ptr_ = std::make_unique<CommandServer>(nh_, ros::this_node::getName(), false);
  command_server_ptr_->registerGoalCallback(boost::bind(&MrsActionlibInterface::actionCallbackGoal, this));
  command_server_ptr_->registerPreemptCallback(boost::bind(&MrsActionlibInterface::actionCallbackPreempt, this));
  command_server_ptr_->start();
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

// | ---------------------- action server callbacks --------------------- |

/*  actionCallbackGoal()//{ */

void MrsActionlibInterface::actionCallbackGoal() {

  have_new_goal_ = false;

  boost::shared_ptr<const mrs_actionlib_interface::commandGoal> temp = command_server_ptr_->acceptNewGoal();

  action_server_goal_ = *temp;

  ROS_INFO("[MrsActionlibInterface]: got a new goal from the action server");

  if (!is_initialized_) {
    action_server_result_.success = false;
    action_server_result_.message = "Not initialized yet";
    ROS_ERROR("[MrsActionlibInterface]: not initialized yet");
    command_server_ptr_->setAborted(action_server_result_);
    return;
  }

  if (!have_all_data_) {
    action_server_result_.success = false;
    action_server_result_.message = "Still waiting for some data";
    ROS_ERROR("[MrsActionlibInterface]: still waiting for some data");
    command_server_ptr_->setAborted(action_server_result_);
    return;
  }

  if (action_server_goal_.command == _ACTION_SERVER_GOAL_.COMMAND_TAKEOFF || action_server_goal_.command == _ACTION_SERVER_GOAL_.COMMAND_LAND ||
      action_server_goal_.command == _ACTION_SERVER_GOAL_.COMMAND_GOTO) {

    have_new_goal_ = true;

  } else {

    ROS_ERROR("[MrsActionlibInterface]: got an unknown goal");
  }
}

//}

/*  actionCallbackPreempt()//{ */

void MrsActionlibInterface::actionCallbackPreempt() {

  if (command_server_ptr_->isActive()) {

    ROS_INFO_ONCE("[MrsActionlibInterface]: preempted");

    // abort the mission
    command_server_ptr_->setPreempted();
  }
}

//}

// | --------------------- timer callbacks -------------------- |

/* callbackMainTimer() //{ */

void MrsActionlibInterface::callbackMainTimer([[maybe_unused]] const ros::TimerEvent& te) {

  if (!got_control_manager_diag_) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[MrsActionlibInterface]: main timer: waiting for control_manager_diagnostics!");
    return;
  }

  if (!got_mavros_state_) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[MrsActionlibInterface]: main timer: waiting for mavros_state!");
    return;
  }

  /* first run after we have all the data //{ */

  if (!have_all_data_) {
    ROS_INFO("[MrsActionlibInterface]: main timer: have all data, activating!");
    have_all_data_ = true;

    // determine the initial state of the UAV once we have all the necesary data

    if (control_manager_diag_.active_tracker == "NullTracker") {
      changeState(LANDED);
    }

    if (control_manager_diag_.flying_normally == true) {
      changeState(IDLE_FLYING);
    }

    if (control_manager_diag_.active_tracker == "LandoffTracker") {
      changeState(TAKEOFF_LANDING);
    }
  }

  //}

  /* deterministic state changes //{ */

  if (control_manager_diag_.active_tracker == "LandoffTracker" && state_ != TAKEOFF_LANDING) {
    changeState(TAKEOFF_LANDING);
  }

  if (control_manager_diag_.flying_normally == true && state_ == TAKEOFF_LANDING) {

    if (landoff_in_progress_) {
      landoff_in_progress_          = false;
      action_server_result_.success = true;
      action_server_result_.message = "Takeoff completed";
      ROS_INFO("[MrsActionlibInterface]: Takeoff completed");
      command_server_ptr_->setSucceeded(action_server_result_);
    }

    changeState(IDLE_FLYING);
  }

  if (control_manager_diag_.active_tracker == "NullTracker" && state_ == TAKEOFF_LANDING) {

    if (landoff_in_progress_) {
      landoff_in_progress_          = false;
      action_server_result_.success = true;
      action_server_result_.message = "Landing completed";
      ROS_INFO("[MrsActionlibInterface]: Landing completed");
      command_server_ptr_->setSucceeded(action_server_result_);
    }

    changeState(LANDED);
  }

  //}

  /* new goal //{ */

  if (have_new_goal_) {

    have_new_goal_ = false;

    switch (action_server_goal_.command) {

        /* case GOTO //{ */

      case _ACTION_SERVER_GOAL_.COMMAND_GOTO: {

        ROS_INFO("[MrsActionlibInterface]: in switch for GOTO");
        break;
      }

        //}

        /* case TAKEOFF //{ */

      case _ACTION_SERVER_GOAL_.COMMAND_TAKEOFF: {

        ROS_INFO("[MrsActionlibInterface]: in switch for TAKEOFF");
        if (state_ != LANDED) {

          ROS_ERROR("[MrsActionlibInterface]: The drone is not landed, it cannot takeoff!");
          action_server_result_.success = false;
          action_server_result_.message = "Takeoff failed";
          ROS_ERROR("[MrsActionlibInterface]: Takeoff failed");
          command_server_ptr_->setAborted(action_server_result_);

        } else {

          landoff_in_progress_ = takeoff();

          if (!landoff_in_progress_) {
            action_server_result_.success = false;
            action_server_result_.message = "Takeoff failed";
            ROS_ERROR("[MrsActionlibInterface]: Takeoff failed");
            command_server_ptr_->setAborted(action_server_result_);
          }
        }
        break;
      }

        //}

        /* case LAND //{ */

      case _ACTION_SERVER_GOAL_.COMMAND_LAND: {
        ROS_INFO("[MrsActionlibInterface]: in switch for LAND");
        break;
      }

        //}

        /* case default //{ */

      default: {
        ROS_INFO("[MrsActionlibInterface]: in switch for UNKNOWN");
        break;
      }

        //}
    }
  }

  //}
}

//}

/* callbackLandoffTimer() //{ */

bool MrsActionlibInterface::takeoff() {

  mavros_msgs::CommandBool command_bool_srv;
  command_bool_srv.request.value = true;

  ROS_WARN("[MrsActionlibInterface]: Calling for UAV arming!");

  bool res = srv_client_mavros_arm_.call(command_bool_srv);

  if (!res) {
    ROS_ERROR("[MrsActionlibInterface]: Service call for arming failed!");
    ROS_ERROR_STREAM("[MrsActionlibInterface]: response: " << command_bool_srv.response.result);
    return false;
  }

  ROS_WARN("[MrsActionlibInterface]: UAV armed!");
  ros::Duration(1.0).sleep();  // Sleep for one second

  ROS_WARN("[MrsActionlibInterface]: Calling for Mavros offboard!");

  mavros_msgs::SetMode set_mode_srv;
  set_mode_srv.request.base_mode   = 0;
  set_mode_srv.request.custom_mode = "offboard";

  res = srv_client_mavros_set_mode_.call(set_mode_srv);

  if (!res) {
    ROS_ERROR("[MrsActionlibInterface]: Service call for offboard failed!");
    return false;
  }

  ROS_WARN("[MrsActionlibInterface]: Offboard set!");
  return true;
}

//}

/* callbackFeedbackTimer() //{ */

void MrsActionlibInterface::callbackFeedbackTimer([[maybe_unused]] const ros::TimerEvent& te) {
}

//}

// | -------------------- topic callbacks ------------------- |

/*  controlManagerDiagCallback()//{ */

void MrsActionlibInterface::controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg) {
  got_control_manager_diag_ = true;
  control_manager_diag_     = *msg;
}

//}

/*  mavrosStateCallback()//{ */

void MrsActionlibInterface::mavrosStateCallback(const mavros_msgs::StateConstPtr& msg) {
  got_mavros_state_ = true;
  mavros_state_     = *msg;
}

//}
// | -------------------- misc routines ------------------- |

/* changeState() //{ */

void MrsActionlibInterface::changeState(uav_state new_state) {
  ROS_INFO_STREAM("[MrsActionlibInterface]: Changing state [" << uav_state_str[state_] << "] --) [" << uav_state_str[new_state] << "]");
  state_ = new_state;
}

//}

/* main //{ */

int main(int argc, char** argv) {
  ros::init(argc, argv, "mrs_actionlib_interface");
  MrsActionlibInterface interface;
  while (ros::ok()) {
    ros::spin();
    return 0;
  }
}

//}
