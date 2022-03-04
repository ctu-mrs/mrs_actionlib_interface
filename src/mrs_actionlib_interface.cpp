/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>

#include <mrs_actionlib_interface/commandAction.h>
#include <actionlib/server/simple_action_server.h>

#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/ValidateReference.h>
#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/PathfinderDiagnostics.h>
#include <mrs_msgs/UavState.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <math.h>

/* class MrsActionlibInterface //{ */

class MrsActionlibInterface {

public:
  MrsActionlibInterface();

private:
  ros::NodeHandle nh_;

  std::string _uav_name_;
  std::string _run_type_;
  bool        _simulation_ = false;

  std::atomic<bool> is_initialized_ = false;

  bool have_all_data_ = false;

  bool have_new_goal_ = false;

  bool takeoff_in_progress_   = false;
  bool landing_in_progress_   = false;
  bool goto_in_progress_      = false;
  bool fresh_pathfinder_diag_ = false;

  bool      armed_ = false;
  ros::Time armed_time_;
  double    armed_timeout_secs_ = 3.0;

  bool      offboard_ = false;
  ros::Time offboard_time_;
  bool      set_offboard_called_ = false;

  double pos_error_threshold_ = 0.1;

  ros::Time motors_on_time_;

  bool motors_ = false;

  ros::Time arming_time_;

  typedef enum
  {
    LANDED,
    TAKEOFF_LANDING,
    IDLE_FLYING,
    GOTO,
    UNKNOWN
  } uav_state;

  typedef enum
  {
    TAKEOFF_INITIAL,
    ARMED,
    ARMED_MOTORS,
    ARMED_MOTORS_OFFBOARD,
    TAKING_OFF,
    TAKEOFF_FINISHED
  } takeoff_state;

  typedef enum
  {
    LANDING_INITIAL,
    LANDING,
    LANDING_FINISHED
  } landing_state;

  std::string last_tracker_ = "NullTracker";

  const std::vector<std::string> uav_state_str{"LANDED", "TAKEOFF_LANDING", "IDLE_FLYING", "GOTO", "UNKNOWN"};
  const std::vector<std::string> takeoff_state_str{"TAKEOFF_INITIAL", "ARMED", "ARMED_MOTORS", "ARMED_MOTORS_OFFBOARD", "TAKING_OFF", "TAKEOFF_FINISHED"};
  const std::vector<std::string> landing_state_str{"LANDING_INITIAL", "LANDING", "LANDING_FINISHED"};

  uav_state     state_         = UNKNOWN;
  takeoff_state takeoff_state_ = TAKEOFF_INITIAL;
  landing_state landing_state_ = LANDING_INITIAL;

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
  ros::Subscriber pathfinder_diagnostics_subscriber_;
  ros::Subscriber uav_state_subscriber_;

  void                                controlManagerDiagCallback(const mrs_msgs::ControlManagerDiagnosticsConstPtr& msg);
  mrs_msgs::ControlManagerDiagnostics control_manager_diag_;
  bool                                got_control_manager_diag_ = false;

  void               mavrosStateCallback(const mavros_msgs::StateConstPtr& msg);
  mavros_msgs::State mavros_state_;
  bool               got_mavros_state_ = false;

  void               uavStateCallback(const mrs_msgs::UavStateConstPtr& msg);
  mrs_msgs::UavState uav_state_;
  bool               got_uav_state_ = false;


  void                            pathfinderDiagnosticsCallback(const mrs_msgs::PathfinderDiagnosticsConstPtr& msg);
  mrs_msgs::PathfinderDiagnostics pathfinder_diagnostics_;
  bool                            got_pathfinder_diagnostics_ = false;

  // | --------------------- timer callbacks -------------------- |

  void callbackMainTimer(const ros::TimerEvent& te);
  void callbackTakeoffTimer(const ros::TimerEvent& te);
  void callbackLandingTimer(const ros::TimerEvent& te);
  void callbackGotoTimer(const ros::TimerEvent& te);
  void callbackFeedbackTimer(const ros::TimerEvent& te);

  ros::Timer main_timer_;
  ros::Timer takeoff_timer_;
  ros::Timer landing_timer_;
  ros::Timer goto_timer_;

  // | --------------------- service clients -------------------- |

  ros::ServiceClient srv_client_mavros_arm_;
  ros::ServiceClient srv_client_mavros_set_mode_;
  ros::ServiceClient srv_client_validate_reference_;
  ros::ServiceClient srv_client_motors_;
  ros::ServiceClient srv_client_takeoff_;
  ros::ServiceClient srv_client_land_;
  ros::ServiceClient srv_client_pathfinder_goto_;

  // | --------------------- misc routines -------------------- |

  void changeMainState(uav_state new_state);
  void changeTakeoffState(takeoff_state new_state);
  void changeLandingState(landing_state new_state);

  bool validateReference();
  bool setMotors(const bool value);

  bool disarm();
  bool takeoff();

  void info_goto(std::string info);
  void info_goto(double rate, std::string info);
  void abort_goto(std::string info);

  void info_takeoff(std::string);
  void info_takeoff(double rate, std::string);
  void abort_takeoff(std::string);

  void info_landing(std::string);
  void info_landing(double rate, std::string);
  void abort_landing(std::string);
};

//}

/* MrsActionlibInterface() //{ */

MrsActionlibInterface::MrsActionlibInterface() {

  nh_ = ros::NodeHandle("~");

  ros::Time::waitForValid();

  nh_.getParam("uav_name", _uav_name_);
  nh_.getParam("run_type", _run_type_);

  if (_run_type_ == "simulation") {
    _simulation_ = true;
  }

  // | --------------------- subscribers -------------------- |

  control_manager_diag_subscriber_ =
      nh_.subscribe("control_manager_diag_in", 1, &MrsActionlibInterface::controlManagerDiagCallback, this, ros::TransportHints().tcpNoDelay());

  mavros_state_subscriber_ = nh_.subscribe("mavros_state_in", 1, &MrsActionlibInterface::mavrosStateCallback, this, ros::TransportHints().tcpNoDelay());
  pathfinder_diagnostics_subscriber_ =
      nh_.subscribe("pathfinder_diagnostics_in", 1, &MrsActionlibInterface::pathfinderDiagnosticsCallback, this, ros::TransportHints().tcpNoDelay());
  uav_state_subscriber_ = nh_.subscribe("uav_state_in", 1, &MrsActionlibInterface::uavStateCallback, this, ros::TransportHints().tcpNoDelay());

  // | --------------------- timer callbacks -------------------- |

  main_timer_    = nh_.createTimer(ros::Rate(1), &MrsActionlibInterface::callbackMainTimer, this);
  takeoff_timer_ = nh_.createTimer(ros::Rate(10), &MrsActionlibInterface::callbackTakeoffTimer, this);
  landing_timer_ = nh_.createTimer(ros::Rate(10), &MrsActionlibInterface::callbackLandingTimer, this);
  goto_timer_    = nh_.createTimer(ros::Rate(10), &MrsActionlibInterface::callbackGotoTimer, this);

  srv_client_mavros_arm_         = nh_.serviceClient<mavros_msgs::CommandBool>("mavros_arm_out");
  srv_client_mavros_set_mode_    = nh_.serviceClient<mavros_msgs::SetMode>("mavros_set_mode_out");
  srv_client_validate_reference_ = nh_.serviceClient<mrs_msgs::ValidateReference>("validate_reference_out");
  srv_client_motors_             = nh_.serviceClient<std_srvs::SetBool>("motors_out");
  srv_client_takeoff_            = nh_.serviceClient<std_srvs::Trigger>("takeoff_out");
  srv_client_land_               = nh_.serviceClient<std_srvs::Trigger>("land_out");
  srv_client_pathfinder_goto_    = nh_.serviceClient<mrs_msgs::Vec4>("pathfinder_goto_out");

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

  if (landing_in_progress_) {
    action_server_result_.success = false;
    action_server_result_.message = "Landing is in progress, cannot accept a new goal!";
    ROS_ERROR("[MrsActionlibInterface]: Landing is in progress, cannot accept a new goal!");
    command_server_ptr_->setAborted(action_server_result_);
    return;
  }

  if (takeoff_in_progress_) {
    action_server_result_.success = false;
    action_server_result_.message = "Takeoff is in progress, cannot accept a new goal!";
    ROS_ERROR("[MrsActionlibInterface]: Takeoff is in progress, cannot accept a new goal!");
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

  if (!got_uav_state_) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[MrsActionlibInterface]: main timer: waiting for uav_state!");
    return;
  }

  if (!got_pathfinder_diagnostics_) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[MrsActionlibInterface]: main timer: waiting for pathfinder_diagnostics!");
    return;
  }


  /* first run after we have all the data //{ */

  if (!have_all_data_) {
    ROS_INFO("[MrsActionlibInterface]: main timer: have all data, activating!");
    have_all_data_ = true;

    // determine the initial state of the UAV once we have all the necesary data

    if (control_manager_diag_.active_tracker == "NullTracker") {
      changeMainState(LANDED);
    }

    if (control_manager_diag_.flying_normally == true) {
      changeMainState(IDLE_FLYING);
    }

    if (control_manager_diag_.active_tracker == "LandoffTracker") {
      changeMainState(TAKEOFF_LANDING);
    }
  }

  //}

  /* deterministic state changes //{ */

  if (control_manager_diag_.active_tracker == "LandoffTracker" && state_ != TAKEOFF_LANDING) {
    changeMainState(TAKEOFF_LANDING);
  }

  if (control_manager_diag_.flying_normally == true && state_ == TAKEOFF_LANDING) {

    /* if (landoff_in_progress_) { */
    /*   landoff_in_progress_          = false; */
    /*   action_server_result_.success = true; */
    /*   action_server_result_.message = "Takeoff completed"; */
    /*   ROS_INFO("[MrsActionlibInterface]: Takeoff completed"); */
    /*   command_server_ptr_->setSucceeded(action_server_result_); */
    /* } */

    changeMainState(IDLE_FLYING);
  }

  if (control_manager_diag_.active_tracker == "NullTracker" && state_ == TAKEOFF_LANDING) {

    /* if (landoff_in_progress_) { */
    /*   landoff_in_progress_          = false; */
    /*   action_server_result_.success = true; */
    /*   action_server_result_.message = "Landing completed"; */
    /*   ROS_INFO("[MrsActionlibInterface]: Landing completed"); */
    /*   command_server_ptr_->setSucceeded(action_server_result_); */
    /* } */

    changeMainState(LANDED);
  }

  //}

  /* new goal //{ */

  if (have_new_goal_) {

    have_new_goal_ = false;

    switch (action_server_goal_.command) {

        /* case GOTO //{ */

      case _ACTION_SERVER_GOAL_.COMMAND_GOTO: {

        mrs_msgs::Vec4 srv;
        srv.request.goal[0] = action_server_goal_.goto_x;
        srv.request.goal[1] = action_server_goal_.goto_y;
        srv.request.goal[2] = action_server_goal_.goto_z;
        srv.request.goal[3] = action_server_goal_.goto_hdg;

        bool res = srv_client_pathfinder_goto_.call(srv);

        if (res) {

          if (srv.response.success) {

            ROS_INFO("[MrsActionlibInterface]: Service call for goto successful");
            goto_in_progress_      = true;
            fresh_pathfinder_diag_ = false;  // we need a fresh pathfinder diagnostics message, and evaluate it in the goto timer

          } else {

            ROS_ERROR("[MrsActionlibInterface]: Service call for goto failed");

            break;
          }

        } else {

          ROS_ERROR("[MrsActionlibInterface]: Service call for goto failed");
          break;
        }
        break;
      }

        //}

        /* case TAKEOFF //{ */

      case _ACTION_SERVER_GOAL_.COMMAND_TAKEOFF: {

        if (state_ != LANDED) {

          ROS_ERROR("[MrsActionlibInterface]: The UAV is not landed, it cannot takeoff!");
          action_server_result_.success = false;
          action_server_result_.message = "UAV is not landed! Takeoff failed";
          ROS_ERROR("[MrsActionlibInterface]: Takeoff failed");
          command_server_ptr_->setAborted(action_server_result_);

        } else {

          takeoff_in_progress_ = true;
        }
        break;
      }

        //}

        /* case LAND //{ */

      case _ACTION_SERVER_GOAL_.COMMAND_LAND: {

        if (state_ != IDLE_FLYING) {

          ROS_ERROR("[MrsActionlibInterface]: The UAV is not flying, it cannot land!");
          action_server_result_.success = false;
          action_server_result_.message = "UAV is not flying, it cannot land! Landing failed";
          ROS_ERROR("[MrsActionlibInterface]: Landing failed");
          command_server_ptr_->setAborted(action_server_result_);

        } else {

          landing_in_progress_ = true;
        }
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

/* callbackTakeoffTimer() //{ */

void MrsActionlibInterface::callbackTakeoffTimer([[maybe_unused]] const ros::TimerEvent& te) {

  if (!takeoff_in_progress_) {
    if (takeoff_state_ != TAKEOFF_INITIAL) {
      changeTakeoffState(TAKEOFF_INITIAL);
    }
    return;
  }

  switch (takeoff_state_) {

    case TAKEOFF_INITIAL: {

      // sanity check:
      if (control_manager_diag_.active_tracker != "NullTracker") {
        abort_takeoff("Null tracker is not active, cannot takeoff!");
        takeoff_in_progress_ = false;
        break;
      }

      if (!validateReference()) {
        abort_takeoff("Reference is not valid! (safety area/bumper)");
        takeoff_in_progress_ = false;
        break;
      }

      if (_simulation_) {

        mavros_msgs::CommandBool command_bool_srv;
        command_bool_srv.request.value = true;

        bool res = srv_client_mavros_arm_.call(command_bool_srv);

        if (!res) {
          disarm();
          abort_takeoff("Service call for arming failed!");
          break;
        }
      }

      if (armed_) {
        info_takeoff(1.0, "Waiting for arming");
        set_offboard_called_ = false;
        changeTakeoffState(ARMED);
      }


      break;
    }

    case ARMED: {

      bool res = setMotors(true);

      if (!res) {

        disarm();
        abort_takeoff("[MrsActionlibInterface]: could not set motors ON");
        break;

      } else {
        changeTakeoffState(ARMED_MOTORS);
      }


      break;
    }

    case ARMED_MOTORS: {

      info_takeoff(1.0, "Waiting for offboard");

      if (offboard_) {

        info_takeoff("Offboard set!");
        changeTakeoffState(ARMED_MOTORS_OFFBOARD);
        break;
      }

      if (_simulation_ && !set_offboard_called_) {

        set_offboard_called_ = true;
        mavros_msgs::SetMode set_mode_srv;
        set_mode_srv.request.base_mode   = 0;
        set_mode_srv.request.custom_mode = "offboard";

        info_takeoff("Setting offboard");
        bool res = srv_client_mavros_set_mode_.call(set_mode_srv);

        if (!res) {
          disarm();
          abort_takeoff("Service call for offboard failed!");
          break;
        }
      }

      if (armed_timeout_secs_ > (ros::Time::now() - armed_time_).toSec()) {
        disarm();
        abort_takeoff("Armed for too long without getting into offboard, disarming!");
        break;
      }

      break;
    }

    case ARMED_MOTORS_OFFBOARD: {

      std_srvs::Trigger srv;

      bool res = srv_client_takeoff_.call(srv);

      if (res) {

        if (srv.response.success) {

          changeTakeoffState(TAKING_OFF);
          break;

        } else {

          abort_takeoff("Taking off failed:" + std::string(srv.response.message));
          break;
        }

      } else {

        abort_takeoff(": service call for taking off failed");
        break;
      }

      break;
    }
    case TAKING_OFF: {
      info_takeoff(1.0, "taking off");

      if (control_manager_diag_.active_tracker != "LandoffTracker" && last_tracker_ == "LandoffTracker") {
        changeTakeoffState(TAKEOFF_FINISHED);
      }

      last_tracker_ = control_manager_diag_.active_tracker;
      break;
    }
    case TAKEOFF_FINISHED: {

      action_server_result_.success = true;
      action_server_result_.message = "Takeoff completed";
      info_takeoff("Takeoff completed");
      command_server_ptr_->setSucceeded(action_server_result_);

      takeoff_in_progress_ = false;
      changeMainState(IDLE_FLYING);
      changeTakeoffState(TAKEOFF_INITIAL);
      break;
    }
  }
}

//}

/* callbackLandingTimer() //{ */

void MrsActionlibInterface::callbackLandingTimer([[maybe_unused]] const ros::TimerEvent& te) {

  if (!landing_in_progress_) {
    if (landing_state_ != LANDING_INITIAL) {
      changeLandingState(LANDING_INITIAL);
    }
    return;
  }

  switch (landing_state_) {

    case LANDING_INITIAL: {

      // sanity check:
      if (control_manager_diag_.flying_normally != true) {
        abort_landing("Cannot land, not flying normally!");
        break;
      }

      info_landing("Call for landing");

      std_srvs::Trigger srv;

      bool res = srv_client_land_.call(srv);

      if (res) {

        if (srv.response.success) {

          info_landing("Service call for landing successful");
          changeLandingState(LANDING);

        } else {

          abort_landing("landing failed: " + std::string(srv.response.message));
          break;
        }

      } else {

        abort_landing("service call for landing failed");
        break;
      }
    }

    case LANDING: {

      info_landing(1.0, "landing");

      if (control_manager_diag_.active_tracker != "LandoffTracker" && last_tracker_ == "LandoffTracker") {
        changeLandingState(LANDING_FINISHED);
      }

      last_tracker_ = control_manager_diag_.active_tracker;
      break;
    }

    case LANDING_FINISHED: {

      action_server_result_.success = true;
      action_server_result_.message = "Landing completed";
      info_landing("Landing completed");
      command_server_ptr_->setSucceeded(action_server_result_);

      landing_in_progress_ = false;
      changeMainState(LANDED);
      changeLandingState(LANDING_INITIAL);
      break;

    }


    break;
  }
}

//}

/* callbackGotoTimer() //{ */

void MrsActionlibInterface::callbackGotoTimer([[maybe_unused]] const ros::TimerEvent& te) {

  if (!goto_in_progress_) {
    return;
  }

  if (goto_in_progress_ && !fresh_pathfinder_diag_) {
    info_goto(1.0, "waiting for fresh pathfinder diagnostics ");
    return;
  }

  double best_goal_err_ = sqrt(pow(pathfinder_diagnostics_.best_goal.x - pathfinder_diagnostics_.desired_reference.x, 2) +
                               pow(pathfinder_diagnostics_.best_goal.y - pathfinder_diagnostics_.desired_reference.y, 2) +
                               pow(pathfinder_diagnostics_.best_goal.z - pathfinder_diagnostics_.desired_reference.z, 2));

  double pos_error = sqrt(pow(uav_state_.pose.position.x - pathfinder_diagnostics_.desired_reference.x, 2) +
                          pow(uav_state_.pose.position.y - pathfinder_diagnostics_.desired_reference.y, 2) +
                          pow(uav_state_.pose.position.z - pathfinder_diagnostics_.desired_reference.z, 2));


  std::stringstream strs1;
  strs1 << std::fixed << std::setprecision(2) << best_goal_err_;
  std::string best_goal_err_str = strs1.str();

  std::stringstream strs2;
  strs2 << std::fixed << std::setprecision(2) << pos_error;
  std::string pos_error_str = strs2.str();

  if (pathfinder_diagnostics_.idle == true && !control_manager_diag_.tracker_status.have_goal) {
    goto_in_progress_ = false;

    if (pos_error < pos_error_threshold_) {
      action_server_result_.success = true;
      action_server_result_.message = "Goto goal reached, position error: " + best_goal_err_str;
      info_goto("Goto goal reached, position error: " + best_goal_err_str);
      command_server_ptr_->setSucceeded(action_server_result_);
    } else {
      action_server_result_.success = false;
      action_server_result_.message = "Goal not reachable, final error in position: " + pos_error_str;
      info_goto("Goal not reachable, final error in position: " + pos_error_str);
      command_server_ptr_->setAborted(action_server_result_);
    }


  } else {
    info_goto(1.0, "Goto is in progress, distance to reference: " + pos_error_str);
  }
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

  if (!armed_ && msg->armed) {
    ros::Time armed_time_ = ros::Time::now();
  }

  if (!offboard_ && msg->mode == "OFFBOARD") {
    ros::Time offboard_time_ = ros::Time::now();
  }

  offboard_ = msg->mode == "OFFBOARD" ? true : false;
  armed_    = msg->armed;
}

//}

/*  uavStateCallback()//{ */

void MrsActionlibInterface::uavStateCallback(const mrs_msgs::UavStateConstPtr& msg) {

  got_uav_state_ = true;
  uav_state_     = *msg;
}

//}

/*  pathfinderDiagnosticsCallback()//{ */

void MrsActionlibInterface::pathfinderDiagnosticsCallback(const mrs_msgs::PathfinderDiagnosticsConstPtr& msg) {

  got_pathfinder_diagnostics_ = true;
  pathfinder_diagnostics_     = *msg;
  fresh_pathfinder_diag_      = true;
}

//}

// | -------------------- misc routines ------------------- |

/* changeMainState() //{ */

void MrsActionlibInterface::changeMainState(uav_state new_state) {
  ROS_INFO_STREAM("[MrsActionlibInterface]: Changing state [" << uav_state_str[state_] << "] to [" << uav_state_str[new_state] << "]");
  state_ = new_state;
}

//}

/* changeTakeoffState() //{ */

void MrsActionlibInterface::changeTakeoffState(takeoff_state new_state) {
  std::stringstream tmp;
  tmp << "Changing takeoff state [" << takeoff_state_str[takeoff_state_] << "] to [" << takeoff_state_str[new_state] << "]";
  info_takeoff(tmp.str());
  takeoff_state_ = new_state;
}

//}

/* changeLandingState() //{ */

void MrsActionlibInterface::changeLandingState(landing_state new_state) {
  ROS_INFO_STREAM("[MrsActionlibInterface]: Changing landing state [" << landing_state_str[landing_state_] << "] to [" << landing_state_str[new_state] << "]");
  landing_state_ = new_state;
}

//}

/* validateReference() //{ */

bool MrsActionlibInterface::validateReference() {

  mrs_msgs::ValidateReference srv_out;

  srv_out.request.reference.header.frame_id = "fcu";

  bool res = srv_client_validate_reference_.call(srv_out);

  if (res) {

    if (srv_out.response.success) {

      ROS_INFO_THROTTLE(1.0, "[MrsActionlibInterface]: current position is valid");
      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[MrsActionlibInterface]: current position is not valid (safety area, bumper)!");
      return false;
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[MrsActionlibInterface]: current position could not be validated");
    return false;
  }
}

//}

/* disarm() //{ */

bool MrsActionlibInterface::disarm() {

  if (!got_mavros_state_) {

    ROS_WARN_THROTTLE(1.0, "[MrsActionlibInterface]: cannot disarm, missing mavros state!");

    return false;
  }

  if (offboard_) {

    ROS_WARN_THROTTLE(1.0, "[MrsActionlibInterface]: cannot disarm, UAV is in offboard mode!");

    return false;
  }

  ROS_INFO_THROTTLE(1.0, "[MrsActionlibInterface]: disarming");

  mavros_msgs::CommandBool srv;
  srv.request.value = 0;

  bool res = srv_client_mavros_arm_.call(srv);

  if (res) {

    if (srv.response.success) {

      ROS_WARN_THROTTLE(1.0, "[MrsActionlibInterface]: UAV disarmed!");

      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[MrsActionlibInterface]: disarming failed");
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[MrsActionlibInterface]: service call for disarming failed");
  }

  return false;
}

//}

/* info_goto() //{ */

void MrsActionlibInterface::info_goto(std::string info) {
  ROS_INFO_STREAM("[MrsActionlibInterface]-------------->[GotoStateMachine]: " << info);
}

void MrsActionlibInterface::info_goto(double rate, std::string info) {
  ROS_INFO_STREAM_THROTTLE(rate, "[MrsActionlibInterface]-------------->[GotoStateMachine]: " << info);
}

//}

/* info_takeoff() //{ */

void MrsActionlibInterface::info_takeoff(std::string info) {
  ROS_INFO_STREAM("[MrsActionlibInterface]-------------->[TakeoffStateMachine]: " << info);
}

void MrsActionlibInterface::info_takeoff(double rate, std::string info) {
  ROS_INFO_STREAM_THROTTLE(rate, "[MrsActionlibInterface]-------------->[TakeoffStateMachine]: " << info);
}

//}

/* info_landing() //{ */

void MrsActionlibInterface::info_landing(std::string info) {
  ROS_INFO_STREAM("[MrsActionlibInterface]-------------->[LandingStateMachine]: " << info);
}

void MrsActionlibInterface::info_landing(double rate, std::string info) {
  ROS_INFO_STREAM_THROTTLE(rate, "[MrsActionlibInterface]-------------->[LandingStateMachine]: " << info);
}

//}

/* abort_goto() //{ */

void MrsActionlibInterface::abort_goto(std::string info) {

  goto_in_progress_ = false;
  ROS_INFO_STREAM("[MrsActionlibInterface]-------------->[GotoStateMachine]: " << info);

  action_server_result_.success = false;
  action_server_result_.message = info;
  command_server_ptr_->setAborted(action_server_result_);
}

//}

/* abort_takeoff() //{ */

void MrsActionlibInterface::abort_takeoff(std::string info) {

  takeoff_in_progress_ = false;
  ROS_INFO_STREAM("[MrsActionlibInterface]-------------->[TakeoffStateMachine]: " << info);
  changeTakeoffState(TAKEOFF_INITIAL);
  setMotors(false);

  action_server_result_.success = false;
  action_server_result_.message = info;
  command_server_ptr_->setAborted(action_server_result_);
}

//}

/* abort_landing() //{ */

void MrsActionlibInterface::abort_landing(std::string info) {

  landing_in_progress_ = false;
  ROS_INFO_STREAM("[MrsActionlibInterface]-------------->[LandingStateMachine]: " << info);
  changeLandingState(LANDING_INITIAL);

  action_server_result_.success = false;
  action_server_result_.message = info;
  command_server_ptr_->setAborted(action_server_result_);
}

//}

/* setMotors() //{ */

bool MrsActionlibInterface::setMotors(const bool value) {

  ROS_INFO_THROTTLE(1.0, "[MrsActionlibInterface]: setting motors %s", value ? "ON" : "OFF");

  std_srvs::SetBool srv;
  srv.request.data = value;

  bool res = srv_client_motors_.call(srv);

  if (res) {

    if (srv.response.success) {

      ROS_INFO_THROTTLE(1.0, "[MrsActionlibInterface]: motors succesfully set to %s", value ? "ON" : "OFF");
      return true;

    } else {

      ROS_ERROR_THROTTLE(1.0, "[MrsActionlibInterface]: setting motors failed: %s", srv.response.message.c_str());
    }

  } else {

    ROS_ERROR_THROTTLE(1.0, "[MrsActionlibInterface]: service call for setting motors failed");
  }

  return false;
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

