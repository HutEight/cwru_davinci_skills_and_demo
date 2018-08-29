//
// Created by William on 27/08/18.
//

#include "cwru_davinci_skills_and_demo/DavinciSkillsCommunicationsManager.h"


DavinciSkillsCommunicationsManager::DavinciSkillsCommunicationsManager(ros::NodeHandle nodeHandle, string name, bool debug) :
    nh_(nodeHandle),
    action_server_(nh_, name,  boost::bind(&DavinciSkillsCommunicationsManager::actionServerExecuteCallback, this, _1), false),
    spinner_(5),
    debug_mode_toggle_(debug) {

  ROS_INFO("Initialising da Vinci Skills Communications Manager MK I.");

  initialiseTransforms();

  initialiseCommunications();

  spinner_.start();


  startActionServer();




  ros::waitForShutdown();




}


bool DavinciSkillsCommunicationsManager::initialiseTransforms() {

  tf::TransformListener tfListener;
  tf::StampedTransform tfResult_one, tfResult_two, tfResult_three;
  DavinciSkillsAuxiliary davinci_skills_auxiliary;
  bool tf_acquired = false;
  int n_tries = 0;

  ROS_INFO("Attempting to get Camera PSMs transforms");

  while (!tf_acquired){
    if (n_tries > 5) break;
    tf_acquired = true;
    try {
      tfListener.lookupTransform("left_camera_link",
                                 "PSM1psm_base_link",
                                 ros::Time(0),
                                 tfResult_one);

      tfListener.lookupTransform("left_camera_link",
                                 "PSM2psm_base_link",
                                 ros::Time(0),
                                 tfResult_two);

      tfListener.lookupTransform("PSM1psm_base_link",
                                 "PSM2psm_base_link",
                                 ros::Time(0),
                                 tfResult_three);

    } catch (tf::TransformException &exception) {
      ROS_WARN("%s", exception.what());
      tf_acquired = false;
      ros::Duration(0.5).sleep();
      ros::spinOnce();
      n_tries++;
    }
  }

  if (!tf_acquired) {

    ROS_ERROR("Could NOT acquire Camera PSMs transforms transform.");
    return false;

  } else {

    ROS_INFO("Transform Info acquired.");

    boost::lock_guard<boost::mutex> guard(mtx_);

    basic_env_transforms_.psm_one_affine_wrt_lt_camera = davinci_skills_auxiliary.convertTFToEigen(tfResult_one);
    basic_env_transforms_.psm_two_affine_wrt_lt_camera = davinci_skills_auxiliary.convertTFToEigen(tfResult_two);
    basic_env_transforms_.psm_two_affine_wrt_psm_one = davinci_skills_auxiliary.convertTFToEigen(tfResult_three);
    basic_env_transforms_.psm_one_affine_wrt_psm_two = basic_env_transforms_.psm_two_affine_wrt_psm_one.inverse();

    if (debug_mode_toggle_) {
      ROS_INFO("Basic Transforms:");
      basic_env_transforms_.printAllTransforms();
    }

    return true;

  }


}


void DavinciSkillsCommunicationsManager::initialiseCommunications() {

}


void DavinciSkillsCommunicationsManager::manageInfromation() {

}


void DavinciSkillsCommunicationsManager::startActionServer() {
  ROS_INFO("Commnucations Action Server Stand By.");
  action_server_.start();
  ROS_INFO("Commnucations Action Server Ready.");
}


void DavinciSkillsCommunicationsManager::runRobotController() {

}


void DavinciSkillsCommunicationsManager::runActiveTasks() {


}



/*
 * It will first switch cases to what types of tasks requested by the Client.
 */
void DavinciSkillsCommunicationsManager::actionServerExecuteCallback(const cwru_davinci_skills_and_demo::CwruDavinciSkillControlGoalConstPtr &goal) {

  ROS_INFO("New Goal Received!");

  int i(0);
  for (i = 0; i < 10; i++) {

    // check that preempt has not been requested by the client
//    if (action_server_.isPreemptRequested() || !ros::ok())
//    {
//      ROS_WARN("Preempted");
//      // set the action state to preempted
//      action_server_.setPreempted();
//      action_result_.success = false;
//      break;
//    }

    std::cout << "CHECKPOINT#" << i << std::endl;
    ros::Duration(1).sleep();

  }

  if (i==10) {
    action_result_.success = true;
    action_server_.setSucceeded(action_result_);
  } else {
    action_result_.success = false;
  }




}

