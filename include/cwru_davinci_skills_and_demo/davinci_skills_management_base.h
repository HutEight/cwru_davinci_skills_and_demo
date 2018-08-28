//
// Created by William on 27/08/18.
//

#ifndef CWRU_DAVINCI_SKILLS_AND_DEMO_DAVINCI_SKILLS_MANAGEMENT_BASE_H
#define CWRU_DAVINCI_SKILLS_AND_DEMO_DAVINCI_SKILLS_MANAGEMENT_BASE_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <string>
#include <math.h>
#include <algorithm>

#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/QuadWord.h>
#include <eigen3/Eigen/src/Geometry/Transform.h>

#include <cwru_davinci_kinematics/davinci_fwd_kinematics.h>
#include <cwru_davinci_kinematics/davinci_inv_kinematics.h>
#include <cwru_davinci_kinematics/davinci_kinematic_definitions.h>

#include <cwru_davinci_control/psm_controller.h>

#include <cwru_davinci_skills_and_demo/CwruDavinciSkillControlAction.h>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32MultiArray.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <map>
#include <queue>
#include <mutex>
#include <iostream>
#include <condition_variable>

#include <ctime>
#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>


namespace cwru_davinci_skills {







} // namespace cwru_davinci_skills



namespace cwru_davinci_skills_communications {

inline void printEigenAffine(Eigen::Affine3d affine)
{
  std::cout << "Rotation: " << std::endl;
  std::cout << affine.linear() << std::endl;
  std::cout << "origin: " << affine.translation().transpose() << std::endl;
}



struct BasicEnvTransforms {

  int index;
  Eigen::Affine3d psm_one_affine_wrt_lt_camera;
  Eigen::Affine3d psm_two_affine_wrt_lt_camera;
  Eigen::Affine3d psm_one_affine_wrt_psm_two;
  Eigen::Affine3d psm_two_affine_wrt_psm_one;
  Eigen::Affine3d tissue_frame_affine_wrt_lt_camera;
  Eigen::Affine3d psm_one_affine_wrt_tissue_frame;
  Eigen::Affine3d psm_two_affine_wrt_tissue_frame;
  Eigen::Affine3d psm_one_affine_wrt_dual_op_frame;
  Eigen::Affine3d psm_two_affine_wrt_dual_op_frame;
  Eigen::Affine3d dual_op_frame_affine_wrt_lt_camera;

  void printAllTransforms() {
    std::cout << "psm_one_affine_wrt_lt_camera: \n";
    printEigenAffine(psm_one_affine_wrt_lt_camera);
    std::cout << "psm_two_affine_wrt_lt_camera: \n";
    printEigenAffine(psm_two_affine_wrt_lt_camera);
    std::cout << "psm_one_affine_wrt_psm_two: \n";
    printEigenAffine(psm_one_affine_wrt_psm_two);
    std::cout << "psm_two_affine_wrt_psm_one: \n";
    printEigenAffine(psm_two_affine_wrt_psm_one);
    std::cout << "tissue_frame_affine_wrt_lt_camera: \n";
    printEigenAffine(tissue_frame_affine_wrt_lt_camera);
    std::cout << "psm_one_affine_wrt_tissue_frame: \n";
    printEigenAffine(psm_one_affine_wrt_tissue_frame);
    std::cout << "psm_two_affine_wrt_tissue_frame: \n";
    printEigenAffine(psm_two_affine_wrt_tissue_frame);
    std::cout << "psm_one_affine_wrt_dual_op_frame: \n";
    printEigenAffine(psm_one_affine_wrt_dual_op_frame);
    std::cout << "psm_two_affine_wrt_dual_op_frame: \n";
    printEigenAffine(psm_two_affine_wrt_dual_op_frame);
    std::cout << "dual_op_frame_affine_wrt_lt_camera: \n";
    printEigenAffine(dual_op_frame_affine_wrt_lt_camera);

  }
};


// Thread safe map adapted from https://stackoverflow.com/a/16075550/5525775 //
// A threadsafe-map.
template <class Key, class T>
class SafeMap
{
 public:
  SafeMap(void)
      : q()
      , m()
  {}

  ~SafeMap(void)
  {}

  // Clear the map map.
  void clear()
  {
    std::lock_guard<std::mutex> lock(m);
    q.clear();
  }

  // Check if a value exists for a given key in the map.
  bool value_exist_at_key(Key key)
  {
    std::lock_guard<std::mutex> lock(m);
    typename std::map<Key,T>::iterator it = q.find(key);
    if (it != q.end()) {
      return true;
    }
    else {
      return false;
    }
  }

  // Add an element to the map.
  void insert_or_assign(Key key, T t)
  {
    std::lock_guard<std::mutex> lock(m);
    q[key] = t;
  }

  // Get an element from the map
  T at(Key key)
  {
    std::lock_guard<std::mutex> lock(m);
    try {
      T val = q.at(key);
      return val;
    }
    catch (const std::out_of_range& e) {
      std::cerr << "Error! " << e.what() << "\n";
      throw e;
    }
  }

 private:
  std::map<Key, T> q;
  mutable std::mutex m;
};

// Thread safe queue adapted from https://stackoverflow.com/a/16075550/5525775 //
// A threadsafe-queue.
template <class T>
class SafeQueue
{
 public:
  SafeQueue(void)
      : q()
      , m()
      , c()
  {}

  ~SafeQueue(void)
  {}

  void clear() {
    std::lock_guard<std::mutex> lock(m);
    std::queue<T>().swap(q);
  }

  bool empty() {
    std::lock_guard<std::mutex> lock(m);
    return q.empty();
  }

  size_t size() {
    std::lock_guard<std::mutex> lock(m);
    return q.size();
  }

  // Add an element to the queue.
  void enqueue(T t)
  {
    std::lock_guard<std::mutex> lock(m);
    q.push(t);
    c.notify_one();
  }

  // Get the "front"-element.
  // If the queue is empty, wait till a element is avaiable.
  T dequeue(void)
  {
    std::unique_lock<std::mutex> lock(m);
    while(q.empty())
    {
      // release lock as long as the wait and reaquire it afterwards.
      c.wait(lock);
    }
    T val = q.front();
    q.pop();
    return val;
  }

 private:
  std::queue<T> q;
  mutable std::mutex m;
  std::condition_variable c;
};


} // namespace cwru_davinci_skills_communications


#endif //CWRU_DAVINCI_SKILLS_AND_DEMO_DAVINCI_SKILLS_MANAGEMENT_BASE_H
