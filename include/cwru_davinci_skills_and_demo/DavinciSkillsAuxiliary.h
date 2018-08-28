//
// Created by William on 27/08/18.
//

#ifndef CWRU_DAVINCI_SKILLS_AND_DEMO_DAVINCISKILLSAUXILIARY_H
#define CWRU_DAVINCI_SKILLS_AND_DEMO_DAVINCISKILLSAUXILIARY_H

#include "cwru_davinci_skills_and_demo/davinci_skills_management_base.h"

using namespace cwru_davinci_skills;
using namespace cwru_davinci_skills_communications;


class DavinciSkillsAuxiliary {
 public:

  inline void printEigenAffine(Eigen::Affine3d affine)
  {
    std::cout << "Rotation: " << std::endl;
    std::cout << affine.linear() << std::endl;
    std::cout << "origin: " << affine.translation().transpose() << std::endl;
  }


  inline Eigen::Affine3d convertTFToEigen(const tf::Transform &t)
  {
    Eigen::Affine3d e;
    for (int i = 0; i < 3; i++)
    {
      e.matrix()(i, 3) = t.getOrigin()[i];
      for (int j = 0; j < 3; j++)
      {
        e.matrix()(i, j) = t.getBasis()[i][j];
      }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++)
      e.matrix()(3, col) = 0;
    e.matrix()(3, 3) = 1;
    return e;
  }

  inline Eigen::Matrix3d rotateAboutAxisX(double phi)
  {
    Eigen::Matrix3d Rx;
    Rx(0, 0) = 1.0;
    Rx(0, 1) = 0.0;
    Rx(0, 2) = 0.0;
    Rx(1, 0) = 0.0;
    Rx(1, 1) = cos(phi);
    Rx(1, 2) = -sin(phi);
    Rx(2, 0) = 0.0;
    Rx(2, 1) = sin(phi);
    Rx(2, 2) = cos(phi);
    return Rx;
  }

  inline Eigen::Matrix3d rotateAboutAxisY(double phi)
  {
    Eigen::Matrix3d Roty;
    Roty(0, 0) = cos(phi);
    Roty(0, 1) = 0;
    Roty(0, 2) = sin(phi);
    Roty(1, 0) = 0;
    Roty(1, 1) = 1.0;
    Roty(1, 2) = 0.0;
    Roty(2, 0) = -sin(phi);
    Roty(2, 1) = 0.0;
    Roty(2, 2) = cos(phi);
    return Roty;
  }

  inline Eigen::Matrix3d rotateAboutAxisZ(double phi)
  {
    Eigen::Matrix3d Rz;
    Rz(0, 0) = cos(phi);
    Rz(0, 1) = -sin(phi);
    Rz(0, 2) = 0.0;
    Rz(1, 0) = sin(phi);
    Rz(1, 1) = cos(phi);
    Rz(1, 2) = 0.0;
    Rz(2, 0) = 0.0;
    Rz(2, 1) = 0.0;
    Rz(2, 2) = 1.0;
    return Rz;
  }


  inline Eigen::Matrix3d rotateAboutVectorK(Eigen::Vector3d k_vec, double phi)
  {
    Eigen::Matrix3d R_k_phi;
    double kx = k_vec(0);
    double ky = k_vec(1);
    double kz = k_vec(2);
    Eigen::Matrix3d K;
    K(0, 0) = 0.0;
    K(0, 1) = -kz;
    K(0, 2) = ky;
    K(1, 0) = kz;
    K(1, 1) = 0.0;
    K(1, 2) = -kx;
    K(2, 0) = -ky;
    K(2, 1) = kx;
    K(2, 2) = 0;
    Eigen::Matrix3d I = Eigen::MatrixXd::Identity(3, 3);
    R_k_phi = I + sin(phi) * K + (1 - cos(phi)) * K * K;
    return R_k_phi;
  }

  inline geometry_msgs::PointStamped convertEigenVec3dToGeoMsgsPtStamped(Eigen::Vector3d vec)
  {
    geometry_msgs::PointStamped pt;
    pt.header.stamp = ros::Time::now();
    pt.header.frame_id = "/left_camera_optical_frame";
    pt.point.x = vec[0];
    pt.point.y = vec[1];
    pt.point.z = vec[2];
    return pt;
  }


  inline Eigen::Vector3d convertPointStampedToEigenVector(const geometry_msgs::PointStamped &pt)
  {
    Eigen::Vector3d vec;
    vec << pt.point.x, pt.point.y, pt.point.z;
    return vec;
  }

  inline Eigen::Affine3d convertTfTransformToEigenAffine(const tf::Transform &t)
  {
    Eigen::Affine3d e;
    for (int i = 0; i < 3; i++)
    {
      e.matrix()(i, 3) = t.getOrigin()[i];
      for (int j = 0; j < 3; j++)
      {
        e.matrix()(i, j) = t.getBasis()[i][j];
      }
    }
    // Fill in identity in last row
    for (int col = 0; col < 3; col++) {
      e.matrix()(3, col) = 0;
    }
    e.matrix()(3, 3) = 1;
    return e;
  }

  inline Eigen::Affine3d convertGeoTransformStampedToEigenAffine(const geometry_msgs::TransformStamped &geoTf) {

    Eigen::Affine3d e;
    Eigen::Vector3d origin;
    Eigen::Matrix3d rotation;
    Eigen::Quaterniond quaternion;

    origin << geoTf.transform.translation.x,
        geoTf.transform.translation.y,
        geoTf.transform.translation.z;

    quaternion.x() = geoTf.transform.rotation.x;
    quaternion.y() = geoTf.transform.rotation.y;
    quaternion.z() = geoTf.transform.rotation.z;
    quaternion.w() = geoTf.transform.rotation.w;

    rotation = quaternion.toRotationMatrix();

    e.linear() = rotation;
    e.translation() = origin;

    return e;
  }

  inline geometry_msgs::TransformStamped convertEigenAffineToGeoTransform(Eigen::Affine3d affine) {

    geometry_msgs::TransformStamped geo_tf;

    Eigen::Quaterniond quaternion;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d origin;

    rotation = affine.linear();
    quaternion = rotation;

    origin = affine.translation();

    geo_tf.transform.rotation.x = quaternion.x();
    geo_tf.transform.rotation.y = quaternion.y();
    geo_tf.transform.rotation.z = quaternion.z();
    geo_tf.transform.rotation.w = quaternion.w();

    geo_tf.transform.translation.x = origin[0];
    geo_tf.transform.translation.y = origin[1];
    geo_tf.transform.translation.z = origin[2];

    return geo_tf;

  }


 private:

};

#endif //CWRU_DAVINCI_SKILLS_AND_DEMO_DAVINCISKILLSAUXILIARY_H
