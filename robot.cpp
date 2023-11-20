#include "utils.h"
#include "robot.h"
#include "distance.h"

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>
#include <chrono>

#include <ros/ros.h>
#include <ros/service.h>

using namespace std;
using namespace Eigen;

// Link

Matrix4d Link::dhMatrix(double q)
{
  Matrix4d dhMat = Matrix4d::Zero();

  double cost = this->jointType == 0 ? cos(q) : this->_cost;
  double sint = this->jointType == 0 ? sin(q) : this->_sint;
  double cosa = _cosa;
  double sina = _sina;
  double d = this->jointType == 1 ? q : this->dHd;
  double a = this->dHa;

  dhMat(0, 0) = cost;
  dhMat(0, 1) = -sint * cosa;
  dhMat(0, 2) = sint * sina;
  dhMat(0, 3) = a * cost;

  dhMat(1, 0) = sint;
  dhMat(1, 1) = cost * cosa;
  dhMat(1, 2) = -cost * sina;
  dhMat(1, 3) = a * sint;

  dhMat(2, 1) = sina;
  dhMat(2, 2) = cosa;
  dhMat(2, 3) = d;

  dhMat(3, 3) = 1;

  return dhMat;
}

Link::Link(int number, double theta, double d, double alpha, double a, int type, string strName)
{
  linkNumber = number;
  dHtheta = theta;
  dHd = d;
  dHalpha = alpha;
  dHa = a;
  jointType = type;
  name = strName;

  _cost = cos(theta);
  _sint = sin(theta);
  _cosa = cos(alpha);
  _sina = sin(alpha);
}

// DistanceLinkObjResult

DistanceLinkObjResult::DistanceLinkObjResult(){};

// DistanceLinkLinkResult

DistanceLinkLinkResult::DistanceLinkLinkResult(){};

// ConstControlResult

ConstControlResult::ConstControlResult(){};

// DistanceRobotObjResult

DistanceLinkObjResult DistanceRobotObjResult::operator[](int i) const
{
  return _structDistances[i];
}

DistanceLinkObjResult DistanceRobotObjResult::getClosest()
{
  DistanceLinkObjResult ds = this->_structDistances[0];
  double minDist = ds.distance;

  for (int i = 0; i < this->noElements; i++)
  {
    if (this->_structDistances[i].distance < minDist)
    {
      minDist = this->_structDistances[i].distance;
      ds = this->_structDistances[i];
    }
  }

  return ds;
}

DistanceLinkObjResult *DistanceRobotObjResult::getItem(int indLink, int indColObjLink)
{
  for (int i = 0; i < this->noElements; i++)
  {
    if (this->_structDistances[i].linkNumber == indLink && this->_structDistances[i].colObjLinkNumber == indColObjLink)
    {
      return &this->_structDistances[i];
    }
  }
  return NULL;
}

void DistanceRobotObjResult::addDistanceLinkObj(DistanceLinkObjResult dlo)
{
  this->_structDistances.push_back(dlo);
  this->noElements++;
}

DistanceRobotObjResult::DistanceRobotObjResult(int noJoints)
{
  this->_noJoints = noJoints;
  this->noElements = 0;
}

DistanceRobotObjResult::DistanceRobotObjResult()
{
  this->noElements = 0;
}

MatrixXd DistanceRobotObjResult::getAllJacobians()
{
  MatrixXd jac(this->noElements, this->_noJoints);

  for (int i = 0; i < this->noElements; i++)
  {
    for (int j = 0; j < this->_noJoints; j++)
    {
      jac(i, j) = this->_structDistances[i].jacDist[j];
    }
  }

  return jac;
}

VectorXd DistanceRobotObjResult::getDistances()
{
  VectorXd dist(this->noElements);

  for (int i = 0; i < this->noElements; i++)
  {
    dist[i] = this->_structDistances[i].distance;
  }

  return dist;
}

// DistanceRobotAutoResult

DistanceLinkLinkResult DistanceRobotAutoResult::operator[](int i) const
{
  return _structDistances[i];
}

DistanceLinkLinkResult DistanceRobotAutoResult::getClosest()
{
  DistanceLinkLinkResult ds = this->_structDistances[0];
  double minDist = ds.distance;

  for (int i = 0; i < this->noElements; i++)
  {
    if (this->_structDistances[i].distance < minDist)
    {
      minDist = this->_structDistances[i].distance;
      ds = this->_structDistances[i];
    }
  }

  return ds;
}

DistanceLinkLinkResult *DistanceRobotAutoResult::getItem(int indLink1, int indColObjLink1, int indLink2,
                                                         int indColObjLink2)
{
  for (int i = 0; i < this->noElements; i++)
  {
    if (this->_structDistances[i].linkNumber1 == indLink1 &&
        this->_structDistances[i].colObjLinkNumber1 == indColObjLink1 &&
        this->_structDistances[i].linkNumber2 == indLink2 &&
        this->_structDistances[i].colObjLinkNumber2 == indColObjLink2)
    {
      return &this->_structDistances[i];
    }
  }
  return NULL;
}

void DistanceRobotAutoResult::addDistanceLinkLink(DistanceLinkLinkResult dll)
{
  this->_structDistances.push_back(dll);
  this->noElements++;
}

DistanceRobotAutoResult::DistanceRobotAutoResult(int noJoints)
{
  this->_noJoints = noJoints;
  this->noElements = 0;
}

DistanceRobotAutoResult::DistanceRobotAutoResult()
{
  this->noElements = 0;
}

MatrixXd DistanceRobotAutoResult::getAllJacobians()
{
  MatrixXd jac(this->noElements, this->_noJoints);

  for (int i = 0; i < this->noElements; i++)
  {
    for (int j = 0; j < this->_noJoints; j++)
    {
      jac(i, j) = this->_structDistances[i].jacDist[j];
    }
  }

  return jac;
}

VectorXd DistanceRobotAutoResult::getDistances()
{
  VectorXd dist(this->noElements);

  for (int i = 0; i < this->noElements; i++)
  {
    dist[i] = this->_structDistances[i].distance;
  }

  return dist;
}

// FreeConfigResult

FreeConfigResult::FreeConfigResult()
{
  this->isFree = true;
  this->errorType = ErrorType::none;
  this->linkNumber1 = -1;
  this->linkNumber2 = -1;
  this->colObjNumber1 = -1;
  this->colObjNumber2 = -1;
  this->jointNumber = -1;
  this->obstacleNumber = -1;
}

// Manipulator

Manipulator::Manipulator(int no)
{
  noJoints = no;
  q = VectorXd::Zero(no);
  qBase = VectorXd::Zero(no);
  htmWorldToBase = Matrix4d::Identity();
  htmBaseToDH0 = Matrix4d::Identity();
  htmDHnToTool = Matrix4d::Identity();
}

Manipulator Manipulator::createKukaKR5()
{
  // Create empty manipulator
  Manipulator manip(6);

  // Create links
  double d_bs = 0.340;
  double d_se = 0.400;
  double d_ew = 0.400;
  double d_wf = 0.126;

  vector<Link> linksKuka;

  Link link1(1, 0, 0.335, -M_PI / 2, 0.075, 0, "j1");
  Link link2(2, 0, 0, 0, 0.365, 0, "j2");
  Link link3(3, 0, 0, M_PI / 2, 0.090, 0, "j3");
  Link link4(4, 0, -0.405, -M_PI / 2, 0, 0, "j4");
  Link link5(5, 0, 0, -M_PI / 2, 0, 0, "j5");
  Link link6(6, 0, 0.080, 0, 0, 0, "j6");

  linksKuka.push_back(link1);
  linksKuka.push_back(link2);
  linksKuka.push_back(link3);
  linksKuka.push_back(link4);
  linksKuka.push_back(link5);
  linksKuka.push_back(link6);

  manip.links = linksKuka;

  // Create collision objects
  FKResult fkres0 = manip.fk();

  // For the first link
  Cylinder *kr5C0_0 =
      new Cylinder(Utils::trn(-0.075, 0.170, 0) * Utils::roty(M_PI / 2) * Utils::rotx(M_PI / 2), 0.12, 0.33);
  manip.links[0].colObjs.push_back(kr5C0_0);
  // manip.links[0].htmCols.push_back(fkres0.htmDH[0].inverse() * (*kr5C0_0).htm);
  manip.links[0].htmCols.push_back((*kr5C0_0).htm);

  Cylinder *kr5C0_1 = new Cylinder(Utils::trn(0, 0, 0.030) * Utils::rotz(M_PI / 2) * Utils::rotx(M_PI), 0.095, 0.30);
  manip.links[0].colObjs.push_back(kr5C0_1);
  // manip.links[0].htmCols.push_back(fkres0.htmDH[0].inverse() * (*kr5C0_1).htm);
  manip.links[0].htmCols.push_back((*kr5C0_1).htm);

  // For the second link
  Box *kr5C1_0 = new Box(Utils::trn(-0.200, 0.020, 0.12) * Utils::roty(M_PI / 2), 0.1, 0.16, 0.5);
  manip.links[1].colObjs.push_back(kr5C1_0);
  // manip.links[1].htmCols.push_back(fkres0.htmDH[1].inverse() * (*kr5C1_0).htm);
  manip.links[1].htmCols.push_back((*kr5C1_0).htm);

  Cylinder *kr5C1_1 = new Cylinder(Utils::trn(0, 0, 0.040) * Utils::rotz(M_PI) * Utils::rotx(M_PI), 0.095, 0.28);
  manip.links[1].colObjs.push_back(kr5C1_1);
  // manip.links[1].htmCols.push_back(fkres0.htmDH[1].inverse() * (*kr5C1_1).htm);
  manip.links[1].htmCols.push_back((*kr5C1_1).htm);

  // For the third link
  Box *kr5C2_0 = new Box(Utils::trn(0, 0, -0.224) * Utils::rotz(-M_PI / 2) * Utils::rotx(-M_PI / 2), 0.143, 0.45, 0.12);
  manip.links[2].colObjs.push_back(kr5C2_0);
  // manip.links[2].htmCols.push_back(fkres0.htmDH[2].inverse() * (*kr5C2_0).htm);
  manip.links[2].htmCols.push_back((*kr5C2_0).htm);

  return manip;
}

Manipulator Manipulator::createGeoTouch()
{
  // Create empty manipulator
  Manipulator manip(5);

  // Create links
  double d_se = 0.400;
  double d_ew = 0.400;
  double d_wf = 0.126;

  vector<Link> linksKuka;

  // Link(int number, double theta, double d, double alpha, double a, int type, string strName);

  Link link1(1, 0, 0, M_PI / 2, 0, 0, "j1");
  Link link2(1, 0, 0, 0, 0.133, 0, "j2");
  Link link3(1, 0, 0, M_PI / 2, 0, 0, "j3");
  Link link4(1, 0, 0.133, -M_PI / 2, 0, 0, "j4");
  Link link5(1, 0, 0, 0, -0.04, 0, "j5");

  // Link link1(1, 0, 0, M_PI / 2, 0, 0, "j1");
  // Link link2(1, 0, 0, 0, 0, 0, "j2");
  // Link link3(1, 0, 0, M_PI / 2, 0, 0, "j3");
  // Link link4(1, 0, 0, -M_PI / 2, 0, 0, "j4");
  // Link link5(1, 0, 0, 0, 0, 0, "j5");

  linksKuka.push_back(link1);
  linksKuka.push_back(link2);
  linksKuka.push_back(link3);
  linksKuka.push_back(link4);
  linksKuka.push_back(link5);
  // linksKuka.push_back(link6);

  manip.htmWorldToBase = Utils::rotz(-M_PI / 2) * Utils::roty(-M_PI / 2);

  manip.links = linksKuka;

  return manip;
}

Manipulator Manipulator::createKukaIIWA()
{
  // Create empty manipulator
  Manipulator manip(7);

  // Create links
  double d_tool = 0.405;
  double d_bs = 0.360;
  double d_se = 0.420;
  double d_ew = 0.400;
  double d_wf = 0.152 + d_tool; // 0.609

  vector<Link> linksKuka;

  Link link1(1, 0, d_bs, -M_PI / 2, 0, 0, "iiwa_joint_1");
  Link link2(2, 0, 0, M_PI / 2, 0, 0, "iiwa_joint_2");
  Link link3(3, 0, d_se, M_PI / 2, 0, 0, "iiwa_joint_3");
  Link link4(4, 0, 0, -M_PI / 2, 0, 0, "iiwa_joint_4");
  Link link5(5, 0, d_ew, -M_PI / 2, 0, 0, "iiwa_joint_5");
  Link link6(6, 0, 0, M_PI / 2, 0, 0, "iiwa_joint_6");
  Link link7(7, 0, d_wf, 0, 0, 0, "iiwa_joint_7");

  linksKuka.push_back(link1);
  linksKuka.push_back(link2);
  linksKuka.push_back(link3);
  linksKuka.push_back(link4);
  linksKuka.push_back(link5);
  linksKuka.push_back(link6);
  linksKuka.push_back(link7);

  manip.links = linksKuka;

  // Create collision objects
  FKResult fkres0 = manip.fk();

  double dz = -0.02;

  // For the first link
  Cylinder *iiwaC0_0 = new Cylinder(Utils::trn(0, 0, 0.15 + dz), 0.1, 0.12);
  manip.links[0].colObjs.push_back(iiwaC0_0);
  manip.links[0].htmCols.push_back(fkres0.htmDH[0].inverse() * (*iiwaC0_0).htm);
  manip.links[0].nameObjs.push_back("c0_0");

  Sphere *iiwaC0_1 = new Sphere(Utils::trn(0.01, 0, 0.33 + dz), 0.12);
  manip.links[0].colObjs.push_back(iiwaC0_1);
  manip.links[0].htmCols.push_back(fkres0.htmDH[0].inverse() * (*iiwaC0_1).htm);
  manip.links[0].nameObjs.push_back("c0_1");

  // For the second link
  Sphere *iiwaC1_0 = new Sphere(Utils::trn(0, 0.03, 0.38 + dz), 0.1);
  manip.links[1].colObjs.push_back(iiwaC1_0);
  manip.links[1].htmCols.push_back(fkres0.htmDH[1].inverse() * (*iiwaC1_0).htm);
  manip.links[1].nameObjs.push_back("c1_0");

  Cylinder *iiwaC1_1 = new Cylinder(Utils::trn(0, 0, 0.54 + dz), 0.075, 0.125);
  manip.links[1].colObjs.push_back(iiwaC1_1);
  manip.links[1].htmCols.push_back(fkres0.htmDH[1].inverse() * (*iiwaC1_1).htm);
  manip.links[1].nameObjs.push_back("c1_1");

  // For the third link
  Cylinder *iiwaC2_0 = new Cylinder(Utils::trn(0, 0, 0.66 + 2 * dz), 0.08, 0.1);
  manip.links[2].colObjs.push_back(iiwaC2_0);
  manip.links[2].htmCols.push_back(fkres0.htmDH[2].inverse() * (*iiwaC2_0).htm);
  manip.links[2].nameObjs.push_back("c2_0");

  Sphere *iiwaC2_1 = new Sphere(Utils::trn(0, 0.01, 0.75 + 2 * dz), 0.11);
  manip.links[2].colObjs.push_back(iiwaC2_1);
  manip.links[2].htmCols.push_back(fkres0.htmDH[2].inverse() * (*iiwaC2_1).htm);
  manip.links[2].nameObjs.push_back("c2_1");

  // For the fourth link
  Sphere *iiwaC3_0 = new Sphere(Utils::trn(0, -0.035, 0.803 + 2 * dz), 0.08);
  manip.links[3].colObjs.push_back(iiwaC3_0);
  manip.links[3].htmCols.push_back(fkres0.htmDH[3].inverse() * (*iiwaC3_0).htm);
  manip.links[3].nameObjs.push_back("c3_0");

  Cylinder *iiwaC3_1 = new Cylinder(Utils::trn(0, 0, 0.935 + 2 * dz), 0.075, 0.125);
  manip.links[3].colObjs.push_back(iiwaC3_1);
  manip.links[3].htmCols.push_back(fkres0.htmDH[3].inverse() * (*iiwaC3_1).htm);
  manip.links[3].nameObjs.push_back("c3_1");

  // For the fifth link
  Cylinder *iiwaC4_0 = new Cylinder(Utils::trn(0, 0, 1.052 + 2 * dz), 0.075, 0.11);
  manip.links[4].colObjs.push_back(iiwaC4_0);
  manip.links[4].htmCols.push_back(fkres0.htmDH[4].inverse() * (*iiwaC4_0).htm);
  manip.links[4].nameObjs.push_back("c4_0");

  Box *iiwaC4_1 = new Box(Utils::trn(0, -0.08, 1.15 + 2 * dz), 0.12, 0.04, 0.17);
  manip.links[4].colObjs.push_back(iiwaC4_1);
  manip.links[4].htmCols.push_back(fkres0.htmDH[4].inverse() * (*iiwaC4_1).htm);
  manip.links[4].nameObjs.push_back("c4_1");

  // For the sixth link
  Cylinder *iiwaC5_0 = new Cylinder(Utils::trn(0, 0, 1.203 + 2 * dz), 0.075, 0.24);
  manip.links[5].colObjs.push_back(iiwaC5_0);
  manip.links[5].htmCols.push_back(fkres0.htmDH[5].inverse() * (*iiwaC5_0).htm);
  manip.links[5].nameObjs.push_back("c5_0");

  // Create joint limits (in rad)
  double dq = 0.095;

  VectorXd qMin(7);
  qMin << -2.967055 + dq, -2.09435 + dq, -2.967055 + dq, -2.09435 + dq, -2.937055 + dq, -2.09435 + dq, -3.054325 + dq;

  VectorXd qMax(7);
  qMax << 2.967055 - dq, 2.09435 - dq, 2.967055 - dq, 2.09435 - dq, 2.937055 - dq, 2.09435 - dq, 3.054325 - dq;

  manip.qMin = qMin;
  manip.qMax = qMax;

  // Create joint velocity limits (in rad/s)
  VectorXd qDotMin(7);
  qDotMin << -1.482, -1.482, -1.740, -1.307, -2.268, -2.355, -2.356;

  VectorXd qDotMax(7);
  qDotMax << 1.482, 1.482, 1.740, 1.307, 2.268, 2.355, 2.356;

  manip.qDotMin = qDotMin;
  manip.qDotMax = qDotMax;

  return manip;
}

Manipulator Manipulator::createKukaIIWASurgery()
{
  // Create empty manipulator
  Manipulator manip(7);

  // Create links
  double d_tool = 0.400;
  double d_bs = 0.360;
  double d_se = 0.420;
  double d_ew = 0.400;
  double d_wf = 0.156 + d_tool; // 0.609

  vector<Link> linksKuka;

  Link link1(1, 0, d_bs, -M_PI / 2, 0, 0, "iiwa_joint_1");
  Link link2(2, 0, 0, M_PI / 2, 0, 0, "iiwa_joint_2");
  Link link3(3, 0, d_se, M_PI / 2, 0, 0, "iiwa_joint_3");
  Link link4(4, 0, 0, -M_PI / 2, 0, 0, "iiwa_joint_4");
  Link link5(5, 0, d_ew, -M_PI / 2, 0, 0, "iiwa_joint_5");
  Link link6(6, 0, 0, M_PI / 2, 0, 0, "iiwa_joint_6");
  Link link7(7, 0, d_wf, 0, 0, 0, "iiwa_joint_7");

  linksKuka.push_back(link1);
  linksKuka.push_back(link2);
  linksKuka.push_back(link3);
  linksKuka.push_back(link4);
  linksKuka.push_back(link5);
  linksKuka.push_back(link6);
  linksKuka.push_back(link7);

  manip.links = linksKuka;

  // Create collision objects
  FKResult fkres0 = manip.fk();

  double dz = -0.02;

  // For the first link
  Cylinder *iiwaC0_0 = new Cylinder(Utils::trn(0, 0, 0.15 + dz), 0.1, 0.12);
  manip.links[0].colObjs.push_back(iiwaC0_0);
  manip.links[0].htmCols.push_back(fkres0.htmDH[0].inverse() * (*iiwaC0_0).htm);
  manip.links[0].nameObjs.push_back("c0_0");

  Sphere *iiwaC0_1 = new Sphere(Utils::trn(0.01, 0, 0.33 + dz), 0.12);
  manip.links[0].colObjs.push_back(iiwaC0_1);
  manip.links[0].htmCols.push_back(fkres0.htmDH[0].inverse() * (*iiwaC0_1).htm);
  manip.links[0].nameObjs.push_back("c0_1");

  // For the second link
  Sphere *iiwaC1_0 = new Sphere(Utils::trn(0, 0.03, 0.38 + dz), 0.1);
  manip.links[1].colObjs.push_back(iiwaC1_0);
  manip.links[1].htmCols.push_back(fkres0.htmDH[1].inverse() * (*iiwaC1_0).htm);
  manip.links[1].nameObjs.push_back("c1_0");

  Cylinder *iiwaC1_1 = new Cylinder(Utils::trn(0, 0, 0.54 + dz), 0.075, 0.125);
  manip.links[1].colObjs.push_back(iiwaC1_1);
  manip.links[1].htmCols.push_back(fkres0.htmDH[1].inverse() * (*iiwaC1_1).htm);
  manip.links[1].nameObjs.push_back("c1_1");

  // For the third link
  Cylinder *iiwaC2_0 = new Cylinder(Utils::trn(0, 0, 0.66 + 2 * dz), 0.08, 0.1);
  manip.links[2].colObjs.push_back(iiwaC2_0);
  manip.links[2].htmCols.push_back(fkres0.htmDH[2].inverse() * (*iiwaC2_0).htm);
  manip.links[2].nameObjs.push_back("c2_0");

  Sphere *iiwaC2_1 = new Sphere(Utils::trn(0, 0.01, 0.75 + 2 * dz), 0.11);
  manip.links[2].colObjs.push_back(iiwaC2_1);
  manip.links[2].htmCols.push_back(fkres0.htmDH[2].inverse() * (*iiwaC2_1).htm);
  manip.links[2].nameObjs.push_back("c2_1");

  // For the fourth link
  Sphere *iiwaC3_0 = new Sphere(Utils::trn(0, -0.035, 0.803 + 2 * dz), 0.08);
  manip.links[3].colObjs.push_back(iiwaC3_0);
  manip.links[3].htmCols.push_back(fkres0.htmDH[3].inverse() * (*iiwaC3_0).htm);
  manip.links[3].nameObjs.push_back("c3_0");

  Cylinder *iiwaC3_1 = new Cylinder(Utils::trn(0, 0, 0.935 + 2 * dz), 0.075, 0.125);
  manip.links[3].colObjs.push_back(iiwaC3_1);
  manip.links[3].htmCols.push_back(fkres0.htmDH[3].inverse() * (*iiwaC3_1).htm);
  manip.links[3].nameObjs.push_back("c3_1");

  // For the fifth link
  Cylinder *iiwaC4_0 = new Cylinder(Utils::trn(0, 0, 1.052 + 2 * dz), 0.075, 0.11);
  manip.links[4].colObjs.push_back(iiwaC4_0);
  manip.links[4].htmCols.push_back(fkres0.htmDH[4].inverse() * (*iiwaC4_0).htm);
  manip.links[4].nameObjs.push_back("c4_0");

  Box *iiwaC4_1 = new Box(Utils::trn(0, -0.08, 1.15 + 2 * dz), 0.12, 0.04, 0.17);
  manip.links[4].colObjs.push_back(iiwaC4_1);
  manip.links[4].htmCols.push_back(fkres0.htmDH[4].inverse() * (*iiwaC4_1).htm);
  manip.links[4].nameObjs.push_back("c4_1");

  // For the sixth link
  Cylinder *iiwaC5_0 = new Cylinder(Utils::trn(0, 0, 1.203 + 2 * dz), 0.075, 0.24);
  manip.links[5].colObjs.push_back(iiwaC5_0);
  manip.links[5].htmCols.push_back(fkres0.htmDH[5].inverse() * (*iiwaC5_0).htm);
  manip.links[5].nameObjs.push_back("c5_0");

  // Create joint limits (in rad)
  double dq = 0.087;

  VectorXd qMin(7);
  qMin << -2.967055 + dq, -2.09435 + dq, -2.967055 + dq, -2.09435 + dq, -2.937055 + dq, -2.09435 + dq, -3.054325 + dq;

  VectorXd qMax(7);
  qMax << 2.967055 - dq, 2.09435 - dq, 2.967055 - dq, 2.09435 - dq, 2.937055 - dq, 2.09435 - dq, 3.054325 - dq;

  manip.qMin = qMin;
  manip.qMax = qMax;

  // Create joint velocity limits (in rad/s)
  VectorXd qDotMin(7);
  qDotMin << -1.482, -1.482, -1.740, -1.307, -2.268, -2.355, -2.356;

  VectorXd qDotMax(7);
  qDotMax << 1.482, 1.482, 1.740, 1.307, 2.268, 2.355, 2.356;

  manip.qDotMin = qDotMin;
  manip.qDotMax = qDotMax;

  // manip.htmDHnToTool = Utils::trn(0, 0, d_tool);

  return manip;
}

Manipulator Manipulator::createKukaIIWAWithTool()
{
  // Create empty manipulator
  Manipulator manip(7);

  // Create links
  double d_bs = 0.340;
  double d_se = 0.400;
  double d_ew = 0.400;
  double d_wf = 0.152;

  vector<Link> linksKuka;

  Link link1(1, 0, d_bs, -M_PI / 2, 0, 0, "iiwa_joint_1");
  Link link2(2, 0, 0, M_PI / 2, 0, 0, "iiwa_joint_2");
  Link link3(3, 0, d_se, M_PI / 2, 0, 0, "iiwa_joint_3");
  Link link4(4, 0, 0, -M_PI / 2, 0, 0, "iiwa_joint_4");
  Link link5(5, 0, d_ew, -M_PI / 2, 0, 0, "iiwa_joint_5");
  Link link6(6, 0, 0, M_PI / 2, 0, 0, "iiwa_joint_6");
  Link link7(7, 0, d_wf, 0, 0, 0, "iiwa_joint_7");

  linksKuka.push_back(link1);
  linksKuka.push_back(link2);
  linksKuka.push_back(link3);
  linksKuka.push_back(link4);
  linksKuka.push_back(link5);
  linksKuka.push_back(link6);
  linksKuka.push_back(link7);

  manip.links = linksKuka;

  // Create collision objects
  FKResult fkres0 = manip.fk();

  double dz = -0.02;

  // For the first link
  Cylinder *iiwaC0_0 = new Cylinder(Utils::trn(0, 0, 0.15 + dz), 0.1, 0.12);
  manip.links[0].colObjs.push_back(iiwaC0_0);
  manip.links[0].htmCols.push_back(fkres0.htmDH[0].inverse() * (*iiwaC0_0).htm);
  manip.links[0].nameObjs.push_back("c0_0");

  Sphere *iiwaC0_1 = new Sphere(Utils::trn(0.01, 0, 0.33 + dz), 0.125);
  manip.links[0].colObjs.push_back(iiwaC0_1);
  manip.links[0].htmCols.push_back(fkres0.htmDH[0].inverse() * (*iiwaC0_1).htm);
  manip.links[0].nameObjs.push_back("c0_1");

  // For the second link
  Sphere *iiwaC1_0 = new Sphere(Utils::trn(0, 0.03, 0.38 + dz), 0.11);
  manip.links[1].colObjs.push_back(iiwaC1_0);
  manip.links[1].htmCols.push_back(fkres0.htmDH[1].inverse() * (*iiwaC1_0).htm);
  manip.links[1].nameObjs.push_back("c1_0");

  Cylinder *iiwaC1_1 = new Cylinder(Utils::trn(0, 0, 0.54 + dz), 0.08, 0.125);
  manip.links[1].colObjs.push_back(iiwaC1_1);
  manip.links[1].htmCols.push_back(fkres0.htmDH[1].inverse() * (*iiwaC1_1).htm);
  manip.links[1].nameObjs.push_back("c1_1");

  // For the third link
  Cylinder *iiwaC2_0 = new Cylinder(Utils::trn(0, 0, 0.66 + 2 * dz), 0.08, 0.1);
  manip.links[2].colObjs.push_back(iiwaC2_0);
  manip.links[2].htmCols.push_back(fkres0.htmDH[2].inverse() * (*iiwaC2_0).htm);
  manip.links[2].nameObjs.push_back("c2_0");

  Sphere *iiwaC2_1 = new Sphere(Utils::trn(0, 0.01, 0.75 + 2 * dz), 0.11);
  manip.links[2].colObjs.push_back(iiwaC2_1);
  manip.links[2].htmCols.push_back(fkres0.htmDH[2].inverse() * (*iiwaC2_1).htm);
  manip.links[2].nameObjs.push_back("c2_1");

  // For the fourth link
  Sphere *iiwaC3_0 = new Sphere(Utils::trn(0, -0.035, 0.803 + 2 * dz), 0.09);
  manip.links[3].colObjs.push_back(iiwaC3_0);
  manip.links[3].htmCols.push_back(fkres0.htmDH[3].inverse() * (*iiwaC3_0).htm);
  manip.links[3].nameObjs.push_back("c3_0");

  Cylinder *iiwaC3_1 = new Cylinder(Utils::trn(0, 0, 0.935 + 2 * dz), 0.075, 0.125);
  manip.links[3].colObjs.push_back(iiwaC3_1);
  manip.links[3].htmCols.push_back(fkres0.htmDH[3].inverse() * (*iiwaC3_1).htm);
  manip.links[3].nameObjs.push_back("c3_1");

  // For the fifth link
  Cylinder *iiwaC4_0 = new Cylinder(Utils::trn(0, 0, 1.052 + 2 * dz), 0.075, 0.11);
  manip.links[4].colObjs.push_back(iiwaC4_0);
  manip.links[4].htmCols.push_back(fkres0.htmDH[4].inverse() * (*iiwaC4_0).htm);
  manip.links[4].nameObjs.push_back("c4_0");

  Box *iiwaC4_1 = new Box(Utils::trn(0, -0.08, 1.15 + 2 * dz), 0.12, 0.04, 0.17);
  manip.links[4].colObjs.push_back(iiwaC4_1);
  manip.links[4].htmCols.push_back(fkres0.htmDH[4].inverse() * (*iiwaC4_1).htm);
  manip.links[4].nameObjs.push_back("c4_1");

  // For the sixth link
  Cylinder *iiwaC5_0 = new Cylinder(Utils::trn(0, 0, 1.203 + 2 * dz), 0.075, 0.24);
  manip.links[5].colObjs.push_back(iiwaC5_0);
  manip.links[5].htmCols.push_back(fkres0.htmDH[5].inverse() * (*iiwaC5_0).htm);
  manip.links[5].nameObjs.push_back("c5_0");

  // For the seventh link
  // Box* iiwaC6_0 = new Box(Utils::trn(0.1, 0, 1.58 + 2 * dz - 0.14 / 2 + 0.02 + 0.015), 0.20, 0.1, 0.6 - 0.14 -
  // 0.015); manip.links[6].colObjs.push_back(iiwaC6_0); manip.links[6].htmCols.push_back(fkres0.htmDH[6].inverse() *
  // (*iiwaC6_0).htm); manip.links[6].nameObjs.push_back("c6_0");

  // Create joint limits (in rad)
  double dq = 0.087;

  VectorXd qMin(7);
  qMin << -2.967055 + dq, -2.09435 + dq, -2.967055 + dq, -2.09435 + dq, -2.937055 + dq, -2.09435 + dq, -3.054325 + dq;

  VectorXd qMax(7);
  qMax << 2.967055 - dq, 2.09435 - dq, 2.967055 - dq, 2.09435 - dq, 2.937055 - dq, 2.09435 - dq, 3.054325 - dq;

  manip.qMin = qMin;
  manip.qMax = qMax;

  // Create joint velocity limits (in rad/s)
  VectorXd qDotMin(7);
  qDotMin << -1.482, -1.482, -1.740, -1.307, -2.268, -2.355, -2.356;

  VectorXd qDotMax(7);
  qDotMax << 1.482, 1.482, 1.740, 1.307, 2.268, 2.355, 2.356;

  manip.qDotMin = qDotMin;
  manip.qDotMax = qDotMax;

  return manip;
}
void Manipulator::setConfig(VectorXd q, Matrix4d customHtmWorldToBase)
{
  // Set the configuration
  this->q = q;

  FKResult fkres = this->fk(q, customHtmWorldToBase);

  // Update all primitive collision objects to the new pose
  for (int i = 0; i < this->noJoints; i++)
  {
    for (int j = 0; j < this->links[i].colObjs.size(); j++)
    {
      (*this->links[i].colObjs[j]).htm = fkres.htmDH[i] * this->links[i].htmCols[j];
    }
  }
}

FKResult Manipulator::fk(VectorXd q, Matrix4d customHtmWorldToBase)
{
  VectorXd q_int = q.rows() == 0 ? this->q : q;
  Matrix4d htmNow = customHtmWorldToBase(3, 3) == 0 ? this->htmWorldToBase : customHtmWorldToBase;
  htmNow = htmNow * this->htmBaseToDH0;

  vector<Matrix4d> listOfHtms;
  double theta, d, alpha, a;

  for (int i = 0; i < this->noJoints; i++)
  {
    htmNow = htmNow * links[i].dhMatrix(q_int[i]);
    listOfHtms.push_back(htmNow);
  }

  FKResult fkres;
  fkres.htmDH = listOfHtms;
  fkres.htmTool = htmNow * this->htmDHnToTool;

  return fkres;
}

FulcrumPointResult Manipulator::computeFulcrumPoint(Vector3d pf, VectorXd q)
{
  FulcrumPointResult fulcrumPointResult;

  // KUKA Current Joints Forward Kinematics
  FKResult fkr = jacGeo(q);

  // KUKA's X-Rotation, Y-Rotation, Z-Rotation, Position - From the Forward Kinematics
  Vector3d xe = fkr.htmTool.block<3, 1>(0, 0);
  Vector3d ye = fkr.htmTool.block<3, 1>(0, 1);
  Vector3d ze = fkr.htmTool.block<3, 1>(0, 2);
  Vector3d pe = fkr.htmTool.block<3, 1>(0, 3);

  // KUKA Jacobian | Linear Velocity Component
  MatrixXd Jv = fkr.jacTool.block<3, 7>(0, 0);

  // KUKA Jacobian | Angular Velocity Component
  MatrixXd Jw = fkr.jacTool.block<3, 7>(3, 0);

  /* Task Function Components */
  // Fx or F1:
  fulcrumPointResult.fx = (xe.transpose() * (pe - pf))[0];
  // Fy or F2
  fulcrumPointResult.fy = (ye.transpose() * (pe - pf))[0];
  // Fz or F3
  fulcrumPointResult.fz = (ze.transpose() * (pe - pf))[0];
  // Computing the distance to target
  fulcrumPointResult.df =
      sqrt(fulcrumPointResult.fx * fulcrumPointResult.fx + fulcrumPointResult.fy * fulcrumPointResult.fy);

  // x component of the RCM Task Function Jacobian
  fulcrumPointResult.jacfx = xe.transpose() * Jv - (pe - pf).transpose() * Utils::S(xe) * Jw;
  // y component of the RCM Task Function Jacobian
  fulcrumPointResult.jacfy = ye.transpose() * Jv - (pe - pf).transpose() * Utils::S(ye) * Jw;

  fulcrumPointResult.fkr = fkr;
  return fulcrumPointResult;
}

FKResult Manipulator::jacGeo(VectorXd q, Matrix4d customHtmWorldToBase)
{
  // Compute the forward kinematics
  FKResult fkres = this->fk(q, customHtmWorldToBase);

  // Now compute the differential kinematics
  vector<MatrixXd> listOfJacs;

  int n = this->noJoints;

  for (int i = 0; i < n; i++)
  {
    MatrixXd newJac = MatrixXd::Zero(6, this->noJoints);
    listOfJacs.push_back(newJac);
  }

  Vector3d p_i, p_j_ant, z_j_ant, v, w;
  Matrix4d htmWorldToDH0 = customHtmWorldToBase(3, 3) == 0 ? this->htmWorldToBase : customHtmWorldToBase;
  htmWorldToDH0 = htmWorldToDH0 * this->htmBaseToDH0;

  for (int i = 0; i < n; i++)
  {
    p_i = fkres.htmDH[i].block<3, 1>(0, 3);
    for (int j = 0; j <= i; j++)
    {
      p_j_ant = (j == 0) ? htmWorldToDH0.block<3, 1>(0, 3) : fkres.htmDH[j - 1].block<3, 1>(0, 3);
      z_j_ant = (j == 0) ? htmWorldToDH0.block<3, 1>(0, 2) : fkres.htmDH[j - 1].block<3, 1>(0, 2);

      v = (this->links[j].jointType == 0) ? Utils::cross(z_j_ant, p_i - p_j_ant) : z_j_ant;
      w = (this->links[j].jointType == 0) ? z_j_ant : Vector3d::Zero();

      listOfJacs[i](0, j) = v(0);
      listOfJacs[i](1, j) = v(1);
      listOfJacs[i](2, j) = v(2);
      listOfJacs[i](3, j) = w(0);
      listOfJacs[i](4, j) = w(1);
      listOfJacs[i](5, j) = w(2);
    }
  }

  fkres.jacDH = listOfJacs;

  Vector3d p_ef = fkres.htmTool.block<3, 1>(0, 3);
  Vector3d p_n = fkres.htmDH[n - 1].block<3, 1>(0, 3);
  MatrixXd Jv = fkres.jacDH[n - 1].block(0, 0, 3, n);
  MatrixXd Jw = fkres.jacDH[n - 1].block(3, 0, 3, n);
  MatrixXd JvTool = Jv - Utils::S(p_ef - p_n) * Jw;
  MatrixXd jacGeoTool(6, n);
  jacGeoTool << JvTool, Jw;

  fkres.jacTool = jacGeoTool;

  return fkres;
}

TaskResult Manipulator::taskFunction(Matrix4d taskHtm, VectorXd q, Matrix4d customHtmWorldToBase)
{
  int n = this->noJoints;
  FKResult fkres = this->jacGeo(q, customHtmWorldToBase);

  Vector3d p_ef = fkres.htmTool.block<3, 1>(0, 3);
  Vector3d x_ef = fkres.htmTool.block<3, 1>(0, 0);
  Vector3d y_ef = fkres.htmTool.block<3, 1>(0, 1);
  Vector3d z_ef = fkres.htmTool.block<3, 1>(0, 2);

  Vector3d p_des = taskHtm.block<3, 1>(0, 3);
  Vector3d x_des = taskHtm.block<3, 1>(0, 0);
  Vector3d y_des = taskHtm.block<3, 1>(0, 1);
  Vector3d z_des = taskHtm.block<3, 1>(0, 2);

  Vector3d r_p = p_ef - p_des;
  Vector3d r_o;
  double e_x = 1 - x_des.transpose() * x_ef;
  double e_y = 1 - y_des.transpose() * y_ef;
  double e_z = 1 - z_des.transpose() * z_ef;
  r_o << e_x, e_y, e_z;

  VectorXd r(6, 1);
  r << r_p, r_o;

  MatrixXd jacr_p = fkres.jacTool.block(0, 0, 3, n);
  MatrixXd Jw = fkres.jacDH[n - 1].block(3, 0, 3, n);
  MatrixXd jacr_o_x = x_des.transpose() * Utils::S(x_ef) * Jw;
  MatrixXd jacr_o_y = y_des.transpose() * Utils::S(y_ef) * Jw;
  MatrixXd jacr_o_z = z_des.transpose() * Utils::S(z_ef) * Jw;

  MatrixXd jacr(6, n);
  jacr << jacr_p, jacr_o_x, jacr_o_y, jacr_o_z;

  double maxErrorPos = max(abs(r[0]), max(abs(r[1]), abs(r[2])));
  double fc = 180 / M_PI;
  double maxErrorOri = fc * max(acos(1 - r[3]), max(acos(1 - r[4]), acos(1 - r[5])));

  TaskResult taskResult;

  taskResult.task = r;
  taskResult.jacTask = jacr;
  taskResult.maxErrorPos = maxErrorPos;
  taskResult.maxErrorOri = maxErrorOri;

  return taskResult;
}

VectorXd Manipulator::ik(Matrix4d desHtm, VectorXd q0, double pTol, double aTol, int noIterMax)
{
  int n = this->noJoints;

  VectorXd q = q0.rows() == 0 ? Utils::randVec(n, -M_PI, M_PI) : q0;

  bool continueLoop = true;
  TaskResult tr;
  double dt = 0.05;
  double error_pos, error_ori;

  int iter = 0;
  int localIter = 0;

  while (continueLoop)
  {
    tr = taskFunction(desHtm, q);
    MatrixXd H = Utils::pinv(MatrixXd::Ones(3, 6), 0.005);
    q = q - dt * (Utils::pinv(tr.jacTask, 0.005) * Utils::vecPow(tr.task, 0, 6, 0.5));

    error_pos = tr.task.block(0, 0, 3, 1).norm();
    error_ori = 0;
    for (int i = 0; i < 3; i++)
    {
      error_ori = max(error_ori, (180 / M_PI) * acos(1 - tr.task[i + 3]));
    }

    continueLoop = !(error_pos <= pTol && error_ori <= aTol) && (iter <= noIterMax);
    iter++;

    if (!(error_pos <= pTol && error_ori <= aTol) && localIter >= 500)
    {
      q = Utils::randVec(n, -M_PI, M_PI);
      localIter = 0;
    }
  }

  // cout << iter << std::endl;
  return q;
}

DistanceRobotObjResult Manipulator::computeDistToObj(GeometricPrimitives *obj, VectorXd q,
                                                     Matrix4d customHtmWorldToBase,
                                                     DistanceRobotObjResult *oldDistStruct, double tol, double h,
                                                     double maxDist)
{
  DistanceRobotObjResult drs(this->noJoints);

  FKResult fkres = this->jacGeo(q, customHtmWorldToBase);

  // Prepare the list of objects with the appropriate pose
  vector<vector<GeometricPrimitives *>> listObjsCopy;

  for (int i = 0; i < this->noJoints; i++)
  {
    vector<GeometricPrimitives *> tempList;
    for (int j = 0; j < this->links[i].colObjs.size(); j++)
    {
      GeometricPrimitives *colObjCopied = (*this->links[i].colObjs[j]).copy();
      (*colObjCopied).htm = fkres.htmDH[i] * this->links[i].htmCols[j];
      tempList.push_back(colObjCopied);
    }
    listObjsCopy.push_back(tempList);
  }

  // Compute all the distances
  Vector3d pointObj;
  DistanceLinkObjResult *dlo;

  for (int i = 0; i < this->noJoints; i++)
  {
    for (int j = 0; j < this->links[i].colObjs.size(); j++)
    {
      if (maxDist >= 10000 || Utils::distanceAABB(obj, listObjsCopy[i][j]) <= maxDist)
      {
        if (oldDistStruct != NULL)
        {
          dlo = (*oldDistStruct).getItem(i, j);
          if (dlo != NULL)
            pointObj = (*dlo).witnessObj;
          else
            pointObj = Vector3d::Random();
        }
        else
          pointObj = Vector3d::Random();

        DistanceStruct ds = GeometricPrimitives::computeDist(obj, listObjsCopy[i][j], pointObj, h, tol);

        DistanceLinkObjResult dloNew;
        dloNew.colObjLinkNumber = j;
        dloNew.linkNumber = i;
        dloNew.distance = ds.distance;
        dloNew.witnessObj = ds.witnessA;
        dloNew.witnessColObjLink = ds.witnessB;

        MatrixXd jacWitnessColObjLink(3, this->noJoints);
        MatrixXd Jv = fkres.jacDH[i].block(0, 0, 3, this->noJoints);
        MatrixXd Jw = fkres.jacDH[i].block(3, 0, 3, this->noJoints);
        Vector3d pointCenter = fkres.htmDH[i].block<3, 1>(0, 3);
        jacWitnessColObjLink = Jv - Utils::S(dloNew.witnessColObjLink - pointCenter) * Jw;

        dloNew.jacDist = jacWitnessColObjLink.transpose() * (dloNew.witnessColObjLink - dloNew.witnessObj) /
                         (0.000001 + dloNew.distance);

        drs.addDistanceLinkObj(dloNew);
      }
    }
  }

  return drs;
}

DistanceRobotAutoResult Manipulator::computeDistAuto(VectorXd q, Matrix4d customHtmWorldToBase,
                                                     DistanceRobotAutoResult *oldDistStruct, double tol, double h,
                                                     double maxDist)
{
  DistanceRobotAutoResult drs(this->noJoints);

  FKResult fkres = this->jacGeo(q, customHtmWorldToBase);

  // Prepare the list of objects with the appropriate pose
  vector<vector<GeometricPrimitives *>> listObjsCopy;

  for (int i = 0; i < this->noJoints; i++)
  {
    vector<GeometricPrimitives *> tempList;
    for (int j = 0; j < this->links[i].colObjs.size(); j++)
    {
      GeometricPrimitives *colObjCopied = (*this->links[i].colObjs[j]).copy();
      (*colObjCopied).htm = fkres.htmDH[i] * this->links[i].htmCols[j];
      tempList.push_back(colObjCopied);
    }
    listObjsCopy.push_back(tempList);
  }

  // Create a list of all possible distance computations
  vector<int> indLink1, indLink2, indObjCol1, indObjCol2;

  for (int i = 0; i < this->noJoints; i++)
    for (int j = i + 2; j < this->noJoints; j++)
      for (int ii = 0; ii < this->links[i].colObjs.size(); ii++)
        for (int jj = 0; jj < this->links[j].colObjs.size(); jj++)
        {
          indLink1.push_back(i);
          indLink2.push_back(j);
          indObjCol1.push_back(ii);
          indObjCol2.push_back(jj);
        }

  // Compute all the distances
  Vector3d pointColObj1;
  DistanceLinkLinkResult *dll;
  int ind1, ind2, indCol1, indCol2;

  for (int k = 0; k < indLink1.size(); k++)
  {
    ind1 = indLink1[k];
    ind2 = indLink2[k];
    indCol1 = indObjCol1[k];
    indCol2 = indObjCol2[k];

    if (maxDist >= 10000 || Utils::distanceAABB(listObjsCopy[ind1][indCol1], listObjsCopy[ind2][indCol2]) <= maxDist)
    {
      if (oldDistStruct != NULL)
      {
        dll = (*oldDistStruct).getItem(ind1, indCol1, ind2, indCol2);
        if (dll != NULL)
          pointColObj1 = (*dll).witnessColObjLink1;
        else
          pointColObj1 = Vector3d::Random();
      }
      else
        pointColObj1 = Vector3d::Random();

      DistanceStruct ds = GeometricPrimitives::computeDist(listObjsCopy[ind1][indCol1], listObjsCopy[ind2][indCol2],
                                                           pointColObj1, h, tol);

      DistanceLinkLinkResult dllNew;

      dllNew.linkNumber1 = ind1;
      dllNew.colObjLinkNumber1 = indCol1;
      dllNew.linkNumber2 = ind2;
      dllNew.colObjLinkNumber2 = indCol2;
      dllNew.witnessColObjLink1 = ds.witnessA;
      dllNew.witnessColObjLink2 = ds.witnessB;
      dllNew.distance = ds.distance;

      MatrixXd jacWitnessColObjLink1(3, this->noJoints);
      MatrixXd jacWitnessColObjLink2(3, this->noJoints);
      MatrixXd Jv1 = fkres.jacDH[ind1].block(0, 0, 3, this->noJoints);
      MatrixXd Jw1 = fkres.jacDH[ind1].block(3, 0, 3, this->noJoints);
      MatrixXd Jv2 = fkres.jacDH[ind2].block(0, 0, 3, this->noJoints);
      MatrixXd Jw2 = fkres.jacDH[ind2].block(3, 0, 3, this->noJoints);

      Vector3d pointCenter1 = fkres.htmDH[ind1].block<3, 1>(0, 3);
      Vector3d pointCenter2 = fkres.htmDH[ind2].block<3, 1>(0, 3);

      jacWitnessColObjLink1 = Jv1 - Utils::S(dllNew.witnessColObjLink1 - pointCenter1) * Jw1;
      jacWitnessColObjLink2 = Jv2 - Utils::S(dllNew.witnessColObjLink2 - pointCenter2) * Jw2;

      dllNew.jacDist = (jacWitnessColObjLink1 - jacWitnessColObjLink2).transpose() *
                       (dllNew.witnessColObjLink1 - dllNew.witnessColObjLink2) / (0.000001 + dllNew.distance);

      drs.addDistanceLinkLink(dllNew);
    }
  }

  return drs;
}

ConstControlResult Manipulator::velocityConstControl(VectorXd q, VelocityConstControlParam param)
{
  ConstControlResult ccr;

  auto start = std::chrono::high_resolution_clock::now();

  MatrixXd A(0, this->noJoints);
  VectorXd b(0), btemp;
  MatrixXd I = MatrixXd::Identity(this->noJoints, this->noJoints);

  // Create all constraints with collisions with the obstacles
  DistanceRobotObjResult dlo, dloOld;
  for (int i = 0; i < param.obstacles.size(); i++)
  {
    dloOld = (param.oldControlStruct == NULL) ? NULL : param.oldControlStruct->distancesObjResult[i];
    dlo = this->computeDistToObj(param.obstacles[i], q, param.customHtmWorldToBase, &dloOld, param.tol, param.h,
                                 param.maxDistAABB);

    ccr.distancesObjResult.push_back(dlo);

    A = Utils::matrixVertStack(A, dlo.getAllJacobians());

    btemp = dlo.getDistances();
    for (int j = 0; j < btemp.rows(); j++)
      btemp[j] = -param.etaObs * (btemp[j] - param.distSafeObs);

    b = Utils::vectorVertStack(b, btemp);
  }

  // Create all constraints with auto collisions
  if (param.considerAutoCollision)
  {
    DistanceRobotAutoResult dra, draOld;
    draOld = (param.oldControlStruct == NULL) ? NULL : param.oldControlStruct->distanceAutoResult;
    dra = this->computeDistAuto(q, param.customHtmWorldToBase, &draOld, param.tol, param.h, param.maxDistAABB);

    ccr.distanceAutoResult = dra;

    A = Utils::matrixVertStack(A, dra.getAllJacobians());

    btemp = dra.getDistances();
    for (int j = 0; j < btemp.rows(); j++)
      btemp[j] = -param.etaAuto * (btemp[j] - param.distSafeAuto);

    b = Utils::vectorVertStack(b, btemp);
  }

  // Create all joint limit constraints
  if (param.considerJointLimits)
  {
    VectorXd q_int = q.rows() == 0 ? this->q : q;

    A = Utils::matrixVertStack(A, I);

    btemp = VectorXd::Zero(this->noJoints);

    for (int i = 0; i < this->noJoints; i++)
      btemp[i] = -param.etaJoint * (q_int[i] - this->qMin[i] - param.distSafeJoint);

    b = Utils::vectorVertStack(b, btemp);

    A = Utils::matrixVertStack(A, -I);

    btemp = VectorXd::Zero(this->noJoints);

    for (int i = 0; i < this->noJoints; i++)
      btemp[i] = -param.etaJoint * (this->qMax[i] - q_int[i] - param.distSafeJoint);

    b = Utils::vectorVertStack(b, btemp);
  }

  // Create all action limit constraints
  if (param.considerActionLimits)
  {
    A = Utils::matrixVertStack(A, I);

    btemp = VectorXd::Zero(this->noJoints);

    for (int i = 0; i < this->noJoints; i++)
      btemp[i] = this->qDotMin[i];

    b = Utils::vectorVertStack(b, btemp);

    A = Utils::matrixVertStack(A, -I);

    btemp = VectorXd::Zero(this->noJoints);

    for (int i = 0; i < this->noJoints; i++)
      btemp[i] = -this->qDotMax[i];

    b = Utils::vectorVertStack(b, btemp);
  }

  // Create H and f matrices
  TaskResult tr = this->taskFunction(param.taskHtm, q, param.customHtmWorldToBase);

  MatrixXd H = 2 * (tr.jacTask.transpose() * tr.jacTask + param.eps * I);

  VectorXd modTask(6);
  modTask[0] = param.kpos * tr.task[0];
  modTask[1] = param.kpos * tr.task[1];
  modTask[2] = param.kpos * tr.task[2];
  modTask[3] = param.kori * sqrt(tr.task[3]);
  modTask[4] = param.kori * sqrt(tr.task[4]);
  modTask[5] = param.kori * sqrt(tr.task[5]);

  VectorXd f = 2 * (tr.jacTask.transpose() * modTask);

  ccr.A = A;
  ccr.b = b;
  ccr.H = H;
  ccr.f = f;

  // Solve the optimization problem
  VectorXd u = Utils::solveQP(H, f, A, b);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  ccr.action = u;
  ccr.milisecondsSpent = duration.count() / 1000.0;
  ccr.taskResult = tr;
  ccr.feasible = u.rows() > 0;

  return ccr;
}

ConstControlResult Manipulator::accelerationConstControl(VectorXd qdot, VectorXd q, AccelerationConstControlParam param)
{
  ConstControlResult ccr;

  auto start = std::chrono::high_resolution_clock::now();

  MatrixXd A(0, this->noJoints);
  VectorXd b(0), btemp;
  MatrixXd I = MatrixXd::Identity(this->noJoints, this->noJoints);

  // Create all constraints with collisions with the obstacles
  DistanceRobotObjResult dlo, dloOld, dloNext;

  for (int i = 0; i < param.obstacles.size(); i++)
  {
    dloOld = (param.oldControlStruct == NULL) ? NULL : param.oldControlStruct->distancesObjResult[i];
    dlo = this->computeDistToObj(param.obstacles[i], q, param.customHtmWorldToBase, &dloOld, param.tol, param.h,
                                 param.maxDistAABB);
    dloNext = this->computeDistToObj(param.obstacles[i], q + qdot * param.dt, param.customHtmWorldToBase, &dlo,
                                     param.tol, param.h, param.maxDistAABB);

    ccr.distancesObjResult.push_back(dlo);

    for (int j = 0; j < dlo.noElements; j++)
    {
      // Try to find the respective element in dloNext.

      DistanceLinkObjResult *dlorNext = dloNext.getItem(dlo[j].linkNumber, dlo[j].colObjLinkNumber);
      if (dlorNext != NULL)
      {
        double dist = dlo[j].distance - param.distSafeObs;
        double distDot = (dlorNext->distance - dlo[j].distance) / param.dt;
        VectorXd gradDot = (dlorNext->jacDist - dlo[j].jacDist) / param.dt;
        double coriolisCentrifugal = (gradDot.transpose() * qdot)[0];
        double K = param.etaObs;

        A = Utils::matrixVertStack(A, dlo[j].jacDist.transpose());
        MatrixXd bn(1, 1);
        bn << -coriolisCentrifugal - 2 * K * distDot - K * K * dist;
        b = Utils::matrixVertStack(b, bn);
      }
    }
  }

  // Create all constraints with auto collisions

  if (param.considerAutoCollision)
  {
    DistanceRobotAutoResult dra, draOld, draNext;

    draOld = (param.oldControlStruct == NULL) ? NULL : param.oldControlStruct->distanceAutoResult;
    dra = this->computeDistAuto(q, param.customHtmWorldToBase, &draOld, param.tol, param.h, param.maxDistAABB);
    draNext = this->computeDistAuto(q + qdot * param.dt, param.customHtmWorldToBase, &dra, param.tol, param.h,
                                    param.maxDistAABB);

    ccr.distanceAutoResult = dra;

    for (int j = 0; j < dra.noElements; j++)
    {
      // Try to find the respective element in draNext.

      DistanceLinkLinkResult *dllrNext =
          draNext.getItem(dra[j].linkNumber1, dra[j].colObjLinkNumber1, dra[j].linkNumber2, dra[j].colObjLinkNumber2);
      if (dllrNext != NULL)
      {
        double dist = dra[j].distance - param.distSafeAuto;
        double distDot = (dllrNext->distance - dra[j].distance) / param.dt;
        VectorXd gradDot = (dllrNext->jacDist - dra[j].jacDist) / param.dt;
        double coriolisCentrifugal = (gradDot.transpose() * qdot)[0];
        double K = param.etaAuto;

        A = Utils::matrixVertStack(A, dra[j].jacDist.transpose());
        MatrixXd bn(1, 1);
        bn << -coriolisCentrifugal - 2 * K * distDot - K * K * dist;
        b = Utils::matrixVertStack(b, bn);
      }
    }
  }

  // Create all joint position limit  constraints
  if (param.considerJointPositionLimits)
  {
    double K = param.etaJointPosition;
    VectorXd q_int = q.rows() == 0 ? this->q : q;

    A = Utils::matrixVertStack(A, I);

    btemp = VectorXd::Zero(this->noJoints);

    for (int i = 0; i < this->noJoints; i++)
      btemp[i] = -2 * K * qdot[i] - K * K * (q_int[i] - this->qMin[i] - param.distSafeJoint);

    b = Utils::vectorVertStack(b, btemp);

    A = Utils::matrixVertStack(A, -I);

    btemp = VectorXd::Zero(this->noJoints);

    for (int i = 0; i < this->noJoints; i++)
      btemp[i] = 2 * K * qdot[i] - K * K * (this->qMax[i] - q_int[i] - param.distSafeJoint);

    b = Utils::vectorVertStack(b, btemp);
  }

  // Create all joint velocity limit  constraints
  if (param.considerJointSpeedLimits)
  {
    double K = param.etaJointVelocity;

    A = Utils::matrixVertStack(A, I);

    btemp = VectorXd::Zero(this->noJoints);

    for (int i = 0; i < this->noJoints; i++)
      btemp[i] = -K * (qdot[i] - this->qDotMin[i]);

    b = Utils::vectorVertStack(b, btemp);

    A = Utils::matrixVertStack(A, -I);

    btemp = VectorXd::Zero(this->noJoints);

    for (int i = 0; i < this->noJoints; i++)
      btemp[i] = -K * (this->qDotMax[i] - qdot[i]);

    b = Utils::vectorVertStack(b, btemp);
  }

  // Create H and f matrices
  TaskResult tr = this->taskFunction(param.taskHtm, q, param.customHtmWorldToBase);
  TaskResult trNext = this->taskFunction(param.taskHtm, q + qdot * param.dt, param.customHtmWorldToBase);

  MatrixXd H = 2 * (tr.jacTask.transpose() * tr.jacTask + param.beta * I);

  VectorXd coriolisCentrifugal = ((trNext.jacTask - tr.jacTask) / (param.dt)) * qdot;

  VectorXd rdot = (trNext.task - tr.task) / (param.dt);

  double K = param.kconv;

  VectorXd f =
      2 * (tr.jacTask.transpose() * (coriolisCentrifugal + 2 * K * rdot + K * K * tr.task) + param.beta * qdot);

  ccr.A = A;
  ccr.b = b;
  ccr.H = H;
  ccr.f = f;

  // Solve the optimization problem
  VectorXd u = Utils::solveQP(H, f, A, b);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

  ccr.action = u;
  ccr.milisecondsSpent = duration.count() / 1000.0;
  ccr.taskResult = tr;
  ccr.feasible = u.rows() > 0;

  return ccr;
}

FreeConfigResult Manipulator::checkFreeConfig(VectorXd q, FreeConfigParam param)
{
  FreeConfigResult fcr;

  VectorXd q_int = q.rows() == 0 ? this->q : q;
  Matrix4d htmNow = param.customHtmWorldToBase(3, 3) == 0 ? this->htmWorldToBase : param.customHtmWorldToBase;
  htmNow = htmNow * this->htmBaseToDH0;

  int i = 0;
  int j = 0;
  int k = 0;

  // Consider joint limits, if needed
  if (param.considerJointLimits)
  {
    // Check lower limits
    i = 0;
    while (fcr.isFree && i < this->noJoints)
    {
      fcr.isFree = q_int[i] >= this->qMin[i] + param.distSafeJoint;
      if (!fcr.isFree)
      {
        fcr.errorType = FreeConfigResult::ErrorType::lowerJointLimit;
        fcr.jointNumber = i;
      }
      i++;
    }

    // Check upper limits
    i = 0;
    while (fcr.isFree && i < this->noJoints)
    {
      fcr.isFree = q_int[i] <= this->qMax[i] - param.distSafeJoint;
      if (!fcr.isFree)
      {
        fcr.errorType = FreeConfigResult::ErrorType::upperJointLimit;
        fcr.jointNumber = i;
      }
      i++;
    }
  }

  // Prepare for collision or auto collision detection
  vector<vector<GeometricPrimitives *>> listObjsCopy;
  if (fcr.isFree && (param.considerAutoCollision || param.obstacles.size() > 0))
  {
    FKResult fkres = this->jacGeo(q, param.customHtmWorldToBase);

    for (i = 0; i < this->noJoints; i++)
    {
      vector<GeometricPrimitives *> tempList;
      for (j = 0; j < this->links[i].colObjs.size(); j++)
      {
        GeometricPrimitives *colObjCopied = (*this->links[i].colObjs[j]).copy();
        (*colObjCopied).htm = fkres.htmDH[i] * this->links[i].htmCols[j];
        tempList.push_back(colObjCopied);
      }
      listObjsCopy.push_back(tempList);
    }
  }

  // Check auto collision, if needed
  if (fcr.isFree && param.considerAutoCollision)
  {
    // Create a list of all possible distance computations
    vector<int> indLink1, indLink2, indObjCol1, indObjCol2;

    for (i = 0; i < this->noJoints; i++)
      for (j = i + 2; j < this->noJoints; j++)
        for (int ii = 0; ii < this->links[i].colObjs.size(); ii++)
          for (int jj = 0; jj < this->links[j].colObjs.size(); jj++)
          {
            indLink1.push_back(i);
            indLink2.push_back(j);
            indObjCol1.push_back(ii);
            indObjCol2.push_back(jj);
          }

    // Compute the distances
    int ind1, ind2, indCol1, indCol2;
    k = 0;

    while (fcr.isFree && k < indLink1.size())
    {
      ind1 = indLink1[k];
      ind2 = indLink2[k];
      indCol1 = indObjCol1[k];
      indCol2 = indObjCol2[k];

      if (param.maxDistAABB >= 10000 ||
          Utils::distanceAABB(listObjsCopy[ind1][indCol1], listObjsCopy[ind2][indCol2]) <= param.maxDistAABB)
      {
        DistanceStruct ds = GeometricPrimitives::computeDist(listObjsCopy[ind1][indCol1], listObjsCopy[ind2][indCol2],
                                                             Vector3d::Random(), param.h, param.tol);

        fcr.isFree = ds.distance > param.distSafeAuto;

        if (!fcr.isFree)
        {
          fcr.errorType = FreeConfigResult::ErrorType::autoCollision;
          fcr.linkNumber1 = ind1;
          fcr.linkNumber2 = ind2;
          fcr.colObjNumber1 = indCol1;
          fcr.colObjNumber2 = indCol2;
        }
      }
      k++;
    }
  }

  // Check collision with obstacles
  i = 0;

  while (fcr.isFree && i < this->noJoints)
  {
    j = 0;
    while (fcr.isFree && j < this->links[i].colObjs.size())
    {
      k = 0;
      while (fcr.isFree && k < param.obstacles.size())
      {
        // Check if the combination of link i and obstacle k is not on the ignore list
        bool ignoreCol = false;
        for (int s = 0; s < param.ignoreCol.size(); s++)
          ignoreCol = ignoreCol || ((param.ignoreCol[s].linkNo == i) && (param.ignoreCol[s].obsNo == k));

        if ((!ignoreCol) && (param.maxDistAABB >= 10000 ||
                             Utils::distanceAABB(param.obstacles[k], listObjsCopy[i][j]) <= param.maxDistAABB))
        {
          DistanceStruct ds = GeometricPrimitives::computeDist(param.obstacles[k], listObjsCopy[i][j],
                                                               Vector3d::Random(), param.h, param.tol);
          fcr.isFree = ds.distance > param.distSafeObs;

          if (!fcr.isFree)
          {
            fcr.errorType = FreeConfigResult::ErrorType::obstacleCollision;
            fcr.linkNumber1 = i;
            fcr.colObjNumber1 = j;
            fcr.obstacleNumber = k;
          }
        }

        k++;
      }
      j++;
    }
    i++;
  }

  return fcr;
}

Manipulator Manipulator::createKukaIIWA_old()
{
  // Create empty manipulator
  Manipulator manip(7);

  // Create links
  double d_bs = 0.360;
  double d_se = 0.420;
  double d_ew = 0.400;
  double d_wf = 0.152 + 0.4; // 0.609

  vector<Link> linksKuka;

  Link link1(1, 0, d_bs, -M_PI / 2, 0, 0, "iiwa_joint_1");
  Link link2(2, 0, 0, M_PI / 2, 0, 0, "iiwa_joint_2");
  Link link3(3, 0, d_se, M_PI / 2, 0, 0, "iiwa_joint_3");
  Link link4(4, 0, 0, -M_PI / 2, 0, 0, "iiwa_joint_4");
  Link link5(5, 0, d_ew, -M_PI / 2, 0, 0, "iiwa_joint_5");
  Link link6(6, 0, 0, M_PI / 2, 0, 0, "iiwa_joint_6");
  Link link7(7, 0, d_wf, 0, 0, 0, "iiwa_joint_7");

  linksKuka.push_back(link1);
  linksKuka.push_back(link2);
  linksKuka.push_back(link3);
  linksKuka.push_back(link4);
  linksKuka.push_back(link5);
  linksKuka.push_back(link6);
  linksKuka.push_back(link7);

  manip.links = linksKuka;

  // Create collision objects
  FKResult fkres0 = manip.fk();

  double dz = -0.02;

  // For the first link
  Cylinder *iiwaC0_0 = new Cylinder(Utils::trn(0, 0, 0.15 + dz), 0.1, 0.12);
  manip.links[0].colObjs.push_back(iiwaC0_0);
  manip.links[0].htmCols.push_back(fkres0.htmDH[0].inverse() * (*iiwaC0_0).htm);
  manip.links[0].nameObjs.push_back("c0_0");

  Sphere *iiwaC0_1 = new Sphere(Utils::trn(0.01, 0, 0.33 + dz), 0.12);
  manip.links[0].colObjs.push_back(iiwaC0_1);
  manip.links[0].htmCols.push_back(fkres0.htmDH[0].inverse() * (*iiwaC0_1).htm);
  manip.links[0].nameObjs.push_back("c0_1");

  // For the second link
  Sphere *iiwaC1_0 = new Sphere(Utils::trn(0, 0.03, 0.38 + dz), 0.1);
  manip.links[1].colObjs.push_back(iiwaC1_0);
  manip.links[1].htmCols.push_back(fkres0.htmDH[1].inverse() * (*iiwaC1_0).htm);
  manip.links[1].nameObjs.push_back("c1_0");

  Cylinder *iiwaC1_1 = new Cylinder(Utils::trn(0, 0, 0.54 + dz), 0.075, 0.125);
  manip.links[1].colObjs.push_back(iiwaC1_1);
  manip.links[1].htmCols.push_back(fkres0.htmDH[1].inverse() * (*iiwaC1_1).htm);
  manip.links[1].nameObjs.push_back("c1_1");

  // For the third link
  Cylinder *iiwaC2_0 = new Cylinder(Utils::trn(0, 0, 0.66 + 2 * dz), 0.08, 0.1);
  manip.links[2].colObjs.push_back(iiwaC2_0);
  manip.links[2].htmCols.push_back(fkres0.htmDH[2].inverse() * (*iiwaC2_0).htm);
  manip.links[2].nameObjs.push_back("c2_0");

  Sphere *iiwaC2_1 = new Sphere(Utils::trn(0, 0.01, 0.75 + 2 * dz), 0.11);
  manip.links[2].colObjs.push_back(iiwaC2_1);
  manip.links[2].htmCols.push_back(fkres0.htmDH[2].inverse() * (*iiwaC2_1).htm);
  manip.links[2].nameObjs.push_back("c2_1");

  // For the fourth link
  Sphere *iiwaC3_0 = new Sphere(Utils::trn(0, -0.035, 0.803 + 2 * dz), 0.08);
  manip.links[3].colObjs.push_back(iiwaC3_0);
  manip.links[3].htmCols.push_back(fkres0.htmDH[3].inverse() * (*iiwaC3_0).htm);
  manip.links[3].nameObjs.push_back("c3_0");

  Cylinder *iiwaC3_1 = new Cylinder(Utils::trn(0, 0, 0.935 + 2 * dz), 0.075, 0.125);
  manip.links[3].colObjs.push_back(iiwaC3_1);
  manip.links[3].htmCols.push_back(fkres0.htmDH[3].inverse() * (*iiwaC3_1).htm);
  manip.links[3].nameObjs.push_back("c3_1");

  // For the fifth link
  Cylinder *iiwaC4_0 = new Cylinder(Utils::trn(0, 0, 1.052 + 2 * dz), 0.075, 0.11);
  manip.links[4].colObjs.push_back(iiwaC4_0);
  manip.links[4].htmCols.push_back(fkres0.htmDH[4].inverse() * (*iiwaC4_0).htm);
  manip.links[4].nameObjs.push_back("c4_0");

  Box *iiwaC4_1 = new Box(Utils::trn(0, -0.08, 1.15 + 2 * dz), 0.12, 0.04, 0.17);
  manip.links[4].colObjs.push_back(iiwaC4_1);
  manip.links[4].htmCols.push_back(fkres0.htmDH[4].inverse() * (*iiwaC4_1).htm);
  manip.links[4].nameObjs.push_back("c4_1");

  // For the sixth link
  Cylinder *iiwaC5_0 = new Cylinder(Utils::trn(0, 0, 1.203 + 2 * dz), 0.075, 0.24);
  manip.links[5].colObjs.push_back(iiwaC5_0);
  manip.links[5].htmCols.push_back(fkres0.htmDH[5].inverse() * (*iiwaC5_0).htm);
  manip.links[5].nameObjs.push_back("c5_0");

  // Create joint limits (in rad)
  double dq = 0.087;

  VectorXd qMin(7);
  qMin << -2.967055 + dq, -2.09435 + dq, -2.967055 + dq, -2.09435 + dq, -2.937055 + dq, -2.09435 + dq, -3.054325 + dq;

  VectorXd qMax(7);
  qMax << 2.967055 - dq, 2.09435 - dq, 2.967055 - dq, 2.09435 - dq, 2.937055 - dq, 2.09435 - dq, 3.054325 - dq;

  manip.qMin = qMin;
  manip.qMax = qMax;

  // Create joint velocity limits (in rad/s)
  VectorXd qDotMin(7);
  qDotMin << -1.482, -1.482, -1.740, -1.307, -2.268, -2.355, -2.356;

  VectorXd qDotMax(7);
  qDotMax << 1.482, 1.482, 1.740, 1.307, 2.268, 2.355, 2.356;

  manip.qDotMin = qDotMin;
  manip.qDotMax = qDotMax;

  return manip;
}

Manipulator Manipulator::createKukaIIWAWithTool_old()
{
  // Create empty manipulator
  Manipulator manip(7);

  // Create links
  double d_bs = 0.360;
  double d_se = 0.420;
  double d_ew = 0.400;
  double d_wf = 0.152 + 0.4;

  vector<Link> linksKuka;

  Link link1(1, 0, d_bs, -M_PI / 2, 0, 0, "iiwa_joint_1");
  Link link2(2, 0, 0, M_PI / 2, 0, 0, "iiwa_joint_2");
  Link link3(3, 0, d_se, M_PI / 2, 0, 0, "iiwa_joint_3");
  Link link4(4, 0, 0, -M_PI / 2, 0, 0, "iiwa_joint_4");
  Link link5(5, 0, d_ew, -M_PI / 2, 0, 0, "iiwa_joint_5");
  Link link6(6, 0, 0, M_PI / 2, 0, 0, "iiwa_joint_6");
  Link link7(7, 0, d_wf, 0, 0, 0, "iiwa_joint_7");

  linksKuka.push_back(link1);
  linksKuka.push_back(link2);
  linksKuka.push_back(link3);
  linksKuka.push_back(link4);
  linksKuka.push_back(link5);
  linksKuka.push_back(link6);
  linksKuka.push_back(link7);

  manip.links = linksKuka;

  // Create collision objects
  FKResult fkres0 = manip.fk();

  double dz = -0.02;

  // For the first link
  Cylinder *iiwaC0_0 = new Cylinder(Utils::trn(0, 0, 0.15 + dz), 0.1, 0.12);
  manip.links[0].colObjs.push_back(iiwaC0_0);
  manip.links[0].htmCols.push_back(fkres0.htmDH[0].inverse() * (*iiwaC0_0).htm);
  manip.links[0].nameObjs.push_back("c0_0");

  Sphere *iiwaC0_1 = new Sphere(Utils::trn(0.01, 0, 0.33 + dz), 0.125);
  manip.links[0].colObjs.push_back(iiwaC0_1);
  manip.links[0].htmCols.push_back(fkres0.htmDH[0].inverse() * (*iiwaC0_1).htm);
  manip.links[0].nameObjs.push_back("c0_1");

  // For the second link
  Sphere *iiwaC1_0 = new Sphere(Utils::trn(0, 0.03, 0.38 + dz), 0.11);
  manip.links[1].colObjs.push_back(iiwaC1_0);
  manip.links[1].htmCols.push_back(fkres0.htmDH[1].inverse() * (*iiwaC1_0).htm);
  manip.links[1].nameObjs.push_back("c1_0");

  Cylinder *iiwaC1_1 = new Cylinder(Utils::trn(0, 0, 0.54 + dz), 0.08, 0.125);
  manip.links[1].colObjs.push_back(iiwaC1_1);
  manip.links[1].htmCols.push_back(fkres0.htmDH[1].inverse() * (*iiwaC1_1).htm);
  manip.links[1].nameObjs.push_back("c1_1");

  // For the third link
  Cylinder *iiwaC2_0 = new Cylinder(Utils::trn(0, 0, 0.66 + 2 * dz), 0.08, 0.1);
  manip.links[2].colObjs.push_back(iiwaC2_0);
  manip.links[2].htmCols.push_back(fkres0.htmDH[2].inverse() * (*iiwaC2_0).htm);
  manip.links[2].nameObjs.push_back("c2_0");

  Sphere *iiwaC2_1 = new Sphere(Utils::trn(0, 0.01, 0.75 + 2 * dz), 0.11);
  manip.links[2].colObjs.push_back(iiwaC2_1);
  manip.links[2].htmCols.push_back(fkres0.htmDH[2].inverse() * (*iiwaC2_1).htm);
  manip.links[2].nameObjs.push_back("c2_1");

  // For the fourth link
  Sphere *iiwaC3_0 = new Sphere(Utils::trn(0, -0.035, 0.803 + 2 * dz), 0.09);
  manip.links[3].colObjs.push_back(iiwaC3_0);
  manip.links[3].htmCols.push_back(fkres0.htmDH[3].inverse() * (*iiwaC3_0).htm);
  manip.links[3].nameObjs.push_back("c3_0");

  Cylinder *iiwaC3_1 = new Cylinder(Utils::trn(0, 0, 0.935 + 2 * dz), 0.075, 0.125);
  manip.links[3].colObjs.push_back(iiwaC3_1);
  manip.links[3].htmCols.push_back(fkres0.htmDH[3].inverse() * (*iiwaC3_1).htm);
  manip.links[3].nameObjs.push_back("c3_1");

  // For the fifth link
  Cylinder *iiwaC4_0 = new Cylinder(Utils::trn(0, 0, 1.052 + 2 * dz), 0.075, 0.11);
  manip.links[4].colObjs.push_back(iiwaC4_0);
  manip.links[4].htmCols.push_back(fkres0.htmDH[4].inverse() * (*iiwaC4_0).htm);
  manip.links[4].nameObjs.push_back("c4_0");

  Box *iiwaC4_1 = new Box(Utils::trn(0, -0.08, 1.15 + 2 * dz), 0.12, 0.04, 0.17);
  manip.links[4].colObjs.push_back(iiwaC4_1);
  manip.links[4].htmCols.push_back(fkres0.htmDH[4].inverse() * (*iiwaC4_1).htm);
  manip.links[4].nameObjs.push_back("c4_1");

  // For the sixth link
  Cylinder *iiwaC5_0 = new Cylinder(Utils::trn(0, 0, 1.203 + 2 * dz), 0.075, 0.24);
  manip.links[5].colObjs.push_back(iiwaC5_0);
  manip.links[5].htmCols.push_back(fkres0.htmDH[5].inverse() * (*iiwaC5_0).htm);
  manip.links[5].nameObjs.push_back("c5_0");

  // For the seventh link
  // Box* iiwaC6_0 = new Box(Utils::trn(0.1, 0, 1.58 + 2 * dz - 0.14 / 2 + 0.02 + 0.015), 0.20, 0.1, 0.6 - 0.14 -
  // 0.015); manip.links[6].colObjs.push_back(iiwaC6_0); manip.links[6].htmCols.push_back(fkres0.htmDH[6].inverse() *
  // (*iiwaC6_0).htm); manip.links[6].nameObjs.push_back("c6_0");

  // Create joint limits (in rad)
  double dq = 0.087;

  VectorXd qMin(7);
  qMin << -2.967055 + dq, -2.09435 + dq, -2.967055 + dq, -2.09435 + dq, -2.937055 + dq, -2.09435 + dq, -3.054325 + dq;

  VectorXd qMax(7);
  qMax << 2.967055 - dq, 2.09435 - dq, 2.967055 - dq, 2.09435 - dq, 2.937055 - dq, 2.09435 - dq, 3.054325 - dq;

  manip.qMin = qMin;
  manip.qMax = qMax;

  // Create joint velocity limits (in rad/s)
  VectorXd qDotMin(7);
  qDotMin << -1.482, -1.482, -1.740, -1.307, -2.268, -2.355, -2.356;

  VectorXd qDotMax(7);
  qDotMax << 1.482, 1.482, 1.740, 1.307, 2.268, 2.355, 2.356;

  manip.qDotMin = qDotMin;
  manip.qDotMax = qDotMax;

  return manip;
}