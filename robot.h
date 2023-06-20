#pragma once

#include "utils.h"
#include "distance.h"
#include "robot.h"

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>
#include <string.h>

using namespace std;
using namespace Eigen;

class Manipulator;

class Link
{
private:
  double _cost;
  double _sint;
  double _cosa;
  double _sina;

public:
  double dHtheta;
  double dHd;
  double dHalpha;
  double dHa;
  int jointType;
  int linkNumber;
  vector<GeometricPrimitives*> colObjs;
  vector<Matrix4d> htmCols;
  vector<string> nameObjs;
  string name;

  Matrix4d dhMatrix(double q);

  Link(int number, double theta, double d, double alpha, double a, int type, string strName);
};

struct FKResult
{
  vector<Matrix4d> htmDH;
  Matrix4d htmTool;
  vector<MatrixXd> jacDH;
  MatrixXd jacTool;
};

struct FulcrumPointResult
{
  double fx;
  double fy;
  double fz;
  double df;
  MatrixXd jacfx;
  MatrixXd jacfy;
  FKResult fkr;
};

struct TaskResult
{
  VectorXd task;
  MatrixXd jacTask;
  double maxErrorPos;
  double maxErrorOri;
};

class DistanceLinkObjResult
{
public:
  int linkNumber;
  int colObjLinkNumber;
  Vector3d witnessColObjLink;
  Vector3d witnessObj;
  double distance;
  VectorXd jacDist;

  DistanceLinkObjResult();
};

class DistanceLinkLinkResult
{
public:
  int linkNumber1;
  int colObjLinkNumber1;
  int linkNumber2;
  int colObjLinkNumber2;
  Vector3d witnessColObjLink1;
  Vector3d witnessColObjLink2;
  double distance;
  VectorXd jacDist;

  DistanceLinkLinkResult();
};

class DistanceRobotObjResult
{
private:
  vector<DistanceLinkObjResult> _structDistances;
  int _noJoints;

public:
  int noElements;
  GeometricPrimitives* obj;
  Manipulator* robot;

  DistanceRobotObjResult(int noJoints);

  DistanceRobotObjResult();

  DistanceLinkObjResult operator[](int i) const;

  DistanceLinkObjResult getClosest();

  DistanceLinkObjResult* getItem(int indLink, int indColObjLink);

  MatrixXd getAllJacobians();

  VectorXd getDistances();

  void addDistanceLinkObj(DistanceLinkObjResult dlo);
};

class DistanceRobotAutoResult
{
private:
  vector<DistanceLinkLinkResult> _structDistances;
  int _noJoints;

public:
  int noElements;
  Manipulator* robot;

  DistanceRobotAutoResult(int noJoints);

  DistanceRobotAutoResult();

  DistanceLinkLinkResult operator[](int i) const;

  DistanceLinkLinkResult getClosest();

  DistanceLinkLinkResult* getItem(int indLink1, int indColObjLink1, int indLink2, int indColObjLink2);

  MatrixXd getAllJacobians();

  VectorXd getDistances();

  void addDistanceLinkLink(DistanceLinkLinkResult dll);
};

class ConstControlResult
{
public:
  VectorXd action;
  TaskResult taskResult;
  vector<DistanceRobotObjResult> distancesObjResult;
  DistanceRobotAutoResult distanceAutoResult;
  MatrixXd H;
  MatrixXd A;
  VectorXd f;
  VectorXd b;
  double milisecondsSpent;
  bool feasible;

  ConstControlResult();
};

struct VelocityConstControlParam
{
  Matrix4d taskHtm;
  vector<GeometricPrimitives*> obstacles;
  ConstControlResult* oldControlStruct = NULL;
  Matrix4d customHtmWorldToBase = Matrix4d::Zero();

  double kpos = 0.5;
  double kori = 0.2;
  double etaObs = 0.5;
  double etaJoint = 0.5;
  double etaAuto = 0.5;
  double distSafeObs = 0.02;
  double distSafeJoint = 0.01;
  double distSafeAuto = 0.02;
  double eps = 0.005;
  double tol = 0.005;
  double maxDistAABB = 0.2;
  double h = 0;
  bool considerAutoCollision = true;
  bool considerJointLimits = true;
  bool considerActionLimits = true;
};

struct AccelerationConstControlParam
{
  Matrix4d taskHtm;
  vector<GeometricPrimitives*> obstacles;
  ConstControlResult* oldControlStruct = NULL;
  Matrix4d customHtmWorldToBase = Matrix4d::Zero();

  double kconv = 2.0;
  double etaObs = 0.5;
  double etaJointPosition = 0.5;
  double etaJointVelocity = 0.5;
  double etaAuto = 0.5;
  double distSafeObs = 0.02;
  double distSafeJoint = 0.01;
  double distSafeAuto = 0.02;
  double tol = 0.005;
  double maxDistAABB = 0.2;
  double beta = 0.005;
  double sigma = 0.5;
  double h = 0;
  double dt = 0.03;
  bool considerAutoCollision = true;
  bool considerJointPositionLimits = true;
  bool considerJointSpeedLimits = true;
  bool considerActionLimits = true;
};

// class OptimConstraint
// {
// public:
//   enum OptimConstraintType
//   {
//     velocityConstUpperJointLimit,
//     velocityConstLowerJointLimit,
//     velocityConstUpperVelLimit,
//     velocityConstLowerVelLimit,
//     velocityConstObstacleCollision,
//     velocityConstAutoCollision,

//     autoCollision,
//     obstacleCollision,
//     none
//   };
//     MatrixXd Arow;
//     double bvalue;
//     OptimConstraintType type;
// };

// class VelocityConstUpperJointLimit : public OptimConstraint
// {
//   public:
//     int jointNumber;
//     double jointValue;
//     double etaJoint;
//     double maxValue;
// };

// class VelocityConstLowerJointLimit : public OptimConstraint
// {
//   public:
//     int jointNumber;
//     double jointValue;
//     double etaJoint;
//     double minValue;
// };

struct IgnoreColLinkObstacle
{
  int linkNo;
  int obsNo;
};

struct FreeConfigParam
{
  vector<GeometricPrimitives*> obstacles = {};
  Matrix4d customHtmWorldToBase = Matrix4d::Zero();

  double distSafeObs = 0.02;
  double distSafeJoint = 0.01;
  double distSafeAuto = 0.02;
  double tol = 0.005;
  double h = 0;
  double maxDistAABB = 0.2;
  bool considerAutoCollision = true;
  bool considerJointLimits = true;
  vector<IgnoreColLinkObstacle> ignoreCol = {};
};

class FreeConfigResult
{
public:
  enum ErrorType
  {
    lowerJointLimit,
    upperJointLimit,
    autoCollision,
    obstacleCollision,
    none
  };
  bool isFree;
  ErrorType errorType;
  int jointNumber;
  int linkNumber1;
  int linkNumber2;
  int colObjNumber1;
  int colObjNumber2;
  int obstacleNumber;

  FreeConfigResult();
};

class Manipulator
{
public:
  int noJoints;
  vector<Link> links;
  VectorXd q;
  VectorXd qBase;
  VectorXd qMin;
  VectorXd qMax;
  VectorXd qDotMin;
  VectorXd qDotMax;
  Matrix4d htmWorldToBase;
  Matrix4d htmBaseToDH0;
  Matrix4d htmDHnToTool;

  Manipulator(int no);

  static Manipulator createKukaKR5();

  static Manipulator createKukaIIWA();

  static Manipulator createKukaIIWA_old();

  static Manipulator createKukaIIWASurgery();

  static Manipulator createKukaIIWAWithTool();

  static Manipulator createKukaIIWAWithTool_old();

  static Manipulator createGeoTouch();

  void setConfig(VectorXd q, Matrix4d customHtmWorldToBase = Matrix4d::Zero());

  FKResult fk(VectorXd q = VectorXd(0), Matrix4d customHtmWorldToBase = Matrix4d::Zero());

  FKResult jacGeo(VectorXd q = VectorXd(0), Matrix4d customHtmWorldToBase = Matrix4d::Zero());

  TaskResult taskFunction(Matrix4d taskHtm, VectorXd q = VectorXd(0), Matrix4d customHtmWorldToBase = Matrix4d::Zero());

  VectorXd ik(Matrix4d desHtm, VectorXd q0 = VectorXd(0), double pTol = 0.001, double aTol = 2, int noIterMax = 2000);

  DistanceRobotObjResult computeDistToObj(GeometricPrimitives* obj, VectorXd q = VectorXd(0),
                                          Matrix4d customHtmWorldToBase = Matrix4d::Zero(),
                                          DistanceRobotObjResult* oldDistStruct = NULL, double tol = 0.0005,
                                          double h = 0, double maxDist = 10000);

  DistanceRobotAutoResult computeDistAuto(VectorXd q = VectorXd(0), Matrix4d customHtmWorldToBase = Matrix4d::Zero(),
                                          DistanceRobotAutoResult* oldDistStruct = NULL, double tol = 0.0005,
                                          double h = 0, double maxDist = 10000);

  ConstControlResult velocityConstControl(VectorXd q = VectorXd(0),
                                          VelocityConstControlParam param = VelocityConstControlParam());

  ConstControlResult accelerationConstControl(VectorXd qdot, VectorXd q = VectorXd(0),
                                              AccelerationConstControlParam param = AccelerationConstControlParam());

  FreeConfigResult checkFreeConfig(VectorXd q = VectorXd(0), FreeConfigParam param = FreeConfigParam());

  FulcrumPointResult computeFulcrumPoint(Vector3d pf, VectorXd q);
};
