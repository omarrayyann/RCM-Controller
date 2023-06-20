#include <iostream>
#include <cmath>
#include <iomanip>
#include <math.h>
#include <vector>
#include <string>
#include <fstream>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <sensor_msgs/JointState.h>
#include "iiwa_msgs/JointPosition.h"
#include "iiwa_msgs/JointQuantity.h"
#include <conio.h>
#include "robot.cpp"
#include "utils.cpp"
#include "distance.cpp"

using namespace std;

// Start of Global variables
VectorXd g_qkuka = VectorXd::Zero(7);
bool g_readedJoints = false;
int g_count = 0;
Vector3d g_pd;
Manipulator g_manip = Manipulator::createKukaIIWA();
bool g_reachedStartingPosition = false;
ofstream g_fileDebug;
TwoTimeSeries g_targetq;
vector<int> g_startIndex = {1};
vector<VectorXd> g_allpds;
double g_tsim = 0;
double lastTimeEEfPosition;
double lastTimeJointsPosition;
ros::Publisher KUKAJointsPublisher;
ros::Publisher KUKAJointsVelocityPublisher;
ros::Publisher KUKAFRIPublisher;
bool act = true;
ros::Time g_startingTime;
Matrix4d PARAM_HTMSTART = Utils::trn(0.45, 0, 0.55 - 0.4) * Utils::roty(3.14);
double PARAM_DT = 1.0 / 250;
Vector3d PARAM_FP;
bool PARAM_ISSIM = false;
double PARAM_TIME = 0.01;
double FILTER_PARAM = 0.01;
double VELCONVERT = 1;
// End of Global variables

// Start of Prototypes
struct Data;
class TimeSeries;
struct TwoTimeSeries;
double sqrtsgn(double x);
VectorXd sqrtsgn(VectorXd x);
double getTime();
bool setConfig(VectorXd q);
bool setConfigSpeed(VectorXd qdot);
void jointsCall(const sensor_msgs::JointState msg);
VectorXd rcmControl(VectorXd qt, Vector3d pd, Vector3d pf);
TwoTimeSeries generatePath(VectorXd p0, double t0, double maxtime);
// End of Prototypes

// Kuka Joints Publisher
bool setConfig(VectorXd q)
{
  if (g_manip.fk(q).htmTool(2, 3) < 0.45 - 0.5 - 1000)
  {
    ROS_INFO("ERROR: Collision of the tool with the ground. Not sending");
    return false;
  }
  else
  {
    bool jointsWithinLimits = true;
    for (int i = 0; i < q.rows(); i++)
      jointsWithinLimits = jointsWithinLimits && (q[i] >= g_manip.qMin[i] && q[i] <= g_manip.qMax[i]);

    if (jointsWithinLimits)
    {
      if (!PARAM_ISSIM)
      {
        std_msgs::Float64MultiArray messageArray;
        iiwa_msgs::JointPosition jointPosition;
        iiwa_msgs::JointQuantity quantity;

        messageArray.data = {q[0], q[1], q[2], q[3], q[4], q[5], q[6]};

        quantity.a1 = q[0];
        quantity.a2 = q[1];
        quantity.a3 = q[2];
        quantity.a4 = q[3];
        quantity.a5 = q[4];
        quantity.a6 = q[5];
        quantity.a7 = q[6];

        jointPosition.position = quantity;
        KUKAFRIPublisher.publish(messageArray);
      }
      else
      {
        g_qkuka = q;
      }

      return true;
    }
    else
    {
      ROS_INFO("ERROR: Joints not within limits. Not sending...");
      return false;
    }
  }
}

// KUKA Joints Subscriber
void jointsCall(const sensor_msgs::JointState msg)
{
  if (!PARAM_ISSIM)
  {
    VectorXd q_kuka_temp = VectorXd::Zero(7);
    q_kuka_temp << msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5],
        msg.position[6];
    g_qkuka = q_kuka_temp;
  }
  g_readedJoints = true;
}

// Control
VectorXd rcmControl(VectorXd qt, Vector3d pd, Vector3d pf)
{
  double K = 14.0;

  VectorXd q = qt;
  double dt = PARAM_DT;
  VectorXd qdot;

  for (int n = 0; n < round(PARAM_DT / dt); n++)
  {
    FulcrumPointResult fpResult = g_manip.computeFulcrumPoint(pf, q);

    MatrixXd A1 = Utils::matrixVertStack(fpResult.jacfx, fpResult.jacfy);
    VectorXd b1 = Utils::vectorVertStack(-K * (fpResult.fx), -K * (fpResult.fy));

    MatrixXd A2 = fpResult.fkr.jacTool.block<3, 7>(0, 0);
    Vector3d pe = fpResult.fkr.htmTool.block<3, 1>(0, 3);
    VectorXd b2 = -(27.0) * (pe - pd); // 27

    vector<MatrixXd> A = {A2, A1};
    vector<VectorXd> b = {b2, b1};

    qdot = Utils::hierarchicalSolve(A, b, 0.0000001); // 0.0005 0.0001
    q += qdot * dt;
  }

  qdot = (q - qt) / PARAM_DT;

  FulcrumPointResult fpResult_next = g_manip.computeFulcrumPoint(pf, q);

  return qdot;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "RCM_Control");
  ros::NodeHandle n;

  ros::Subscriber KUKASubscriber1 = n.subscribe("/iiwa/joint_states", 100, jointsCall);
  KUKAFRIPublisher = n.advertise<std_msgs::Float64MultiArray>("/iiwa/PositionController/command", 3);

  ros::Rate loop_rate(1 / PARAM_DT);

  VelocityConstControlParam param;
  param.taskHtm = PARAM_HTMSTART;
  param.obstacles = {};
  param.considerActionLimits = false;
  param.considerAutoCollision = false;
  param.considerJointLimits = true;
  param.kpos = 2.0;
  param.kori = 0.4;

  PARAM_FP = PARAM_HTMSTART.block<3, 1>(0, 3);
  PARAM_FP[2] += 0.10;
  g_pd = PARAM_HTMSTART.block<3, 1>(0, 3);

  double t_reached;

  if (PARAM_ISSIM)
    g_readedJoints = true;

  TimeSeries qpath;
  double tend, tstart;
  VectorXd qr;

  Vector3d p0;

  VectorXd PARAM_STARTQ0 = VectorXd::Zero(7);

  PARAM_STARTQ0 << 0.61916, 1.42963, -1.60960, -1.60650, 1.43321, 1.59124, -1.25610;

  Matrix4d htmstart = g_manip.fk(PARAM_STARTQ0).htmTool;
  PARAM_FP = htmstart.block<3, 1>(0, 3) - 0.1 * htmstart.block<3, 1>(0, 2);
  g_pd = htmstart.block<3, 1>(0, 3);

  while (ros::ok())
  {
    // Mode: going to starting position
    if (g_readedJoints && !g_reachedStartingPosition)
    {
      setConfigSpeed(-1.0 * sqrtsgn(g_qkuka - PARAM_STARTQ0));
      g_reachedStartingPosition = (g_qkuka - PARAM_STARTQ0).norm() <= 0.005;

      if (g_reachedStartingPosition)
      {
        g_startingTime = ros::Time::now();
        g_count = 0;
        p0 = g_manip.fk(g_qkuka).htmTool.block<3, 1>(0, 3);
      }
    }

    // Mode: fulcrum point constraint
    if (g_readedJoints && g_reachedStartingPosition)
    {

      if (!(g_count % ((int)(PARAM_TIME / PARAM_DT))))
      {
        Vector3d p_real = g_manip.fk(g_qkuka).htmTool.block<3, 1>(0, 3);
        double tstart = getTime();
        g_targetq = generatePath(p0, getTime(), PARAM_TIME);
        g_startIndex.push_back(g_startIndex[g_startIndex.size() - 1] + g_targetq.b.size());
        for (int s = 0; s < g_targetq.b.size(); s++)
          g_allpds.push_back(g_targetq.b.data[s]);
      }
      else
      {
        setConfig(g_targetq.a.atTime(getTime()));
      }
    }

    g_count++;
    ros::spinOnce();
    loop_rate.sleep();

    if (PARAM_ISSIM)
      g_tsim += PARAM_DT;
  }
}

// Helper Classes, Structures and Functions
struct Data
{
  VectorXd data;
  double timeStamp;
};

class TimeSeries
{
public:
  vector<VectorXd> data;
  vector<double> timeStamp;

  void add(VectorXd v, double t)
  {
    data.push_back(v);
    timeStamp.push_back(t);
  }
  void add(double v, double t)
  {
    VectorXd vv = VectorXd::Zero(1);
    vv << v;
    data.push_back(vv);
    timeStamp.push_back(t);
  }

  int size() { return data.size(); }
  VectorXd atTime(double t)
  {
    if (t < timeStamp[0])
    {
      ROS_INFO_STREAM("U");
      return data[0];
    }

    int k = 0;
    while (!(timeStamp[k] <= t && timeStamp[k + 1] > t) && k + 1 < timeStamp.size())
      k++;

    if (k + 1 == timeStamp.size())
    {
      ROS_INFO_STREAM("V");
      return data[k - 2];
    }
    else
    {
      double alpha = (t - timeStamp[k]) / (timeStamp[k + 1] - timeStamp[k]);
      return (1 - alpha) * data[k] + alpha * data[k + 1];
    }
  }
  string print(int n) { return std::to_string(timeStamp[n]) + " " + Utils::printVectorOctave(data[n]) + " "; }
};

struct TwoTimeSeries
{
  TimeSeries a;
  TimeSeries b;
};

double sqrtsgn(double x)
{
  return ((x > 0) ? 1 : -1) * sqrt(abs(x));
}

VectorXd sqrtsgn(VectorXd x)
{
  VectorXd y = VectorXd::Zero(x.rows());

  for (int i = 0; i < x.rows(); i++)
    y[i] = sqrtsgn(x[i]);

  return y;
}

double getTime()
{
  if (!PARAM_ISSIM)
    return (ros::Time::now() - g_startingTime).toSec();
  else
    return g_tsim;
}

bool setConfigSpeed(VectorXd qdot)
{
  bool speedWithinLimits = true;
  for (int i = 0; i < qdot.rows(); i++)
    speedWithinLimits =
        speedWithinLimits && (qdot[i] >= 1.5 * g_manip.qDotMin[i] && qdot[i] <= 1.5 * g_manip.qDotMax[i]);

  if (speedWithinLimits)
    setConfig(g_qkuka + PARAM_DT * qdot);
  else
    ROS_INFO_STREAM("ERROR: Speet outside limits... not sending");
}

TwoTimeSeries generatePath(VectorXd p0, double t0, double maxtime)
{
  VectorXd q = g_qkuka;
  double t = t0;
  Vector3d vlin_des;

  TimeSeries path;
  TimeSeries desp;

  double dt = 3 * PARAM_DT; // 3 * PARAM_DT
  while (t - t0 < maxtime + 0.1)
  {
    vlin_des << 0, 0.02 * cos(2 * 3.14 * t / 10), 0;
    Vector3d deltap = Vector3d::Zero(3);
    double rho = min(t / 5, 1);

    deltap << 0.03 * rho * cos(2 * 3.14 * t / 20), 0.03 * sin(2 * 3.14 * t / 20),
        -0.04 * rho + 0.06 * sin(2 * 3.14 * t / 40);

    VectorXd qdot = rcmControl(q, p0 + deltap, PARAM_FP);
    desp.add(p0 + deltap, t);
    q += dt * qdot;
    t += dt;
    path.add(q, t);
  }
  TwoTimeSeries tts;
  tts.a = path;
  tts.b = desp;

  return tts;
}
// End of Helper Classes and Structures