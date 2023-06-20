
#include "utils.h"
#include "distance.h"

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>

#include "quadprogpp/src/QuadProg++.hh"
#include "quadprogpp/src/QuadProg++.cc"

using namespace std;
using namespace Eigen;

Matrix4d Utils::rotx(double theta)
{
  double cost = cos(theta);
  double sint = sin(theta);

  Matrix4d rot;
  rot << 1, 0, 0, 0, 0, cost, -sint, 0, 0, sint, cost, 0, 0, 0, 0, 1;

  return rot;
}
Matrix4d Utils::roty(double theta)
{
  double cost = cos(theta);
  double sint = sin(theta);

  Matrix4d rot;
  rot << cost, 0, sint, 0, 0, 1, 0, 0, -sint, 0, cost, 0, 0, 0, 0, 1;

  return rot;
}
Matrix4d Utils::rotz(double theta)
{
  double cost = cos(theta);
  double sint = sin(theta);

  Matrix4d rot;
  rot << cost, -sint, 0, 0, sint, cost, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;

  return rot;
}
Matrix4d Utils::trn(Vector3d v)
{
  Matrix4d trn;
  trn << 1, 0, 0, v(0), 0, 1, 0, v(1), 0, 0, 1, v(2), 0, 0, 0, 1;

  return trn;
}
Matrix4d Utils::trn(double x, double y, double z)
{
  Matrix4d trn;

  trn << 1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1;

  return trn;
}
Matrix3d Utils::S(Vector3d v)
{
  Matrix3d res = Matrix3d::Zero();

  res(0, 1) = -v(2);
  res(1, 0) = v(2);
  res(0, 2) = v(1);
  res(2, 0) = -v(1);
  res(1, 2) = -v(0);
  res(2, 1) = v(0);

  return res;
}
Vector3d Utils::cross(Vector3d a, Vector3d b)
{
  return Utils::S(a) * b;
}
MatrixXd Utils::pinv(MatrixXd M, double eps)
{
  int m = M.cols();
  MatrixXd MT = M.transpose();
  return (eps * MatrixXd::Identity(m, m) + MT * M).inverse() * MT;
}
VectorXd Utils::vecPow(VectorXd v, int indStart, int indEnd, double h)
{
  // int n = v.rows();
  VectorXd w = v;

  for (int i = indStart; i < indEnd; i++)
  {
    w[i] = (v[i] > 0 ? 1 : -1) * pow(abs(v[i]), h);
  }

  return w;
}
double Utils::rand(double vMin, double vMax)
{
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<double> dist(vMin, vMax);

  return dist(mt);
}
VectorXd Utils::randVec(int n, double vMin, double vMax)
{
  VectorXd v(n);

  for (int i = 0; i < n; i++)
  {
    v[i] = rand(vMin, vMax);
  }

  return v;
}
Vector3d Utils::rpy(Matrix4d rot)
{
  double cos_pitch = sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2));
  double sin_pitch = -rot(2, 0);
  Vector3d angles = Vector3d::Zero();

  if (cos_pitch > 0.001)
  {
    double roll = atan2(rot(2, 1), rot(2, 2));
    double pitch = atan2(-rot(2, 0), cos_pitch);
    double yaw = atan2(rot(1, 0), rot(0, 0));
    angles << roll, pitch, yaw;
    return angles;
  }
  else
  {
    if (sin_pitch > 0)
    {
      double pitch = M_PI / 2;
      double yaw = 0;
      double roll = atan2(rot(0, 1), rot(0, 2));
      angles << roll, pitch, yaw;
      return angles;
    }
    else
    {
      double pitch = -M_PI / 2;
      double yaw = 0;
      double roll = atan2(-rot(0, 1), -rot(0, 2));
      angles << roll, pitch, yaw;
      return angles;
    }
  }
}

double Utils::distanceAABB(GeometricPrimitives* objA, GeometricPrimitives* objB)
{
  AxisAlignedBoundingBox AABB_A = (*objA).getAABB();
  AxisAlignedBoundingBox AABB_B = (*objB).getAABB();

  Vector3d delta = (*objA).htm.block<3, 1>(0, 3) - (*objB).htm.block<3, 1>(0, 3);

  double dx = max(abs(delta[0]) - (AABB_A.width + AABB_B.width) / 2.0, 0.0);
  double dy = max(abs(delta[1]) - (AABB_A.depth + AABB_B.depth) / 2.0, 0.0);
  double dz = max(abs(delta[2]) - (AABB_A.height + AABB_B.height) / 2.0, 0.0);

  return sqrt(dx * dx + dy * dy + dz * dz);
}

vector<VectorXd> Utils::upsample(vector<VectorXd> points, double dist)
{
  vector<VectorXd> upsampledPoints;
  VectorXd currentPoint;
  VectorXd nextPoint;
  VectorXd dir;
  double curDist;
  int jmax;

  for (int i = 1; i < points.size(); i++)
  {
    currentPoint = points[i - 1];
    nextPoint = points[i];
    curDist = (currentPoint - nextPoint).norm();

    jmax = (int)floor(curDist / dist) - 1;

    dir = (nextPoint - currentPoint) / (0.0000001 + curDist);

    for (int j = 0; j <= jmax; j++)
    {
      upsampledPoints.push_back(currentPoint + ((double)j) * dist * dir);
    }
  }
  upsampledPoints.push_back(nextPoint);

  return upsampledPoints;
}

VectorFieldResult Utils::vectorField(VectorXd q, vector<VectorXd> points, double alpha, bool openCurve,
                                     double percentLengthStop)
{
  // Find the closest point in the curve
  double dmin = (points[0] - q).norm();
  double dminTemp;
  int ind = 0;
  vector<double> s;
  s.push_back(0);

  for (int i = 0; i < points.size(); i++)
  {
    dminTemp = (points[i] - q).norm();
    if (dminTemp < dmin)
    {
      dmin = dminTemp;
      ind = i;
    }
    if (i > 0) s.push_back(s[s.size() - 1] + (points[i] - points[i - 1]).norm());
  }

  VectorXd pi = points[ind];

  // Compute the normal vector
  VectorXd N = (pi - q) / ((pi - q).norm() + 0.00001);

  // Compute the tangent vector
  VectorXd T;

  if (openCurve)
  {
    if (ind == 0)
      T = (points[1] - points[0]).normalized();
    else
      T = (points[ind] - points[ind - 1]).normalized();
  }
  else
  {
    if (ind == 0)
      T = (points[0] - points[points.size() - 1]).normalized();
    else
      T = (points[ind] - points[ind - 1]).normalized();
  }

  // Compute the G and H gains
  double G = (2 / M_PI) * atan(alpha * sqrt(dmin));
  double H = sqrt(1 - 0.9999 * G * G);

  // Compute the final vector field:
  VectorXd v = G * N + H * T;

  // Scale if the curve is open
  double sCurrent = s[ind];
  double sMaximum = s[s.size() - 1];
  double mult;
  if (openCurve)
  {
    mult = sqrt(abs((1 - (sCurrent - percentLengthStop * sMaximum) / ((1 - percentLengthStop) * sMaximum))));
    if (sCurrent > percentLengthStop * sMaximum) v = mult * v;
  }

  // if(openCurve)
  //     cout<<dmin<<std::endl;

  VectorFieldResult vfr;

  vfr.vector = v;
  vfr.distance = dmin;
  vfr.index = ind;

  return vfr;
}

VectorXd Utils::solveQP(MatrixXd H, VectorXd f, MatrixXd A, VectorXd b, MatrixXd Aeq, VectorXd beq)
{
  // Solve min_u (u'*H*u)/2 + f'*u
  // such that:
  // A*u >= b
  // Aeq*u = beq
  // The function assumes that H is a positive definite function (the problem is strictly convex)

  int n = H.rows();

  if (A.rows() == 0 && A.cols() == 0)
  {
    A = MatrixXd::Zero(0, n);
    b = VectorXd::Zero(0);
  }

  if (Aeq.rows() == 0 && Aeq.cols() == 0)
  {
    Aeq = MatrixXd::Zero(0, n);
    beq = VectorXd::Zero(0);
  }

  int meq = Aeq.rows();
  int mineq = A.rows();

  quadprogpp::Matrix<double> H_aux, Aeq_aux, A_aux;
  quadprogpp::Vector<double> f_aux, beq_aux, b_aux, u_aux;

  H_aux.resize(n, n);
  for (int i = 0; i < n; i++)
    for (int j = 0; j < n; j++) H_aux[i][j] = H(i, j);

  f_aux.resize(n);
  for (int i = 0; i < n; i++) f_aux[i] = f[i];

  Aeq_aux.resize(n, meq);
  for (int i = 0; i < n; i++)
    for (int j = 0; j < meq; j++) Aeq_aux[i][j] = Aeq(j, i);

  beq_aux.resize(meq);
  for (int j = 0; j < meq; j++) beq_aux[j] = -beq[j];

  A_aux.resize(n, mineq);
  for (int i = 0; i < n; i++)
    for (int j = 0; j < mineq; j++) A_aux[i][j] = A(j, i);

  b_aux.resize(mineq);
  for (int j = 0; j < mineq; j++) b_aux[j] = -b[j];

  u_aux.resize(n);

  double val = solve_quadprog(H_aux, f_aux, Aeq_aux, beq_aux, A_aux, b_aux, u_aux);

  if (val > 1.0E50)
  {
    // Problem is unfeasible
    VectorXd u(0);
    return u;
  }
  else
  {
    // Problem is feasible
    VectorXd u(n);

    for (int i = 0; i < n; i++) u[i] = u_aux[i];

    return u;
  }
}

MatrixXd Utils::matrixVertStack(MatrixXd A1, MatrixXd A2)
{
  MatrixXd A(A1.rows() + A2.rows(), A1.cols());
  A << A1, A2;
  return A;
}

VectorXd Utils::vectorVertStack(VectorXd v1, VectorXd v2)
{
  VectorXd v(v1.rows() + v2.rows());
  v << v1, v2;
  return v;
}

VectorXd Utils::vectorVertStack(double v1, VectorXd v2)
{
  VectorXd v(1 + v2.rows());
  v << v1, v2;
  return v;
}

VectorXd Utils::vectorVertStack(VectorXd v1, double v2)
{
  VectorXd v(v1.rows() + 1);
  v << v1, v2;
  return v;
}

VectorXd Utils::vectorVertStack(double v1, double v2)
{
  VectorXd v(2);
  v << v1, v2;
  return v;
}

string Utils::printNumber(double x, int nochar)
{
  double P1 = pow(10, nochar - 3);
  double P2 = 1 / P1;

  double y = P2 * round(x * P1);
  string str;
  if (x >= 0)
    str = " " + std::to_string(y).substr(0, nochar - 1);
  else
    str = std::to_string(y).substr(0, nochar - 1);

  while (str.size() < nochar) str += "0";

  return str;
}

string Utils::printVector(VectorXd v)
{
  string str = "[";
  for (int i = 0; i < v.rows() - 1; i++) str += printNumber(v[i]) + ", ";

  str += printNumber(v[v.rows() - 1]) + "]";

  return str;
}

string Utils::printVectorOctave(VectorXd v)
{
  string str = "[";
  for (int i = 0; i < v.rows() - 1; i++) str += printNumber(v[i]) + " ";

  str += printNumber(v[v.rows() - 1]) + "]";

  return str;
}

string Utils::printMatrix(MatrixXd M)
{
  string str = "";

  for (int i = 0; i < M.rows(); i++)
  {
    str += (i == 0) ? "[" : " ";
    for (int j = 0; j < M.cols(); j++) str += Utils::printNumber(M(i, j)) + "  ";

    str += (i == M.rows() - 1) ? "]" : "\n";
  }

  return str;
}

string Utils::printMatrixPython(MatrixXd M)
{
  string str = "np.matrix([";

  for (int i = 0; i < M.rows(); i++)
  {
    str += "[";
    for (int j = 0; j < M.cols(); j++) str += Utils::printNumber(M(i, j)) + ((j == M.cols() - 1) ? " " : ", ");

    str += (i == M.rows() - 1) ? "]" : "], ";
  }

  return str + "])";
}

double Utils::softMin(vector<double> v, double h)
{
  double valMin = 100000;
  double sum = 0;

  for (int i = 0; i < v.size(); i++) valMin = min(valMin, v[i]);

  for (int i = 0; i < v.size(); i++)
  {
    sum += exp(-(v[i] - valMin) / h);
  }

  return valMin - h * log(sum / ((double)v.size()));
}

MatrixXd Utils::nullSpace(MatrixXd A)
{
  FullPivLU<MatrixXd> lu(A);
  MatrixXd Anull = lu.kernel();

  if (Anull.cols() == 1 && Anull.norm() <= 0.001)
    return MatrixXd::Zero(A.cols(), 0);
  else
    return Anull;
}

VectorXd Utils::hierarchicalSolve(vector<MatrixXd> A, vector<VectorXd> b, double eps)
{
  VectorXd xsol = Utils::pinv(A[0], eps) * b[0];
  MatrixXd nullA = Utils::nullSpace(A[0]);

  if (nullA.cols() == 0 || A.size() == 1)
    return xsol;
  else
  {
    vector<MatrixXd> Anew;
    vector<VectorXd> bnew;
    for (int i = 1; i < A.size(); i++)
    {
      Anew.push_back(A[i] * nullA);
      bnew.push_back(b[i] - A[i] * xsol);
    }
    VectorXd ysol = Utils::hierarchicalSolve(Anew, bnew, eps);
    return xsol + nullA * ysol;
  }
}

SoftSelectMinResult Utils::softSelectMin(vector<double> v, vector<VectorXd> vVec, double h)
{
  double valMin = 100000;
  double sum = 0;
  double tempVal;
  VectorXd sumv = VectorXd::Zero(vVec[0].rows());

  for (int i = 0; i < v.size(); i++) valMin = min(valMin, v[i]);

  for (int i = 0; i < v.size(); i++)
  {
    tempVal = exp(-(v[i] - valMin) / h);
    sum += tempVal;
    sumv += tempVal * vVec[i];
  }

  SoftSelectMinResult ssmr;

  ssmr.selected = sumv / sum;
  ssmr.residue = -h * log(sum / ((double)v.size()));
  ssmr.trueMin = valMin;
  ssmr.softMin = ssmr.trueMin + ssmr.residue;

  return ssmr;
}