#pragma once

#include "utils.h"

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <list>
#include <math.h>
#include <vector>
#include <random>
#include <memory>


using namespace std;
using namespace Eigen;


struct AxisAlignedBoundingBox
{
    double width;
    double depth;
    double height;
};

struct DistanceStruct
{
    double distance;
    Vector3d witnessA;
    Vector3d witnessB;
};

class PointCloud;

class GeometricPrimitives
{
public:
    Matrix4d htm;
    virtual AxisAlignedBoundingBox getAABB() = 0;
    virtual double distanceToPoint(Vector3d point, double h = 0) = 0;
    virtual Vector3d projection(Vector3d point, double h = 0) = 0;
    virtual GeometricPrimitives* copy() = 0;
    virtual PointCloud generatePointCloud(double delta=0.01) = 0;

    static DistanceStruct computeDist(GeometricPrimitives* objA, GeometricPrimitives* objB, Vector3d pointA0 = Vector3d::Random(), double h = 0,
    double tol=0.001);
};

class Box : public GeometricPrimitives
{
public:
    double width;
    double depth;
    double height;

    Box(Matrix4d htm0, double widthVal, double depthVal, double heightVal);

    AxisAlignedBoundingBox getAABB();
    double distanceToPoint(Vector3d point, double h = 0);
    Vector3d projection(Vector3d point, double h = 0);
    GeometricPrimitives* copy();
    PointCloud generatePointCloud(double delta=0.01);
};

class Cylinder : public GeometricPrimitives
{
public:
    double radius;
    double height;

    Cylinder(Matrix4d htm0, double radiusVal, double heightVal);
    AxisAlignedBoundingBox getAABB();
    double distanceToPoint(Vector3d point, double h = 0);
    Vector3d projection(Vector3d point, double h = 0);
    GeometricPrimitives* copy();
    PointCloud generatePointCloud(double delta=0.01);

};

class Sphere : public GeometricPrimitives
{
public:
    double radius;

    Sphere(Matrix4d htm0, double radiusVal);
    AxisAlignedBoundingBox getAABB();
    double distanceToPoint(Vector3d point, double h = 0);
    Vector3d projection(Vector3d point, double h = 0);
    GeometricPrimitives* copy();
    PointCloud generatePointCloud(double delta=0.01);

};

class PointCloud : public GeometricPrimitives
{
public:
    vector<Vector3d> points;
    double minx, maxx;
    double miny, maxy;
    double minz, maxz;


    PointCloud(vector<Vector3d> pointsVal);
    AxisAlignedBoundingBox getAABB();
    double distanceToPoint(Vector3d point, double h = 0);
    Vector3d projection(Vector3d point, double h = 0);
    GeometricPrimitives* copy();
    PointCloud generatePointCloud(double delta=0.01);

};