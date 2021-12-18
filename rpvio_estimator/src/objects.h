#ifndef VOXBLOX_SIMULATION_OBJECTS_H_
#define VOXBLOX_SIMULATION_OBJECTS_H_

/**
 * This file is a modified from 'voxblox' repo
 **/

#include <algorithm>
#include <iostream>

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// #include "voxblox/core/common.h"
// #include "voxblox/core/voxel.h"

typedef Eigen::Vector3d Point;
typedef double FloatingPoint;

// Constants used across the library.
constexpr FloatingPoint kEpsilon = 1e-6; /**< Used for coordinates. */
constexpr float kFloatEpsilon = 1e-6;    /**< Used for weights. */

struct Color
{
    Color() : r(0), g(0), b(0), a(0) {}
    Color(uint8_t _r, uint8_t _g, uint8_t _b) : Color(_r, _g, _b, 255) {}
    Color(uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _a)
        : r(_r), g(_g), b(_b), a(_a) {}

    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;

    static Color blendTwoColors(const Color &first_color,
                                FloatingPoint first_weight,
                                const Color &second_color,
                                FloatingPoint second_weight)
    {
        FloatingPoint total_weight = first_weight + second_weight;

        first_weight /= total_weight;
        second_weight /= total_weight;

        Color new_color;
        new_color.r = static_cast<uint8_t>(
            round(first_color.r * first_weight + second_color.r * second_weight));
        new_color.g = static_cast<uint8_t>(
            round(first_color.g * first_weight + second_color.g * second_weight));
        new_color.b = static_cast<uint8_t>(
            round(first_color.b * first_weight + second_color.b * second_weight));
        new_color.a = static_cast<uint8_t>(
            round(first_color.a * first_weight + second_color.a * second_weight));

        return new_color;
    }

    // Now a bunch of static colors to use! :)
    static const Color White() { return Color(255, 255, 255); }
    static const Color Black() { return Color(0, 0, 0); }
    static const Color Gray() { return Color(127, 127, 127); }
    static const Color Red() { return Color(255, 0, 0); }
    static const Color Green() { return Color(0, 255, 0); }
    static const Color Blue() { return Color(0, 0, 255); }
    static const Color Yellow() { return Color(255, 255, 0); }
    static const Color Orange() { return Color(255, 127, 0); }
    static const Color Purple() { return Color(127, 0, 255); }
    static const Color Teal() { return Color(0, 255, 255); }
    static const Color Pink() { return Color(255, 0, 127); }
};

// Should this be full virtual?
/**
 * Base class for simulator objects. Each object allows an exact ground-truth
 * sdf to be created for it.
 */
class Object
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // A wall is an infinite plane.
    enum Type
    {
        kSphere = 0,
        kPlane,
        kCuboid
    };

    Object(const Point &center, Type type)
        : Object(center, type, Color::White(), 0 /*id*/) {}
    Object(const Point &center, Type type, const Color &color)
        : Object(center, type, color, 0 /*id*/) {}
    Object(const Point &center, Type type, const Color &color, int id)
        : center_(center), type_(type), color_(color), id_(id) {}
    virtual ~Object() {}

    /// Map-building accessors.
    virtual FloatingPoint getDistanceToPoint(const Point &point) const = 0;

    Color getColor() const { return color_; }
    Type getType() const { return type_; }
    int getId() const { return id_; }

    /// Raycasting accessors.
    virtual bool getRayIntersection(const Point &ray_origin,
                                    const Point &ray_direction,
                                    FloatingPoint max_dist,
                                    Point *intersect_point,
                                    FloatingPoint *intersect_dist) const = 0;

    virtual void setParameters(Point center, Point normal, FloatingPoint breadth, FloatingPoint width, FloatingPoint height) = 0;

protected:
    Point center_;
    Type type_;
    Color color_;
    int id_;
};

class Sphere : public Object
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Sphere(const Point &center, FloatingPoint radius)
        : Object(center, Type::kSphere), radius_(radius) {}
    Sphere(const Point &center, FloatingPoint radius, const Color &color)
        : Object(center, Type::kSphere, color), radius_(radius) {}

    virtual FloatingPoint getDistanceToPoint(const Point &point) const
    {
        FloatingPoint distance = (center_ - point).norm() - radius_;
        return distance;
    }

    virtual bool getRayIntersection(const Point &ray_origin,
                                    const Point &ray_direction,
                                    FloatingPoint max_dist,
                                    Point *intersect_point,
                                    FloatingPoint *intersect_dist) const
    {
        // From https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
        // x = o + dl is the ray equation
        // r = sphere radius, c = sphere center
        FloatingPoint under_square_root =
            pow(ray_direction.dot(ray_origin - center_), 2) -
            (ray_origin - center_).squaredNorm() + pow(radius_, 2);

        // No real roots = no intersection.
        if (under_square_root < 0.0)
        {
            return false;
        }

        FloatingPoint d =
            -(ray_direction.dot(ray_origin - center_)) - sqrt(under_square_root);

        // Intersection behind the origin.
        if (d < 0.0)
        {
            return false;
        }
        // Intersection greater than max dist, so no intersection in the sensor
        // range.
        if (d > max_dist)
        {
            return false;
        }

        *intersect_point = ray_origin + d * ray_direction;
        *intersect_dist = d;
        return true;
    }

    virtual void setParameters(Point center, Point normal, FloatingPoint breadth, FloatingPoint width) {}

protected:
    FloatingPoint radius_;
};

/// Requires normal being passed in to ALREADY BE NORMALIZED!!!!
class PlaneObject : public Object
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PlaneObject(const Point &center, const Point &normal)
        : Object(center, Type::kPlane), normal_(normal) {}
    PlaneObject(const Point &center, const Point &normal, const Color &color)
        : Object(center, Type::kPlane, color), normal_(normal)
    {
        CHECK_NEAR(normal.norm(), 1.0, 1e-3);
    }

    virtual FloatingPoint getDistanceToPoint(const Point &point) const
    {
        // Compute the 'd' in ax + by + cz + d = 0:
        // This is actually the scalar product I guess.
        FloatingPoint d = -normal_.dot(center_);
        FloatingPoint p = d / normal_.norm();

        FloatingPoint distance = normal_.dot(point) + p;
        return distance;
    }

    virtual bool getRayIntersection(const Point &ray_origin,
                                    const Point &ray_direction,
                                    FloatingPoint max_dist,
                                    Point *intersect_point,
                                    FloatingPoint *intersect_dist) const
    {
        // From https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection
        // Following notation of sphere more...
        // x = o + dl is the ray equation
        // n = normal, c = plane 'origin'
        FloatingPoint denominator = ray_direction.dot(normal_);
        if (std::abs(denominator) < kEpsilon)
        {
            // Lines are parallel, no intersection.
            return false;
        }
        FloatingPoint d = (center_ - ray_origin).dot(normal_) / denominator;
        if (d < 0.0)
        {
            return false;
        }
        if (d > max_dist)
        {
            return false;
        }
        *intersect_point = ray_origin + d * ray_direction;
        *intersect_dist = d;
        return true;
    }

    virtual void setParameters(Point center, Point normal, FloatingPoint breadth, FloatingPoint width) {}

protected:
    Point normal_;
};

// Assumed that the normal is horizontal
class CuboidObject : public Object
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CuboidObject(const Point &center, const Point &normal, const FloatingPoint &breadth, const FloatingPoint &width, const FloatingPoint &height)
        : Object(center, Type::kCuboid), normal_(normal), breadth_(breadth), width_(width), height_(height)
    {
        setIsometryFromNormal();
    }
    CuboidObject(const Point &center, const Point &normal, const FloatingPoint &breadth, const FloatingPoint &width, const FloatingPoint &height, const Color &color)
        : Object(center, Type::kCuboid, color), normal_(normal), breadth_(breadth), width_(width), height_(height)
    {
        setIsometryFromNormal();
    }
    CuboidObject(const Point &center, const Point &normal, const FloatingPoint &breadth, const FloatingPoint &width, const FloatingPoint &height, const Color &color, const int id)
        : Object(center, Type::kCuboid, color, id), normal_(normal), breadth_(breadth), width_(width), height_(height)
    {
        setIsometryFromNormal();
    }
    CuboidObject(const vector<Point>& vertices)
    {
        Point center(0, 0, 0);

        for (int i = 0; i < vertices.size(); i++){
            center += vertices[i];
        }
        center = center / vertices.size();


    }

    void computeNormalFromVertices()
    {

    }

    void computeHalfPlanes()
    {
        
    }

    void setIsometryFromNormal()
    {
        Point normal;
        normal << normal_.x(), normal_.y(), normal_.z();
        normal.normalize();

        Point x_axis;
        x_axis << 1.0, 0.0, 0.0;

        // Compute the rotation of x-axis w.r.t normal
        FloatingPoint cos_theta = x_axis.dot(normal);
        FloatingPoint sin_theta = std::sqrt((FloatingPoint)1.0 - cos_theta * cos_theta);

        iso_box2world_.translation() = center_;
        iso_box2world_.linear() << cos_theta, -sin_theta, 0.0,
            sin_theta, cos_theta, 0.0,
            0.0, 0.0, 1.0;
    }

    virtual FloatingPoint getDistanceToPoint(const Point &point) const
    {
        // TODO: Add "SDF of a box" youtube link

        Point p;
        p << point.x(), point.y(), point.z();

        p = iso_box2world_.inverse() * point;

        Point R;
        R.x() = width_ / 2.0;
        R.y() = breadth_ / 2.0;
        R.z() = height_ / 2.0;

        Point q = p.cwiseAbs() - R;

        FloatingPoint distance = q.cwiseMax(0.0).norm();// + std::min(q.maxCoeff(), (FloatingPoint)0.0);

        return distance;
    }

    virtual bool getRayIntersection(const Point &ray_origin,
                                    const Point &ray_direction,
                                    FloatingPoint max_dist,
                                    Point *intersect_point,
                                    FloatingPoint *intersect_dist) const
    {
        return false;
    }

    virtual void setParameters(Point center, Point normal, FloatingPoint breadth, FloatingPoint width, FloatingPoint height)
    {
        center_ = center;
        normal_ = normal;
        breadth_ = breadth;
        width_ = width;
        height_ = height;

        setIsometryFromNormal();
    }

protected:
    FloatingPoint breadth_;
    FloatingPoint width_;
    FloatingPoint height_;
    Point normal_;

    Eigen::Transform<FloatingPoint, 3, Eigen::Isometry> iso_box2world_;
};

#endif // VOXBLOX_SIMULATION_OBJECTS_H_