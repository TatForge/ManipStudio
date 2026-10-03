#ifndef ARMSHAPE_H
#define ARMSHAPE_H

#include "my_math.h"

#include <array>
#include <cstdlib>
#include <string>
#include <vector>

// Shape of the arm, built from the DH table, used both for the collision
// check (as convex hulls) and for display. Every part is rigid in one body:
// body -1 is the base standing on the floor, body k (0..5) moves with joint
// frame k. In modified DH, frame k+1 is reached from frame k by a(k) along
// x(k) and then d(k+1) along z(k+1); those two legs become tubes, each joint
// a drum on its axis, a bend between two legs a sphere.
struct ArmPart
{
    enum Kind { Cylinder, Sphere };
    Kind kind;
    int body;
    RoboMath::Vector3 from, to;     // axis end points in the body frame (m); sphere: both = centre
    scalar radius;                  // m
    std::string name;               // "base", "joint 1".."joint 6", "link 1".."link 5", "flange"
};

class ArmShape
{
public:
    // drum radius of every joint (m), tapering towards the wrist
    static const std::array<scalar, JOINTS_COUNT> &DefaultJointRadii();
    // dh: one row per joint with columns a, alpha, d (Robo::state_matrix)
    static std::vector<ArmPart> Build(const RoboMath::Matrix &dh,
                                      const std::array<scalar, JOINTS_COUNT> &joint_radii = DefaultJointRadii());
    // vertices (body frame) of a convex polyhedron that contains the part
    static std::vector<RoboMath::Vector3> HullPoints(const ArmPart &part, int segments = 16);
    // parts of the same or of neighbouring bodies touch by construction and are not checked
    static bool Adjacent(int body_a, int body_b) { return std::abs(body_a - body_b) <= 1; }
};

#endif // ARMSHAPE_H
