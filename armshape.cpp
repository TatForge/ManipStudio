#include "armshape.h"

#include <cmath>

using RoboMath::Vector3;

namespace
{
constexpr scalar kDrumLength = 2.3;     // drum length / drum radius
constexpr scalar kLinkRatio = 0.72;     // tube radius / radius of the joint it starts at
constexpr scalar kMinLeg = 1e-4;        // shorter legs are left out
constexpr scalar kFlangeThickness = 0.012;

Vector3 Add(const Vector3 &a, const Vector3 &b) { return Vector3(a.x + b.x, a.y + b.y, a.z + b.z); }
Vector3 Sub(const Vector3 &a, const Vector3 &b) { return Vector3(a.x - b.x, a.y - b.y, a.z - b.z); }
Vector3 Scale(const Vector3 &a, scalar s) { return Vector3(a.x * s, a.y * s, a.z * s); }
Vector3 Cross(const Vector3 &a, const Vector3 &b)
{
    return Vector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
scalar Norm(const Vector3 &a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }

ArmPart Cylinder(int body, const Vector3 &from, const Vector3 &to, scalar radius, const std::string &name)
{
    return {ArmPart::Cylinder, body, from, to, radius, name};
}
}

const std::array<scalar, JOINTS_COUNT> &ArmShape::DefaultJointRadii()
{
    static const std::array<scalar, JOINTS_COUNT> radii{{0.075, 0.068, 0.058, 0.045, 0.04, 0.035}};
    return radii;
}

std::vector<ArmPart> ArmShape::Build(const RoboMath::Matrix &dh, const std::array<scalar, JOINTS_COUNT> &r)
{
    std::vector<ArmPart> parts;
    // pedestal from the floor to the drum of the first joint, which turns about the vertical
    // (frame 0 is translated by a(0), d(0) only, see Solver::SolveDirectKinematic)
    const Vector3 o0(dh.at(0, 0), 0, dh.at(0, 2));
    const Vector3 foot(o0.x, o0.y, 0);
    parts.push_back(Cylinder(-1, foot, Vector3(o0.x, o0.y, o0.z - r[0] * kDrumLength / 2), r[0] * 1.05, "base"));
    parts.push_back(Cylinder(-1, foot, Vector3(o0.x, o0.y, 0.02), r[0] * 1.6, "base"));

    for (int k = 0; k < JOINTS_COUNT; k++)
    {
        const std::string joint = "joint " + std::to_string(k + 1);
        const scalar half = r[k] * kDrumLength / 2;
        parts.push_back(Cylinder(k, Vector3(0, 0, -half), Vector3(0, 0, half), r[k], joint));
        if (k == JOINTS_COUNT - 1)
            break;

        // in frame k: frame k+1 sits at (a, 0, 0) + d * z(k+1), z(k+1) = RotX(alpha) * z
        const scalar a = dh.at(k, 0), alpha = dh.at(k, 1), d = dh.at(k + 1, 2);
        const Vector3 bend(a, 0, 0);
        const Vector3 next = Add(bend, Scale(Vector3(0, -std::sin(alpha), std::cos(alpha)), d));
        const scalar tube = r[k] * kLinkRatio;
        const std::string link = "link " + std::to_string(k + 1);
        const bool leg1 = std::abs(a) > kMinLeg, leg2 = std::abs(d) > kMinLeg;
        if (leg1)
            parts.push_back(Cylinder(k, Vector3(0, 0, 0), bend, tube, link));
        if (leg2)
            parts.push_back(Cylinder(k, bend, next, tube, link));
        if (leg1 && leg2)
            parts.push_back({ArmPart::Sphere, k, bend, bend, tube, link});
    }

    const int last = JOINTS_COUNT - 1;
    const scalar half = r[last] * kDrumLength / 2;
    parts.push_back(Cylinder(last, Vector3(0, 0, half), Vector3(0, 0, half + kFlangeThickness), r[last] * 1.15, "flange"));
    return parts;
}

std::vector<Vector3> ArmShape::HullPoints(const ArmPart &part, int segments)
{
    std::vector<Vector3> points;
    // circumscribed polygons, so the hull contains the round part
    const scalar grow = 1.0 / std::cos(M_PI / segments);
    if (part.kind == ArmPart::Cylinder)
    {
        const Vector3 axis = Sub(part.to, part.from);
        const scalar length = Norm(axis);
        if (length <= 0)
            return points;
        const Vector3 w = Scale(axis, 1 / length);
        const Vector3 helper = std::abs(w.x) < 0.9 ? Vector3(1, 0, 0) : Vector3(0, 1, 0);
        Vector3 u = Cross(w, helper);
        u = Scale(u, 1 / Norm(u));
        const Vector3 v = Cross(w, u);
        const scalar rr = part.radius * grow;
        for (int i = 0; i < segments; i++)
        {
            const scalar t = 2 * M_PI * i / segments;
            const Vector3 offset = Add(Scale(u, rr * std::cos(t)), Scale(v, rr * std::sin(t)));
            points.push_back(Add(part.from, offset));
            points.push_back(Add(part.to, offset));
        }
        return points;
    }
    // sphere: latitude rings plus the poles; rings scaled so the polyhedron contains the sphere
    const int rings = std::max(3, segments / 2);
    const scalar rr = part.radius * grow * grow;
    points.push_back(Add(part.from, Vector3(0, 0, rr)));
    points.push_back(Add(part.from, Vector3(0, 0, -rr)));
    for (int i = 1; i < rings; i++)
    {
        const scalar phi = M_PI * i / rings;
        for (int j = 0; j < segments; j++)
        {
            const scalar t = 2 * M_PI * j / segments;
            points.push_back(Add(part.from, Vector3(rr * std::sin(phi) * std::cos(t), rr * std::sin(phi) * std::sin(t), rr * std::cos(phi))));
        }
    }
    return points;
}
