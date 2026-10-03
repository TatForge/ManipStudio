#include "visuallayout.h"

#include <cmath>

namespace VisualLayout
{
namespace
{
// drum radius per joint, tapering towards the wrist (metres)
const float kJointRadius[6] = {0.075f, 0.068f, 0.058f, 0.045f, 0.04f, 0.035f};
constexpr float kDrumLength = 2.3f;        // drum length / radius
constexpr float kLinkRatio = 0.72f;        // tube radius / radius of the joint it starts at
constexpr float kMinLeg = 1e-4f;           // shorter legs are not drawn

QVector3D Origin(const QMatrix4x4 &m) { return m.column(3).toVector3D(); }
QVector3D AxisZ(const QMatrix4x4 &m) { return m.column(2).toVector3D().normalized(); }

Part Cylinder(Role role, int joint, const QVector3D &from, const QVector3D &to, float radius)
{
    return {Shape::Cylinder, role, joint, from, to, radius};
}

Part Sphere(Role role, int joint, const QVector3D &at, float radius)
{
    return {Shape::Sphere, role, joint, at, at, radius};
}
}

QQuaternion Part::Rotation() const
{
    const QVector3D dir = to - from;
    if (dir.lengthSquared() < 1e-12f)
        return QQuaternion();
    return QQuaternion::rotationTo(QVector3D(0, 1, 0), dir.normalized());
}

QString RoleName(Role role)
{
    switch (role)
    {
    case Role::Base: return QStringLiteral("base");
    case Role::Joint: return QStringLiteral("joint");
    case Role::Link: return QStringLiteral("link");
    case Role::Flange: return QStringLiteral("flange");
    }
    return QString();
}

QVector<Part> Build(const QVector<QMatrix4x4> &frames)
{
    QVector<Part> parts;
    if (frames.size() < 6)
        return parts;

    // pedestal from the floor up to the first joint, which turns about the vertical
    const QVector3D o0 = Origin(frames[0]);
    const QVector3D foot(0, 0, 0);
    parts << Cylinder(Role::Base, -1, foot, QVector3D(o0.x(), o0.y(), o0.z() - kJointRadius[0] * kDrumLength / 2),
                      kJointRadius[0] * 1.05f);
    parts << Cylinder(Role::Base, -1, foot, QVector3D(0, 0, 0.02f), kJointRadius[0] * 1.6f);

    for (int i = 0; i < 6; i++)
    {
        const QVector3D o = Origin(frames[i]);
        const QVector3D z = AxisZ(frames[i]);
        const float r = kJointRadius[i];
        const float half = r * kDrumLength / 2;
        parts << Cylinder(Role::Joint, i, o - z * half, o + z * half, r);
        if (i == 5)
            break;

        // leg 1: a(i) along x(i); leg 2: d(i+1) along z(i+1)
        const QVector3D next = Origin(frames[i + 1]);
        const QVector3D z_next = AxisZ(frames[i + 1]);
        const float d_next = QVector3D::dotProduct(next - o, z_next);
        const QVector3D bend = next - z_next * d_next;
        const float tube = r * kLinkRatio;
        const bool leg1 = (bend - o).length() > kMinLeg;
        const bool leg2 = std::abs(d_next) > kMinLeg;
        if (leg1)
            parts << Cylinder(Role::Link, i, o, bend, tube);
        if (leg2)
            parts << Cylinder(Role::Link, i, bend, next, tube);
        if (leg1 && leg2)
            parts << Sphere(Role::Link, i, bend, tube);
    }

    // tool flange at the end of the last joint
    const QVector3D o5 = Origin(frames[5]);
    const QVector3D z5 = AxisZ(frames[5]);
    const float half5 = kJointRadius[5] * kDrumLength / 2;
    parts << Cylinder(Role::Flange, 5, o5 + z5 * half5, o5 + z5 * (half5 + 0.012f), kJointRadius[5] * 1.15f);
    return parts;
}
}
