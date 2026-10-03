#include "visuallayout.h"

#include <cmath>

namespace VisualLayout
{
namespace
{
QVector3D ToQ(const RoboMath::Vector3 &v)
{
    return QVector3D(float(v.x), float(v.y), float(v.z));
}

Role RoleOf(const std::string &name)
{
    if (name.rfind("base", 0) == 0)
        return Role::Base;
    if (name.rfind("joint", 0) == 0)
        return Role::Joint;
    if (name.rfind("flange", 0) == 0)
        return Role::Flange;
    return Role::Link;
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

QVector<Part> Build(const QVector<QMatrix4x4> &frames, const std::vector<ArmPart> &shape)
{
    QVector<Part> parts;
    if (frames.size() < 6)
        return parts;
    for (const ArmPart &a : shape)
    {
        QVector3D from = ToQ(a.from), to = ToQ(a.to);
        if (a.body >= 0)
        {
            from = frames[a.body].map(from);
            to = frames[a.body].map(to);
        }
        parts.append({a.kind == ArmPart::Sphere ? Shape::Sphere : Shape::Cylinder, RoleOf(a.name), a.body, from, to, float(a.radius)});
    }
    return parts;
}
}
