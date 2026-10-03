#ifndef VISUALLAYOUT_H
#define VISUALLAYOUT_H

#include "armshape.h"

#include <QMatrix4x4>
#include <QQuaternion>
#include <QString>
#include <QVector3D>
#include <QVector>

// Display model of the arm: the parts of the core's ArmShape (the same ones
// the collision check uses as convex hulls) placed by the joint frames.
namespace VisualLayout
{
enum class Shape { Cylinder, Sphere };
enum class Role { Base, Joint, Link, Flange };

struct Part
{
    Shape shape;
    Role role;
    int joint;              // joint the part belongs to / starts at (-1: base)
    QVector3D from, to;     // cylinder axis end points (metres); sphere: both = centre
    float radius;           // metres

    QVector3D Center() const { return (from + to) / 2; }
    float Length() const { return (to - from).length(); }
    // rotation of a Y-up primitive onto the from -> to axis
    QQuaternion Rotation() const;
};

QString RoleName(Role role);

// frames: base -> joint i frames TR[0..5] (metres); shape: Solver::GetArmParts()
QVector<Part> Build(const QVector<QMatrix4x4> &frames, const std::vector<ArmPart> &shape);
}

#endif // VISUALLAYOUT_H
