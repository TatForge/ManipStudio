#ifndef VISUALLAYOUT_H
#define VISUALLAYOUT_H

#include <QMatrix4x4>
#include <QQuaternion>
#include <QString>
#include <QVector3D>
#include <QVector>

// Display model of the arm built from the joint frames alone, so it always
// matches the kinematics (the STL parts in MeshLayout are the much coarser
// collision model). In modified DH, frame i+1 is reached from frame i by
// a(i) along x(i) and then d(i+1) along z(i+1): each of the two legs becomes
// a tube, every joint a drum on its axis, bends get a sphere.
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

// frames: base -> joint i frames TR[0..5] (metres)
QVector<Part> Build(const QVector<QMatrix4x4> &frames);
}

#endif // VISUALLAYOUT_H
