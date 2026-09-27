#ifndef MESHLAYOUT_H
#define MESHLAYOUT_H

#include <QMatrix4x4>
#include <QString>
#include <QVector3D>
#include <QVector>

// Which STL is drawn where. Mirrors Solver::MoveMeshes, so the 3D view shows
// exactly the geometry the collision check works with.
namespace MeshLayout
{
struct Part
{
    QString name;
    QString file;       // relative to the data directory
    bool obstacle;
};

const QVector<Part> &Parts();

// Placement (rotation + translation, metres) of every part for the joint
// frames TR[0..5] of one pose.
QVector<QMatrix4x4> Placements(const QVector<QMatrix4x4> &frames);
}

// Triangles of an STL file (binary or ASCII), flat normals, metres.
struct StlMesh
{
    QVector<QVector3D> positions;       // 3 per triangle
    QVector<QVector3D> normals;         // one per vertex (flat shading)
    QVector3D min, max;
    int TriangleCount() const { return int(positions.size() / 3); }
};

bool ReadStlMesh(const QString &path, StlMesh &mesh, QString *error = nullptr);

#endif // MESHLAYOUT_H
