#ifndef MESHLAYOUT_H
#define MESHLAYOUT_H

#include <QMatrix4x4>
#include <QString>
#include <QVector3D>
#include <QVector>

// The obstacle of the collision check, an STL cube placed as in the Solver
// constructor. (The arm itself is drawn from ArmShape, see VisualLayout.)
namespace MeshLayout
{
QString ObstacleFile();             // relative to the data directory
QMatrix4x4 ObstaclePlacement();     // metres, world
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
