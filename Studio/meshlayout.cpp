#include "meshlayout.h"

#include "constants.h"

#include <QFile>
#include <QTextStream>

#include <cstring>
#include <limits>

namespace MeshLayout
{
QString ObstacleFile()
{
    return QStringLiteral("STLmodels/cube0.5m.stl");
}

QMatrix4x4 ObstaclePlacement()
{
    QMatrix4x4 m;
    m.translate(float(SMESH + 0.05), float(SMESH - 0.05), 0.f);
    return m;
}
}

namespace
{
void AddTriangle(StlMesh &mesh, const QVector3D &a, const QVector3D &b, const QVector3D &c)
{
    const QVector3D n = QVector3D::normal(a, b, c);
    for (const QVector3D &v : {a, b, c})
    {
        mesh.positions.append(v);
        mesh.normals.append(n);
    }
}

bool ReadBinary(const QByteArray &data, StlMesh &mesh)
{
    if (data.size() < 84)
        return false;
    quint32 count;
    std::memcpy(&count, data.constData() + 80, 4);
    if (qint64(data.size()) != 84 + qint64(count) * 50)
        return false;
    const char *p = data.constData() + 84;
    for (quint32 t = 0; t < count; t++, p += 50)
    {
        float f[12];
        std::memcpy(f, p, sizeof f);
        AddTriangle(mesh, {f[3], f[4], f[5]}, {f[6], f[7], f[8]}, {f[9], f[10], f[11]});
    }
    return true;
}

bool ReadAscii(const QByteArray &data, StlMesh &mesh)
{
    QTextStream in(data);
    QVector<QVector3D> v;
    QString word;
    while (!in.atEnd())
    {
        in >> word;
        if (word == QLatin1String("vertex"))
        {
            float x, y, z;
            in >> x >> y >> z;
            v.append({x, y, z});
            if (v.size() == 3)
            {
                AddTriangle(mesh, v[0], v[1], v[2]);
                v.clear();
            }
        }
    }
    return mesh.TriangleCount() > 0;
}
}

bool ReadStlMesh(const QString &path, StlMesh &mesh, QString *error)
{
    mesh = StlMesh();
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        if (error)
            *error = QStringLiteral("cannot open ") + path;
        return false;
    }
    const QByteArray data = file.readAll();
    // binary files exported by some tools also start with "solid", so the size check decides
    const bool ok = ReadBinary(data, mesh) || (data.trimmed().startsWith("solid") && ReadAscii(data, mesh));
    if (!ok)
    {
        mesh = StlMesh();
        if (error)
            *error = QStringLiteral("not a valid STL file: ") + path;
        return false;
    }
    constexpr float inf = std::numeric_limits<float>::max();
    mesh.min = {inf, inf, inf};
    mesh.max = {-inf, -inf, -inf};
    for (const QVector3D &p : mesh.positions)
        for (int k = 0; k < 3; k++)
        {
            mesh.min[k] = std::min(mesh.min[k], p[k]);
            mesh.max[k] = std::max(mesh.max[k], p[k]);
        }
    return true;
}
