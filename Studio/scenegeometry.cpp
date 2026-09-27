#include "scenegeometry.h"

#include "meshlayout.h"

#include <QByteArray>
#include <QVector3D>

#include <limits>

void StlGeometry::setSource(const QUrl &source)
{
    if (m_source == source)
        return;
    m_source = source;
    clear();
    m_triangles = 0;
    m_error.clear();

    StlMesh mesh;
    if (!ReadStlMesh(source.isLocalFile() ? source.toLocalFile() : source.toString(), mesh, &m_error))
    {
        qWarning("StlGeometry: %s", qPrintable(m_error));
        update();
        emit sourceChanged();
        return;
    }
    QByteArray data;
    data.resize(mesh.positions.size() * 6 * int(sizeof(float)));
    float *p = reinterpret_cast<float *>(data.data());
    for (int i = 0; i < mesh.positions.size(); i++)
    {
        const QVector3D &v = mesh.positions[i];
        const QVector3D &n = mesh.normals[i];
        *p++ = v.x(); *p++ = v.y(); *p++ = v.z();
        *p++ = n.x(); *p++ = n.y(); *p++ = n.z();
    }
    setVertexData(data);
    setStride(6 * sizeof(float));
    setPrimitiveType(PrimitiveType::Triangles);
    addAttribute(Attribute::PositionSemantic, 0, Attribute::F32Type);
    addAttribute(Attribute::NormalSemantic, 3 * sizeof(float), Attribute::F32Type);
    setBounds(mesh.min, mesh.max);
    m_triangles = mesh.TriangleCount();
    update();
    emit sourceChanged();
}

void LineGeometry::setPoints(const QVariantList &points)
{
    m_points = points;
    Rebuild();
    emit pointsChanged();
}

void LineGeometry::setSegments(bool segments)
{
    if (m_segments == segments)
        return;
    m_segments = segments;
    Rebuild();
    emit pointsChanged();
}

void LineGeometry::Rebuild()
{
    clear();
    QByteArray data;
    data.resize(m_points.size() * 3 * int(sizeof(float)));
    float *p = reinterpret_cast<float *>(data.data());
    constexpr float inf = std::numeric_limits<float>::max();
    QVector3D lo(inf, inf, inf), hi(-inf, -inf, -inf);
    for (const QVariant &var : m_points)
    {
        const QVector3D v = var.value<QVector3D>();
        *p++ = v.x(); *p++ = v.y(); *p++ = v.z();
        for (int k = 0; k < 3; k++)
        {
            lo[k] = std::min(lo[k], v[k]);
            hi[k] = std::max(hi[k], v[k]);
        }
    }
    setVertexData(data);
    setStride(3 * sizeof(float));
    setPrimitiveType(m_segments ? PrimitiveType::Lines : PrimitiveType::LineStrip);
    addAttribute(Attribute::PositionSemantic, 0, Attribute::F32Type);
    if (!m_points.isEmpty())
        setBounds(lo, hi);
    update();
}
