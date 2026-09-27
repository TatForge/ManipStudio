#ifndef SCENEGEOMETRY_H
#define SCENEGEOMETRY_H

#include <QQuick3DGeometry>
#include <QUrl>
#include <QVariantList>
#include <QtQml/qqmlregistration.h>

// Triangles of an STL file, flat shaded.
class StlGeometry : public QQuick3DGeometry
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(QUrl source READ source WRITE setSource NOTIFY sourceChanged)
    Q_PROPERTY(int triangleCount READ triangleCount NOTIFY sourceChanged)
    Q_PROPERTY(QString error READ error NOTIFY sourceChanged)
public:
    QUrl source() const { return m_source; }
    void setSource(const QUrl &source);
    int triangleCount() const { return m_triangles; }
    QString error() const { return m_error; }

signals:
    void sourceChanged();

private:
    QUrl m_source;
    int m_triangles = 0;
    QString m_error;
};

// Unlit line work: a strip through `points`, or separate segments (pairs of
// points) when `segments` is set. Points are QVector3D.
class LineGeometry : public QQuick3DGeometry
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(QVariantList points READ points WRITE setPoints NOTIFY pointsChanged)
    Q_PROPERTY(bool segments READ segments WRITE setSegments NOTIFY pointsChanged)
public:
    QVariantList points() const { return m_points; }
    void setPoints(const QVariantList &points);
    bool segments() const { return m_segments; }
    void setSegments(bool segments);

signals:
    void pointsChanged();

private:
    void Rebuild();

    QVariantList m_points;
    bool m_segments = false;
};

#endif // SCENEGEOMETRY_H
