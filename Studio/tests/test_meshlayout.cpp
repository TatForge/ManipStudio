// The 3D view must show the same geometry the collision check uses: the
// display parts (VisualLayout over Solver::GetArmParts) are compared with the
// hulls Solver::MoveMeshes places; plus the obstacle and the STL reader.

// White-box access to Solver's meshes (as Tests/solver_test_access.h does):
// dependencies first, with normal access rules.
#include <QSettings>
#include <QSharedMemory>
#include <QString>
#include <qsettings.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "my_math.h"
#include "robo.h"
#include "meshcollider.h"
#include "armshape.h"

#define private public
#define protected public
#include "sharedmem.h"
#include "emulator.h"
#include "solver.h"
#undef protected
#undef private

#include "meshlayout.h"
#include "visuallayout.h"

#include <algorithm>

#include <QDir>
#include <QTemporaryDir>
#include <QtTest>

class TestMeshLayout : public QObject
{
    Q_OBJECT
private:
    static QMatrix4x4 ToQ(const Matrix4x4 &m)
    {
        QMatrix4x4 r;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                r(i, j) = float(m.at(i, j));
        return r;
    }

    // joint frames of the project's robot for joint angles in degrees
    static QVector<QMatrix4x4> Frames(const QVector<double> &deg)
    {
        Robo robot(JOINTS_COUNT);
        robot.LoadManip();
        Solver solver(robot);
        Angles a;
        for (int i = 0; i < JOINTS_COUNT; i++)
            a.q[i] = ToRad(deg[i]);
        Matrix4x4 TR[JOINTS_COUNT];
        solver.SolveDirectKinematic(a, TR);
        QVector<QMatrix4x4> frames;
        for (const auto &m : TR)
            frames.append(ToQ(m));
        return frames;
    }

    // the arm shape of the project's robot (body frames)
    static std::vector<ArmPart> Shape()
    {
        Robo robot(JOINTS_COUNT);
        robot.LoadManip();
        Solver solver(robot);
        return solver.GetArmParts();
    }

    static QVector<VisualLayout::Part> Display(const QVector<double> &deg) { return VisualLayout::Build(Frames(deg), Shape()); }

    static QVector3D Origin(const QMatrix4x4 &m) { return m.column(3).toVector3D(); }

    static QVector<VisualLayout::Part> Links(const QVector<VisualLayout::Part> &parts, int joint)
    {
        QVector<VisualLayout::Part> out;
        for (const auto &p : parts)
            if (p.role == VisualLayout::Role::Link && p.joint == joint && p.shape == VisualLayout::Shape::Cylinder)
                out << p;
        return out;
    }

    static void AddPoses()
    {
        QTest::addColumn<QVector<double>>("deg");
        QTest::newRow("zero") << QVector<double>{0, 0, 0, 0, 0, 0};
        QTest::newRow("folded") << QVector<double>{0, -45, 90, 45, 90, 0};
        QTest::newRow("work") << QVector<double>{17, -29, 52, 11, -23, 6};
        QTest::newRow("wrist") << QVector<double>{-120, -100, -60, 150, 110, -300};
    }

private slots:
    void initTestCase() { QVERIFY(QDir::setCurrent(QStringLiteral(MANIP_DATA_DIR))); }

    // --- display model built from the joint frames ---

    void displayLinksConnectEveryJoint_data() { AddPoses(); }
    void displayLinksConnectEveryJoint()
    {
        QFETCH(QVector<double>, deg);
        const QVector<QMatrix4x4> frames = Frames(deg);
        const auto parts = VisualLayout::Build(frames, Shape());
        for (int i = 0; i < 5; i++)
        {
            const auto links = Links(parts, i);
            QVERIFY2(!links.isEmpty(), qPrintable(QString::number(i)));
            // an unbroken tube from joint i to joint i + 1
            QVERIFY((links.first().from - Origin(frames[i])).length() < 1e-5);
            QVERIFY((links.last().to - Origin(frames[i + 1])).length() < 1e-5);
            for (int k = 1; k < links.size(); k++)
                QVERIFY((links[k].from - links[k - 1].to).length() < 1e-5);
        }
    }

    void displayLinksHaveTheDhLengths()
    {
        // zero pose: shoulder -> elbow and elbow -> wrist legs along x are a(1), a(2)
        const auto parts = Display({0, 0, 0, 0, 0, 0});
        const auto shoulder = Links(parts, 1);
        const auto elbow = Links(parts, 2);
        QVERIFY(!shoulder.isEmpty() && !elbow.isEmpty());
        QVERIFY(qAbs(shoulder.first().Length() - 0.375f) < 1e-5);
        QVERIFY(qAbs(elbow.first().Length() - 0.295f) < 1e-5);
        // the shoulder tube starts on the shoulder axis and ends above the elbow axis
        QVERIFY((shoulder.first().from - QVector3D(0, 0.1f, 0.2325f)).length() < 1e-5);
        QVERIFY((shoulder.first().to - QVector3D(0.375f, 0.1f, 0.2325f)).length() < 1e-5);
    }

    void displayDrumsSitOnTheJointAxes_data() { AddPoses(); }
    void displayDrumsSitOnTheJointAxes()
    {
        QFETCH(QVector<double>, deg);
        const QVector<QMatrix4x4> frames = Frames(deg);
        int drums = 0;
        for (const auto &p : VisualLayout::Build(frames, Shape()))
        {
            if (p.role != VisualLayout::Role::Joint)
                continue;
            drums++;
            const QVector3D z = frames[p.joint].column(2).toVector3D().normalized();
            QVERIFY((p.Center() - Origin(frames[p.joint])).length() < 1e-5);
            QVERIFY(qAbs(QVector3D::dotProduct((p.to - p.from).normalized(), z)) > 1 - 1e-5);
            // a Y-up primitive rotated by Rotation() points along the drum
            QVERIFY((p.Rotation().rotatedVector(QVector3D(0, 1, 0)) - (p.to - p.from).normalized()).length() < 1e-4);
        }
        QCOMPARE(drums, 6);
    }

    void displayModelStructureDoesNotDependOnThePose()
    {
        const auto a = Display({0, 0, 0, 0, 0, 0});
        const auto b = Display({-120, -100, -60, 150, 110, -300});
        QCOMPARE(a.size(), b.size());
        for (int k = 0; k < a.size(); k++)
        {
            QVERIFY(a[k].role == b[k].role && a[k].shape == b[k].shape && a[k].joint == b[k].joint);
            QCOMPARE(a[k].radius, b[k].radius);
            QVERIFY(qAbs(a[k].Length() - b[k].Length()) < 1e-5);
        }
    }

    void displayBaseStandsOnTheFloor()
    {
        const QVector<QMatrix4x4> frames = Frames({30, 0, 0, 0, 0, 0});
        bool found = false;
        for (const auto &p : VisualLayout::Build(frames, Shape()))
            if (p.role == VisualLayout::Role::Base)
            {
                found = true;
                QCOMPARE(p.from, QVector3D(0, 0, 0));
                QVERIFY(p.to.z() > 0 && p.to.z() < Origin(frames[0]).z());
            }
        QVERIFY(found);
        QVERIFY(VisualLayout::Build({}, Shape()).isEmpty());
    }

    void readsProjectStlFiles()
    {
        StlMesh mesh;
        QString error;
        QVERIFY2(ReadStlMesh(QStringLiteral("STLmodels/base.stl"), mesh, &error), qPrintable(error));
        QCOMPARE(mesh.TriangleCount(), 92);
        QCOMPARE(mesh.normals.size(), mesh.positions.size());
        QVERIFY(qAbs(mesh.min.z()) < 1e-4);
        QVERIFY(qAbs(mesh.max.z() - 0.185f) < 1e-3);
        QVERIFY(qAbs(mesh.max.x() - 0.062f) < 1e-3);

        QVERIFY2(ReadStlMesh(MeshLayout::ObstacleFile(), mesh, &error), qPrintable(error));
        QCOMPARE(mesh.TriangleCount(), 12);
        // flat normals are unit length (no degenerate triangles)
        for (const QVector3D &n : mesh.normals)
            QVERIFY(qAbs(n.length() - 1.f) < 1e-3);
    }

    void readsAsciiStl()
    {
        QTemporaryDir dir;
        const QString path = dir.filePath(QStringLiteral("tri.stl"));
        QFile f(path);
        QVERIFY(f.open(QIODevice::WriteOnly));
        f.write("solid t\n facet normal 0 0 1\n  outer loop\n   vertex 0 0 0\n   vertex 1 0 0\n   vertex 0 1 0\n"
                "  endloop\n endfacet\nendsolid t\n");
        f.close();
        StlMesh mesh;
        QVERIFY(ReadStlMesh(path, mesh));
        QCOMPARE(mesh.TriangleCount(), 1);
        QCOMPARE(mesh.normals[0], QVector3D(0, 0, 1));
        QCOMPARE(mesh.max, QVector3D(1, 1, 0));
    }

    void rejectsMissingAndBrokenFiles()
    {
        StlMesh mesh;
        QString error;
        QVERIFY(!ReadStlMesh(QStringLiteral("STLmodels/no_such.stl"), mesh, &error));
        QVERIFY(error.contains(QStringLiteral("cannot open")));

        QTemporaryDir dir;
        const QString path = dir.filePath(QStringLiteral("broken.stl"));
        QFile f(path);
        QVERIFY(f.open(QIODevice::WriteOnly));
        f.write(QByteArray(84, '\0') + QByteArray("\x05\0\0\0", 4));   // header claims more than there is
        f.close();
        QVERIFY(!ReadStlMesh(path, mesh, &error));
        QCOMPARE(mesh.TriangleCount(), 0);
    }

    // what the 3D view draws is what the collision check tests
    void displayPartsAreTheSolverHulls_data() { AddPoses(); }
    void displayPartsAreTheSolverHulls()
    {
        QFETCH(QVector<double>, deg);
        Robo robot(JOINTS_COUNT);
        QVERIFY(robot.LoadManip());
        Solver solver(robot);
        Angles a;
        for (int i = 0; i < JOINTS_COUNT; i++)
            a.q[i] = ToRad(deg[i]);
        Matrix4x4 TR[JOINTS_COUNT];
        solver.SolveDirectKinematic(a, TR);
        solver.MoveMeshes(TR);
        QVector<QMatrix4x4> frames;
        for (const auto &m : TR)
            frames.append(ToQ(m));
        const auto parts = VisualLayout::Build(frames, solver.GetArmParts());
        QCOMPARE(size_t(parts.size()), solver.GetArmParts().size());
        QCOMPARE(solver.m_meshes_buff.size(), size_t(parts.size()) + 1);    // + the obstacle
        for (int i = 0; i < parts.size(); i++)
        {
            const auto &p = parts[i];
            const MeshCollider &hull = solver.m_meshes_buff[i];
            for (ushort k = 0; k < hull.Size(); k++)
            {
                const QVector3D v(float(hull.point(k).x), float(hull.point(k).y), float(hull.point(k).z));
                const QVector3D ab = p.to - p.from;
                const float t = ab.lengthSquared() > 0 ? std::clamp(QVector3D::dotProduct(v - p.from, ab) / ab.lengthSquared(), 0.f, 1.f) : 0.f;
                const float d = (v - (p.from + ab * t)).length();
                QVERIFY2(d > p.radius - 1e-4f && d < p.radius * 1.05f, qPrintable(QStringLiteral("part %1 vertex %2: %3").arg(i).arg(k).arg(d)));
            }
        }
    }

    void obstacleSitsWhereTheSolverPutsIt()
    {
        Robo robot(JOINTS_COUNT);
        QVERIFY(robot.LoadManip());
        Solver solver(robot);
        // the solver moves its obstacle mesh once, in the constructor
        const MeshCollider &cube = solver.m_meshes.back();
        Vector3 lo(1e9, 1e9, 1e9), hi(-1e9, -1e9, -1e9);
        for (ushort k = 0; k < cube.Size(); k++)
        {
            const Vector3 p = cube.point(k);
            lo = Vector3(std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z));
            hi = Vector3(std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z));
        }
        StlMesh mesh;
        QVERIFY(ReadStlMesh(MeshLayout::ObstacleFile(), mesh));
        const QMatrix4x4 place = MeshLayout::ObstaclePlacement();
        const QVector3D vlo = place.map(mesh.min), vhi = place.map(mesh.max);
        QVERIFY((vlo - QVector3D(float(lo.x), float(lo.y), float(lo.z))).length() < 1e-4);
        QVERIFY((vhi - QVector3D(float(hi.x), float(hi.y), float(hi.z))).length() < 1e-4);
    }
};

QTEST_GUILESS_MAIN(TestMeshLayout)
#include "test_meshlayout.moc"
