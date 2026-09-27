// Scenario tests of the GUI backend: the same calls QML makes, run headless
// against the real core (settings and STL meshes from MANIP_DATA_DIR).
#include "robotbackend.h"

#include <QSignalSpy>
#include <QtTest>

#include <memory>

class TestRobotBackend : public QObject
{
    Q_OBJECT
private:
    std::unique_ptr<RobotBackend> m_backend;

    static constexpr int kTimeoutMs = 20000;

    // waits until the worker has answered every request sent so far
    void WaitPose()
    {
        QSignalSpy spy(m_backend.get(), &RobotBackend::poseChanged);
        QVERIFY(spy.wait(kTimeoutMs));
        while (spy.wait(50))
            ;
    }

    static double At(const QVariantList &l, int i) { return l.at(i).toDouble(); }

private slots:
    void init()
    {
        m_backend = std::make_unique<RobotBackend>(QStringLiteral(MANIP_DATA_DIR));
        QTRY_VERIFY_WITH_TIMEOUT(m_backend->ready(), kTimeoutMs);
        QTRY_COMPARE_WITH_TIMEOUT(m_backend->tcp().size(), 6, kTimeoutMs);
    }

    void cleanup() { m_backend.reset(); }

    void loadsJointLimitsFromSettings()
    {
        const QVariantList limits = m_backend->limits();
        QCOMPARE(limits.size(), 6);
        QCOMPARE(qRound(limits[1].toMap()[QStringLiteral("min")].toDouble()), -150);
        QCOMPARE(qRound(limits[1].toMap()[QStringLiteral("max")].toDouble()), 30);
        QVERIFY(!m_backend->busy());
    }

    void zeroPoseTcpMatchesKinematics()
    {
        const QVariantList tcp = m_backend->tcp();
        QVERIFY(qAbs(At(tcp, 0) - 0.67) < 1e-3);
        QVERIFY(qAbs(At(tcp, 1) - 0.2431) < 1e-3);
        QVERIFY(qAbs(At(tcp, 2) - 0.0615) < 1e-3);
        QCOMPARE(m_backend->pose().frames.size(), 6);
        // the last frame origin is the TCP the panel shows
        const QVariantList frames = m_backend->frames();
        QCOMPARE(frames.size(), 6);
        const QVector3D flange = frames.last().value<QVector3D>();
        QVERIFY(qAbs(flange.x() - At(tcp, 0)) < 1e-5);
        QVERIFY(qAbs(flange.y() - At(tcp, 1)) < 1e-5);
        QVERIFY(qAbs(flange.z() - At(tcp, 2)) < 1e-5);
    }

    void jogIsClampedToJointLimits()
    {
        m_backend->setJoint(1, 80);
        QCOMPARE(qRound(At(m_backend->joints(), 1)), 30);
        m_backend->setJoint(0, -500);
        QCOMPARE(qRound(At(m_backend->joints(), 0)), -170);
        WaitPose();
        // the computed pose is for the clamped angles, not the requested ones
        QVERIFY(qAbs(m_backend->pose().joints_rad[1] - qDegreesToRadians(30.0)) < 1e-9);
    }

    void jogMovesTcp()
    {
        const QVariantList before = m_backend->tcp();
        m_backend->setJoint(1, -30);
        WaitPose();
        const QVariantList after = m_backend->tcp();
        // the shoulder lifts the arm: the tool rises by ~0.36 m
        QVERIFY2(At(after, 2) - At(before, 2) > 0.3, qPrintable(QStringLiteral("z %1 -> %2").arg(At(before, 2)).arg(At(after, 2))));
    }

    void moveJPlaysBackToTarget()
    {
        const QVariantList target{20, -40, 60, 10, 45, -30};
        QSignalSpy finished(m_backend.get(), &RobotBackend::motionFinished);
        QSignalSpy errors(m_backend.get(), &RobotBackend::errorOccurred);
        m_backend->moveJ(target, 90, 180);
        QVERIFY(m_backend->busy());
        QVERIFY(finished.wait(kTimeoutMs));
        QCOMPARE(errors.count(), 0);
        QVERIFY(m_backend->sampleCount() > 10);
        QVERIFY(m_backend->duration() > 0.3);
        QCOMPARE(m_backend->progress(), 1.0);
        QVERIFY(!m_backend->playing());
        for (int i = 0; i < 6; i++)
            QVERIFY2(qAbs(At(m_backend->joints(), i) - At(target, i)) < 1e-3, qPrintable(QString::number(i)));
    }

    void moveJReportsTcpPathFromStartToTarget()
    {
        const QVariantList start = m_backend->tcp();
        QSignalSpy finished(m_backend.get(), &RobotBackend::motionFinished);
        m_backend->moveJ({20, -40, 60, 10, 45, -30}, 90, 180);
        QVERIFY(finished.wait(kTimeoutMs));
        WaitPose();
        const QVariantList path = m_backend->tcpPath();
        QCOMPARE(path.size(), m_backend->sampleCount());
        const QVector3D first = path.first().value<QVector3D>(), last = path.last().value<QVector3D>();
        const QVariantList end = m_backend->tcp();
        QVERIFY((first - QVector3D(At(start, 0), At(start, 1), At(start, 2))).length() < 1e-4);
        QVERIFY((last - QVector3D(At(end, 0), At(end, 1), At(end, 2))).length() < 1e-4);
    }

    void scenePartsFollowThePose()
    {
        const QVariantList parts = m_backend->meshParts();
        QCOMPARE(parts.size(), 9);
        for (const QVariant &p : parts)
            QVERIFY(QFile::exists(p.toMap()[QStringLiteral("source")].toUrl().toLocalFile()));
        QVERIFY(parts.last().toMap()[QStringLiteral("obstacle")].toBool());
        QCOMPARE(m_backend->meshPoses().size(), 9);

        // the last joint part sits at frame 4, the tool frame at the flange (TCP)
        const QVariantMap tool = m_backend->toolPose();
        const QVector3D flange = tool[QStringLiteral("position")].value<QVector3D>();
        const QVariantList tcp = m_backend->tcp();
        QVERIFY((flange - QVector3D(At(tcp, 0), At(tcp, 1), At(tcp, 2))).length() < 1e-5);

        const QVector3D before = m_backend->meshPoses()[7].toMap()[QStringLiteral("position")].value<QVector3D>();
        m_backend->setJoint(0, 90);
        WaitPose();
        const QVector3D after = m_backend->meshPoses()[7].toMap()[QStringLiteral("position")].value<QVector3D>();
        // base rotation by 90 deg: (x, y) -> (-y, x)
        QVERIFY(qAbs(after.x() + before.y()) < 1e-4);
        QVERIFY(qAbs(after.y() - before.x()) < 1e-4);
        QVERIFY(qAbs(after.z() - before.z()) < 1e-4);
    }

    void moveJOutsideLimitsReportsError()
    {
        QSignalSpy errors(m_backend.get(), &RobotBackend::errorOccurred);
        m_backend->moveJ({0, 60, 0, 0, 0, 0}, 90, 180);
        QVERIFY(errors.wait(kTimeoutMs));
        QVERIFY(errors.first().first().toString().contains(QStringLiteral("outside its limit")));
        QVERIFY(!m_backend->busy());
        QCOMPARE(m_backend->sampleCount(), 0);
    }

    void moveJToCurrentPoseReportsError()
    {
        QSignalSpy errors(m_backend.get(), &RobotBackend::errorOccurred);
        m_backend->moveJ(m_backend->joints(), 90, 180);
        QVERIFY(errors.wait(kTimeoutMs));
        QVERIFY(!m_backend->busy());
    }

    void moveJWhileBusyIsRejected()
    {
        QSignalSpy errors(m_backend.get(), &RobotBackend::errorOccurred);
        QSignalSpy finished(m_backend.get(), &RobotBackend::motionFinished);
        m_backend->moveJ({10, 0, 0, 0, 0, 0}, 90, 180);
        m_backend->moveJ({20, 0, 0, 0, 0, 0}, 90, 180);
        QCOMPARE(errors.count(), 1);
        QVERIFY(finished.wait(kTimeoutMs));
        QVERIFY(qAbs(At(m_backend->joints(), 0) - 10) < 1e-3);
    }
};

QTEST_GUILESS_MAIN(TestRobotBackend)
#include "test_robotbackend.moc"
