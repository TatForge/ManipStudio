// Scenario tests of the GUI backend: the same calls QML makes, run headless
// against the real core (settings and STL meshes from MANIP_DATA_DIR).
#include "robotbackend.h"

#include <QDir>
#include <QFile>
#include <QSignalSpy>
#include <QTemporaryDir>
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

    static QVariantMap Peak(const RobotBackend &b, int joint) { return b.peaks().at(joint).toMap(); }

    // plans and plays the default test move to the end
    void RunMove(const QVariantList &target = {20, -40, 60, 10, 45, -30})
    {
        QSignalSpy finished(m_backend.get(), &RobotBackend::motionFinished);
        m_backend->moveJ(target, 90, 180);
        QVERIFY(finished.wait(kTimeoutMs));
    }

    // a copy of the project's settings file and meshes
    static bool MakeDataDir(QTemporaryDir &dir)
    {
        const QDir src(QStringLiteral(MANIP_DATA_DIR));
        if (!QFile::copy(src.filePath(QStringLiteral("ManipulatorSettigns.ini")), QDir(dir.path()).filePath(QStringLiteral("ManipulatorSettigns.ini"))))
            return false;
        QFile::setPermissions(QDir(dir.path()).filePath(QStringLiteral("ManipulatorSettigns.ini")), QFile::ReadOwner | QFile::WriteOwner);
        QDir(dir.path()).mkdir(QStringLiteral("STLmodels"));
        for (const QString &f : QDir(src.filePath(QStringLiteral("STLmodels"))).entryList(QDir::Files))
            if (!QFile::copy(src.filePath(QStringLiteral("STLmodels/") + f), QDir(dir.path()).filePath(QStringLiteral("STLmodels/") + f)))
                return false;
        return true;
    }

    void StartIn(const QString &dir)
    {
        m_backend = std::make_unique<RobotBackend>(dir);
        QTRY_VERIFY_WITH_TIMEOUT(m_backend->ready(), kTimeoutMs);
        QTRY_COMPARE_WITH_TIMEOUT(m_backend->tcp().size(), 6, kTimeoutMs);
    }

    // applies and waits until the solver has been rebuilt
    void Apply(const QVariantMap &settings, bool save)
    {
        QSignalSpy changed(m_backend.get(), &RobotBackend::settingsChanged);
        m_backend->applySettings(settings, save);
        QVERIFY(changed.wait(kTimeoutMs));
        QTRY_VERIFY_WITH_TIMEOUT(!m_backend->busy(), kTimeoutMs);
    }

    static QVariantMap SetIn(QVariantMap map, const QString &list, int index, const QString &key, const QVariant &value)
    {
        QVariantList l = map[list].toList();
        QVariantMap row = l[index].toMap();
        row[key] = value;
        l[index] = row;
        map[list] = l;
        return map;
    }

    static QVector3D Xyz(const QVariantList &tcp) { return QVector3D(tcp[0].toFloat(), tcp[1].toFloat(), tcp[2].toFloat()); }

    // waits for the end of a planned motion (or its error)
    bool WaitMotion()
    {
        QSignalSpy finished(m_backend.get(), &RobotBackend::motionFinished);
        QSignalSpy errors(m_backend.get(), &RobotBackend::errorOccurred);
        for (int waited = 0; waited < kTimeoutMs && finished.isEmpty() && errors.isEmpty(); waited += 50)
            QTest::qWait(50);
        if (!errors.isEmpty())
            qWarning("motion failed: %s", qPrintable(errors.first().first().toString()));
        // the pose of the last sample may still be on its way from the worker
        QTest::qWait(150);
        return !finished.isEmpty();
    }

    // a pose in the middle of the work space, away from singularities
    void GoToWorkPose()
    {
        m_backend->setJoints({17, -29, 52, 11, -23, 6});
        WaitPose();
    }

    static double DistanceToSegment(const QVector3D &p, const QVector3D &a, const QVector3D &b)
    {
        const QVector3D ab = b - a;
        const float t = std::clamp(QVector3D::dotProduct(p - a, ab) / ab.lengthSquared(), 0.f, 1.f);
        return (p - (a + ab * t)).length();
    }

    double ShoulderPeak()
    {
        return Peak(*m_backend, 1)[QStringLiteral("peak")].toDouble();
    }

    // data directory with the project's model but a lower motor limit for one joint
    static bool MakeDataDirWithMomentLimit(QTemporaryDir &dir, int joint, double limit)
    {
        const QDir src(QStringLiteral(MANIP_DATA_DIR));
        QFile ini(src.filePath(QStringLiteral("ManipulatorSettigns.ini")));
        if (!ini.open(QIODevice::ReadOnly))
            return false;
        QString text = QString::fromUtf8(ini.readAll());
        const int section = text.indexOf(QStringLiteral("[max%20moments]"));
        const int at = text.indexOf(QRegularExpression(QStringLiteral("\r?\n%1=").arg(joint)), section);
        if (section < 0 || at < 0)
            return false;
        const int value = text.indexOf(QLatin1Char('='), at) + 1;
        const int end = text.indexOf(QRegularExpression(QStringLiteral("\r?\n")), value);
        text.replace(value, end - value, QString::number(limit, 'g', 17));
        QFile out(QDir(dir.path()).filePath(QStringLiteral("ManipulatorSettigns.ini")));
        if (!out.open(QIODevice::WriteOnly) || out.write(text.toUtf8()) < 0)
            return false;
        out.close();
        QDir(dir.path()).mkdir(QStringLiteral("STLmodels"));
        for (const QString &f : QDir(src.filePath(QStringLiteral("STLmodels"))).entryList(QDir::Files))
            if (!QFile::copy(src.filePath(QStringLiteral("STLmodels/") + f), QDir(dir.path()).filePath(QStringLiteral("STLmodels/") + f)))
                return false;
        return true;
    }

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
        // the obstacle is the only STL part
        const QVariantList parts = m_backend->meshParts();
        QCOMPARE(parts.size(), 1);
        QVERIFY(QFile::exists(parts[0].toMap()[QStringLiteral("source")].toUrl().toLocalFile()));
        QVERIFY(parts[0].toMap()[QStringLiteral("obstacle")].toBool());
        QCOMPARE(m_backend->meshPoses().size(), 1);
        // the arm is drawn from the core's shape: one drum per joint and more
        QVERIFY(m_backend->visualPartCount() >= 14);
        QCOMPARE(m_backend->visualParts().size(), m_backend->visualPartCount());

        // the tool frame sits at the flange (TCP)
        const QVariantMap tool = m_backend->toolPose();
        const QVector3D flange = tool[QStringLiteral("position")].value<QVector3D>();
        const QVariantList tcp = m_backend->tcp();
        QVERIFY((flange - QVector3D(At(tcp, 0), At(tcp, 1), At(tcp, 2))).length() < 1e-5);

        auto lastDrum = [this] {
            QVector3D at;
            for (const QVariant &v : m_backend->visualParts())
                if (v.toMap()[QStringLiteral("role")] == QLatin1String("joint"))
                    at = v.toMap()[QStringLiteral("position")].value<QVector3D>();
            return at;
        };
        const QVector3D before = lastDrum();
        m_backend->setJoint(0, 90);
        WaitPose();
        const QVector3D after = lastDrum();
        // base rotation by 90 deg: (x, y) -> (-y, x)
        QVERIFY(qAbs(after.x() + before.y()) < 1e-4);
        QVERIFY(qAbs(after.y() - before.x()) < 1e-4);
        QVERIFY(qAbs(after.z() - before.z()) < 1e-4);
    }

    void momentsCoverEveryPlannedSample()
    {
        RunMove();
        QCOMPARE(m_backend->maxMoments().size(), 6);
        QCOMPARE(qRound(At(m_backend->maxMoments(), 0)), 180);
        QCOMPARE(m_backend->momentCount(), m_backend->sampleCount());
        QVERIFY(m_backend->trajectoryError().isEmpty());
        for (int j = 0; j < 6; j++)
        {
            const QList<QPointF> tau = m_backend->Series(QStringLiteral("moment"), j);
            QCOMPARE(tau.size(), m_backend->sampleCount());
            QCOMPARE(tau.first().x(), 0.0);
            QVERIFY(qAbs(tau.last().x() - m_backend->duration()) < 1e-9);
        }
        // the shoulder holds the arm against gravity: clearly non-zero moment
        double shoulder = 0;
        for (const QPointF &p : m_backend->Series(QStringLiteral("moment"), 1))
            shoulder = std::max(shoulder, std::abs(p.y()));
        QVERIFY(shoulder > 5);
    }

    void peaksAreWithinTheMotorLimitsOfAnAcceptedMove()
    {
        RunMove();
        const QVariantList peaks = m_backend->peaks();
        QCOMPARE(peaks.size(), 6);
        for (int j = 0; j < 6; j++)
        {
            const QVariantMap p = peaks[j].toMap();
            QVERIFY(!p[QStringLiteral("exceeded")].toBool());
            QVERIFY(p[QStringLiteral("ratio")].toDouble() <= 1.0);
            QVERIFY(p[QStringLiteral("time")].toDouble() <= m_backend->duration() + 1e-9);
            // the peak is the largest |tau| of the curve
            double peak = 0;
            for (const QPointF &pt : m_backend->Series(QStringLiteral("moment"), j))
                peak = std::max(peak, std::abs(pt.y()));
            QVERIFY(qAbs(p[QStringLiteral("peak")].toDouble() - peak) < 1e-6);
        }
    }

    void kinematicSeriesEndAtTheTarget()
    {
        const QVariantList target{20, -40, 60, 10, 45, -30};
        RunMove(target);
        for (int j = 0; j < 6; j++)
        {
            QVERIFY(qAbs(m_backend->Series(QStringLiteral("position"), j).last().y() - At(target, j)) < 1e-3);
            QVERIFY(qAbs(m_backend->Series(QStringLiteral("velocity"), j).last().y()) < 1.0);
        }
        // synchronized MoveJ: the joint with the largest travel reaches the commanded speed
        double top = 0;
        for (int j = 0; j < 6; j++)
            for (const QPointF &p : m_backend->Series(QStringLiteral("velocity"), j))
                top = std::max(top, std::abs(p.y()));
        QVERIFY2(top > 80 && top < 92, qPrintable(QString::number(top)));
        const QVariantMap range = m_backend->seriesRange(QStringLiteral("position"));
        QVERIFY(range[QStringLiteral("yMin")].toDouble() <= -40 + 1e-3);
        QVERIFY(range[QStringLiteral("yMax")].toDouble() >= 60 - 1e-3);
        QVERIFY(m_backend->Series(QStringLiteral("nonsense"), 0).isEmpty());
    }

    void seekShowsThePoseAtThatTime()
    {
        RunMove();
        const double half = m_backend->duration() / 2;
        m_backend->seek(half);
        QVERIFY(!m_backend->playing());
        QVERIFY(qAbs(m_backend->time() - half) <= 0.0051);
        const int k = qRound(m_backend->time() * 100);   // dt = 10 ms
        for (int j = 0; j < 6; j++)
            QVERIFY(qAbs(At(m_backend->joints(), j) - m_backend->Series(QStringLiteral("position"), j).at(k).y()) < 1e-6);
        WaitPose();
        // replay continues from there and ends at the target
        QSignalSpy finished(m_backend.get(), &RobotBackend::motionFinished);
        m_backend->play();
        QVERIFY(m_backend->playing());
        QVERIFY(finished.wait(kTimeoutMs));
        QCOMPARE(m_backend->progress(), 1.0);
    }

    void exportsTheTrajectoryAsCsv()
    {
        QTemporaryDir dir;
        const QUrl file = QUrl::fromLocalFile(QDir(dir.path()).filePath(QStringLiteral("move.csv")));
        QVERIFY(!m_backend->exportCsv(file));      // nothing planned yet
        RunMove();
        QVERIFY(m_backend->exportCsv(file));
        QFile f(file.toLocalFile());
        QVERIFY(f.open(QIODevice::ReadOnly | QIODevice::Text));
        const QStringList lines = QString::fromUtf8(f.readAll()).split(QLatin1Char('\n'), Qt::SkipEmptyParts);
        QCOMPARE(lines.size(), m_backend->sampleCount() + 1);
        const QStringList header = lines.first().split(QLatin1Char(','));
        QCOMPARE(header.size(), 1 + 18 + 6);
        QCOMPARE(header[1], QStringLiteral("q1_deg"));
        QCOMPARE(header.last(), QStringLiteral("tau6_Nm"));
        const QStringList last = lines.last().split(QLatin1Char(','));
        QCOMPARE(last.size(), header.size());
        QVERIFY(qAbs(last[1].toDouble() - 20) < 1e-3);
        QVERIFY(qAbs(last[0].toDouble() - m_backend->duration()) < 1e-6);
    }

    void motorLimitStopsThePlanAndShowsWhere()
    {
        // |tau2| along the default move, then a limit half way between its extremes
        RunMove();
        double lo = 1e9, hi = 0;
        for (const QPointF &p : m_backend->Series(QStringLiteral("moment"), 1))
        {
            lo = std::min(lo, std::abs(p.y()));
            hi = std::max(hi, std::abs(p.y()));
        }
        const int full = m_backend->momentCount();
        QVERIFY(hi - lo > 1.0);
        const double limit = (lo + hi) / 2;

        QTemporaryDir dir;
        QVERIFY(MakeDataDirWithMomentLimit(dir, 1, limit));
        m_backend = std::make_unique<RobotBackend>(dir.path());
        QTRY_VERIFY_WITH_TIMEOUT(m_backend->ready(), kTimeoutMs);
        QVERIFY(qAbs(At(m_backend->maxMoments(), 1) - limit) < 1e-6);
        m_backend->setJoints({0, 0, 0, 0, 0, 0});
        WaitPose();

        QSignalSpy errors(m_backend.get(), &RobotBackend::errorOccurred);
        m_backend->moveJ({20, -40, 60, 10, 45, -30}, 90, 180);
        QVERIFY(errors.wait(kTimeoutMs));
        QVERIFY(!m_backend->busy());
        QVERIFY(!m_backend->trajectoryError().isEmpty());
        QCOMPARE(m_backend->sampleCount(), 0);
        QVERIFY(m_backend->momentCount() > 0);
        QVERIFY(m_backend->momentCount() < full);
        const QVariantMap p = Peak(*m_backend, 1);
        QVERIFY(p[QStringLiteral("exceeded")].toBool());
        // the last evaluated sample is the one over the limit
        QVERIFY(std::abs(m_backend->Series(QStringLiteral("moment"), 1).last().y()) > limit);
    }

    void settingsExposeTheModelInUiUnits()
    {
        const QVariantMap st = m_backend->settings();
        const QVariantList dh = st[QStringLiteral("dh")].toList();
        QCOMPARE(dh.size(), 6);
        QCOMPARE(dh[1].toMap()[QStringLiteral("a")].toDouble(), 0.375);
        QCOMPARE(qRound(dh[0].toMap()[QStringLiteral("alpha")].toDouble()), -90);    // degrees
        const QVariantList limits = st[QStringLiteral("limits")].toList();
        QCOMPARE(qRound(limits[1].toMap()[QStringLiteral("min")].toDouble()), -150);
        QCOMPARE(st[QStringLiteral("maxMoments")].toList()[0].toDouble(), 180.0);
        QCOMPARE(st[QStringLiteral("floorZ")].toDouble(), 0.0);
        QCOMPARE(st[QStringLiteral("payload")].toMap()[QStringLiteral("mass")].toDouble(), 5.0);
        QCOMPARE(st[QStringLiteral("links")].toList()[1].toMap()[QStringLiteral("mass")].toDouble(), 2.9325);
        QCOMPARE(st[QStringLiteral("links")].toList()[1].toMap()[QStringLiteral("inertia")].toList().size(), 6);
        QCOMPARE(st[QStringLiteral("jointRadii")].toList()[0].toDouble(), 0.075);
    }

    // a heavier payload, applied without saving: the next plan needs more shoulder torque,
    // the settings file stays as it was
    void appliedPayloadChangesTheMomentsButNotTheFile()
    {
        QTemporaryDir dir;
        QVERIFY(MakeDataDir(dir));
        StartIn(dir.path());
        QFile ini(QDir(dir.path()).filePath(QStringLiteral("ManipulatorSettigns.ini")));
        QVERIFY(ini.open(QIODevice::ReadOnly));
        const QByteArray before = ini.readAll();
        ini.close();

        RunMove();
        const double light = ShoulderPeak();
        QVariantMap st = m_backend->settings();
        QVariantMap payload = st[QStringLiteral("payload")].toMap();
        payload[QStringLiteral("mass")] = 15.0;
        st[QStringLiteral("payload")] = payload;
        Apply(st, false);
        QCOMPARE(m_backend->sampleCount(), 0);          // the old plan is gone
        QCOMPARE(m_backend->settings()[QStringLiteral("payload")].toMap()[QStringLiteral("mass")].toDouble(), 15.0);
        QVERIFY(m_backend->status().contains(QStringLiteral("not saved")));

        m_backend->setJoints({0, 0, 0, 0, 0, 0});
        WaitPose();
        RunMove();
        QVERIFY2(ShoulderPeak() > light + 20, qPrintable(QStringLiteral("%1 -> %2").arg(light).arg(ShoulderPeak())));

        QVERIFY(ini.open(QIODevice::ReadOnly));
        QCOMPARE(ini.readAll(), before);
    }

    void savedSettingsAreLoadedOnTheNextStart()
    {
        QTemporaryDir dir;
        QVERIFY(MakeDataDir(dir));
        StartIn(dir.path());
        QVariantMap st = SetIn(m_backend->settings(), QStringLiteral("dh"), 1, QStringLiteral("a"), 0.4);
        st = SetIn(st, QStringLiteral("limits"), 0, QStringLiteral("max"), 100.0);
        QVariantList radii = st[QStringLiteral("jointRadii")].toList();
        radii[2] = 0.07;
        st[QStringLiteral("jointRadii")] = radii;
        Apply(st, true);
        QVERIFY(m_backend->status().contains(QStringLiteral("saved to")));

        m_backend.reset();
        StartIn(dir.path());
        const QVariantMap back = m_backend->settings();
        QCOMPARE(back[QStringLiteral("dh")].toList()[1].toMap()[QStringLiteral("a")].toDouble(), 0.4);
        QVERIFY(qAbs(back[QStringLiteral("limits")].toList()[0].toMap()[QStringLiteral("max")].toDouble() - 100) < 1e-9);
        QCOMPARE(back[QStringLiteral("jointRadii")].toList()[2].toDouble(), 0.07);
        // the longer upper arm reaches further: zero-pose TCP x = 0.4 + 0.295
        QVERIFY(qAbs(At(m_backend->tcp(), 0) - 0.695) < 1e-6);
    }

    void invalidSettingsAreRejectedAndChangeNothing()
    {
        const QVariantMap before = m_backend->settings();
        QSignalSpy errors(m_backend.get(), &RobotBackend::errorOccurred);
        m_backend->applySettings(SetIn(before, QStringLiteral("limits"), 2, QStringLiteral("min"), 200.0), false);
        QVERIFY(errors.wait(kTimeoutMs));
        QVERIFY(errors.first().first().toString().contains(QStringLiteral("lower limit")));
        QTRY_VERIFY_WITH_TIMEOUT(!m_backend->busy(), kTimeoutMs);
        QCOMPARE(m_backend->settings(), before);

        // a field that is not a number never reaches the solver
        errors.clear();
        m_backend->applySettings(SetIn(before, QStringLiteral("dh"), 0, QStringLiteral("d"), QStringLiteral("abc")), false);
        QCOMPARE(errors.count(), 1);
        QVERIFY(!m_backend->busy());
    }

    void reloadDiscardsUnsavedChanges()
    {
        QVariantMap st = m_backend->settings();
        QVariantMap payload = st[QStringLiteral("payload")].toMap();
        payload[QStringLiteral("mass")] = 9.0;
        st[QStringLiteral("payload")] = payload;
        Apply(st, false);
        QCOMPARE(m_backend->settings()[QStringLiteral("payload")].toMap()[QStringLiteral("mass")].toDouble(), 9.0);

        QSignalSpy changed(m_backend.get(), &RobotBackend::settingsChanged);
        m_backend->reloadSettings();
        QVERIFY(changed.wait(kTimeoutMs));
        QTRY_VERIFY_WITH_TIMEOUT(!m_backend->busy(), kTimeoutMs);
        QCOMPARE(m_backend->settings()[QStringLiteral("payload")].toMap()[QStringLiteral("mass")].toDouble(), 5.0);
        QVERIFY(m_backend->status().contains(QStringLiteral("reloaded")));
    }

    void jointsAreKeptInsideNewLimits()
    {
        m_backend->setJoint(0, 100);
        Apply(SetIn(m_backend->settings(), QStringLiteral("limits"), 0, QStringLiteral("max"), 50.0), false);
        QVERIFY(qAbs(At(m_backend->joints(), 0) - 50) < 1e-9);
        QVERIFY(qAbs(m_backend->limits()[0].toMap()[QStringLiteral("max")].toDouble() - 50) < 1e-9);
    }

    // ------------------------------------------------------------ MoveL / MoveC / TCP jog

    void moveLRunsStraightToTheTarget()
    {
        GoToWorkPose();
        const QVariantList start = m_backend->tcp();
        QVariantList target = start;
        target[1] = start[1].toDouble() + 0.05;
        target[2] = start[2].toDouble() - 0.03;
        m_backend->moveL(target, 0.1, 0.5);
        QVERIFY(WaitMotion());
        const QVariantList end = m_backend->tcp();
        QVERIFY2((Xyz(end) - Xyz(target)).length() < 1e-3, qPrintable(QString::number((Xyz(end) - Xyz(target)).length())));
        for (int k = 3; k < 6; k++)
            QVERIFY(qAbs(At(end, k) - At(start, k)) < 0.5);         // orientation kept (deg)
        for (const QVariant &p : m_backend->tcpPath())
            QVERIFY(DistanceToSegment(p.value<QVector3D>(), Xyz(start), Xyz(target)) < 1e-3);
        QVERIFY(m_backend->momentCount() == m_backend->sampleCount());
    }

    // +360 deg about an axis is the same pose: no full turn of the wrist
    void moveLTakesTheShortWayRoundTheAngles()
    {
        GoToWorkPose();
        QVariantList target = m_backend->tcp();
        target[3] = target[3].toDouble() + 360;
        QSignalSpy errors(m_backend.get(), &RobotBackend::errorOccurred);
        m_backend->moveL(target, 0.1, 0.5);
        QVERIFY(errors.wait(kTimeoutMs));
        QVERIFY2(errors.first().first().toString().contains(QStringLiteral("equal")), qPrintable(errors.first().first().toString()));
    }

    void moveCPassesThroughTheMiddlePoint()
    {
        GoToWorkPose();
        const QVariantList start = m_backend->tcp();
        const QVector3D a = Xyz(start);
        const QVector3D b = a + QVector3D(-0.05f, 0, -0.05f), c = a + QVector3D(-0.1f, 0, 0);
        QVariantList end = start;
        end[0] = c.x();
        end[1] = c.y();
        end[2] = c.z();
        m_backend->moveC({b.x(), b.y(), b.z()}, end, 0.1, 0.5);
        QVERIFY(WaitMotion());
        QVERIFY((Xyz(m_backend->tcp()) - c).length() < 1e-3);
        // a half circle of radius 5 cm around a + (-0.05, 0, 0)
        const QVector3D centre = a + QVector3D(-0.05f, 0, 0);
        double closest = 1;
        for (const QVariant &v : m_backend->tcpPath())
        {
            const QVector3D p = v.value<QVector3D>();
            QVERIFY(qAbs((p - centre).length() - 0.05) < 1e-3);
            closest = std::min(closest, double((p - b).length()));
        }
        QVERIFY(closest < 2e-3);
    }

    void jogMovesTheToolAlongAnAxis()
    {
        GoToWorkPose();
        const QVariantList before = m_backend->tcp();
        m_backend->jogTcp(0, 0.02);             // +20 mm along X
        QVERIFY(WaitMotion());
        const QVariantList after = m_backend->tcp();
        QVERIFY(qAbs(At(after, 0) - At(before, 0) - 0.02) < 1e-3);
        QVERIFY(qAbs(At(after, 1) - At(before, 1)) < 1e-3);
        QVERIFY(qAbs(At(after, 2) - At(before, 2)) < 1e-3);

        m_backend->jogTcp(5, 10);               // +10 deg yaw
        QVERIFY(WaitMotion());
        const QVariantList turned = m_backend->tcp();
        QVERIFY(qAbs(At(turned, 5) - At(after, 5) - 10) < 0.5);
        QVERIFY((Xyz(turned) - Xyz(after)).length() < 1e-3);
    }

    // ------------------------------------------------------------ program of points

    void programRunsItsStepsInOrder()
    {
        GoToWorkPose();
        const QVariantList work = m_backend->joints();
        QVariantList pose = m_backend->tcp();
        pose[2] = pose[2].toDouble() + 0.05;
        m_backend->clearProgram();
        m_backend->addStep({{QStringLiteral("type"), QStringLiteral("J")}, {QStringLiteral("joints"), QVariantList{0, -45, 90, 45, 90, 0}},
                            {QStringLiteral("speed"), 90}, {QStringLiteral("accel"), 180}});
        m_backend->addStep({{QStringLiteral("type"), QStringLiteral("J")}, {QStringLiteral("joints"), work},
                            {QStringLiteral("speed"), 90}, {QStringLiteral("accel"), 180}});
        m_backend->addStep({{QStringLiteral("type"), QStringLiteral("L")}, {QStringLiteral("pose"), pose},
                            {QStringLiteral("speed"), 0.1}, {QStringLiteral("accel"), 0.5}});
        QCOMPARE(m_backend->program().size(), 3);
        m_backend->runProgram();
        QVERIFY(WaitMotion());
        QVERIFY((Xyz(m_backend->tcp()) - Xyz(pose)).length() < 1e-3);
        // one continuous trajectory: no jumps between consecutive samples
        for (int j = 0; j < 6; j++)
        {
            const QList<QPointF> q = m_backend->Series(QStringLiteral("position"), j);
            for (int k = 1; k < q.size(); k++)
                QVERIFY2(qAbs(q[k].y() - q[k - 1].y()) < 2.0, qPrintable(QStringLiteral("joint %1 sample %2").arg(j).arg(k)));
        }
        QCOMPARE(m_backend->momentCount(), m_backend->sampleCount());
    }

    void programStopsAtTheFailingStep()
    {
        m_backend->clearProgram();
        m_backend->addStep({{QStringLiteral("type"), QStringLiteral("J")}, {QStringLiteral("joints"), QVariantList{10, -20, 30, 0, 0, 0}},
                            {QStringLiteral("speed"), 90}, {QStringLiteral("accel"), 180}});
        m_backend->addStep({{QStringLiteral("type"), QStringLiteral("J")}, {QStringLiteral("joints"), QVariantList{10, 60, 30, 0, 0, 0}},
                            {QStringLiteral("speed"), 90}, {QStringLiteral("accel"), 180}});
        QSignalSpy errors(m_backend.get(), &RobotBackend::errorOccurred);
        m_backend->runProgram();
        QVERIFY(errors.wait(kTimeoutMs));
        const QString msg = errors.first().first().toString();
        QVERIFY2(msg.contains(QStringLiteral("2")) && msg.contains(QStringLiteral("outside its limit")), qPrintable(msg));
        QCOMPARE(m_backend->sampleCount(), 0);
        QVERIFY(!m_backend->busy());
    }

    void programSkipsStepsThatDoNotMove()
    {
        m_backend->clearProgram();
        for (int i = 0; i < 2; i++)
            m_backend->addStep({{QStringLiteral("type"), QStringLiteral("J")}, {QStringLiteral("joints"), QVariantList{15, -30, 40, 0, 20, 0}},
                                {QStringLiteral("speed"), 90}, {QStringLiteral("accel"), 180}});
        m_backend->runProgram();
        QVERIFY(WaitMotion());
        QVERIFY(qAbs(At(m_backend->joints(), 0) - 15) < 1e-3);
    }

    void programIsEditable()
    {
        m_backend->clearProgram();
        for (int i = 0; i < 3; i++)
            m_backend->addStep({{QStringLiteral("type"), QStringLiteral("J")}, {QStringLiteral("joints"), QVariantList{double(i), 0, 0, 0, 0, 0}},
                                {QStringLiteral("speed"), 90}, {QStringLiteral("accel"), 180}});
        m_backend->moveStep(2, -2);              // last to first
        QCOMPARE(m_backend->program()[0].toMap()[QStringLiteral("joints")].toList()[0].toDouble(), 2.0);
        m_backend->removeStep(1);
        QCOMPARE(m_backend->program().size(), 2);
        QCOMPARE(m_backend->program()[1].toMap()[QStringLiteral("joints")].toList()[0].toDouble(), 1.0);
        QSignalSpy errors(m_backend.get(), &RobotBackend::errorOccurred);
        m_backend->addStep({{QStringLiteral("type"), QStringLiteral("X")}});
        QCOMPARE(errors.count(), 1);
        QCOMPARE(m_backend->program().size(), 2);
    }

    void programSurvivesSaveAndLoad()
    {
        m_backend->clearProgram();
        m_backend->addStep({{QStringLiteral("type"), QStringLiteral("J")}, {QStringLiteral("joints"), QVariantList{1, 2, 3, 4, 5, 6}},
                            {QStringLiteral("speed"), 45}, {QStringLiteral("accel"), 90}});
        m_backend->addStep({{QStringLiteral("type"), QStringLiteral("C")}, {QStringLiteral("mid"), QVariantList{0.3, 0.1, 0.4}},
                            {QStringLiteral("pose"), QVariantList{0.35, 0.1, 0.35, 180, 0, 90}}, {QStringLiteral("speed"), 0.05}, {QStringLiteral("accel"), 0.2}});
        const QVariantList before = m_backend->program();
        QTemporaryDir dir;
        const QUrl file = QUrl::fromLocalFile(QDir(dir.path()).filePath(QStringLiteral("p.json")));
        QVERIFY(m_backend->saveProgram(file));
        m_backend->clearProgram();
        QVERIFY(m_backend->loadProgram(file));
        QCOMPARE(m_backend->program(), before);
        QVERIFY(!m_backend->loadProgram(QUrl::fromLocalFile(QDir(dir.path()).filePath(QStringLiteral("none.json")))));
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
