// Motor set-up over CAN: the ODrive frames byte by byte as documented, the
// SLCAN text format, the simulated ODrive, and MotorHub scenarios run against
// the simulator through the same protocol code a real bus uses.
#include "motorhub.h"
#include "odriveprotocol.h"
#include "simulatedodrive.h"
#include "slcan.h"

#include <QSignalSpy>
#include <QTemporaryDir>
#include <QtTest>

#include <cmath>

using namespace odrive;

namespace
{
QByteArray Hex(const char *hex)
{
    return QByteArray::fromHex(hex);
}
}

class MotorTests : public QObject
{
    Q_OBJECT

    static constexpr int kTimeoutMs = 5000;

    QTemporaryDir m_dir;
    std::unique_ptr<MotorHub> m_hub;
    SimulatedCanBus *m_bus = nullptr;

    QVariantMap Axis(int i) const { return m_hub->axes().at(i).toMap(); }
    double Position(int i) const { return Axis(i)[QStringLiteral("position")].toDouble(); }

    // hub on a simulated bus with axes at nodes 0..2
    void Start(bool calibrated = true)
    {
        m_hub = std::make_unique<MotorHub>(m_dir.filePath(QStringLiteral("motors.ini")));
        auto bus = std::make_unique<SimulatedCanBus>(QVector<int>{0, 1, 2}, calibrated);
        m_bus = bus.get();
        QVERIFY(m_hub->ConnectTransport(std::move(bus), QStringLiteral("test")));
        QTRY_VERIFY_WITH_TIMEOUT(Axis(0)[QStringLiteral("online")].toBool(), kTimeoutMs);
        QTRY_VERIFY_WITH_TIMEOUT(Axis(0)[QStringLiteral("hasPosition")].toBool(), kTimeoutMs);
        // an estimate may come before the first heartbeat tells the state
        QTRY_VERIFY_WITH_TIMEOUT(Axis(0)[QStringLiteral("state")].toInt() != Undefined, kTimeoutMs);
    }

    void Enable(int i)
    {
        m_hub->enable(i);
        QTRY_VERIFY_WITH_TIMEOUT(Axis(i)[QStringLiteral("closedLoop")].toBool(), kTimeoutMs);
    }

private slots:
    void cleanup()
    {
        m_hub.reset();
        m_bus = nullptr;
    }

    // ------------------------------------------------------------ protocol

    void frameIdIsNodeShiftedByFive()
    {
        QCOMPARE(FrameId(0, Cmd::Heartbeat), 0x001u);
        QCOMPARE(FrameId(1, Cmd::SetInputPos), 0x02Cu);
        QCOMPARE(FrameId(0x3F, Cmd::ClearErrors), 0x7F8u);
        QCOMPARE(NodeOf(0x02C), 1);
        QCOMPARE(CmdOf(0x02C), 0x0Cu);
    }

    void commandsAreEncodedAsDocumented()
    {
        CanFrame f = SetAxisState(1, ClosedLoop);
        QCOMPARE(f.id, 0x027u);
        QCOMPARE(f.data, Hex("08000000"));
        // 1.0f = 0x3F800000, vel_ff 0.5 turn/s -> 500, torque_ff -0.25 N*m -> -250
        f = SetInputPos(2, 1.0f, 0.5f, -0.25f);
        QCOMPARE(f.id, 0x04Cu);
        QCOMPARE(f.data, Hex("0000803ff40106ff"));
        f = SetControllerMode(0, PositionControl, TrapTraj);
        QCOMPARE(f.data, Hex("0300000005000000"));
        f = SetInputVel(0, -2.0f);
        QCOMPARE(f.data, Hex("000000c000000000"));
        f = SetLimits(0, 10.0f, 20.0f);
        QCOMPARE(f.data, Hex("000020410000a041"));
        f = Request(3, Cmd::GetEncoderEstimates);
        QVERIFY(f.rtr);
        QCOMPARE(f.id, 0x069u);
        QVERIFY(f.data.isEmpty());
    }

    void repliesAreDecoded()
    {
        CanFrame hb;
        hb.id = FrameId(4, Cmd::Heartbeat);
        hb.data = Hex("4000000008000000");
        Heartbeat h;
        QVERIFY(DecodeHeartbeat(hb, h));
        QCOMPARE(h.axis_error, 0x40u);
        QCOMPARE(h.state, int(ClosedLoop));
        CanFrame est;
        est.id = FrameId(4, Cmd::GetEncoderEstimates);
        est.data = Hex("0000c03f000080bf");   // 1.5 turns, -1.0 turn/s
        float pos = 0, vel = 0;
        QVERIFY(DecodeFloats(est, pos, vel));
        QCOMPARE(pos, 1.5f);
        QCOMPARE(vel, -1.0f);
        QCOMPARE(ErrorText(0x40 | 0x800 | 0x10000), QStringLiteral("MOTOR_FAILED | WATCHDOG_EXPIRED | 0x10000"));
        QCOMPARE(StateName(ClosedLoop), QStringLiteral("CLOSED_LOOP"));
    }

    void slcanTextRoundTrips()
    {
        const CanFrame f = SetInputVel(1, 1.0f);
        QCOMPARE(slcan::Encode(f), QByteArray("t02D80000803F00000000\r"));
        QCOMPARE(slcan::Encode(Request(1, Cmd::GetIq)), QByteArray("r0348\r"));
        CanFrame back;
        QVERIFY(slcan::Decode("t02D80000803F00000000", back));
        QCOMPARE(back, f);
        QVERIFY(slcan::Decode("t02D80000803F000000001A2B", back));     // with a timestamp
        QCOMPARE(back, f);
        QVERIFY(slcan::Decode("r0348", back));
        QVERIFY(back.rtr);
        QCOMPARE(back.length, 8);
        QVERIFY(!slcan::Decode("T0000002D0", back));    // extended frames are not used
        QVERIFY(!slcan::Decode("t02D8zz", back));
        QCOMPARE(slcan::BitrateCommand(250000), QByteArray("S5\r"));
        QVERIFY(slcan::BitrateCommand(300000).isEmpty());
    }

    // ------------------------------------------------------------ simulator

    void simulatorRefusesClosedLoopUntilCalibrated()
    {
        SimulatedOdrive axis(0, false);
        QVector<CanFrame> out;
        axis.Receive(SetAxisState(0, ClosedLoop), out);
        QCOMPARE(axis.state, int(Idle));
        QVERIFY(axis.error & ErrorInvalidState);
        axis.Receive(ClearErrors(0), out);
        axis.Receive(SetAxisState(0, FullCalibration), out);
        QCOMPARE(axis.state, int(FullCalibration));
        for (int i = 0; i < 200; i++)
            axis.Step(0.01);
        QCOMPARE(axis.state, int(Idle));
        axis.Receive(SetAxisState(0, ClosedLoop), out);
        QCOMPARE(axis.state, int(ClosedLoop));
    }

    void simulatorFollowsATrapezoidalMove()
    {
        SimulatedOdrive axis(0);
        QVector<CanFrame> out;
        axis.Receive(SetAxisState(0, ClosedLoop), out);
        axis.Receive(SetControllerMode(0, PositionControl, TrapTraj), out);
        axis.Receive(SetTrajVelLimit(0, 1.0f), out);
        axis.Receive(SetInputPos(0, 2.0f), out);
        double peak = 0;
        for (int i = 0; i < 500; i++)
        {
            axis.Step(0.01);
            peak = std::max(peak, std::abs(axis.velocity));
        }
        QVERIFY(std::abs(axis.position - 2.0) < 1e-4);
        QVERIFY(peak <= 1.0 + 1e-9);
        QVERIFY(peak > 0.95);
        axis.Receive(Request(0, Cmd::GetEncoderEstimates), out);
        QCOMPARE(out.size(), 1);
        float pos = 0, vel = 0;
        QVERIFY(DecodeFloats(out[0], pos, vel));
        QCOMPARE(pos, 2.0f);
    }

    // ------------------------------------------------------------ MotorHub on the simulated bus

    void axesComeOnlineFromHeartbeats()
    {
        Start();
        const QVariantMap a = Axis(0);
        QCOMPARE(a[QStringLiteral("stateName")].toString(), QStringLiteral("IDLE"));
        QCOMPARE(a[QStringLiteral("node")].toInt(), 0);
        QVERIFY(m_hub->connected());
        // the estimates are polled with remote requests
        bool polled = false;
        for (const CanFrame &f : m_bus->sent)
            polled = polled || (f.rtr && f.id == FrameId(0, Cmd::GetEncoderEstimates));
        QVERIFY(polled);
    }

    // joint angle <-> motor turns through gear, direction and zero
    void moveToReachesTheJointAngleThroughTheGear()
    {
        Start();
        QVERIFY(m_hub->setAxisConfig(0, {{QStringLiteral("gear"), 10.0}, {QStringLiteral("direction"), -1},
                                         {QStringLiteral("zero"), 1.0}, {QStringLiteral("maxSpeed"), 360.0},
                                         {QStringLiteral("maxAccel"), 720.0}}));
        Enable(0);
        m_hub->moveTo(0, 18.0);
        // the motor at 0 turns is the joint at 36 deg; 18 deg * 10 / 360 = 0.5 turn, reversed, from the zero at 1 turn
        QTRY_VERIFY_WITH_TIMEOUT(std::abs(m_bus->Axis(0)->position - 0.5) < 1e-3, kTimeoutMs);
        QTRY_VERIFY_WITH_TIMEOUT(std::abs(Position(0) - 18.0) < 0.05, kTimeoutMs);
        QCOMPARE(Axis(0)[QStringLiteral("target")].toDouble(), 18.0);
        // the trajectory limits were sent in motor units: 360 deg/s * 10 / 360 = 10 turn/s
        QVERIFY(std::abs(m_bus->Axis(0)->traj_vel - 10.0) < 1e-6);
        QVERIFY(std::abs(m_bus->Axis(0)->traj_accel - 20.0) < 1e-6);
        QVERIFY(!m_hub->Series(0, QStringLiteral("position")).isEmpty());
    }

    void targetsAreKeptInsideTheJointRange()
    {
        Start();
        m_hub->setJointLimits({QVariantMap{{QStringLiteral("min"), -30.0}, {QStringLiteral("max"), 45.0}}});
        QVERIFY(m_hub->setAxisConfig(0, {{QStringLiteral("maxSpeed"), 360.0}, {QStringLiteral("maxAccel"), 1440.0}}));
        Enable(0);
        m_hub->moveTo(0, 90.0);
        QCOMPARE(Axis(0)[QStringLiteral("target")].toDouble(), 45.0);
        QTRY_VERIFY_WITH_TIMEOUT(std::abs(Position(0) - 45.0) < 0.05, kTimeoutMs);
        // jogging further out does nothing
        m_hub->jog(0, 1);
        QVERIFY(!Axis(0)[QStringLiteral("jogging")].toBool());
    }

    void moveNeedsClosedLoop()
    {
        Start();
        QSignalSpy errors(m_hub.get(), &MotorHub::errorOccurred);
        m_hub->moveTo(0, 10);
        QCOMPARE(errors.count(), 1);
        QVERIFY(errors.first().first().toString().contains(QStringLiteral("enable")));
    }

    // hold-to-run: the jog stops by itself when the repeats stop coming
    void jogStopsWhenTheButtonIsNoLongerHeld()
    {
        Start();
        QVERIFY(m_hub->setAxisConfig(0, {{QStringLiteral("maxSpeed"), 90.0}, {QStringLiteral("maxAccel"), 720.0}}));
        Enable(0);
        m_hub->jog(0, 1);
        QTRY_VERIFY_WITH_TIMEOUT(Position(0) > 5.0, kTimeoutMs);
        QVERIFY(Axis(0)[QStringLiteral("jogging")].toBool());
        // no more jog() calls: stopped within the timeout and decelerated to rest
        QTRY_VERIFY_WITH_TIMEOUT(!Axis(0)[QStringLiteral("jogging")].toBool(), MotorHub::kJogTimeoutMs + 500);
        QTRY_VERIFY_WITH_TIMEOUT(std::abs(m_bus->Axis(0)->velocity) < 1e-3, kTimeoutMs);
        const double stopped = Position(0);
        QTest::qWait(200);
        QVERIFY(std::abs(Position(0) - stopped) < 0.05);
        QVERIFY(Axis(0)[QStringLiteral("closedLoop")].toBool());
    }

    void stopAllHoldsAndDisableAllFreesTheMotors()
    {
        Start();
        QVERIFY(m_hub->setAxisConfig(0, {{QStringLiteral("maxSpeed"), 90.0}, {QStringLiteral("maxAccel"), 360.0}}));
        Enable(0);
        m_hub->moveTo(0, 170);
        QTRY_VERIFY_WITH_TIMEOUT(Position(0) > 20, kTimeoutMs);
        m_hub->stopAll();
        QTRY_VERIFY_WITH_TIMEOUT(std::abs(m_bus->Axis(0)->velocity) < 1e-3, kTimeoutMs);
        QVERIFY(Position(0) < 150);
        QVERIFY(Axis(0)[QStringLiteral("closedLoop")].toBool());
        m_hub->disableAll();
        QTRY_COMPARE_WITH_TIMEOUT(Axis(0)[QStringLiteral("stateName")].toString(), QStringLiteral("IDLE"), kTimeoutMs);
    }

    void uncalibratedMotorReportsTheErrorUntilCalibrated()
    {
        Start(false);
        QSignalSpy errors(m_hub.get(), &MotorHub::errorOccurred);
        m_hub->enable(0);
        QTRY_VERIFY_WITH_TIMEOUT(Axis(0)[QStringLiteral("errorText")].toString().contains(QStringLiteral("INVALID_STATE")), kTimeoutMs);
        QVERIFY(!errors.isEmpty());
        m_hub->clearErrors(0);
        QTRY_VERIFY_WITH_TIMEOUT(Axis(0)[QStringLiteral("error")].toString().isEmpty(), kTimeoutMs);
        m_hub->calibrate(0);
        QTRY_COMPARE_WITH_TIMEOUT(Axis(0)[QStringLiteral("stateName")].toString(), QStringLiteral("CALIBRATION"), kTimeoutMs);
        QTRY_COMPARE_WITH_TIMEOUT(Axis(0)[QStringLiteral("stateName")].toString(), QStringLiteral("IDLE"), kTimeoutMs);
        Enable(0);
    }

    void silentAxisGoesOffline()
    {
        Start();
        Enable(0);
        QSignalSpy errors(m_hub.get(), &MotorHub::errorOccurred);
        m_bus->SetSilent(0, true);
        QTRY_VERIFY_WITH_TIMEOUT(!Axis(0)[QStringLiteral("online")].toBool(), MotorHub::kOfflineMs + 1000);
        QVERIFY(!errors.isEmpty());
        QVERIFY(errors.last().first().toString().contains(QStringLiteral("node 0")));
        m_bus->SetSilent(0, false);
        QTRY_VERIFY_WITH_TIMEOUT(Axis(0)[QStringLiteral("online")].toBool(), kTimeoutMs);
    }

    void zeroHereMakesThePresentPositionZero()
    {
        Start();
        m_bus->Axis(0)->position = 3.25;
        QTRY_VERIFY_WITH_TIMEOUT(std::abs(Axis(0)[QStringLiteral("motorTurns")].toDouble() - 3.25) < 1e-6, kTimeoutMs);
        m_hub->setZeroHere(0);
        QCOMPARE(Axis(0)[QStringLiteral("zero")].toDouble(), 3.25);
        QVERIFY(std::abs(Position(0)) < 1e-6);
    }

    void settingsSurviveSaveAndLoad()
    {
        Start();
        m_hub->setAxisCount(3);
        QVERIFY(m_hub->setAxisConfig(2, {{QStringLiteral("node"), 5}, {QStringLiteral("gear"), 50.0}, {QStringLiteral("direction"), -1},
                                         {QStringLiteral("zero"), 1.5}, {QStringLiteral("currentLimit"), 7.5}}));
        QVERIFY(m_hub->saveConfig());
        m_hub.reset();
        MotorHub loaded(m_dir.filePath(QStringLiteral("motors.ini")));
        QCOMPARE(loaded.axisCount(), 3);
        const QVariantMap a = loaded.axes().at(2).toMap();
        QCOMPARE(a[QStringLiteral("node")].toInt(), 5);
        QCOMPARE(a[QStringLiteral("gear")].toDouble(), 50.0);
        QCOMPARE(a[QStringLiteral("direction")].toInt(), -1);
        QCOMPARE(a[QStringLiteral("zero")].toDouble(), 1.5);
        QCOMPARE(a[QStringLiteral("currentLimit")].toDouble(), 7.5);
    }

    void invalidSettingsAreRejected()
    {
        Start();
        QSignalSpy errors(m_hub.get(), &MotorHub::errorOccurred);
        QVERIFY(!m_hub->setAxisConfig(0, {{QStringLiteral("gear"), 0.0}}));
        QVERIFY(!m_hub->setAxisConfig(0, {{QStringLiteral("direction"), 2}}));
        QVERIFY(!m_hub->setAxisConfig(0, {{QStringLiteral("node"), 64}}));
        QVERIFY(!m_hub->setAxisConfig(0, {{QStringLiteral("maxSpeed"), QStringLiteral("fast")}}));
        QCOMPARE(errors.count(), 4);
        QCOMPARE(Axis(0)[QStringLiteral("gear")].toDouble(), 1.0);
    }

    void simulatorIsAlwaysOffered()
    {
        MotorHub hub(m_dir.filePath(QStringLiteral("x.ini")));
        QCOMPARE(hub.backends().first(), QStringLiteral("simulator"));
        QVERIFY(hub.connectBus(QStringLiteral("simulator"), QString(), 250000));
        QVERIFY(hub.connected());
        QVERIFY(!hub.connectBus(QStringLiteral("no-such-backend"), QString(), 250000));
        QVERIFY(!hub.connected());
    }
};

QTEST_MAIN(MotorTests)
#include "test_motors.moc"
