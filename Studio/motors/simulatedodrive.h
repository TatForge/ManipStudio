#ifndef SIMULATEDODRIVE_H
#define SIMULATEDODRIVE_H

#include "cantransport.h"
#include "odriveprotocol.h"

#include <QElapsedTimer>
#include <QTimer>
#include <QVector>

#include <memory>
#include <vector>

// One ODrive axis in software, answering the CAN protocol: states and
// calibration, control and input modes, limits, a simple motor model,
// estimates, current and errors. Positions in motor turns.
class SimulatedOdrive
{
public:
    explicit SimulatedOdrive(int node, bool calibrated = true);

    // handles a frame addressed to this axis; replies go to `out`
    void Receive(const CanFrame &f, QVector<CanFrame> &out);
    void Step(double dt);
    CanFrame HeartbeatFrame() const;

    int node;
    bool calibrated;
    int state = odrive::Idle;
    quint32 error = 0;
    int control_mode = odrive::PositionControl;
    int input_mode = odrive::Passthrough;
    double position = 0, velocity = 0;      // turns, turns/s
    double input_pos = 0, input_vel = 0, input_torque = 0;
    double vel_limit = 2, current_limit = 10;
    double traj_vel = 2, traj_accel = 2, traj_decel = 2;
    double iq = 0;                          // A
    double calibration_left = 0;            // s
    double bus_voltage = 24;

private:
    void Accelerate(double v_target, double accel, double dt);
};

// A bus with simulated axes: replies come back asynchronously like on a real
// bus, heartbeats every 100 ms.
class SimulatedCanBus : public CanTransport
{
    Q_OBJECT
public:
    explicit SimulatedCanBus(const QVector<int> &nodes, bool calibrated = true, QObject *parent = nullptr);

    bool Open(QString *error) override;
    void Close() override;
    bool Send(const CanFrame &frame) override;

    SimulatedOdrive *Axis(int node);
    // a silent axis neither answers nor sends heartbeats (unplugged, powered off)
    void SetSilent(int node, bool silent);
    // every frame sent to the bus, for tests
    QVector<CanFrame> sent;

private:
    void Tick();
    void Deliver(const QVector<CanFrame> &frames);

    std::vector<std::unique_ptr<SimulatedOdrive>> m_axes;
    QVector<int> m_silent;
    QTimer m_timer;
    QElapsedTimer m_clock;
    double m_since_heartbeat = 0;
    bool m_open = false;
};

#endif // SIMULATEDODRIVE_H
