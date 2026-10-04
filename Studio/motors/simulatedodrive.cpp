#include "simulatedodrive.h"

#include <algorithm>
#include <cmath>

using namespace odrive;

SimulatedOdrive::SimulatedOdrive(int node, bool calibrated) : node(node), calibrated(calibrated)
{
}

void SimulatedOdrive::Receive(const CanFrame &f, QVector<CanFrame> &out)
{
    if (NodeOf(f.id) != node)
        return;
    const Cmd cmd = Cmd(CmdOf(f.id));
    if (f.rtr)
    {
        CanFrame reply;
        reply.id = f.id;
        reply.data = QByteArray(8, '\0');
        switch (cmd)
        {
        case Cmd::Heartbeat: out.append(HeartbeatFrame()); return;
        case Cmd::GetEncoderEstimates:
            PutF32(reply.data, 0, float(position));
            PutF32(reply.data, 4, float(velocity));
            break;
        case Cmd::GetIq:
            PutF32(reply.data, 0, float(iq));
            PutF32(reply.data, 4, float(iq * 0.98));
            break;
        case Cmd::GetBusVoltageCurrent:
            PutF32(reply.data, 0, float(bus_voltage));
            PutF32(reply.data, 4, float(std::abs(iq) * 0.1));
            break;
        case Cmd::GetError: PutU32(reply.data, 0, error); break;
        default: return;
        }
        out.append(reply);
        return;
    }
    const QByteArray &d = f.data;
    switch (cmd)
    {
    case Cmd::SetAxisState:
    {
        if (d.size() < 4)
            return;
        const int requested = int(U32(d, 0));
        if (requested == Idle)
            state = Idle;
        else if (requested == ClosedLoop)
        {
            if (!calibrated || error)
            {
                // the firmware refuses and stays where it is
                error |= ErrorInvalidState;
                state = Idle;
                return;
            }
            state = ClosedLoop;
            // the controller starts from where the motor is
            input_pos = position;
            input_vel = 0;
            input_torque = 0;
        }
        else if (requested == FullCalibration || requested == MotorCalibration || requested == EncoderOffsetCalibration)
        {
            if (state != Idle)
            {
                error |= ErrorInvalidState;
                return;
            }
            state = requested;
            calibration_left = requested == FullCalibration ? 1.0 : 0.5;
        }
        return;
    }
    case Cmd::SetControllerMode:
        if (d.size() < 8)
            return;
        if (int(U32(d, 0)) == PositionControl && control_mode != PositionControl)
            input_pos = position;
        control_mode = int(U32(d, 0));
        input_mode = int(U32(d, 4));
        return;
    case Cmd::SetInputPos:
        if (d.size() >= 4)
            input_pos = F32(d, 0);
        return;
    case Cmd::SetInputVel:
        if (d.size() >= 4)
            input_vel = F32(d, 0);
        return;
    case Cmd::SetInputTorque:
        if (d.size() >= 4)
            input_torque = F32(d, 0);
        return;
    case Cmd::SetLimits:
        if (d.size() >= 8)
        {
            vel_limit = F32(d, 0);
            current_limit = F32(d, 4);
        }
        return;
    case Cmd::SetTrajVelLimit:
        if (d.size() >= 4)
            traj_vel = F32(d, 0);
        return;
    case Cmd::SetTrajAccelLimits:
        if (d.size() >= 8)
        {
            traj_accel = F32(d, 0);
            traj_decel = F32(d, 4);
        }
        return;
    case Cmd::ClearErrors: error = 0; return;
    default: return;
    }
}

void SimulatedOdrive::Accelerate(double v_target, double accel, double dt)
{
    const double dv = std::clamp(v_target - velocity, -accel * dt, accel * dt);
    iq = std::clamp(dv / dt * 0.5 + (velocity != 0 ? std::copysign(0.2, velocity) : 0.0), -current_limit, current_limit);
    velocity += dv;
}

void SimulatedOdrive::Step(double dt)
{
    if (dt <= 0)
        return;
    if (calibration_left > 0)
    {
        calibration_left -= dt;
        if (calibration_left <= 0)
        {
            calibrated = true;
            state = Idle;
        }
        return;
    }
    if (state != ClosedLoop)
    {
        // free running: friction brings the rotor to rest
        velocity *= std::exp(-5 * dt);
        if (std::abs(velocity) < 1e-6)
            velocity = 0;
        position += velocity * dt;
        iq = 0;
        return;
    }
    switch (control_mode)
    {
    case PositionControl:
    {
        const bool traj = input_mode == TrapTraj;
        const double v_max = traj ? std::min(traj_vel, vel_limit) : vel_limit;
        const double a_max = traj ? traj_accel : 50.0;
        const double d_max = traj ? traj_decel : 50.0;
        const double e = input_pos - position;
        // fastest speed that still stops at the target
        const double v_target = std::copysign(std::min(v_max, std::sqrt(2 * d_max * std::abs(e))), e);
        Accelerate(v_target, std::abs(v_target) < std::abs(velocity) ? d_max : a_max, dt);
        // arrives within this step (or is slow and close): settle exactly on the target
        if (std::abs(e) <= std::abs(velocity) * dt || (std::abs(e) < 1e-4 && std::abs(velocity) < d_max * dt))
        {
            position = input_pos;
            velocity = 0;
        }
        else
            position += velocity * dt;
        break;
    }
    case VelocityControl:
        Accelerate(std::clamp(input_vel, -vel_limit, vel_limit), input_mode == VelRamp ? traj_accel : 50.0, dt);
        position += velocity * dt;
        break;
    case TorqueControl:
    default:
        velocity = std::clamp(velocity + (input_torque * 10 - 0.5 * velocity) * dt, -vel_limit, vel_limit);
        iq = std::clamp(input_torque * 4, -current_limit, current_limit);
        position += velocity * dt;
        break;
    }
}

CanFrame SimulatedOdrive::HeartbeatFrame() const
{
    CanFrame f;
    f.id = FrameId(node, Cmd::Heartbeat);
    f.data = QByteArray(8, '\0');
    PutU32(f.data, 0, error);
    f.data[4] = char(state);
    return f;
}

SimulatedCanBus::SimulatedCanBus(const QVector<int> &nodes, bool calibrated, QObject *parent) : CanTransport(parent)
{
    qRegisterMetaType<CanFrame>();
    for (int n : nodes)
        m_axes.push_back(std::make_unique<SimulatedOdrive>(n, calibrated));
    m_timer.setInterval(5);
    m_timer.setTimerType(Qt::PreciseTimer);
    connect(&m_timer, &QTimer::timeout, this, &SimulatedCanBus::Tick);
}

bool SimulatedCanBus::Open(QString *)
{
    m_open = true;
    m_clock.start();
    m_timer.start();
    return true;
}

void SimulatedCanBus::Close()
{
    m_open = false;
    m_timer.stop();
}

bool SimulatedCanBus::Send(const CanFrame &frame)
{
    if (!m_open)
        return false;
    sent.append(frame);
    QVector<CanFrame> replies;
    for (auto &axis : m_axes)
        if (!m_silent.contains(axis->node))
            axis->Receive(frame, replies);
    Deliver(replies);
    return true;
}

SimulatedOdrive *SimulatedCanBus::Axis(int node)
{
    for (auto &axis : m_axes)
        if (axis->node == node)
            return axis.get();
    return nullptr;
}

void SimulatedCanBus::SetSilent(int node, bool silent)
{
    m_silent.removeAll(node);
    if (silent)
        m_silent.append(node);
}

void SimulatedCanBus::Tick()
{
    const double dt = m_clock.restart() / 1000.0;
    for (auto &axis : m_axes)
        axis->Step(std::min(dt, 0.05));
    m_since_heartbeat += dt;
    if (m_since_heartbeat < 0.1)
        return;
    m_since_heartbeat = 0;
    QVector<CanFrame> beats;
    for (auto &axis : m_axes)
        if (!m_silent.contains(axis->node))
            beats.append(axis->HeartbeatFrame());
    Deliver(beats);
}

void SimulatedCanBus::Deliver(const QVector<CanFrame> &frames)
{
    for (const CanFrame &f : frames)
        QMetaObject::invokeMethod(this, [this, f] {
            if (m_open)
                emit FrameReceived(f);
        }, Qt::QueuedConnection);
}
