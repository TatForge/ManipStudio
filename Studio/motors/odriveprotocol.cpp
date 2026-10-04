#include "odriveprotocol.h"

#include <QStringList>

#include <cmath>
#include <cstring>

namespace odrive
{
namespace
{
CanFrame Make(int node, Cmd cmd, int size)
{
    CanFrame f;
    f.id = FrameId(node, cmd);
    f.data = QByteArray(size, '\0');
    return f;
}

qint16 Scaled(float v)
{
    const float s = std::round(v * 1000.f);
    return qint16(std::max(-32768.f, std::min(32767.f, s)));
}
}

quint32 FrameId(int node, Cmd cmd)
{
    return (quint32(node & kMaxNode) << 5) | quint32(cmd);
}

int NodeOf(quint32 id)
{
    return int(id >> 5) & kMaxNode;
}

quint32 CmdOf(quint32 id)
{
    return id & 0x1F;
}

quint32 U32(const QByteArray &d, int at)
{
    quint32 v = 0;
    for (int i = 3; i >= 0; i--)
        v = (v << 8) | quint8(d.at(at + i));
    return v;
}

float F32(const QByteArray &d, int at)
{
    const quint32 bits = U32(d, at);
    float v;
    std::memcpy(&v, &bits, sizeof v);
    return v;
}

qint16 S16(const QByteArray &d, int at)
{
    return qint16(quint16(quint8(d.at(at))) | quint16(quint8(d.at(at + 1))) << 8);
}

void PutU32(QByteArray &d, int at, quint32 v)
{
    for (int i = 0; i < 4; i++)
        d[at + i] = char((v >> (8 * i)) & 0xFF);
}

void PutF32(QByteArray &d, int at, float v)
{
    quint32 bits;
    std::memcpy(&bits, &v, sizeof v);
    PutU32(d, at, bits);
}

void PutS16(QByteArray &d, int at, qint16 v)
{
    d[at] = char(quint16(v) & 0xFF);
    d[at + 1] = char(quint16(v) >> 8);
}

CanFrame SetAxisState(int node, AxisState state)
{
    CanFrame f = Make(node, Cmd::SetAxisState, 4);
    PutU32(f.data, 0, quint32(state));
    return f;
}

CanFrame SetControllerMode(int node, ControlMode control, InputMode input)
{
    CanFrame f = Make(node, Cmd::SetControllerMode, 8);
    PutU32(f.data, 0, quint32(control));
    PutU32(f.data, 4, quint32(input));
    return f;
}

CanFrame SetInputPos(int node, float turns, float vel_ff, float torque_ff)
{
    CanFrame f = Make(node, Cmd::SetInputPos, 8);
    PutF32(f.data, 0, turns);
    PutS16(f.data, 4, Scaled(vel_ff));
    PutS16(f.data, 6, Scaled(torque_ff));
    return f;
}

CanFrame SetInputVel(int node, float turns_s, float torque_ff)
{
    CanFrame f = Make(node, Cmd::SetInputVel, 8);
    PutF32(f.data, 0, turns_s);
    PutF32(f.data, 4, torque_ff);
    return f;
}

CanFrame SetInputTorque(int node, float nm)
{
    CanFrame f = Make(node, Cmd::SetInputTorque, 4);
    PutF32(f.data, 0, nm);
    return f;
}

CanFrame SetLimits(int node, float vel_limit, float current_limit)
{
    CanFrame f = Make(node, Cmd::SetLimits, 8);
    PutF32(f.data, 0, vel_limit);
    PutF32(f.data, 4, current_limit);
    return f;
}

CanFrame SetTrajVelLimit(int node, float turns_s)
{
    CanFrame f = Make(node, Cmd::SetTrajVelLimit, 4);
    PutF32(f.data, 0, turns_s);
    return f;
}

CanFrame SetTrajAccelLimits(int node, float accel, float decel)
{
    CanFrame f = Make(node, Cmd::SetTrajAccelLimits, 8);
    PutF32(f.data, 0, accel);
    PutF32(f.data, 4, decel);
    return f;
}

CanFrame ClearErrors(int node)
{
    // 0.6.x reads an "identify" byte here, 0.5.x ignores the data
    return Make(node, Cmd::ClearErrors, 1);
}

CanFrame Reboot(int node)
{
    return Make(node, Cmd::Reboot, 0);
}

CanFrame Request(int node, Cmd cmd)
{
    CanFrame f;
    f.id = FrameId(node, cmd);
    f.rtr = true;
    f.length = 8;
    return f;
}

bool DecodeHeartbeat(const CanFrame &f, Heartbeat &h)
{
    if (f.rtr || CmdOf(f.id) != quint32(Cmd::Heartbeat) || f.data.size() < 5)
        return false;
    h.axis_error = U32(f.data, 0);
    h.state = quint8(f.data.at(4));
    return true;
}

bool DecodeFloats(const CanFrame &f, float &first, float &second)
{
    if (f.rtr || f.data.size() < 8)
        return false;
    first = F32(f.data, 0);
    second = F32(f.data, 4);
    return true;
}

QString StateName(int state)
{
    switch (state)
    {
    case Undefined: return QStringLiteral("UNDEFINED");
    case Idle: return QStringLiteral("IDLE");
    case StartupSequence: return QStringLiteral("STARTUP");
    case FullCalibration: return QStringLiteral("CALIBRATION");
    case MotorCalibration: return QStringLiteral("MOTOR_CALIBRATION");
    case EncoderIndexSearch: return QStringLiteral("INDEX_SEARCH");
    case EncoderOffsetCalibration: return QStringLiteral("ENCODER_CALIBRATION");
    case ClosedLoop: return QStringLiteral("CLOSED_LOOP");
    case LockinSpin: return QStringLiteral("LOCKIN_SPIN");
    case EncoderDirFind: return QStringLiteral("ENCODER_DIR_FIND");
    case Homing: return QStringLiteral("HOMING");
    }
    return QStringLiteral("STATE_%1").arg(state);
}

QString ErrorText(quint32 axis_error)
{
    static const struct
    {
        quint32 bit;
        const char *name;
    } known[] = {{ErrorInvalidState, "INVALID_STATE"},       {ErrorMotorFailed, "MOTOR_FAILED"},
                 {ErrorSensorlessFailed, "SENSORLESS_FAILED"}, {ErrorEncoderFailed, "ENCODER_FAILED"},
                 {ErrorControllerFailed, "CONTROLLER_FAILED"}, {ErrorWatchdogExpired, "WATCHDOG_EXPIRED"}};
    QStringList parts;
    quint32 rest = axis_error;
    for (const auto &k : known)
        if (axis_error & k.bit)
        {
            parts << QString::fromLatin1(k.name);
            rest &= ~k.bit;
        }
    if (rest)
        parts << QStringLiteral("0x%1").arg(rest, 0, 16);
    return parts.join(QStringLiteral(" | "));
}
}
