#ifndef ODRIVEPROTOCOL_H
#define ODRIVEPROTOCOL_H

#include "canframe.h"

#include <QString>

// ODrive "CAN Simple" protocol: frame id = node << 5 | command, little-endian
// signals. Only the commands that are the same in firmware 0.5.x (the usual
// clones) and 0.6.x are used. Positions are motor turns, speeds turns/s.
namespace odrive
{
enum class Cmd : quint32
{
    Heartbeat = 0x01,
    GetError = 0x03,
    SetAxisState = 0x07,
    GetEncoderEstimates = 0x09,
    SetControllerMode = 0x0B,
    SetInputPos = 0x0C,
    SetInputVel = 0x0D,
    SetInputTorque = 0x0E,
    SetLimits = 0x0F,
    SetTrajVelLimit = 0x11,
    SetTrajAccelLimits = 0x12,
    GetIq = 0x14,
    Reboot = 0x16,
    GetBusVoltageCurrent = 0x17,
    ClearErrors = 0x18,
};

enum AxisState
{
    Undefined = 0,
    Idle = 1,
    StartupSequence = 2,
    FullCalibration = 3,
    MotorCalibration = 4,
    EncoderIndexSearch = 6,
    EncoderOffsetCalibration = 7,
    ClosedLoop = 8,
    LockinSpin = 9,
    EncoderDirFind = 10,
    Homing = 11,
};

enum ControlMode
{
    VoltageControl = 0,
    TorqueControl = 1,
    VelocityControl = 2,
    PositionControl = 3,
};

enum InputMode
{
    Inactive = 0,
    Passthrough = 1,
    VelRamp = 2,
    PosFilter = 3,
    TrapTraj = 5,
};

// firmware 0.5.x axis error bits (0.6.x reports other codes; shown as hex then)
enum AxisError : quint32
{
    ErrorInvalidState = 0x1,
    ErrorMotorFailed = 0x40,
    ErrorSensorlessFailed = 0x80,
    ErrorEncoderFailed = 0x100,
    ErrorControllerFailed = 0x200,
    ErrorWatchdogExpired = 0x800,
};

constexpr int kMaxNode = 0x3F;

quint32 FrameId(int node, Cmd cmd);
int NodeOf(quint32 id);
quint32 CmdOf(quint32 id);

CanFrame SetAxisState(int node, AxisState state);
CanFrame SetControllerMode(int node, ControlMode control, InputMode input);
// vel_ff in turns/s and torque_ff in N*m are sent with 0.001 resolution
CanFrame SetInputPos(int node, float turns, float vel_ff = 0, float torque_ff = 0);
CanFrame SetInputVel(int node, float turns_s, float torque_ff = 0);
CanFrame SetInputTorque(int node, float nm);
CanFrame SetLimits(int node, float vel_limit, float current_limit);
CanFrame SetTrajVelLimit(int node, float turns_s);
CanFrame SetTrajAccelLimits(int node, float accel, float decel);
CanFrame ClearErrors(int node);
CanFrame Reboot(int node);
// remote request for a Get_* message (or the heartbeat)
CanFrame Request(int node, Cmd cmd);

struct Heartbeat
{
    quint32 axis_error = 0;
    int state = Undefined;
};
bool DecodeHeartbeat(const CanFrame &f, Heartbeat &h);
// the two float32 signals of Get_Encoder_Estimates, Get_Iq, Get_Bus_Voltage_Current, ...
bool DecodeFloats(const CanFrame &f, float &first, float &second);

// little-endian field access, also used by the simulator
quint32 U32(const QByteArray &d, int at);
float F32(const QByteArray &d, int at);
qint16 S16(const QByteArray &d, int at);
void PutU32(QByteArray &d, int at, quint32 v);
void PutF32(QByteArray &d, int at, float v);
void PutS16(QByteArray &d, int at, qint16 v);

// "IDLE", "CLOSED_LOOP", ... for the UI
QString StateName(int state);
// names of the known 0.5.x bits, "" for none; unknown bits are listed in hex
QString ErrorText(quint32 axis_error);
}

#endif // ODRIVEPROTOCOL_H
