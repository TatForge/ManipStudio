#ifndef MOTORHUB_H
#define MOTORHUB_H

#include "cantransport.h"

#include <QElapsedTimer>
#include <QObject>
#include <QPointF>
#include <QStringList>
#include <QTimer>
#include <QVariantList>
#include <QVector>

#include <memory>

// Motor set-up and test over CAN (ODrive): connects to a bus, maps every joint
// to an ODrive axis (node id, gear ratio, direction, zero), keeps the live
// state of the axes and sends the test commands. QML context property `motors`.
//
// Units on this side are joint degrees; the axis works in motor turns:
//   turns = zero + direction * deg / 360 * gear
class MotorHub : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool connected READ connected NOTIFY connectedChanged)
    Q_PROPERTY(QString busName READ busName NOTIFY connectedChanged)
    Q_PROPERTY(QStringList backends READ backends CONSTANT)
    Q_PROPERTY(int axisCount READ axisCount WRITE setAxisCount NOTIFY configChanged)
    Q_PROPERTY(QVariantList axes READ axes NOTIFY axesChanged)
    Q_PROPERTY(QString status READ status NOTIFY statusChanged)

public:
    static constexpr int kMaxAxes = 6;
    // an axis that sent nothing for this long is offline
    static constexpr int kOfflineMs = 500;
    // a held jog button must repeat jog() at least this often
    static constexpr int kJogTimeoutMs = 300;

    // config_file: ini with the motor settings (group "motors")
    explicit MotorHub(const QString &config_file, QObject *parent = nullptr);
    ~MotorHub() override;

    bool connected() const { return m_bus != nullptr; }
    QString busName() const { return m_bus_name; }
    // "simulator" first, then the CAN adapters of this build
    QStringList backends() const;
    Q_INVOKABLE QStringList interfaces(const QString &backend) const;
    Q_INVOKABLE bool connectBus(const QString &backend, const QString &interface, int bitrate);
    // uses `transport` (opened here); for tests and the simulator
    bool ConnectTransport(std::unique_ptr<CanTransport> transport, const QString &name);
    Q_INVOKABLE void disconnectBus();
    CanTransport *Transport() const { return m_bus.get(); }

    int axisCount() const { return m_count; }
    void setAxisCount(int count);
    // per axis: {index, name, node, online, state, stateName, error, errorText, closedLoop, busy,
    // hasPosition, position (deg), velocity (deg/s), current (A), busVoltage (V), target (deg or NaN),
    // motorTurns, gear, direction, zero (turns), maxSpeed (deg/s), maxAccel (deg/s^2), currentLimit (A),
    // min, max (deg)}
    QVariantList axes() const;
    QString status() const;

    // {node, gear, direction, zero, maxSpeed, maxAccel, currentLimit}; missing keys stay
    Q_INVOKABLE bool setAxisConfig(int axis, const QVariantMap &config);
    // the joint's present position becomes its zero
    Q_INVOKABLE void setZeroHere(int axis);
    Q_INVOKABLE bool saveConfig();
    Q_INVOKABLE bool loadConfig();
    // joint range from the robot settings: [{min, max}] in degrees, as robot.limits
    Q_INVOKABLE void setJointLimits(const QVariantList &limits);

    // ODrive full calibration: the motor beeps and turns, the joint must be free to move
    Q_INVOKABLE void calibrate(int axis);
    // closed loop, holding the present position
    Q_INVOKABLE void enable(int axis);
    // idle: the motor is free (an arm may fall!)
    Q_INVOKABLE void disable(int axis);
    Q_INVOKABLE void clearErrors(int axis);
    // trapezoidal move to a joint angle (clamped to the joint range)
    Q_INVOKABLE void moveTo(int axis, double deg);
    // hold-to-run: direction -1 / +1 at maxSpeed; repeat within kJogTimeoutMs, 0 stops
    Q_INVOKABLE void jog(int axis, int direction);
    // every axis in closed loop slows down and holds
    Q_INVOKABLE void stopAll();
    // every axis to idle
    Q_INVOKABLE void disableAll();

    // recent history for the plots (x: seconds): "position", "target", "velocity", "current"
    QVector<QPointF> Series(int axis, const QString &kind) const;
    Q_INVOKABLE bool fillSeries(QObject *series, int axis, const QString &kind) const;
    // {xMin, xMax, yMin, yMax} over the given kinds of one axis
    Q_INVOKABLE QVariantMap seriesRange(int axis, const QStringList &kinds) const;
    Q_INVOKABLE void retranslate() { emit statusChanged(); }

signals:
    void connectedChanged();
    void configChanged();
    void axesChanged();
    void statusChanged();
    void errorOccurred(const QString &message);

private:
    struct Axis
    {
        // configuration
        int node = 0;
        double gear = 1;            // motor turns per joint turn
        int direction = 1;
        double zero = 0;            // motor turns at joint zero
        double max_speed = 30;      // deg/s
        double max_accel = 60;      // deg/s^2
        double current_limit = 10;  // A
        // live state
        qint64 last_seen = -1;      // ms on m_clock
        bool online = false;
        int state = 0;
        quint32 error = 0;
        bool has_position = false;
        double turns = 0, turns_s = 0;
        double iq = 0, bus_voltage = 0;
        double target = qQNaN();    // deg
        bool jogging = false;
        int jog_direction = 0;
        qint64 jog_deadline = 0;
        QVector<QPointF> position, target_history, velocity, current;
    };

    void OnFrame(const CanFrame &frame);
    void Poll();
    bool Send(const CanFrame &frame);
    Axis *Get(int axis, const char *action);
    double ToDeg(const Axis &a, double turns) const;
    double ToTurns(const Axis &a, double deg) const;
    double TurnsPerDeg(const Axis &a) const { return a.gear / 360.0; }
    void SendLimits(const Axis &a);
    void Hold(Axis &a);
    void StopJog(Axis &a);
    void Record(QVector<QPointF> &history, double value);
    void SetStatus(const char *source, const QStringList &args = {});
    void Fail(const char *source, const QStringList &args = {});
    double Now() const { return m_clock.elapsed() / 1000.0; }
    QString Name(int axis) const { return QStringLiteral("q%1").arg(axis + 1); }

    QString m_config_file;
    std::unique_ptr<CanTransport> m_bus;
    QString m_bus_name;
    int m_count = 1;
    Axis m_axes[kMaxAxes];
    QVector<QPair<double, double>> m_limits;    // deg
    QTimer m_poll;
    QTimer m_publish;
    QElapsedTimer m_clock;
    int m_tick = 0;
    bool m_dirty = false;
    const char *m_status_source = nullptr;
    QStringList m_status_args;
};

#endif // MOTORHUB_H
