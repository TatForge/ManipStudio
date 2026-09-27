#ifndef ROBOTBACKEND_H
#define ROBOTBACKEND_H

#include "solverworker.h"

#include <QElapsedTimer>
#include <QObject>
#include <QThread>
#include <QTimer>
#include <QVariantList>

// The single object QML talks to. Angles are in degrees on this side,
// radians on the solver side. Heavy work runs in SolverWorker's thread;
// a computed trajectory is then played back here in real time.
class RobotBackend : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool ready READ ready NOTIFY readyChanged)
    Q_PROPERTY(bool busy READ busy NOTIFY busyChanged)
    Q_PROPERTY(bool playing READ playing NOTIFY playingChanged)
    Q_PROPERTY(QVariantList joints READ joints NOTIFY poseChanged)
    Q_PROPERTY(QVariantList tcp READ tcp NOTIFY poseChanged)
    Q_PROPERTY(QVariantList frames READ frames NOTIFY poseChanged)
    Q_PROPERTY(QVariantList limits READ limits NOTIFY readyChanged)
    Q_PROPERTY(double progress READ progress NOTIFY progressChanged)
    Q_PROPERTY(double duration READ duration NOTIFY trajectoryChanged)
    Q_PROPERTY(int sampleCount READ sampleCount NOTIFY trajectoryChanged)
    Q_PROPERTY(QString status READ status NOTIFY statusChanged)
    Q_PROPERTY(QString dataDir READ dataDir CONSTANT)
public:
    explicit RobotBackend(const QString &data_dir, QObject *parent = nullptr);
    ~RobotBackend() override;

    bool ready() const { return m_ready; }
    bool busy() const { return m_busy; }
    bool playing() const { return m_play_timer.isActive(); }
    QVariantList joints() const;
    QVariantList tcp() const;
    // origins of the joint frames (QVector3D, metres), base to flange
    QVariantList frames() const;
    QVariantList limits() const;
    double progress() const;
    double duration() const;
    int sampleCount() const;
    QString status() const { return m_status; }
    QString dataDir() const { return m_data_dir; }
    const ArmPose &pose() const { return m_pose; }

    // Jog: sets joint angles directly (clamped to the limits).
    Q_INVOKABLE void setJoint(int index, double deg);
    Q_INVOKABLE void setJoints(const QVariantList &deg);
    // Plans a synchronized joint move from the current angles and plays it back.
    Q_INVOKABLE void moveJ(const QVariantList &target_deg, double speed_deg_s, double accel_deg_s2);
    Q_INVOKABLE void stop();
    Q_INVOKABLE QVariantList zeroPose() const;
    Q_INVOKABLE QVariantList internalPose() const;

signals:
    void readyChanged();
    void busyChanged();
    void playingChanged();
    void poseChanged();
    void progressChanged();
    void trajectoryChanged();
    void statusChanged();
    void errorOccurred(const QString &message);
    void motionFinished();

    // to the worker thread
    void requestPose(const QVector<double> &joints_rad);
    void requestMoveJ(const QVector<double> &from_rad, const QVector<double> &to_rad, double speed, double accel, double dt_ms);

private:
    void OnInitialized(const QVector<double> &lo, const QVector<double> &hi, double floor_z);
    void OnPose(const ArmPose &pose);
    void OnTrajectory(const QVector<float> &samples, double dt_ms);
    void OnFailed(const QString &what, const QString &message);
    void OnPlayTick();
    void ShowSample(int index);
    void SetBusy(bool busy);
    void SetStatus(const QString &status);

    QString m_data_dir;
    QThread m_thread;
    SolverWorker *m_worker;
    bool m_ready = false;
    bool m_busy = false;
    QString m_status;
    QVector<double> m_lo, m_hi;
    QVector<double> m_target_rad;       // latest commanded angles
    ArmPose m_pose;                     // latest computed pose
    QVector<float> m_samples;
    double m_dt_ms = 0;
    int m_sample = 0;
    QTimer m_play_timer;
    QElapsedTimer m_play_clock;
};

#endif // ROBOTBACKEND_H
