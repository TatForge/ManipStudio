#ifndef ROBOTBACKEND_H
#define ROBOTBACKEND_H

#include "solverworker.h"

#include <QElapsedTimer>
#include <QStringList>
#include <QObject>
#include <QThread>
#include <QTimer>
#include <QVariantList>
#include <QVariantMap>
#include <QPointF>
#include <QUrl>

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
    Q_PROPERTY(double floorZ READ floorZ NOTIFY readyChanged)
    Q_PROPERTY(QVariantList meshParts READ meshParts NOTIFY readyChanged)
    Q_PROPERTY(QVariantList meshPoses READ meshPoses NOTIFY poseChanged)
    Q_PROPERTY(QVariantMap toolPose READ toolPose NOTIFY poseChanged)
    Q_PROPERTY(int visualPartCount READ visualPartCount NOTIFY poseChanged)
    Q_PROPERTY(QVariantList visualParts READ visualParts NOTIFY poseChanged)
    Q_PROPERTY(QVariantList tcpPath READ tcpPath NOTIFY trajectoryChanged)
    Q_PROPERTY(QVariantList maxMoments READ maxMoments NOTIFY readyChanged)
    Q_PROPERTY(int momentCount READ momentCount NOTIFY trajectoryChanged)
    Q_PROPERTY(QVariantList peaks READ peaks NOTIFY trajectoryChanged)
    Q_PROPERTY(QString trajectoryError READ trajectoryError NOTIFY trajectoryChanged)
    Q_PROPERTY(double time READ time NOTIFY progressChanged)
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
    double floorZ() const { return m_floor_z; }
    // {name, source (file URL), obstacle} of the STL parts: the obstacle, see MeshLayout
    QVariantList meshParts() const;
    // {position (QVector3D, m), rotation (QQuaternion)} of every STL part
    QVariantList meshPoses() const;
    // display model built from the joint frames (see VisualLayout); the count
    // depends only on the geometry, so QML can keep one delegate per part
    int visualPartCount() const;
    // {shape ("cylinder" / "sphere"), role, joint, position, rotation, scale}
    // per part; scale maps Quick3D's 100-unit primitives onto the part (metres)
    QVariantList visualParts() const;
    // flange frame {position, rotation}
    QVariantMap toolPose() const;
    // flange positions (QVector3D, m) along the last planned trajectory
    QVariantList tcpPath() const;
    // motor limits |tau| (N*m) from the settings file
    QVariantList maxMoments() const;
    // samples with computed moments; after a failed plan, up to the failing one
    int momentCount() const { return m_moments.size() / 6; }
    // per joint: {peak |tau|, time (s), limit, ratio = peak / limit, exceeded}
    QVariantList peaks() const;
    QString trajectoryError() const { return m_trajectory_error; }
    // playback position, seconds
    double time() const { return m_sample * m_dt_ms / 1000.0; }
    double progress() const;
    double duration() const;
    int sampleCount() const;
    // translated on every read, so it follows the current UI language
    QString status() const;
    QString dataDir() const { return m_data_dir; }
    const ArmPose &pose() const { return m_pose; }

    // Curves of the last trajectory for the plots, x in seconds:
    // "moment" N*m, "position" deg, "velocity" deg/s, "acceleration" deg/s^2.
    QList<QPointF> Series(const QString &kind, int joint) const;
    // {xMax, yMin, yMax} over all joints of one kind
    Q_INVOKABLE QVariantMap seriesRange(const QString &kind) const;
    // replaces the points of a QtCharts QXYSeries with Series(kind, joint)
    Q_INVOKABLE bool fillSeries(QObject *series, const QString &kind, int joint) const;
    // t, q, q', q'' (deg) and tau (N*m) of every sample as CSV
    Q_INVOKABLE bool exportCsv(const QUrl &file) const;

    // Jog: sets joint angles directly (clamped to the limits).
    Q_INVOKABLE void setJoint(int index, double deg);
    Q_INVOKABLE void setJoints(const QVariantList &deg);
    // Plans a synchronized joint move from the current angles and plays it back.
    Q_INVOKABLE void moveJ(const QVariantList &target_deg, double speed_deg_s, double accel_deg_s2);
    Q_INVOKABLE void stop();
    // replays the last trajectory from the current position (from the start at the end)
    Q_INVOKABLE void play();
    // jumps to a moment of the last trajectory (seconds), pausing playback
    Q_INVOKABLE void seek(double seconds);
    Q_INVOKABLE QVariantList zeroPose() const;
    Q_INVOKABLE QVariantList internalPose() const;
    // re-emits the language dependent texts after a language switch
    Q_INVOKABLE void retranslate() { emit statusChanged(); }

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
    void OnInitialized(const QVector<double> &lo, const QVector<double> &hi, double floor_z, const QVector<double> &max_moments,
                       const std::vector<ArmPart> &shape);
    void OnPose(const ArmPose &pose);
    void OnTrajectory(const PlannedTrajectory &trajectory);
    void OnFailed(const QString &what, const QString &message);
    void OnPlayTick();
    void ShowSample(int index);
    void SetBusy(bool busy);
    // source: an untranslated QT_TR_NOOP string with %1, %2... for `args`
    void SetStatus(const char *source, const QStringList &args = {});

    QString m_data_dir;
    QThread m_thread;
    SolverWorker *m_worker;
    bool m_ready = false;
    bool m_busy = false;
    const char *m_status_source = nullptr;
    QStringList m_status_args;
    QVector<double> m_lo, m_hi;
    QVector<double> m_target_rad;       // latest commanded angles
    ArmPose m_pose;                     // latest computed pose
    QVector<float> m_samples;
    QVector<QVector3D> m_tcp_path;
    QVector<float> m_moments;           // 6 per sample
    QVector<double> m_max_moments;
    std::vector<ArmPart> m_shape;      // the arm shape of the core, body frames
    QString m_trajectory_error;
    int m_play_from = 0;
    double m_floor_z = 0;
    double m_dt_ms = 0;
    int m_sample = 0;
    QTimer m_play_timer;
    QElapsedTimer m_play_clock;
};

#endif // ROBOTBACKEND_H
