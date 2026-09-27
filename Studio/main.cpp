#include "robotbackend.h"

#include <QApplication>
#include <QCommandLineParser>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>
#include <QQuickWindow>
#include <QTimer>

int main(int argc, char *argv[])
{
    // QtCharts' ChartView needs a QApplication, not just QGuiApplication
    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("ManipStudio"));
    QQuickStyle::setStyle(QStringLiteral("Material"));

    QCommandLineParser parser;
    parser.addHelpOption();
    QCommandLineOption data_option(QStringLiteral("data"), QStringLiteral("Directory with ManipulatorSettigns.ini and STLmodels/."),
                                   QStringLiteral("dir"), QStringLiteral(MANIP_DATA_DIR));
    parser.addOption(data_option);
    QCommandLineOption shot_option(QStringLiteral("screenshot"), QStringLiteral("Save the window to <file> once the model is loaded, then quit."),
                                   QStringLiteral("file"));
    parser.addOption(shot_option);
    parser.process(app);

    RobotBackend robot(parser.value(data_option));
    QObject::connect(&robot, &RobotBackend::statusChanged, &robot, [&robot] { qInfo("%s", qPrintable(robot.status())); });
    QObject::connect(&robot, &RobotBackend::errorOccurred, &robot, [](const QString &m) { qWarning("%s", qPrintable(m)); });

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty(QStringLiteral("robot"), &robot);
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreationFailed, &app, [] { QCoreApplication::exit(1); },
                     Qt::QueuedConnection);
    // Qt 6.4 has no loadFromModule(); the module's files live under qrc:/<URI>/
    engine.load(QUrl(QStringLiteral("qrc:/Studio/Main.qml")));

    if (parser.isSet(shot_option))
    {
        const QString file = parser.value(shot_option);
        // give the scene a moment to render the loaded pose
        auto grab = [&engine, file] {
            QTimer::singleShot(1500, [&engine, file] {
                auto *window = engine.rootObjects().isEmpty() ? nullptr : qobject_cast<QQuickWindow *>(engine.rootObjects().first());
                const bool ok = window && window->grabWindow().save(file);
                if (!ok)
                    qWarning("screenshot: cannot save %s", qPrintable(file));
                QCoreApplication::exit(ok ? 0 : 1);
            });
        };
        if (robot.ready())
            grab();
        else
            QObject::connect(&robot, &RobotBackend::readyChanged, &app, grab);
    }
    return app.exec();
}
