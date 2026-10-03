#include "robotbackend.h"
#include "translation.h"

#include <QApplication>
#include <QCommandLineParser>
#include <QFileInfo>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QQuickStyle>
#include <QQuickWindow>
#include <QTimer>

namespace
{
// "10,-20,30,0,45,0" -> six angles in degrees; empty list if malformed
QVariantList ParseAngles(const QString &text)
{
    QVariantList out;
    for (const QString &part : text.split(QLatin1Char(','), Qt::SkipEmptyParts))
    {
        bool ok = false;
        const double v = part.trimmed().toDouble(&ok);
        if (!ok)
            return {};
        out.append(v);
    }
    return out.size() == 6 ? out : QVariantList();
}
}

int main(int argc, char *argv[])
{
    // QtCharts' ChartView needs a QApplication, not just QGuiApplication
    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("ManipStudio"));
    QApplication::setOrganizationName(QStringLiteral("ManipStudio"));
    QQuickStyle::setStyle(QStringLiteral("Material"));

    QCommandLineParser parser;
    parser.addHelpOption();
    QCommandLineOption data_option(QStringLiteral("data"), QStringLiteral("Directory with ManipulatorSettigns.ini and STLmodels/."),
                                   QStringLiteral("dir"), QStringLiteral(MANIP_DATA_DIR));
    QCommandLineOption joints_option(QStringLiteral("joints"), QStringLiteral("Initial joint angles, degrees: q1,...,q6."),
                                     QStringLiteral("angles"));
    QCommandLineOption movej_option(QStringLiteral("movej"), QStringLiteral("Run MoveJ to these angles (degrees) after loading."),
                                    QStringLiteral("angles"));
    QCommandLineOption shot_option(QStringLiteral("screenshot"),
                                   QStringLiteral("Save the window to <file> once loaded (after --movej finishes), then quit."),
                                   QStringLiteral("file"));
    QCommandLineOption lang_option(QStringLiteral("lang"), QStringLiteral("UI language: en or ru (default: last used, else system)."),
                                   QStringLiteral("code"));
    QCommandLineOption settings_option(QStringLiteral("settings"), QStringLiteral("Open the settings panel once loaded."));
    parser.addOptions({data_option, joints_option, movej_option, shot_option, lang_option, settings_option});
    parser.process(app);

    const QVariantList joints = ParseAngles(parser.value(joints_option));
    const QVariantList movej = ParseAngles(parser.value(movej_option));
    if ((parser.isSet(joints_option) && joints.isEmpty()) || (parser.isSet(movej_option) && movej.isEmpty()))
    {
        qCritical("--joints / --movej expect six comma separated angles in degrees");
        return 2;
    }

    // resolved now: the backend changes the working directory to the data directory
    const QString shot = parser.isSet(shot_option) ? QFileInfo(parser.value(shot_option)).absoluteFilePath() : QString();

    RobotBackend robot(parser.value(data_option));
    QObject::connect(&robot, &RobotBackend::statusChanged, &robot, [&robot] { qInfo("%s", qPrintable(robot.status())); });
    QObject::connect(&robot, &RobotBackend::errorOccurred, &robot, [](const QString &m) { qWarning("%s", qPrintable(m)); });

    LanguageManager languages;
    languages.setLanguage(parser.isSet(lang_option) ? parser.value(lang_option) : LanguageManager::InitialLanguage());

    QQmlApplicationEngine engine;
    // the Studio module (QML files and the C++ geometry types) lives at qrc:/Studio
    engine.addImportPath(QStringLiteral("qrc:/"));
    engine.rootContext()->setContextProperty(QStringLiteral("robot"), &robot);
    engine.rootContext()->setContextProperty(QStringLiteral("languages"), &languages);
    QObject::connect(&languages, &LanguageManager::languageChanged, &engine, [&engine, &robot] {
        engine.retranslate();
        robot.retranslate();
    });
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreationFailed, &app, [] { QCoreApplication::exit(1); },
                     Qt::QueuedConnection);
    // Qt 6.4 has no loadFromModule(); the module's files live under qrc:/<URI>/
    engine.load(QUrl(QStringLiteral("qrc:/Studio/Main.qml")));

    // give the scene a moment to render, then save the window
    auto grab = [&engine, shot] {
        QTimer::singleShot(1500, [&engine, shot] {
            auto *window = engine.rootObjects().isEmpty() ? nullptr : qobject_cast<QQuickWindow *>(engine.rootObjects().first());
            const bool ok = window && window->grabWindow().save(shot);
            if (!ok)
                qWarning("screenshot: cannot save %s", qPrintable(shot));
            QCoreApplication::exit(ok ? 0 : 1);
        });
    };
    const bool open_settings = parser.isSet(settings_option);
    auto start = [&robot, &engine, joints, movej, shot, grab, open_settings] {
        if (open_settings && !engine.rootObjects().isEmpty())
            if (QObject *panel = engine.rootObjects().first()->findChild<QObject *>(QStringLiteral("settingsPanel")))
                QMetaObject::invokeMethod(panel, "open");
        if (!joints.isEmpty())
            robot.setJoints(joints);
        if (!movej.isEmpty())
        {
            // the pose request above is queued before the plan, so MoveJ starts from `joints`
            QTimer::singleShot(100, &robot, [&robot, movej] { robot.moveJ(movej, 60, 120); });
            if (!shot.isEmpty())
            {
                QObject::connect(&robot, &RobotBackend::motionFinished, &robot, grab);
                QObject::connect(&robot, &RobotBackend::errorOccurred, &robot, grab);
            }
        }
        else if (!shot.isEmpty())
            grab();
    };
    if (robot.ready())
        start();
    else
        QObject::connect(&robot, &RobotBackend::readyChanged, &app, start, Qt::SingleShotConnection);
    return app.exec();
}
