// UI languages: the Russian .ts loads and switches the texts, and every
// translatable string of the QML files and RobotBackend has a translation,
// so a new qsTr() without one fails here instead of showing up in English.
#include "robotbackend.h"
#include "translation.h"

#include <QDir>
#include <QFile>
#include <QRegularExpression>
#include <QSettings>
#include <QTemporaryDir>
#include <QtTest>

class TestTranslation : public QObject
{
    Q_OBJECT
private:
    static QString Ru(const char *context, const char *source)
    {
        TsTranslator t;
        if (!t.LoadTs(QStringLiteral(":/i18n/studio_ru.ts")))
            return QString();
        return t.translate(context, source);
    }

    // every string literal passed to one of `functions` in `file`
    static QStringList Literals(const QString &file, const QString &functions)
    {
        QFile f(file);
        if (!f.open(QIODevice::ReadOnly))
            return {};
        const QString text = QString::fromUtf8(f.readAll());
        const QRegularExpression re(QStringLiteral("\\b(?:%1)\\(\"((?:[^\"\\\\]|\\\\.)*)\"").arg(functions));
        QStringList out;
        for (auto it = re.globalMatch(text); it.hasNext();)
            out << it.next().captured(1).replace(QStringLiteral("\\\""), QStringLiteral("\""));
        return out;
    }

private slots:
    void initTestCase()
    {
        // keep the saved language of the real app untouched
        QCoreApplication::setOrganizationName(QStringLiteral("ManipStudioTests"));
        QCoreApplication::setApplicationName(QStringLiteral("StudioI18nTests"));
        QSettings().clear();
    }

    void cleanupTestCase() { QSettings().clear(); }

    void russianFileLoads()
    {
        TsTranslator t;
        QString error;
        QVERIFY2(t.LoadTs(QStringLiteral(":/i18n/studio_ru.ts"), &error), qPrintable(error));
        QVERIFY(t.MessageCount() > 40);
        QCOMPARE(t.translate("Main", "ready"), QStringLiteral("готов"));
        QCOMPARE(t.translate("ResultsPanel", "°/s²"), QStringLiteral("°/с²"));
        // unknown strings fall back to the source (empty = no translation)
        QVERIFY(t.translate("Main", "no such text").isEmpty());
        QVERIFY(t.translate("NoSuchContext", "ready").isEmpty());
    }

    void everyQmlStringIsTranslated()
    {
        const QDir qml(QStringLiteral(STUDIO_SOURCE_DIR "/qml"));
        int checked = 0;
        QStringList missing;
        for (const QString &name : qml.entryList({QStringLiteral("*.qml")}, QDir::Files))
        {
            const QByteArray context = QFileInfo(name).completeBaseName().toUtf8();
            for (const QString &source : Literals(qml.filePath(name), QStringLiteral("qsTr")))
            {
                checked++;
                if (Ru(context.constData(), source.toUtf8().constData()).isEmpty())
                    missing << QStringLiteral("%1: \"%2\"").arg(name, source);
            }
        }
        QVERIFY(checked > 40);
        QVERIFY2(missing.isEmpty(), qPrintable(missing.join(QLatin1Char('\n'))));
    }

    void everyBackendStringIsTranslated()
    {
        const QStringList sources = Literals(QStringLiteral(STUDIO_SOURCE_DIR "/robotbackend.cpp"), QStringLiteral("tr|QT_TR_NOOP"));
        QVERIFY(sources.size() >= 8);
        QStringList missing;
        for (const QString &source : sources)
            if (Ru("RobotBackend", source.toUtf8().constData()).isEmpty())
                missing << source;
        QVERIFY2(missing.isEmpty(), qPrintable(missing.join(QLatin1Char('\n'))));
    }

    void unfinishedTranslationsAreIgnored()
    {
        QTemporaryDir dir;
        const QString path = QDir(dir.path()).filePath(QStringLiteral("t.ts"));
        QFile f(path);
        QVERIFY(f.open(QIODevice::WriteOnly));
        f.write("<?xml version=\"1.0\" encoding=\"utf-8\"?><TS version=\"2.1\" language=\"ru_RU\"><context><name>C</name>"
                "<message><source>done</source><translation>готово</translation></message>"
                "<message><source>draft</source><translation type=\"unfinished\">черновик</translation></message>"
                "</context></TS>");
        f.close();
        TsTranslator t;
        QVERIFY(t.LoadTs(path));
        QCOMPARE(t.MessageCount(), 1);
        QCOMPARE(t.translate("C", "done"), QStringLiteral("готово"));
        QVERIFY(t.translate("C", "draft").isEmpty());
        QVERIFY(!t.LoadTs(QDir(dir.path()).filePath(QStringLiteral("none.ts"))));
    }

    void switchingLanguageTranslatesAndIsRemembered()
    {
        LanguageManager languages;
        QCOMPARE(languages.language(), QStringLiteral("en"));
        QCOMPARE(languages.languages().size(), 2);
        QSignalSpy changed(&languages, &LanguageManager::languageChanged);

        languages.setLanguage(QStringLiteral("ru"));
        QCOMPARE(changed.count(), 1);
        QCOMPARE(QCoreApplication::translate("JointPanel", "Joints"), QStringLiteral("Суставы"));
        QCOMPARE(LanguageManager::InitialLanguage(), QStringLiteral("ru"));

        languages.setLanguage(QStringLiteral("xx"));     // unknown: ignored
        QCOMPARE(languages.language(), QStringLiteral("ru"));
        QCOMPARE(changed.count(), 1);

        languages.setLanguage(QStringLiteral("en"));
        QCOMPARE(QCoreApplication::translate("JointPanel", "Joints"), QStringLiteral("Joints"));
        QCOMPARE(LanguageManager::InitialLanguage(), QStringLiteral("en"));
        QCOMPARE(changed.count(), 2);
    }

    void backendStatusFollowsTheLanguage()
    {
        RobotBackend robot(QStringLiteral(MANIP_DATA_DIR));
        QTRY_VERIFY_WITH_TIMEOUT(robot.ready(), 20000);
        QVERIFY2(robot.status().startsWith(QStringLiteral("Model loaded, floor at z = 0")), qPrintable(robot.status()));

        LanguageManager languages;
        QSignalSpy status(&robot, &RobotBackend::statusChanged);
        languages.setLanguage(QStringLiteral("ru"));
        robot.retranslate();
        QCOMPARE(status.count(), 1);
        // the already shown status is re-rendered, with the same arguments
        QCOMPARE(robot.status(), QStringLiteral("Модель загружена, пол на высоте z = 0 м"));
        languages.setLanguage(QStringLiteral("en"));
        QCOMPARE(robot.status(), QStringLiteral("Model loaded, floor at z = 0 m"));
    }
};

QTEST_GUILESS_MAIN(TestTranslation)
#include "test_translation.moc"
