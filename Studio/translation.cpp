#include "translation.h"

#include <QCoreApplication>
#include <QFile>
#include <QLocale>
#include <QSettings>
#include <QXmlStreamReader>

namespace
{
struct Language
{
    const char *code;
    const char *name;
};
// English is the source language and needs no file
const Language kLanguages[] = {{"en", "English"}, {"ru", "Русский"}};

QString TsPath(const QString &code)
{
    return QStringLiteral(":/i18n/studio_%1.ts").arg(code);
}

bool Known(const QString &code)
{
    for (const Language &l : kLanguages)
        if (code == QLatin1String(l.code))
            return true;
    return false;
}
}

bool TsTranslator::LoadTs(const QString &path, QString *error)
{
    m_messages.clear();
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
    {
        if (error)
            *error = QStringLiteral("cannot open ") + path;
        return false;
    }
    QXmlStreamReader xml(&file);
    QString context, source, translation;
    bool finished = false;
    while (!xml.atEnd())
    {
        xml.readNext();
        if (!xml.isStartElement())
            continue;
        const auto name = xml.name();
        if (name == QLatin1String("context"))
            context.clear();
        else if (name == QLatin1String("name"))
            context = xml.readElementText();
        else if (name == QLatin1String("message"))
        {
            source.clear();
            translation.clear();
            finished = false;
        }
        else if (name == QLatin1String("source"))
            source = xml.readElementText();
        else if (name == QLatin1String("translation"))
        {
            const auto type = xml.attributes().value(QLatin1String("type"));
            finished = type != QLatin1String("unfinished") && type != QLatin1String("obsolete") && type != QLatin1String("vanished");
            translation = xml.readElementText();
            if (finished && !translation.isEmpty())
                m_messages[context].insert(source, translation);
        }
    }
    if (xml.hasError())
    {
        if (error)
            *error = path + QStringLiteral(": ") + xml.errorString();
        m_messages.clear();
        return false;
    }
    return true;
}

QString TsTranslator::translate(const char *context, const char *source_text, const char *, int) const
{
    const auto ctx = m_messages.constFind(QString::fromUtf8(context));
    if (ctx == m_messages.constEnd())
        return QString();
    return ctx->value(QString::fromUtf8(source_text));
}

int TsTranslator::MessageCount() const
{
    int n = 0;
    for (const auto &ctx : m_messages)
        n += ctx.size();
    return n;
}

LanguageManager::LanguageManager(QObject *parent) : QObject(parent)
{
}

LanguageManager::~LanguageManager()
{
    if (m_translator)
        QCoreApplication::removeTranslator(m_translator);
    delete m_translator;
}

void LanguageManager::setLanguage(const QString &code)
{
    if (!Known(code) || code == m_language)
        return;
    if (m_translator)
    {
        QCoreApplication::removeTranslator(m_translator);
        delete m_translator;
        m_translator = nullptr;
    }
    if (code != QLatin1String("en"))
    {
        auto *translator = new TsTranslator;
        QString error;
        if (!translator->LoadTs(TsPath(code), &error))
        {
            qWarning("LanguageManager: %s", qPrintable(error));
            delete translator;
            return;
        }
        m_translator = translator;
        QCoreApplication::installTranslator(m_translator);
    }
    m_language = code;
    QSettings().setValue(QStringLiteral("ui/language"), m_language);
    emit languageChanged();
}

QVariantList LanguageManager::languages() const
{
    QVariantList out;
    for (const Language &l : kLanguages)
        out.append(QVariantMap{{QStringLiteral("code"), QString::fromLatin1(l.code)}, {QStringLiteral("name"), QString::fromUtf8(l.name)}});
    return out;
}

QString LanguageManager::InitialLanguage()
{
    const QString saved = QSettings().value(QStringLiteral("ui/language")).toString();
    if (Known(saved))
        return saved;
    const QString system = QLocale::system().name().section(QLatin1Char('_'), 0, 0);
    return Known(system) ? system : QStringLiteral("en");
}
