#ifndef TRANSLATION_H
#define TRANSLATION_H

#include <QHash>
#include <QObject>
#include <QString>
#include <QTranslator>
#include <QVariantList>

// QTranslator that reads Qt Linguist .ts files directly, so the build needs
// no lrelease. Only finished translations are used; the rest fall back to
// the (English) source text.
class TsTranslator : public QTranslator
{
public:
    using QTranslator::QTranslator;
    bool LoadTs(const QString &path, QString *error = nullptr);
    QString translate(const char *context, const char *source_text, const char *disambiguation = nullptr, int n = -1) const override;
    bool isEmpty() const override { return m_messages.isEmpty(); }
    int MessageCount() const;

private:
    QHash<QString, QHash<QString, QString>> m_messages;     // context -> source -> translation
};

// UI language: English (the source strings) or a translation from
// :/i18n/studio_<code>.ts. QML reads `language` and calls setLanguage();
// whoever owns the QML engine retranslates it on languageChanged.
class LanguageManager : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString language READ language WRITE setLanguage NOTIFY languageChanged)
    Q_PROPERTY(QVariantList languages READ languages CONSTANT)
public:
    explicit LanguageManager(QObject *parent = nullptr);
    ~LanguageManager() override;

    QString language() const { return m_language; }
    void setLanguage(const QString &code);
    // [{code, name}] of the available languages
    QVariantList languages() const;
    // the saved choice, else the system language if available, else English
    static QString InitialLanguage();

signals:
    void languageChanged();

private:
    QString m_language = QStringLiteral("en");
    TsTranslator *m_translator = nullptr;
};

#endif // TRANSLATION_H
