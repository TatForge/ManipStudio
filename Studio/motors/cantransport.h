#ifndef CANTRANSPORT_H
#define CANTRANSPORT_H

#include "canframe.h"

#include <QObject>
#include <QString>

// A CAN bus connection: the simulator, an SLCAN serial adapter or a Qt CAN plugin.
class CanTransport : public QObject
{
    Q_OBJECT
public:
    using QObject::QObject;
    // false and *error set if the bus cannot be opened
    virtual bool Open(QString *error) = 0;
    virtual void Close() = 0;
    virtual bool Send(const CanFrame &frame) = 0;

signals:
    void FrameReceived(const CanFrame &frame);
    // the connection broke or the adapter reported a bus error
    void ErrorOccurred(const QString &message);
};

#endif // CANTRANSPORT_H
