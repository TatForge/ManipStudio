#ifndef CANFRAME_H
#define CANFRAME_H

#include <QByteArray>
#include <QMetaType>

// A classic CAN 2.0 frame with an 11-bit identifier. A remote request (RTR)
// frame carries no data; `length` is the DLC it asks for.
struct CanFrame
{
    quint32 id = 0;
    bool rtr = false;
    QByteArray data;        // 0..8 bytes, empty for an RTR frame
    int length = 0;         // DLC of an RTR frame

    bool operator==(const CanFrame &o) const
    {
        return id == o.id && rtr == o.rtr && data == o.data && (!rtr || length == o.length);
    }
};
Q_DECLARE_METATYPE(CanFrame)

#endif // CANFRAME_H
