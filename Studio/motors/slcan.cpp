#include "slcan.h"

namespace slcan
{
QByteArray Encode(const CanFrame &f)
{
    QByteArray out;
    out += f.rtr ? 'r' : 't';
    out += QByteArray::number(f.id & 0x7FF, 16).rightJustified(3, '0').toUpper();
    const int dlc = f.rtr ? f.length : int(f.data.size());
    out += char('0' + qBound(0, dlc, 8));
    if (!f.rtr)
        out += f.data.left(8).toHex().toUpper();
    out += '\r';
    return out;
}

bool Decode(const QByteArray &line, CanFrame &f)
{
    if (line.size() < 5 || (line[0] != 't' && line[0] != 'r'))
        return false;
    bool ok = false;
    const quint32 id = line.mid(1, 3).toUInt(&ok, 16);
    if (!ok || id > 0x7FF)
        return false;
    const int dlc = line[4] - '0';
    if (dlc < 0 || dlc > 8)
        return false;
    CanFrame out;
    out.id = id;
    out.rtr = line[0] == 'r';
    if (out.rtr)
        out.length = dlc;
    else
    {
        // adapters may append a timestamp after the data: ignore it
        const QByteArray hex = line.mid(5, 2 * dlc);
        if (hex.size() != 2 * dlc)
            return false;
        for (char c : hex)
            if (!isxdigit(uchar(c)))
                return false;
        out.data = QByteArray::fromHex(hex);
    }
    f = out;
    return true;
}

QByteArray BitrateCommand(int bitrate)
{
    static const int rates[] = {10000, 20000, 50000, 100000, 125000, 250000, 500000, 800000, 1000000};
    for (int i = 0; i < 9; i++)
        if (rates[i] == bitrate)
            return QByteArray("S") + char('0' + i) + '\r';
    return {};
}
}
