#ifndef SLCAN_H
#define SLCAN_H

#include "canframe.h"

// SLCAN (Lawicel) ASCII protocol of serial USB-CAN adapters (CANable with slcan
// firmware, many clones): "t<id:3><dlc><data hex>\r" for data frames,
// "r<id:3><dlc>\r" for remote requests.
namespace slcan
{
QByteArray Encode(const CanFrame &f);
// one line without its trailing '\r'; extended (T/R) frames are refused
bool Decode(const QByteArray &line, CanFrame &f);
// "S6\r" for 500 kbit/s; empty for a bit rate the protocol cannot set
QByteArray BitrateCommand(int bitrate);
}

#endif // SLCAN_H
