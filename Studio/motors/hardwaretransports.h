#ifndef HARDWARETRANSPORTS_H
#define HARDWARETRANSPORTS_H

#include "cantransport.h"

#include <QStringList>

#include <memory>

// Real CAN adapters. Each kind is compiled in only if its Qt module was found
// (SerialPort for SLCAN, SerialBus for the Qt CAN plugins).
namespace can_hardware
{
// "slcan" (USB-serial adapters such as CANable) and the Qt CAN plugins
// ("socketcan", "peakcan", ...), whichever are available in this build
QStringList Backends();
// serial ports for "slcan", the plugin's interfaces otherwise
QStringList Interfaces(const QString &backend);
// nullptr and *error set for an unknown backend
std::unique_ptr<CanTransport> Create(const QString &backend, const QString &interface, int bitrate, QString *error);
}

#endif // HARDWARETRANSPORTS_H
