#include "hardwaretransports.h"

#include "slcan.h"

#ifdef MANIP_HAVE_SERIALPORT
#include <QSerialPort>
#include <QSerialPortInfo>
#endif
#ifdef MANIP_HAVE_SERIALBUS
#include <QCanBus>
#include <QCanBusDevice>
#include <QCanBusFrame>
#include <QVariant>
#endif

namespace
{
#ifdef MANIP_HAVE_SERIALPORT
// USB-serial adapter speaking SLCAN
class SlcanTransport : public CanTransport
{
public:
    SlcanTransport(const QString &port, int bitrate) : m_bitrate(bitrate)
    {
        m_port.setPortName(port);
        // a USB CDC device ignores the baud rate; real UART adapters use this one
        m_port.setBaudRate(115200);
        connect(&m_port, &QSerialPort::readyRead, this, [this] { Read(); });
        connect(&m_port, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError e) {
            if (e != QSerialPort::NoError && e != QSerialPort::TimeoutError)
                emit ErrorOccurred(m_port.errorString());
        });
    }
    ~SlcanTransport() override { Close(); }

    bool Open(QString *error) override
    {
        const QByteArray rate = slcan::BitrateCommand(m_bitrate);
        if (rate.isEmpty())
        {
            *error = QStringLiteral("SLCAN cannot use %1 bit/s").arg(m_bitrate);
            return false;
        }
        if (!m_port.open(QIODevice::ReadWrite))
        {
            *error = m_port.errorString();
            return false;
        }
        // close a channel left open, set the bit rate, open
        m_port.write("\r\r\rC\r" + rate + "O\r");
        return m_port.waitForBytesWritten(500);
    }
    void Close() override
    {
        if (m_port.isOpen())
        {
            m_port.write("C\r");
            m_port.waitForBytesWritten(100);
            m_port.close();
        }
    }
    bool Send(const CanFrame &frame) override
    {
        return m_port.isOpen() && m_port.write(slcan::Encode(frame)) > 0;
    }

private:
    void Read()
    {
        m_buffer += m_port.readAll();
        int end;
        while ((end = m_buffer.indexOf('\r')) >= 0)
        {
            QByteArray line = m_buffer.left(end);
            m_buffer.remove(0, end + 1);
            // a BEL means the adapter refused a command
            if (line.startsWith('\a'))
            {
                emit ErrorOccurred(QStringLiteral("the adapter refused a command"));
                line.remove(0, 1);
            }
            CanFrame f;
            if (slcan::Decode(line, f))
                emit FrameReceived(f);
        }
        if (m_buffer.size() > 4096)
            m_buffer.clear();
    }

    QSerialPort m_port;
    QByteArray m_buffer;
    int m_bitrate;
};
#endif

#ifdef MANIP_HAVE_SERIALBUS
// any CAN plugin of Qt SerialBus
class QtCanTransport : public CanTransport
{
public:
    QtCanTransport(const QString &plugin, const QString &interface, int bitrate)
        : m_plugin(plugin), m_interface(interface), m_bitrate(bitrate)
    {
    }
    ~QtCanTransport() override { Close(); }

    bool Open(QString *error) override
    {
        QString create_error;
        m_device.reset(QCanBus::instance()->createDevice(m_plugin, m_interface, &create_error));
        if (!m_device)
        {
            *error = create_error;
            return false;
        }
        m_device->setConfigurationParameter(QCanBusDevice::BitRateKey, QVariant(m_bitrate));
        connect(m_device.get(), &QCanBusDevice::framesReceived, this, [this] {
            while (m_device && m_device->framesAvailable())
            {
                const QCanBusFrame in = m_device->readFrame();
                if (!in.isValid() || in.hasExtendedFrameFormat() ||
                    (in.frameType() != QCanBusFrame::DataFrame && in.frameType() != QCanBusFrame::RemoteRequestFrame))
                    continue;
                CanFrame f;
                f.id = in.frameId();
                f.rtr = in.frameType() == QCanBusFrame::RemoteRequestFrame;
                if (f.rtr)
                    f.length = int(in.payload().size());
                else
                    f.data = in.payload();
                emit FrameReceived(f);
            }
        });
        connect(m_device.get(), &QCanBusDevice::errorOccurred, this, [this](QCanBusDevice::CanBusError) {
            emit ErrorOccurred(m_device->errorString());
        });
        if (!m_device->connectDevice())
        {
            *error = m_device->errorString();
            m_device.reset();
            return false;
        }
        return true;
    }
    void Close() override
    {
        if (m_device)
            m_device->disconnectDevice();
        m_device.reset();
    }
    bool Send(const CanFrame &frame) override
    {
        if (!m_device)
            return false;
        QCanBusFrame out(frame.id, frame.rtr ? QByteArray(frame.length, '\0') : frame.data);
        // for a remote request the payload length is the requested DLC
        if (frame.rtr)
            out.setFrameType(QCanBusFrame::RemoteRequestFrame);
        return m_device->writeFrame(out);
    }

private:
    QString m_plugin, m_interface;
    int m_bitrate;
    std::unique_ptr<QCanBusDevice> m_device;
};
#endif
}

namespace can_hardware
{
QStringList Backends()
{
    QStringList out;
#ifdef MANIP_HAVE_SERIALPORT
    out << QStringLiteral("slcan");
#endif
#ifdef MANIP_HAVE_SERIALBUS
    for (const QString &plugin : QCanBus::instance()->plugins())
        if (plugin != QLatin1String("virtualcan"))
            out << plugin;
#endif
    return out;
}

QStringList Interfaces(const QString &backend)
{
    QStringList out;
#ifdef MANIP_HAVE_SERIALPORT
    if (backend == QLatin1String("slcan"))
    {
        for (const QSerialPortInfo &port : QSerialPortInfo::availablePorts())
            out << port.portName();
        return out;
    }
#endif
#ifdef MANIP_HAVE_SERIALBUS
    for (const QCanBusDeviceInfo &info : QCanBus::instance()->availableDevices(backend))
        out << info.name();
#endif
    Q_UNUSED(backend)
    return out;
}

std::unique_ptr<CanTransport> Create(const QString &backend, const QString &interface, int bitrate, QString *error)
{
#ifdef MANIP_HAVE_SERIALPORT
    if (backend == QLatin1String("slcan"))
        return std::make_unique<SlcanTransport>(interface, bitrate);
#endif
#ifdef MANIP_HAVE_SERIALBUS
    if (QCanBus::instance()->plugins().contains(backend))
        return std::make_unique<QtCanTransport>(backend, interface, bitrate);
#endif
    Q_UNUSED(interface)
    Q_UNUSED(bitrate)
    *error = QStringLiteral("CAN backend \"%1\" is not available in this build").arg(backend);
    return nullptr;
}
}
