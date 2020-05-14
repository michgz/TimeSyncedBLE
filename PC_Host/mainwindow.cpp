#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QtCharts>
using namespace QtCharts;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);


    QChart *chart = new QChart;
    chart->setAnimationOptions(QChart::NoAnimation);

    ui->chartv->setChart(chart);


}

MainWindow::~MainWindow()
{
    delete ui;
}


#include <QBluetoothDeviceDiscoveryAgent>
#include <QBluetoothAddress>
#include <QLowEnergyController>
#include <QBluetoothDeviceInfo>
#include <QTimer>

#include <QFile>
#include <QDateTime>


// The old values
//#define SERVICE_UUID      "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
//#define READ_CHAR_UUID    "6e400003-b5a3-f393-e0a9-e50e24dcca9e"
//#define WRITE_CHAR_UUID   "6e400002-b5a3-f393-e0a9-e50e24dcca9e"


//The new values
#define SERVICE_UUID      "21171523-4740-4aa5-b66b-5d2c6851cc5c"
#define READ_CHAR_UUID    "21171524-4740-4aa5-b66b-5d2c6851cc5c"
#define WRITE_CHAR_UUID   "21170002-4740-4aa5-b66b-5d2c6851cc5c"



void Worker::timeOut0016(void)
{
    timer0016.stop();
    disconnect(&timer0016, &QTimer::timeout, this, &Worker::timeOut0016);

    // disable notification
    servs->writeDescriptor(
                servs->characteristic(QBluetoothUuid(QString(READ_CHAR_UUID))).descriptor(
                                                    QBluetoothUuid::ClientCharacteristicConfiguration)
                                                            , QByteArray::fromHex("0000"));

    // de-establish hook into notifications
    disconnect(servs, SIGNAL(characteristicChanged(QLowEnergyCharacteristic,QByteArray)),
            this, SLOT(gotReading0016(QLowEnergyCharacteristic,QByteArray)));
}

void Worker::timeOut000A(void)
{
    timer000A.stop();
    disconnect(&timer000A, &QTimer::timeout, this, &Worker::timeOut000A);

    // disable notification
    servs->writeDescriptor(
                servs->characteristic(QBluetoothUuid(QString(READ_CHAR_UUID))).descriptor(
                                                    QBluetoothUuid::ClientCharacteristicConfiguration)
                                                            , QByteArray::fromHex("0000"));

    // de-establish hook into notifications
    disconnect(servs, SIGNAL(characteristicChanged(QLowEnergyCharacteristic,QByteArray)),
            this, SLOT(gotReading000A(QLowEnergyCharacteristic,QByteArray)));
}

static float U16toF(uint16_t x)
{
    if (x >= 0x8000U)
    {
        return 1./1024.*((float)(((signed int)x) - 65536));
    }
    else
    {
        return 1./1024.*((float)(signed int)x);
    }

}



void Worker::timeOut000E(void)
{
    timer000E.stop();
    disconnect(&timer000E, &QTimer::timeout, this, &Worker::timeOut000E);

    // disable notification
    servs->writeDescriptor(
                servs->characteristic(QBluetoothUuid(QString(READ_CHAR_UUID))).descriptor(
                                                    QBluetoothUuid::ClientCharacteristicConfiguration)
                                                            , QByteArray::fromHex("0000"));

    // de-establish hook into notifications
    disconnect(servs, SIGNAL(characteristicChanged(QLowEnergyCharacteristic,QByteArray)),
            this, SLOT(gotReading000E(QLowEnergyCharacteristic,QByteArray)));



    // Handle everything that's been received
    emit debugLog("Finished");

    emit debugLog(QString("Got %1 bytes").arg(rxbuf->bytesAvailable()));

    emit gotTrace(rxbuf);

    rxbuf = nullptr;
    delete rxed; rxed = nullptr;
}

uint32_t Worker::ByteArrayGetU32(QByteArray &arr, int index)
{
    if (arr.length() < 4*index)
    {
        // Failed!
        return 0;
    }

    return   (((uint32_t)arr[4*index + 0]) << 0)
            + (((uint32_t)arr[4*index + 1]) << 8)
            + (((uint32_t)arr[4*index + 2]) << 16)
            + (((uint32_t)arr[4*index + 3]) << 24);
}

void Worker::startRead000A(uint32_t timeToRead)
{
    if (!!writeChar && !!readChar)
    {

        emit debugLog("B");

        // Start time-out timer
        timer000A.setSingleShot(true);
        connect(&timer000A, &QTimer::timeout, this, &Worker::timeOut000A);
        timer000A.start(1500);

        // establish hook into notifications
        connect(servs, SIGNAL(characteristicChanged(QLowEnergyCharacteristic,QByteArray)),
                this, SLOT(gotReading000A(QLowEnergyCharacteristic,QByteArray)));

        // enable notification
        servs->writeDescriptor(
                    servs->characteristic(QBluetoothUuid(QString(READ_CHAR_UUID))).descriptor(
                                                        QBluetoothUuid::ClientCharacteristicConfiguration)
                                                                , QByteArray::fromHex("0100"));

        // Write the value
        QByteArray arr = QByteArray::fromHex("0A000000");

        arr += QByteArray::fromRawData((const char *) &timeToRead, 4);

        servs->writeCharacteristic(
                servs->characteristic(QBluetoothUuid(QString(WRITE_CHAR_UUID)))
                    , arr, QLowEnergyService::WriteMode::WriteWithResponse);

    }
}

void Worker::startRead000E(void)
{
    if (!!writeChar && !!readChar)
    {

        emit debugLog("F");


        // Start time-out timer
        timer000E.setSingleShot(true);
        connect(&timer000E, &QTimer::timeout, this, &Worker::timeOut000E);
        timer000E.setInterval(5000);
        timer000E.start();

#if 0
        if (rxed != nullptr)
        {
            // Error!
            throw;
        }

        rxed = new QByteArray();
        rxed->reserve(438*23+ 100);
        rxed->append("1234");  // Start with something, to make sure future appends are deep copies.
#endif

        // establish hook into notifications
        connect(servs, SIGNAL(characteristicChanged(QLowEnergyCharacteristic,QByteArray)),
                this, SLOT(gotReading000E(QLowEnergyCharacteristic,QByteArray)));

            connect(servs, SIGNAL(characteristicRead(const QLowEnergyCharacteristic,const QByteArray)), this, SLOT(finishedReading000E(const QLowEnergyCharacteristic,const QByteArray)));

        // enable notification
        servs->writeDescriptor(
                    servs->characteristic(QBluetoothUuid(QString(READ_CHAR_UUID))).descriptor(
                                                        QBluetoothUuid::ClientCharacteristicConfiguration)
                                                                , QByteArray::fromHex("0100"));

        // Write the value
        servs->writeCharacteristic(
                servs->characteristic(QBluetoothUuid(QString(WRITE_CHAR_UUID)))
                    , QByteArray::fromHex("0E000000FFFFFFFF")
                        , QLowEnergyService::WriteMode::WriteWithResponse);

    }
}


void Worker::finishedReading000A(const QLowEnergyCharacteristic &x,const QByteArray &y)
{
    emit debugLog("E");

    disconnect(servs, SIGNAL(characteristicRead(const QLowEnergyCharacteristic,const QByteArray)), this, SLOT(finishedReading000A(const QLowEnergyCharacteristic,const QByteArray)));


    if (y.length() >= 8)
    {
        QTimer::singleShot(2000, this, &Worker::startRead000E);
    }
}

void Worker::gotReading000A(QLowEnergyCharacteristic x,QByteArray y)
{
    Q_UNUSED(x);

    emit debugLog("D");

    emit foundService(QString("Got %1 Bytes").arg(y.length()));

    timeOut000A();   // This tidies it up, even though haven't really timed out


    connect(servs, SIGNAL(characteristicRead(const QLowEnergyCharacteristic,const QByteArray)), this, SLOT(finishedReading000A(const QLowEnergyCharacteristic,const QByteArray)));

    // read
    servs->readCharacteristic(
                servs->characteristic(QBluetoothUuid(QString(READ_CHAR_UUID))));

    int i;
    QString ss;
    QTextStream strText(&ss);
    for (i = 0; i < y.length() && i < 16; i ++)
    {
        strText << QString("%1 ").arg((int)(unsigned char)y[i], 2, 16, QLatin1Char('0'));
    }

    emit foundPacket(ss);

}





void Worker::finishedReading000E(const QLowEnergyCharacteristic &x,const QByteArray &y)
{
    emit debugLog("H");
#if 0
    disconnect(servs, SIGNAL(characteristicRead(const QLowEnergyCharacteristic,const QByteArray)), this, SLOT(finishedReading000E(const QLowEnergyCharacteristic,const QByteArray)));

    // establish hook into notifications
    connect(servs, SIGNAL(characteristicChanged(QLowEnergyCharacteristic,QByteArray)),
            this, SLOT(gotReading000E(QLowEnergyCharacteristic,QByteArray)));

    // enable notification
    servs->writeDescriptor(
                servs->characteristic(QBluetoothUuid(QString(READ_CHAR_UUID))).descriptor(
                                                    QBluetoothUuid::ClientCharacteristicConfiguration)
                                                            , QByteArray::fromHex("0100"));
#endif
}

static int rr = 0;

void Worker::gotReading000E(QLowEnergyCharacteristic x,QByteArray y)
{
   // Q_UNUSED(x);


    timer000E.start();  // reset the timer

    emit debugLog("G" + QString("%1").arg(y.length()) + " " + QString("%1").arg(x.handle()));

    rr ++;
      emit foundService(QString("Got a total of %1 Packets. This one %2 bytes").arg(rr).arg(y.length()));



#if 0
    emit foundService(QString("Got %1 Bytes").arg(y.length()));

    timeOut000E();   // This tidies it up, even though haven't really timed out


    connect(servs, SIGNAL(characteristicRead(const QLowEnergyCharacteristic,const QByteArray)), this, SLOT(finishedReading000E(const QLowEnergyCharacteristic,const QByteArray)));

    // read
    servs->readCharacteristic(
                servs->characteristic(QBluetoothUuid(QString(READ_CHAR_UUID))));



    int i;
    QString ss;
    QTextStream strText(&ss);
    for (i = 0; i < y.length() && i < 16; i ++)
    {
        strText << QString("%1 ").arg((int)(unsigned char)y[i], 2, 16);
    }

    emit foundPacket(ss);
#endif


    if (y.length() >= 5)
    {
        emit foundPacket(QString("%1 %2 %3 %4 %5").arg((int)(unsigned char)y[0], 2, 16, QLatin1Char('0'))
                .arg((int)(unsigned char)y[1], 2, 16, QLatin1Char('0'))
                .arg((int)(unsigned char)y[2], 2, 16, QLatin1Char('0'))
                .arg((int)(unsigned char)y[3], 2, 16, QLatin1Char('0'))
                .arg((int)(unsigned char)y[4], 2, 16, QLatin1Char('0')));
    }

    //rxed->append(y);

    rxbuf->open(QIODevice::WriteOnly|               QIODevice::Append);
    rxbuf->write(y);
    rxbuf->close();

    // read
 //   servs->readCharacteristic(
 //               servs->characteristic(QBluetoothUuid(QString(READ_CHAR_UUID))));


}















static uint32_t u;

static uint32_t total_rxed = 0;

void Worker::finishedReading0016(const QLowEnergyCharacteristic &x,const QByteArray &y)
{
    disconnect(servs, SIGNAL(characteristicRead(const QLowEnergyCharacteristic,const QByteArray)), this, SLOT(finishedReading0016(const QLowEnergyCharacteristic,const QByteArray)));

    if (u != 0)
    {
        startRead000A(u);
    }
    u = 0;
}

void Worker::gotReading0016(QLowEnergyCharacteristic x,QByteArray y)
{
    Q_UNUSED(x);
    emit foundService(QString("Got %1 Bytes").arg(y.length()));

    total_rxed += y.length();
    emit foundServicesCount(QString("Got total %1 Bytes").arg(total_rxed));

    //timeOut0016();   // This tidies it up, even though haven't really timed out


    if (false)
    {
        if (y.length() >= 8)
        {
            if (ByteArrayGetU32(y, 0) == 0x80000016UL)
            {
                //startRead000A(ByteArrayGetU32(y, 1));
                u = ByteArrayGetU32(y, 1);
            }
        }

        connect(servs, SIGNAL(characteristicRead(const QLowEnergyCharacteristic,const QByteArray)), this, SLOT(finishedReading0016(const QLowEnergyCharacteristic,const QByteArray)));
    }

    // read
    if (false)
    {
        servs->readCharacteristic(
                    servs->characteristic(QBluetoothUuid(QString(READ_CHAR_UUID))));
    }

    //if (rxbuf == nullptr)
    {
        rxbuf = new QBuffer(&y, this);

        //rxbuf->open(QIODevice::WriteOnly|               QIODevice::Append);
        //rxbuf->write(y);
        //rxbuf->close();

        emit gotTrace(rxbuf);

        rxbuf = nullptr;
    }


    if (false)
    {
        int i;
        QString ss;
        QTextStream strText(&ss);
        for (i = 0; i < y.length() && i < 16; i ++)
        {
            strText << QString("%1 ").arg((int)(unsigned char)y[i], 2, 16, QLatin1Char('0'));
        }

        emit foundPacket(ss);
    }

}


void Worker::tryWriting(void)
{
    if (!!writeChar && !!readChar)
    {
        // Start time-out timer
        timer0016.setSingleShot(true);
        //connect(&timer0016, &QTimer::timeout, this, &Worker::timeOut0016);
        //timer0016.start(1500);

        // establish hook into notifications
        connect(servs, SIGNAL(characteristicChanged(QLowEnergyCharacteristic,QByteArray)),
                this, SLOT(gotReading0016(QLowEnergyCharacteristic,QByteArray)));

        // enable notification
        servs->writeDescriptor(
                    servs->characteristic(QBluetoothUuid(QString(READ_CHAR_UUID))).descriptor(
                                                        QBluetoothUuid::ClientCharacteristicConfiguration)
                                                                , QByteArray::fromHex("0100"));

        if (false)
        {
            // Write the value -- currently not needed
            servs->writeCharacteristic(
                    servs->characteristic(QBluetoothUuid(QString(WRITE_CHAR_UUID)))
                        , QByteArray::fromHex("16000000FFFFFFFF")
                            , QLowEnergyService::WriteMode::WriteWithResponse);
        }

    }
}


void Worker::discoveredService(const QBluetoothUuid &newService)
{
    emit foundService(newService.toString());
}

void Worker::characteristicChanged(QLowEnergyCharacteristic x,QByteArray y)
{
    Q_UNUSED(x);
    emit foundService(QString("Got %1 Bytes").arg(y.length()));

    int i;
    QString ss;
    QTextStream strText(&ss);
    for (i = 0; i < y.length() && i < 16; i ++)
    {
        strText << QString("%1 ").arg((int)(unsigned char)y[i], 2, 16, QLatin1Char('0'));
    }

    emit foundPacket(ss);

}

void Worker::on_service(QLowEnergyService::ServiceState newState)
{
    bool isKnownService = false;

    if (newState == QLowEnergyService::ServiceDiscovered)
    {
        emit foundService(QString("Char count: %1").arg(servs->characteristics().count()));
        QString ss;
        QTextStream strText(&ss);
        for(QLowEnergyCharacteristic x : servs->characteristics())
        {
            if (x.isValid())
            {
                strText << x.uuid().toString() << "\r\n";
                if (x.uuid() == QBluetoothUuid(QString(READ_CHAR_UUID)))
                {
                    if ((x.properties() & 0x10) != 0)
                    {
                        readChar = &x;
                        isKnownService = true;
                    }

                    emit foundConnection(QString("props %1").arg(x.properties(), 2, 16));
                }
                else if (x.uuid() == QBluetoothUuid(QString(WRITE_CHAR_UUID)))
                {
                    if ((x.properties() & 0x0C) != 0)
                    {
                        writeChar = &x;
                        isKnownService = true;
                    }
                }
            }
        }
        emit foundCharacteristic(ss);
    }

    if (isKnownService)
    {
        QTimer::singleShot(2000, this, &Worker::tryWriting);
    }

}


void Worker::startDiscoveringServers(QLowEnergyService * servs_in)
{
    servs = servs_in;
    connect(servs, SIGNAL(stateChanged(QLowEnergyService::ServiceState)), this, SLOT(on_service(QLowEnergyService::ServiceState)));
    servs->discoverDetails();
}



void Worker::discoveredAllServices(void)
{
    int c = qctl->services().count();
    emit foundServicesCount(QString("Count: %1").arg(c));

    int i;

    for (i = 0; i < c; i ++)
    {
        if (qctl->services()[i] == QBluetoothUuid(QString(SERVICE_UUID)))
        {
            startDiscoveringServers(qctl->createServiceObject(qctl->services()[i]));
        }
    }

}

void Worker::deviceDisconnected(void)
{
   emit foundServicesCount("Whoops! Disconnected");
}

void Worker::deviceConnected(void)
{
    emit foundConnection("Connected to it!");
    if (!!qctl)
    {
        qctl->discoverServices();
    }
}

void Worker::doDisconnect(void)
{
    if (!!qctl)
    {
        disconnect(qctl, &QLowEnergyController::disconnected, this, &Worker::deviceDisconnected);
        qctl->disconnectFromDevice();
    }
}


void MainWindow::on_ble_error(QLowEnergyController::Error newError)
{

  ui->errorLabel->setText(QString("Error: %1").arg(newError));

}

static QString fname;

void Worker::startIt(void)
{
    fname = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss'.txt'");

    if (qctl)
    {
        connect(qctl, &QLowEnergyController::connected, this, &Worker::deviceConnected);
        connect(qctl, &QLowEnergyController::disconnected, this, &Worker::deviceDisconnected);

        qctl->connectToDevice();
    }
}

Worker::Worker(QObject *parent, const QBluetoothDeviceInfo &device) : QObject(parent), timer0016(this), timer000A(this), timer000E(this)
{
    rxed = nullptr;
    rxbuf = nullptr;
    qctl = QLowEnergyController::createCentral(device);
    connect(qctl, &QLowEnergyController::discoveryFinished, this, &Worker::discoveredAllServices);
}

const float frequency = 125.;

void MainWindow::processBuffer(QBuffer * buf)
{
    QLineSeries *series1 = new QLineSeries;
    series1->setName("X");
    QLineSeries *series2 = new QLineSeries;
    series2->setName("Y");
    QLineSeries *series3 = new QLineSeries;
    series3->setName("Z");

    QString fname1 = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss'.txt'");

    QFile file(fname1);
    if (file.open(QIODevice::WriteOnly | QIODevice::NewOnly | QIODevice::Text))
    {
        if (buf->open(QIODevice::ReadOnly))
        {
            QTextStream out(&file);

            int i;
            for (i = 0; ; i ++)
            {
                QByteArray b = buf->read(8);
                if (b.length() < 8)
                {
                    break; // Finished reading
                }
                if (true)   // true to include the raw hex in the output
                {
                    int j;
                    for(j = 0; j < 8; j ++)
                    {
                        out << QString("%1 ").arg((int)(unsigned char)b[j], 2, 16, QLatin1Char('0'));
                    }
                    out << "  \t\t .. ";
                }
                float xyz;
                xyz = U16toF((uint16_t)(unsigned char)b[0] + 256*(uint16_t)(unsigned char)b[1]);
                out << QString::number(xyz, 'g', 4);
                if (i > 0)  // (for some reason a stray 8-byte message ends up in the first slot)
                    series1->append(((float)i)/frequency, xyz);

                xyz = U16toF((uint16_t)(unsigned char)b[2] + 256*(uint16_t)(unsigned char)b[3]);
                out << "," << QString::number(xyz, 'g', 4);
                if (i > 0)  // (for some reason a stray 8-byte message ends up in the first slot)
                    series2->append(((float)i)/frequency, xyz);

                xyz = U16toF((uint16_t)(unsigned char)b[4] + 256*(uint16_t)(unsigned char)b[5]);
                out << "," << QString::number(xyz, 'g', 4);
                if (i > 0)  // (for some reason a stray 8-byte message ends up in the first slot)
                    series3->append(((float)i)/frequency, xyz);
                out << "," << QString::number((uint16_t)b[6] + 256*(uint16_t)b[7]) << "\r\n";
            }
            buf->close();
        }
        file.close();
    }

    delete buf;

    QChart * const theChart = ui->chartv->chart();

    if (theChart == nullptr)
        return;

    theChart->removeAllSeries();

    theChart->addSeries(series1);
    theChart->addSeries(series2);
    theChart->addSeries(series3);

    theChart->createDefaultAxes();

}


// Just write the raw hex
void MainWindow::processBuffer_simple(QBuffer * buf)
{
    QFile file(fname);
    if (file.open(QIODevice::Append | QIODevice::Text))
    {
        QTextStream out(&file);

        out << QDateTime::currentDateTime().toString("HH:mm:ss.zzz    ");

        if (buf->open(QIODevice::ReadOnly))
        {
            int i;
            for (i = 0; ; i ++)
            {
                QByteArray b = buf->read(16);
                int j;
                for(j = 0; j < b.length(); j ++)
                {
                    out << QString("%1 ").arg((int)(unsigned char)b[j], 2, 16, QLatin1Char('0'));
                }
                out << "\r\n";
                if (b.length() < 16)
                {
                    break; // Finished reading
                }
            }
            buf->close();
        }
        file.close();
    }

    delete buf;
}



void MainWindow::on_connectButton_clicked(){}

// In your local slot, read information about the found devices
void MainWindow::deviceDiscovered(const QBluetoothDeviceInfo &device)
{
    QString newText;
    QTextStream(&newText) << "Found new device:" << device.name() << '(' << device.address().toString() << ')';

    ui->debugLabel->setText(newText);

    if (/*device.address() == QBluetoothAddress("F1:C8:1A:8D:37:8B")
            ||*/
        device.address() == QBluetoothAddress("DC:9E:80:E8:B2:83")
            )
    {
        //if (device.name().left(8) == "nRF Rela" )
        {
            Worker * wrkr = new Worker(this, device);

            connect(wrkr, &Worker::foundService, [=]( const QString &uuidValue ) { ui->uuidLabel->setText(uuidValue); });

            connect(wrkr, &Worker::foundCharacteristic, [=]( const QString &uuidValue ) { ui->label_11->setText(uuidValue); });

            connect(wrkr, &Worker::foundServicesCount, [=]( const QString &uuidValue ) { ui->uuid2label->setText(uuidValue); });

            connect(wrkr, &Worker::foundConnection, [=]( const QString &uuidValue ) { ui->uuidLabel->setText(uuidValue); });

            connect(wrkr, &Worker::foundPacket, [=]( const QString &uuidValue ) { ui->label_10->append(uuidValue/* + QString("\n")*/);});

            connect(wrkr, &Worker::debugLog, [=]( const QString &uuidValue ) { ui->label_10->append(uuidValue/* + QString("\n")*/);});

            QBuffer * buf_1 = new QBuffer();
            wrkr->setBuffer(buf_1);

            connect(wrkr, &Worker::gotTrace, this, &MainWindow::processBuffer_simple);

            connect(ui->connectButton, &QPushButton::clicked, wrkr, &Worker::startIt);
            connect(ui->disconnectButton, &QPushButton::clicked, wrkr, &Worker::doDisconnect);

        }
    }
}

void MainWindow::on_bleStartButton_clicked()
{
    // Create a discovery agent and connect to its signals
    QBluetoothDeviceDiscoveryAgent *discoveryAgent = new QBluetoothDeviceDiscoveryAgent(this);
    connect(discoveryAgent, SIGNAL(deviceDiscovered(QBluetoothDeviceInfo)),
            this, SLOT(deviceDiscovered(QBluetoothDeviceInfo)));

    // Start a discovery
    discoveryAgent->start(QBluetoothDeviceDiscoveryAgent::LowEnergyMethod);

}

