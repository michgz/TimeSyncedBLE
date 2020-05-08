#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QBluetoothDeviceInfo>
#include <QBluetoothUuid>
#include <QLowEnergyController>
#include <QBuffer>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    //QLowEnergyController* qctl;

private slots:
    void on_bleStartButton_clicked();

    void on_connectButton_clicked();

protected slots:
    void deviceDiscovered(const QBluetoothDeviceInfo &device);
   // void deviceConnected(void);
    //void discoveredService(const QBluetoothUuid &newService);
  void on_ble_error(QLowEnergyController::Error newError);
  //void discoveredAllServices(void);

      void processBuffer(QBuffer *);
      void processBuffer_simple(QBuffer * buf);

 // void on_service(QLowEnergyService::ServiceState newState);

private:

    Ui::MainWindow *ui;
};

#include <QTimer>


class Worker: public QObject
{
    Q_OBJECT

public:

    Worker(QObject *parent = {}) : QObject(parent) {}

    Worker(QObject *parent, const QBluetoothDeviceInfo &);

protected:
    QLowEnergyController * qctl;
    QLowEnergyService *	servs ;

QLowEnergyCharacteristic * writeChar;
QLowEnergyCharacteristic * readChar;
public:
    void startDiscoveringServers(QLowEnergyService *);
    void discoveredAllServices(void);


    static uint32_t ByteArrayGetU32(QByteArray &arr, int index);


    QTimer timer0016;
    QTimer timer000A;
    QTimer timer000E;

    QByteArray * rxed;
    QBuffer * rxbuf;

    void setBuffer(QBuffer * buf) {rxbuf = buf;}
private:
    void startRead000A(uint32_t timeToRead);

public slots:
    void discoveredService(const QBluetoothUuid &newService);
    void on_service(QLowEnergyService::ServiceState newState);
    void deviceConnected(void);
    void deviceDisconnected(void);
    void startIt(void);
    void characteristicChanged(QLowEnergyCharacteristic,QByteArray);
    void tryWriting(void);



    // Op-code 0016 (get current time)
    void gotReading0016(QLowEnergyCharacteristic x,QByteArray y);
    void timeOut0016(void);
    void finishedReading0016(const QLowEnergyCharacteristic &x,const QByteArray &y);

    // Op-code 000A (get reading)
    void gotReading000A(QLowEnergyCharacteristic x,QByteArray y);
    void timeOut000A(void);
    void finishedReading000A(const QLowEnergyCharacteristic &x,const QByteArray &y);

    // Op-code 000E (read out values)
    void startRead000E(void);
    void gotReading000E(QLowEnergyCharacteristic x,QByteArray y);
    void timeOut000E(void);
    void finishedReading000E(const QLowEnergyCharacteristic &x,const QByteArray &y);

signals:
    void foundService(QString);
    void foundServicesCount(QString);
    void foundCharacteristic(QString);
    void foundConnection(QString);
    void foundPacket(QString);
    void debugLog(QString);
    void gotTrace(QBuffer *);

};














#endif // MAINWINDOW_H
