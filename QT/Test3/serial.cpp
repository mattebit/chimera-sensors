#include <QObject>
#include <QtDebug>
#include <QAbstractAxis>
#include <QValueAxis>
#include <QEventLoop>
#include <QTimer>

#include "serial.h"
#include "string.h"
#include "graph.h"

static QByteArray serial_data;

static int serial_counter = 0;

static QSerialPort *serial;

Mine_Serial::Mine_Serial(QObject *parent) : QObject(parent){

}

void Mine_Serial::connect(){
    qDebug() << "Connecting";
    QObject::connect(serial, &QSerialPort::readyRead, [=]{
        serial_data = serial->readLine();

    });
    QTimer *tim1 = new QTimer();
    tim1->setInterval(12);
    tim1->start();
    QObject::connect(tim1, &QTimer::timeout, [=]{
        read_serial();
        clear_serial();
    });
    qDebug() << "Connected";
}

void Mine_Serial::read_serial(){
    gr->add_points(serial_data);

    /*
    if(serial_counter >= 0){
        serial_data = serial->readLine(10);
        serial_counter = 0;
    }
    else{
        serial_counter ++;
    }*/
}

bool Mine_Serial::setup_serial(){
    bool result = false;

    qDebug() << "Opening Serial Port";

    serial = new QSerialPort();

    serial->setPortName("/dev/ttyACM1");
    //serial->setBaudRate(QSerialPort::Baud115200);
    serial->setBaudRate(115200);
    serial->setDataBits(QSerialPort::Data8);
    serial->setFlowControl(QSerialPort::NoFlowControl);
    serial->setParity(QSerialPort::NoParity);

    if(serial->open(QSerialPort::ReadWrite)){
        //acm1 opened
        qDebug() << "ACM1 opened";
        result = true;
    }
    else{
        serial->setPortName("/dev/ttyACM2");
        if(serial->open(QSerialPort::ReadWrite)){
            qDebug() << "ACM2 opened";
            result = true;
            //acm2 opened
        }
        else{
            serial->setPortName("/dev/ttyACM3");
            if(serial->open(QSerialPort::ReadWrite)){
                //acm3 opened
                qDebug() << "ACM3 opened";
                result = true;
            }
            else{
                qDebug() << "Failed to Open Serial Port";
                //failed to open port
            }
        }
    }

    return result;
}

void Mine_Serial::clear_serial(){
    serial->clear();
}
