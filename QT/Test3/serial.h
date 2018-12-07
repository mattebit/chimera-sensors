#ifndef SERIAL_H
#define SERIAL_H

#include <QObject>
#include "QtSerialPort/QSerialPort"
#include <QtWidgets/QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QAbstractAxis>

class Graph;

class Mine_Serial : public QObject{
    Q_OBJECT

public:
    explicit Mine_Serial(){}

public:
    Mine_Serial(QObject *parent);

    Graph *gr = nullptr;

    void connect();

    bool setup_serial();

    void read_serial();

    void clear_serial();
};

#endif // SERIAL_H
