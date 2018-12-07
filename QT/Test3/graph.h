#ifndef GRAPH_H
#define GRAPH_H

#include <QObject>
#include "QtSerialPort/QSerialPort"
#include <QtWidgets/QMainWindow>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QAbstractAxis>

class Graph : public QObject{

    Q_OBJECT

public:
    explicit Graph(){}

public:
    Graph(QObject *parent);

    void graph_init(void);

    void add_points(QByteArray serial_data);

private:
    QMainWindow window;
};


#endif // GRAPH_H
