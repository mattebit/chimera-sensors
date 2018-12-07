#include <QObject>
#include <QtCharts/QLineSeries>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>
#include <QAbstractAxis>
#include <QValueAxis>
#include <QTimer>
#include "graph.h"
#include "serial.h"
#include "QtDebug"

QT_CHARTS_USE_NAMESPACE

static QLineSeries *series;
static QChart *chart;
static QChartView *chartView;

static QValueAxis *axisX;
static QValueAxis *axisY;

static const char *pointer;
static double x = 0;static double y = 0;

static int range = 1000;
static int maxY = 10;
static int y_scaler = 1;
static int serial_counter = 15;

static QVector<QPointF> points;

Mine_Serial s(nullptr);

Graph::Graph(QObject *parent) : QObject(parent){

}

void Graph::graph_init(void){

    series = new QLineSeries();
    chart = new QChart();
    axisX = new QValueAxis;
    axisY = new QValueAxis;

    QPen pen(Qt::red);
    pen.setWidth(1);
    series->setPen(pen);

    //chart->setAnimationOptions(QChart::AllAnimations);
    chart->setTheme(QChart::ChartThemeDark);

    chart->legend()->hide();
    chart->addSeries(series);
    chart->createDefaultAxes();
    chart->setTitle("Fucking Beautifull GRAPH");

    chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    axisX->setRange(0, 100);
    axisY->setRange(0, 1);
    chart->setAxisX(axisX, series);
    chart->setAxisY(axisY, series);

    axisX->setMin(0);
    axisX->setMax(range);

    window.setCentralWidget(chartView);
    window.resize(1800, 600);
    window.show();

    for(int i = 0; i < 600; i++){
        points.append(QPointF(0, 0));
    }
    //points.reserve(int(range));

}

void Graph::add_points(QByteArray buffer){
    pointer = strstr(buffer, "y:");
    if(pointer != nullptr){
        x ++;
        pointer += 2;
        y = atof(pointer);
        if(y < -6000)
        qDebug() << y;
        if(x >= range){
            if(y != 0){
            axisX->setMin(x - range);
            axisX->setMax(x);
            points.append(QPointF(x, y));
            points.removeFirst();
            }
        }
        else{
            points.append(QPointF(x, y));
        }

        series->replace(points);
        if(y >= (maxY * 0.9)){
            y_scaler ++;
            maxY *= y_scaler;
            axisY->setRange(-maxY, maxY);
        }
    }
}


