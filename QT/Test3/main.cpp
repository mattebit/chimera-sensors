#include "mainwindow.h"
#include <QApplication>
#include <QtDebug>
#include <QtQml/QQmlContext>
#include "serial.h"
#include "graph.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    Graph gr (nullptr);

    gr.graph_init();

    Mine_Serial ser(nullptr);

    if(ser.setup_serial()){
        ser.connect();
    }

    return a.exec();
}
