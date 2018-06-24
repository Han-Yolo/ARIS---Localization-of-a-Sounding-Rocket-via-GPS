#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "DGPSMesGen.h"
#include "PositionAveraging.h"

#include "GPSDataTypes.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

    void printStatus(QString status);
    void printError(QString status);
    void endAll(void);

    void on_Stop_button_clicked();
    void on_Start_button_clicked();

    void on_refPosLoad_button_clicked();

    void on_refPosMeasure_button_clicked();

    void on_refPosSet_button_clicked();

    void on_refPosSave_button_clicked();

private:

    Ui::MainWindow *ui;

    DGPSMesGen *app = nullptr;
    PositionAveraging *posAveraging = nullptr;

    WGS84 referencePos;
    bool referencePosValid = false;

    QColor defColor = QColor(0, 0, 0, 200);
    QColor errColor = QColor(255, 0, 0, 200);

    void closeApplication(void);
    void closePositionAveraging(void);
};

#endif // MAINWINDOW_H
