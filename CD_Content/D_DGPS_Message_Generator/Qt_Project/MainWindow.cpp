#include <QFileDialog>
#include <QInputDialog>

#include "MainWindow.h"
#include "ui_mainwindow.h"
#include "RefPosDialog.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->Status_output->setTextColor(defColor);

    ui->Port_input->setText("/dev/ttyACM0");
    ui->Baud_input->setText("9600");

    ui->Port_output->setText("/dev/ttyACM1");
    ui->Baud_output->setText("9600");
}

MainWindow::~MainWindow()
{
    delete ui;
}


/* ----- DGPS message generator slots ----- */

void MainWindow::on_Start_button_clicked()
{
    if(app == nullptr)
    {
        if(posAveraging != nullptr)
        {
            printError("position averaging running");
            return;
        }

        if(!referencePosValid)
        {
            printError("no reference position set");
            return;
        }

        app = new DGPSMesGen(referencePos, ui->Port_input->text(), ui->Baud_input->text().toUInt(),
                             ui->Port_output->text(), ui->Baud_output->text().toUInt());
        connect(app, SIGNAL(finished()), app, SLOT(deleteLater()));
        connect(app, SIGNAL(printStatus(QString)), this, SLOT(printStatus(QString)));
        connect(app, SIGNAL(printError(QString)), this, SLOT(printError(QString)));
        connect(app, SIGNAL(endAll()), this, SLOT(endAll()));
        app->start();
    }
}

void MainWindow::on_Stop_button_clicked()
{
    closeApplication();
}


/* ----- reference position slots ----- */

void MainWindow::on_refPosSet_button_clicked()
{
    RefPosDialog *dialog = new RefPosDialog(this);
    dialog->exec();
    WGS84 pos = dialog->getPos();
    delete dialog;

    if((pos.x == 0.0) || (pos.y == 0.0) || (pos.z == 0.0)) return;

    referencePos = pos;
    referencePosValid = true;

    printStatus("reference position set to WGS84 coordinates:");
    printStatus("    x: " + QString::number(pos.x, 'f', 2) + "m");
    printStatus("    y: " + QString::number(pos.y, 'f', 2) + "m");
    printStatus("    z: " + QString::number(pos.z, 'f', 2) + "m");
}

void MainWindow::on_refPosMeasure_button_clicked()
{
    if(posAveraging == nullptr)
    {
        if(app != nullptr)
        {
            printError("DGPS message generation running");
            return;
        }

        posAveraging = new PositionAveraging(ui->Port_input->text(), ui->Baud_input->text().toUInt());
        connect(posAveraging, SIGNAL(finished()), posAveraging, SLOT(deleteLater()));
        connect(posAveraging, SIGNAL(printStatus(QString)), this, SLOT(printStatus(QString)));
        connect(posAveraging, SIGNAL(printError(QString)), this, SLOT(printError(QString)));
        connect(posAveraging, SIGNAL(endAll()), this, SLOT(endAll()));
        posAveraging->start();

        ui->refPosMeasure_button->setText("Stop Measurement");
    }
    else
    {
        WGS84 avgPos = posAveraging->getAvgPosition();
        closePositionAveraging();

        referencePos = avgPos;
        referencePosValid = true;
        ui->refPosMeasure_button->setText("Measure");

        printStatus("reference position set to WGS84 coordinates:");
        printStatus("    x: " + QString::number(avgPos.x, 'f', 2) + "m");
        printStatus("    y: " + QString::number(avgPos.y, 'f', 2) + "m");
        printStatus("    z: " + QString::number(avgPos.z, 'f', 2) + "m");
    }
}

void MainWindow::on_refPosSave_button_clicked()
{
    if(!referencePosValid)
    {
        printError("no reference position set");
        return;
    }

    QString path = "";
    if(QDir("Reference").exists()) path = "Reference";
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save Reference Position"), path, tr("*.wgs84"));
    if(fileName.isEmpty()) return;

    if(!fileName.endsWith(".wgs84")) fileName += ".wgs84";

    QFile file(fileName);
    if(file.open(QIODevice::WriteOnly))
    {
        QDataStream out(&file);
        out << referencePos;
        printStatus("reference position saved to file");
    }
    else
    {
        printError("could not save reference position");
    }
    file.close();
}

void MainWindow::on_refPosLoad_button_clicked()
{
    if(app != nullptr)
    {
        printError("DGPS message generation running");
        return;
    }

    if(posAveraging != nullptr)
    {
        printError("position averaging running");
        return;
    }

    QString path = "";
    if(QDir("Reference").exists()) path = "Reference";
    QString fileName = QFileDialog::getOpenFileName(this, tr("Load Reference Position"), path, tr("*.wgs84"));
    if(fileName.isEmpty()) return;

    QFile file(fileName);
    if(file.open(QIODevice::ReadOnly))
    {
        QDataStream in(&file);
        WGS84 refPos;
        in >> refPos;
        referencePos = refPos;
        referencePosValid = true;

        printStatus("reference position set to WGS84 coordinates:");
        printStatus("    x: " + QString::number(refPos.x, 'f', 2) + "m");
        printStatus("    y: " + QString::number(refPos.y, 'f', 2) + "m");
        printStatus("    z: " + QString::number(refPos.z, 'f', 2) + "m");
    }
    else
    {
        printError("could not load reference position");
    }
}


/* ----- close functions ----- */

void MainWindow::closeApplication(void)
{
    if(app != nullptr && app->isRunning())
    {
        app->requestInterruption();
        app->wait();
        delete app;
        app = nullptr;
    }
}

void MainWindow::closePositionAveraging(void)
{
    if(posAveraging != nullptr && posAveraging->isRunning())
    {
        posAveraging->requestInterruption();
        posAveraging->wait();
        delete posAveraging;
        posAveraging = nullptr;
    }
}


/* ----- print slots ----- */

void MainWindow::printStatus(QString status)
{
    ui->Status_output->append(status);
}

void MainWindow::printError(QString status)
{
    ui->Status_output->setTextColor(errColor);
    ui->Status_output->append(status);
    ui->Status_output->setTextColor(defColor);
}

void MainWindow::endAll(void)
{
    closeApplication();
    closePositionAveraging();
}
