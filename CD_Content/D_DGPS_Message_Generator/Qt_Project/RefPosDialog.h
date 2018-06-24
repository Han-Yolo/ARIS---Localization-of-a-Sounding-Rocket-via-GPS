#ifndef REFPOSDIALOG_H
#define REFPOSDIALOG_H

#include <QDialog>

#include "GPSDataTypes.h"


namespace Ui {
class RefPosDialog;
}

class RefPosDialog : public QDialog
{
    Q_OBJECT

public:

    explicit RefPosDialog(QWidget *parent = 0);
    ~RefPosDialog();

    WGS84 getPos(void) { return pos; }

private slots:

    void on_buttonBox_accepted();

private:

    Ui::RefPosDialog *ui;

    WGS84 pos;

};

#endif // REFPOSDIALOG_H
