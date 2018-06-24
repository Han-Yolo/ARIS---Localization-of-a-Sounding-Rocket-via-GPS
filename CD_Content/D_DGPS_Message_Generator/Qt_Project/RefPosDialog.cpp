#include "RefPosDialog.h"
#include "ui_refposdialog.h"

RefPosDialog::RefPosDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RefPosDialog)
{
    ui->setupUi(this);
}

RefPosDialog::~RefPosDialog()
{
    delete ui;
}

void RefPosDialog::on_buttonBox_accepted()
{
    pos.x = ui->doubleSpinBox_x->value();
    pos.y = ui->doubleSpinBox_y->value();
    pos.z = ui->doubleSpinBox_z->value();
}
