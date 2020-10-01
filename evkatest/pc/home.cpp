#include "home.h"
#include "ui_home.h"
#include "QDebug"

#include <QFileDialog>
#include <QMessageBox>
#include <thread>
#include <chrono>

Home::Home(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::Home)
{
    ui->setupUi(this);

    // ******************** //
    // **** TABLE VIEW **** //
    // ******************** //
    modelTargets = new QStandardItemModel;
    modelTargets->setHorizontalHeaderLabels(QStringList({"PAN", "TILT", "X", "Y", "Z"}));

    modelResults = new QStandardItemModel;
    modelResults->setHorizontalHeaderLabels(QStringList({"Nokta", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10",
                                                         "11", "12", "13", "14", "15", "16", "17", "18", "19", "20"}));

    ui->tableTargets->setModel(modelTargets);
    ui->tableResults->setModel(modelResults);

    connect(ui->tableTargets->selectionModel(), SIGNAL(selectionChanged(const QItemSelection &, const QItemSelection &)), this, SLOT(targetSelectionChanged(const QItemSelection &, const QItemSelection &)));
    connect(ui->tableResults->selectionModel(), SIGNAL(selectionChanged(const QItemSelection &, const QItemSelection &)), this, SLOT(resultSelectionChanged(const QItemSelection &, const QItemSelection &)));

    // ************************ //
    // **** STATUS WIDGETS **** //
    // ************************ //
    TCPStatus = new QLabel(this);
    SPStatus = new QLabel(this);

    setSPStatus(false);
    setTCPStatus(false);

    ui->statusbar->addWidget(SPStatus,1);
    ui->statusbar->addWidget(TCPStatus,0);

    // ********************* //
    // **** SERIAL PORT **** //
    // ********************* //
    sp = new QSerialPort(this);

    connect(sp, SIGNAL(readyRead()), this, SLOT(readData()));
    connect(sp, &QSerialPort::errorOccurred,[=](QSerialPort::SerialPortError err){
        if(err > QSerialPort::NoError)
            QMessageBox::warning(this, "ComauArduino",
                             QString("Bağlantı hatası. \n Serial port error: %1").arg(err),
                             QMessageBox::Ok);
    });

    // ********************************** //
    // **** INITIALIZE RESULTS TABLE **** //
    // ********************************** //
    /*
    modelResults->setRowCount(POINT_NUMBER*2);

    for (size_t i = 0; i < POINT_NUMBER; i++) {
        QStandardItem *item = new QStandardItem(QString::number(i+1));
        ui->tableResults->setSpan(i*2,0,2,1);
        modelResults->setItem(i*2, 0, item);
    }*/

    modelResults->setRowCount(POINT_NUMBER*6);

    for (size_t i = 0; i < POINT_NUMBER; i++) {
        QStandardItem *item = new QStandardItem(QString::number(i+1));
        ui->tableResults->setSpan(i*6,0,6,1);
        modelResults->setItem(i*6, 0, item);
    }

}

Home::~Home()
{
    delete ui;
}

void Home::readData()
{
    QByteArray data = sp->readAll();
    const char *values = data.constData();

    ui->spConsole->appendHtml(QString("<strong><font color=\"red\">Gelen:</font></strong> %1").arg(QString::fromStdString(data.toStdString())));

    //int16_t x=0,y=0,z=0,fi=0,theta=0,psi=0;
    memcpy(&x, &values[0], 4);
    memcpy(&y, &values[4], 4);
    memcpy(&z, &values[8], 4);
    memcpy(&panEnc, &values[12], 4);
    memcpy(&tiltEnc, &values[16], 4);
    memcpy(&wireEnc, &values[20], 4);
    memcpy(&theta1, &values[24], 4);
    memcpy(&theta2, &values[28], 4);

    //qDebug() << wireEnc;

    /*memcpy(&fi, &values[6], 2);
    memcpy(&theta, &values[8], 2);
    memcpy(&psi, &values[10], 2);



    fiBuffer[0] = fi;
    thetaBuffer[0] = theta;

    int32_t sumFi = 0;
    int32_t sumTheta = 0;

    for(size_t i = SAMPLE_NUMBER-1; i > 0; i--)
    {
        fiBuffer[i] = fiBuffer[i-1];
        thetaBuffer[i] = thetaBuffer[i-1];
    }

    for(size_t j = 0; j < SAMPLE_NUMBER; j++)
    {
        sumFi += fiBuffer[j];
        sumTheta += thetaBuffer[j];
    }

    sumFi /= SAMPLE_NUMBER;
    sumTheta /= SAMPLE_NUMBER;*/

    ui->digitX->display(x);
    ui->digitY->display(y);
    ui->digitZ->display(z);
    ui->digitFi->display(panEnc);
    ui->digitTheta->display(tiltEnc);
    ui->digitPsi->display(wireEnc);
    ui->digitSFi->display(theta1*180/3.141592654);
    ui->digitSTheta->display(theta2*180/3.141592654);

/*
    ui->digitFi->display(fi);
    ui->digitTheta->display(theta);
    ui->digitPsi->display(psi);

    ui->digitSFi->display(sumFi);
    ui->digitSTheta->display(sumTheta);

    // insert point into table
    if(x==1)
        insertResult(sumFi,sumTheta);*/



}

void Home::insertResult(int32_t fi,int32_t theta)
{


    QStandardItem *fiItem = new QStandardItem(QString::number(fi));
    QStandardItem *thetaItem = new QStandardItem(QString::number(theta));

    modelResults->setItem(ui->spinBoxPoint->value()*2, ui->spinBoxTestNum->value(), fiItem);
    modelResults->setItem(ui->spinBoxPoint->value()*2+1, ui->spinBoxTestNum->value(), thetaItem);

    if(ui->spinBoxPoint->value() < POINT_NUMBER-1){
        ui->spinBoxPoint->stepUp();
    }else{
        ui->spinBoxPoint->setValue(0);
        ui->spinBoxTestNum->stepUp();
    }
}

void Home::insertResult()
{
    QStandardItem *panItem = new QStandardItem(QString::number(panEnc));
    QStandardItem *tiltItem = new QStandardItem(QString::number(tiltEnc));
    QStandardItem *wireItem = new QStandardItem(QString::number(wireEnc));
    QStandardItem *xItem = new QStandardItem(QString::number(x));
    QStandardItem *yItem = new QStandardItem(QString::number(y));
    QStandardItem *zItem = new QStandardItem(QString::number(z));

    int topRow = ui->spinBoxPoint->value()*6;

    modelResults->setItem(topRow, ui->spinBoxTestNum->value(), panItem);
    modelResults->setItem(topRow+1, ui->spinBoxTestNum->value(), tiltItem);
    modelResults->setItem(topRow+2, ui->spinBoxTestNum->value(), wireItem);
    modelResults->setItem(topRow+3, ui->spinBoxTestNum->value(), xItem);
    modelResults->setItem(topRow+4, ui->spinBoxTestNum->value(), yItem);
    modelResults->setItem(topRow+5, ui->spinBoxTestNum->value(), zItem);

    if(ui->spinBoxPoint->value() < POINT_NUMBER-1){
        ui->spinBoxPoint->stepUp();
    }else{
        ui->spinBoxPoint->setValue(0);
        ui->spinBoxTestNum->stepUp();
    }
}

void Home::writeData(const QByteArray &data)
{
    sp->write(data);
}

void Home::targetSelectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
{
    ui->inputPan->setValue(modelTargets->index(selected.first().top(), 0).data().toDouble());
    ui->inputTilt->setValue(modelTargets->index(selected.first().top(), 1).data().toDouble());
    ui->inputX->setValue(modelTargets->index(selected.first().top(), 2).data().toDouble());
    ui->inputY->setValue(modelTargets->index(selected.first().top(), 3).data().toDouble());
    ui->inputZ->setValue(modelTargets->index(selected.first().top(), 4).data().toDouble());
}

void Home::resultSelectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
{
}

void Home::on_action_import_csv_triggered()
{
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setViewMode(QFileDialog::Detail);

    QStringList fileNames;

    if (dialog.exec() == QDialog::Accepted)
    {

        fileNames = dialog.selectedFiles();

        QFile file(fileNames[0]);
        if (!file.open(QIODevice::ReadOnly)) {
            QMessageBox::warning(this, "ComauArduino",
                                 QString("Dosya okuma hatası: %s").arg(file.errorString()),
                                 QMessageBox::Ok);
            return;
        }

        modelTargets->removeRows(0,modelTargets->rowCount());

        int lineIndex = 0;

        while (!file.atEnd()) {


            QString line = file.readLine();

            QStringList values = line.split(',', QString::SkipEmptyParts);


            for (int j = 0; j < values.size(); j++) {
                QString value = values.at(j);

                QStandardItem *item = new QStandardItem(value);
                modelTargets->setItem(lineIndex, j, item);
            }

            lineIndex++;
        }

    }
}

void Home::on_action_connect_sp_toggled(bool checked)
{
    if(checked)
    {
        QSerialPortInfo portToUse;
        foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
        {
            QString s = QObject::tr("Port:") + info.portName() + "\n"
                        + QObject::tr("Location:") + info.systemLocation() + "\n"
                        + QObject::tr("Description:") + info.description() + "\n"
                        + QObject::tr("Manufacturer:") + info.manufacturer() + "\n"
                        + QObject::tr("Serial number:") + info.serialNumber() + "\n"
                        + QObject::tr("Vendor Identifier:") + (info.hasVendorIdentifier() ? QString::number(info.vendorIdentifier(), 16) : QString()) + "\n"
                        + QObject::tr("Product Identifier:") + (info.hasProductIdentifier() ? QString::number(info.productIdentifier(), 16) : QString()) + "\n"
                        + QObject::tr("Busy:") + (info.isBusy() ? QObject::tr("Yes") : QObject::tr("No")) + "\n";

            if(!info.isBusy() && (info.description().contains("Arduino") || info.manufacturer().contains("Arduino")
                                                                                 | info.manufacturer().contains("1a86")))
                portToUse = info;

            qDebug() << s;
        }

        if(portToUse.isNull() || !portToUse.isValid())
        {
            QMessageBox::warning(this, "ComauArduino",
                                 QString("Port geçerli değil: %1").arg(portToUse.portName()),
                                 QMessageBox::Ok);
            return;
        }


        sp->setPortName(portToUse.portName());
        sp->setBaudRate(QSerialPort::Baud9600);
        sp->setDataBits(QSerialPort::Data8);
        sp->setParity(QSerialPort::NoParity);
        sp->setStopBits(QSerialPort::OneStop);
        sp->setFlowControl(QSerialPort::NoFlowControl);
        if (sp->open(QIODevice::ReadWrite)) {
            qDebug() << "Connected to" << portToUse.description() << "on" << portToUse.portName();
            setSPStatus(true, portToUse.systemLocation());
        } else {
            qCritical() << "Serial Port error:" << sp->errorString();
        }
    }else{
        sp->close();
        setSPStatus(false);
    }
}

void Home::on_action_connect_tcpip_toggled(bool checked)
{
    if(checked)
    {
        TCPServer.listen(QHostAddress::Any, 4200);
        connect(&TCPServer, SIGNAL(newConnection()), this, SLOT(onClientConnected()));
        setTCPStatus(true, TCPServer.serverAddress().toString());
    }else{
        TCPServer.close();
        TCPServer.disconnect();
        TCPSockets.clear();
    }
}

void Home::onClientConnected()
{
    QTcpSocket* clientSocket = TCPServer.nextPendingConnection();
    connect(clientSocket, SIGNAL(readyRead()), this, SLOT(onTCPReadyRead()));
    connect(clientSocket, SIGNAL(stateChanged(QAbstractSocket::SocketState)), this, SLOT(onSocketStateChanged(QAbstractSocket::SocketState)));

    setTCPStatus(true, TCPServer.serverAddress().toString(), clientSocket->localAddress().toString());

    TCPSockets.push_back(clientSocket);

    ui->action_go->setEnabled(true);
/*
    for (QTcpSocket* socket : TCPSockets) {
        socket->write(QByteArray::fromStdString(clientSocket->peerAddress().toString().toStdString() + " connected to server !\n"));
    }*/
}

void Home::onSocketStateChanged(QAbstractSocket::SocketState socketState)
{
    if (socketState == QAbstractSocket::UnconnectedState)
    {
        QTcpSocket* sender = static_cast<QTcpSocket*>(QObject::sender());
        TCPSockets.removeOne(sender);
        sender->disconnect();

    }
}

void Home::onTCPReadyRead()
{
    QTcpSocket* sender = static_cast<QTcpSocket*>(QObject::sender());
    QByteArray datas = sender->readAll();

    ui->tcpConsole->appendHtml(QString("<strong><font color=\"red\">Gelen:</font></strong> %1").arg(QString::fromStdString(datas.toStdString())));

    // delay time
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    ui->action_go->setDisabled(false);

    if(datas.toInt() != 0){
        QMessageBox::warning(this, "ComauArduino",
                             QString("Robot pozisyona gidemedi. \n Gelen kod: %1").arg(datas.toInt()),
                             QMessageBox::Ok);
    }else
    {
        insertResult();

        if(ui->action_serial_movement->isChecked())
        {
            int selectedRow = ui->tableTargets->selectionModel()->selectedRows().first().row();

            if(selectedRow < modelTargets->rowCount()-1)
            {
                ui->tableTargets->selectRow(selectedRow+1);
                on_action_go_triggered();
            }else{
                ui->action_serial_movement->setChecked(false);
                ui->action_go->setDisabled(false);
            }




        }
    }

    /*
    for (QTcpSocket* socket : TCPSockets) {
            socket->write(QByteArray::fromStdString(sender->peerAddress().toString().toStdString() + ": " + datas.toStdString()));
    }*/
}

void Home::setTCPStatus(bool connected, QString hostDesc, QString clientDesc)
{
    if(connected)
        TCPStatus->setText(QString("TCP/IP Bağlantısı: <strong><font color=\"green\">AÇIK</font></strong> (<strong>Host:</strong> %1) | (<strong>Client:</strong> %2)").arg(hostDesc).arg(clientDesc));
    else
        TCPStatus->setText("TCP/IP Bağlantısı: <strong><font color=\"red\">KAPALI</font></strong>");
}

void Home::setSPStatus(bool connected, QString port)
{
    if(connected)
        SPStatus->setText(QString("Seri Port Bağlantısı: <strong><font color=\"green\">AÇIK</font> (%1)</strong>").arg(port));
    else
        SPStatus->setText("Seri Port Bağlantısı: <strong><font color=\"red\">KAPALI</font></strong>");
}

void Home::on_lineEditSP_returnPressed()
{
    writeData(ui->lineEditSP->text().toLatin1());
    ui->spConsole->appendHtml(QString("<strong><font color=\"green\">Giden:</font></strong> %1").arg(QString::fromStdString(ui->lineEditSP->text().toStdString())));
    ui->lineEditSP->clear();
}


void Home::on_lineEditTCP_returnPressed()
{
    QByteArray b("\x08\x03",2);
    int x = -430;
    int y = -571;
    int z = 71;

   QByteArray qba;

   for (QTcpSocket* socket : TCPSockets) {

       qba.setNum(x);
       socket->write(qba);
       socket->write(QByteArray::fromStdString("\n"));

       qba.setNum(y);
       socket->write(qba);
       socket->write(QByteArray::fromStdString("\n"));

       qba.setNum(z);
       socket->write(qba);
       socket->write(QByteArray::fromStdString("\n"));
   }

   ui->tcpConsole->appendHtml(QString("<strong><font color=\"green\">Giden:</font></strong> %1").arg(QString::fromStdString(ui->lineEditTCP->text().toStdString())));
   ui->lineEditTCP->clear();
}

void Home::on_action_serial_movement_toggled(bool checked)
{
    if(checked)
    {
        ui->action_go->setDisabled(true);
        if(modelTargets->rowCount() > 0)
        {
            // no TCP Client
            if(TCPSockets.count() == 0)
            {

                QMessageBox::warning(this, "EvkaTest",
                                     QString("TCP Client bağlı değil, lütfen önce bağlantı sağlayın."),
                                     QMessageBox::Ok);
                ui->action_serial_movement->setChecked(false);
                return;
            }

            if(!ui->tableTargets->selectionModel()->hasSelection())
                ui->tableTargets->selectRow(0);

            on_action_go_triggered();

        }else
        {
            ui->action_go->setDisabled(false);
            QMessageBox::warning(this, "EvkaTest",
                                 QString("Tabloda veri yok."),
                                 QMessageBox::Ok);
            ui->action_serial_movement->setChecked(false);
        }


    }
    else{

    }


}

void Home::on_action_go_triggered()
{
    ui->action_go->setDisabled(true);

    int x = static_cast<int>(ui->inputX->value());
    int y = static_cast<int>(ui->inputY->value());
    int z = static_cast<int>(ui->inputZ->value());

    QByteArray payload;
    QByteArray NL = QByteArray::fromStdString("\n");

    for (QTcpSocket* socket : TCPSockets) {

        payload.setNum(x);
        socket->write(payload.append(NL));

        payload.setNum(y);
        socket->write(payload.append(NL));

        payload.setNum(z);
        socket->write(payload.append(NL));
    }
}

void Home::on_actionGoGo_triggered()
{

    int16_t s1 = static_cast<int16_t>(ui->inputPan->value());
    int16_t s2 = static_cast<int16_t>(ui->inputTilt->value());

    QByteArray qba;
    qba.append(QByteArray::number(s1));
    qba.append(QByteArray::number(s2));

    writeData(qba);
}

void Home::on_action_export_csv_triggered()
{


    QString filename = QFileDialog::getSaveFileName(this);

    qDebug() << filename;

    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "ComauArduino",
                             QString("Dosya yazma hatası: %s").arg(file.errorString()),
                             QMessageBox::Ok);
        return;
    }

    QTextStream out(&file);

    int rowC = modelResults->rowCount();
    int colC = modelResults->columnCount();

    for(int i = 0; i < rowC; i++)
    {
        for(int j = 0; j < colC; j++)
        {
            out << modelResults->index(i,j).data().toString();

            if(j < colC-1)
                out << ",";
            else
                out << "\n";
        }
    }

    file.close();
}
