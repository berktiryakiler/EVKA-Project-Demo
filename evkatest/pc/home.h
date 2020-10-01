#ifndef HOME_H
#define HOME_H

#include <QMainWindow>
#include <QStandardItemModel>
#include <QTableWidgetItem>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QLabel>
#include <QTcpServer>
#include <QTcpSocket>

#define SAMPLE_NUMBER 2
#define POINT_NUMBER 5

QT_BEGIN_NAMESPACE
namespace Ui { class Home; }
QT_END_NAMESPACE

class Home : public QMainWindow
{
    Q_OBJECT

public:
    Home(QWidget *parent = nullptr);
    ~Home();

private slots:

    void on_action_import_csv_triggered();
    void targetSelectionChanged(const QItemSelection &selected, const QItemSelection &deselected);
    void resultSelectionChanged(const QItemSelection &selected, const QItemSelection &deselected);

    void on_action_connect_sp_toggled(bool checked);

    void on_action_connect_tcpip_toggled(bool checked);
    void readData();


    void on_lineEditSP_returnPressed();

    void onClientConnected();
    void onTCPReadyRead();
    void onSocketStateChanged(QAbstractSocket::SocketState socketState);



    void on_lineEditTCP_returnPressed();

    void on_action_serial_movement_toggled(bool checked);

    void on_action_go_triggered();

    void on_actionGoGo_triggered();

    void on_action_export_csv_triggered();

private:
    Ui::Home *ui;
    QStandardItemModel *modelTargets;
    QStandardItemModel *modelResults;
    QSerialPort *sp;

    int16_t fiBuffer[SAMPLE_NUMBER] = {0};
    int16_t thetaBuffer[SAMPLE_NUMBER] = {0};

    void writeData(const QByteArray &data);

    QLabel *TCPStatus;
    QLabel *SPStatus;
    void setSPStatus(bool connected, QString port = "");
    void setTCPStatus(bool connected, QString hostDesc = " - ", QString clientDesc = " - ");

    void createResultsTable();
    void insertResult(int32_t fi, int32_t theta);
    void insertResult();

    QTcpServer TCPServer;
    QList<QTcpSocket*>  TCPSockets;

    float x=0,y=0,z=0,theta1=0,theta2=0;
    int panEnc=0,tiltEnc=0,wireEnc=0;

    int colCursor = 1;
};
#endif // HOME_H
