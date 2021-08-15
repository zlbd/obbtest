#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "QCollision.h"
#include <QPainter>
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

protected:
    void paintEvent(QPaintEvent *) override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;

private:
    float degreesToRadians(float degree);
    void testObbCollision();
private:
    Ui::MainWindow *ui;
    QVector2D mousePressPosition;
    void paintHexagon(QPaintEvent *);
};

#endif // MAINWINDOW_H
