#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMouseEvent>
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *e)
{
    paintHexagon(e);
}

void MainWindow::paintHexagon(QPaintEvent */*e*/)
{
   /*
    * 绘制多边形 本例中直接写的六边形
    * int   count  多边形的边数
    * float radius 多边形半径
    */
   int count = 6;
   int radius = 100;

   /*
    * 注意：
    * 在使用QPainter绘图之前，必须先指定绘图设备。
    * 下方指定的是在本窗口中绘制。
    * 如果没有指定绘图设备，则在之后的绘图中将无效。
    * 指定绘图设备的方法还可以使用begin() end()组合。
    * QPainter在同一时刻，只能在一个设备上绘制，如果将其切换至另一个设备，将是无效的。
    */
   QPainter painter(this);

   //设置为抗锯齿,并且设置画笔颜色为浅灰
   painter.setRenderHint(QPainter::Antialiasing);
   painter.setPen(Qt::lightGray);

   /*
    * 绘图设备的坐标原点(0,0)在左上角，水平向右增长，垂直向下增长。
    * 首先先平移坐标原点，让原点在绘图设备的中心
    */
   painter.translate(width()/2., height()/2.);

   //开始绘制多边形，并为每个区块上色
   for (int i = 0; i < count; ++i)
   {
       //设中心点到边的垂线与半径的夹角为degree=(360/count)/2即：
       float degree = 180.f/count;

       //先将坐标进行旋转，然后再绘制
       painter.rotate(2 * degree);

       //设边长的一半为wid,则wid = radius*sin(degree)
       //原点到边的距离为hei，则hei = radius*cos(degree)
       //在程序中三角函数都是以弧度为基准，所以要先将角度转化成弧度。
       //头文件要包含QtMath
       float radian = degreesToRadians(degree);
       float wid    = radius * sin(radian);
       float hei    = radius * cos(radian);

       //绘制该三角区块
       QPainterPath path;
       path.lineTo(-wid, -hei);
       path.lineTo(wid, -hei);
       path.lineTo(0, 0);
       painter.drawPath(path);

       //随机生成一个颜色，作为该区块的颜色，并上色
       QColor color(qrand()%255, qrand()%255, qrand()%255, 150);
       painter.fillPath(path, color);
   }
}

void MainWindow::mousePressEvent(QMouseEvent *e)
{
    // Save mouse press position
    mousePressPosition = QVector2D(e->localPos());
}

void MainWindow::mouseReleaseEvent(QMouseEvent *e)
{
    // Mouse release position - mouse press position
    QVector2D diff = QVector2D(e->localPos()) - mousePressPosition;
    Q_UNUSED(diff);

    testObbCollision();
}

float MainWindow::degreesToRadians(float degree)
{
    return degree*M_PI/180;
}

void MainWindow::testObbCollision()
{
    static int s_cnt = 0;
    s_cnt++;
    //----------------------------------
    static Collision::OBB target1 = {
        QVector2D(2.f, 2.f),
        QSizeF(2*sqrt(2), 2*sqrt(2)),
        45*M_PI/180
    };
    static Collision::OBB target2;
    target2 = target1;
    target2.pivot = target1.pivot + QVector2D(1.f * s_cnt, 0.f);

    bool bOBBOverlap = Collision::isOBBOverlap(target1, target2);
    printf("bOBBOverlap = %d\n", bOBBOverlap);fflush(stdout);
}


