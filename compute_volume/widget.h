#ifndef WIDGET_H
#define WIDGET_H

#include "QPaintEvent"
#include "ui_widget.h"
#include <QTimer>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "vtkRenderer.h"
#include "QVTKOpenGLWidget.h"
#include "QVTKOpenGLNativeWidget.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkInteractorStyleTrackballCamera.h"
#include "vtkSmartPointer.h"
#include "QDateTime.h"
#include <QTimerEvent>
#include <Qvector>
#include <qDebug.h>

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

private slots:
    //void on_compute_Button_clicked();
    double compute_volume(std::vector<pcl::PointXYZRGB> cloud);
    void ComputeDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inplane, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outplane,
        double A, double B, double C, double D, double dis);
signals:
    void starting(std::string PLYFileName);
    void inspection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
    void compution(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);


private:
    Ui::Widget *ui = new Ui::Widget();
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindowInteractor> iren;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_extra;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inner;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    double mass=0.0;
    double volume = 0.0;

};
#endif // WIDGET_H
