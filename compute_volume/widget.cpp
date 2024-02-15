#include "widget.h"
#include "ui_widget.h"
#include "QFileDialog"
#include <stdio.h>

#include <iostream>
#include <QVTKWidget.h>
#include "qdebug.h"

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/crop_box.h>
#include<pcl/filters/radius_outlier_removal.h>

#include <ctime>
#include <cstdlib>
#include <windows.h>
#include <cmath>
#include <iomanip>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common.h>
#include <pcl/octree/octree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <vector>


#include <vtkAutoInit.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkDelaunay2D.h>
#include <vtkMath.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkDelaunay3D.h>
#include <vtkMassProperties.h>
#include <vtkTriangleFilter.h>
#include <readthread.h>


using namespace pcl;
using namespace std;


VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingFreeType)


Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);

    readPointcloud* read=new readPointcloud;
    detection* detect=new detection;
    compute* com=new compute;

    connect(this,&Widget::starting,read,&readPointcloud::recive);
    connect(ui->device_Button,&QPushButton::clicked,this,[=]()
    {

        QString filePath = QFileDialog::getOpenFileName(this, tr("select file"), QDir::homePath(), tr("all file(*.*)"));
        char*  PLYFileName;
        QByteArray ba = filePath.toLatin1();
        PLYFileName=ba.data();
        qDebug()<<PLYFileName;
        emit starting(PLYFileName);
        read->start();
        ui->device_Button->setStyleSheet("background-color:rgb(0,170,0)");
    });

    qRegisterMetaType<QVector<pcl::PointXYZRGB>>("QVector<pcl::PointXYZRGB>");
    connect(this,&Widget::inspection, detect, &detection::recivepoint);

    connect(read,&readPointcloud::sendPointcloud,this,[=](QVector<pcl::PointXYZRGB> cloud_out)
    {
        cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        for(int i=0;i<cloud_out.size();i++){
            cloud->push_back(cloud_out[i]);
        }

        viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb3(cloud);
        viewer->addPointCloud(cloud,rgb3, "cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

        ui->meshwidget->SetRenderWindow(viewer->getRenderWindow());
        viewer->setupInteractor(ui->meshwidget->GetInteractor(), ui->meshwidget->GetRenderWindow());


        viewer->updatePointCloud(cloud,rgb3, "cloud");
        viewer->setBackgroundColor(.0, .0, .0);
        viewer->addCoordinateSystem(0.5);
        viewer->resetCamera();
        ui->meshwidget->update();


    });


    connect(ui->detection_Button,&QPushButton::clicked,this,[=]()
    {
        emit inspection(cloud);
        detect->start();
        ui->detection_Button->setStyleSheet("background-color:rgb(0,170,0)");
    });
    qRegisterMetaType<QVector<pcl::PointXYZRGB>>("QVector<pcl::PointXYZRGB>");
    connect(detect,&detection::finish,this,[=](QVector<pcl::PointXYZRGB> cloud_inn,
            QVector<pcl::PointXYZRGB> cloud_ex,QVector<pcl::PointXYZRGB> cloud_tar)
    {

        cloud_extra.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_inner.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_target.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(int i=0;i<cloud_inn.size();i++){
            cloud_inner->push_back(cloud_inn[i]);
        }
        for(int i=0;i<cloud_ex.size();i++){
            cloud_extra->push_back(cloud_ex[i]);
        }
        for(int i=0;i<cloud_tar.size();i++){
            cloud_target->push_back(cloud_tar[i]);

        }
        viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> single_color(cloud_target,"z");
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(cloud_extra);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(cloud_inner);

        viewer->addPointCloud(cloud_extra,rgb1, "cloud_extra");
        viewer->addPointCloud(cloud_inner,rgb2, "cloud_inner");
        viewer->addPointCloud(cloud_target, single_color, "cloud_target");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_extra");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_inner");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_target");
        viewer->setBackgroundColor(.0, .0, .0);
        viewer->addCoordinateSystem(0.5);

        ui->meshwidget->SetRenderWindow(viewer->getRenderWindow());
        viewer->setupInteractor(ui->meshwidget->GetInteractor(), ui->meshwidget->GetRenderWindow());

        viewer->updatePointCloud(cloud_extra,rgb1, "cloud_extra");
        viewer->updatePointCloud(cloud_inner,rgb2, "cloud_inner");
        viewer->updatePointCloud(cloud_target, single_color, "cloud_target");
        viewer->resetCamera();
        ui->meshwidget->update();

    });

    connect(this,&Widget::compution,com,&compute::receivepoint);
    connect(ui->compute_Button,&QPushButton::clicked,this,[=]()
    {
        emit compution(cloud_target);
        com->start();
        ui->compute_Button->setStyleSheet("background-color:rgb(0,170,0)");
    });
    connect(com,&compute::finish,this,[=](double volume)
    {

        QString str_volume = QString::number(volume);
        ui->show_volume_label->setText(str_volume);
        ui->show_volume_label->show();

        mass=volume*1.3;
        QString str_mass = QString::number(mass);
        ui->show_mass_label->setText(str_mass);
        ui->show_mass_label->show();

    });





}

Widget::~Widget()
{
    delete ui;
}

//compute the distance
void Widget::ComputeDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inplane, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outplane,
    double A, double B, double C, double D, double dis) {
    double numerator = 0, denom = 0, dist = 0;
    for (int i = 0; i < cloud_in->size(); i++) {
        /*cout << cloud_in->points[i].x<<" " << cloud_in->points[i].y<< " " << cloud_in->points[i].z << endl;*/
        numerator = (A * cloud_in->points[i].x + B * cloud_in->points[i].y + C * cloud_in->points[i].z) + D;
        denom = sqrt(A * A + B * B + C * C);
        dist = abs(numerator / denom);
        /*cout << dist << endl;*/
        if (dist > dis) { //0.2 best
            cloud_outplane->push_back(cloud_in->points[i]);
        }
        else {
            cloud_inplane->push_back(cloud_in->points[i]);
        }
        numerator = 0;
    }

}




double Widget::compute_volume(vector<pcl::PointXYZRGB> cloud) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
    double min_height = 999999;
    for (int i = 0; i < cloud.size(); i++) {
        cloud_temp->push_back(pcl::PointXYZ(cloud[i].x, cloud[i].y, cloud[i].z));
        if (cloud[i].y < min_height) {
            min_height = cloud[i].y;
        }
    }

    double step = 0.01;
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud_temp, minPt, maxPt);
    int cell_x = int((maxPt.x - minPt.x) / step) + 1;
    int cell_z = int((maxPt.z - minPt.z) / step) + 1;


    int** cell = new int* [cell_x];//先创建指针的指针，指向一个指针数组
    double** height = new double* [cell_x];

    double avg_height = 0.0;
    for (int i = 0; i < cloud_temp->size(); i++) {
        avg_height += abs(cloud_temp->points[i].z- min_height);
    }
    avg_height = avg_height / cloud_temp->size();

    for (int i = 0; i < cell_x; i++)
    {
        cell[i] = new int[cell_z];
        height[i] = new double[cell_z];
        for (int j = 0; j < cell_z; j++)
        {
            cell[i][j] = 0;//对二维数组中的元素赋值
            height[i][j] = 0.0;
        }
    }
    //double distance = 0.0;
    for (int i = 0; i < cloud_temp->size(); i++) {
        int x = (cloud_temp->points[i].x - minPt.x) / step;
        int z = (cloud_temp->points[i].z - minPt.z) / step;
        cell[x][z]++;

        height[x][z]= height[x][z]+ min_height;
    }

    int max_sum = -1;
    for (int i = 0; i < cell_x; i++)
    {
        for (int j = 0; j < cell_z; j++)
        {
            if (cell[i][j] > max_sum) {
                max_sum = cell[i][j];
            }
        }
    }
    int cell_sum = 0;
    int remove_sum = 0;
    double volume1 = 0.0,volume2=0.0;
    for (int i = 0; i < cell_x; i++)
    {
        for (int j = 0; j < cell_z; j++)
        {
            if (cell[i][j] > 0 && cell[i][j] < int(max_sum * 0.03)) {
                remove_sum++;
                volume1 += (height[i][j] / cell[i][j]) * step * step;
            }
            if (cell[i][j] > 0) {
                cell_sum++;
                volume2 += (height[i][j] / cell[i][j]) * step * step;

            }
        }
    }
    double volume=abs(volume2 - volume1);

    /*volume = (cell_sum ) * avg_height;*/
    return volume;
}




