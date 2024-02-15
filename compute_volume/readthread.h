#ifndef READTHREAD_H
#define READTHREAD_H

#include <QObject>
#include <QThread>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <QVector>
#include <QMetaType>
#include "iostream"
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
#include <pcl/common/common.h>
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
#include <vector>

using namespace std;
class readPointcloud : public QThread
{
    Q_OBJECT
public:
    explicit readPointcloud(QObject *parent = nullptr);
    void recive(string PLYFileName);

private:
    void run() override;
    string FileName;

private slots:



signals:
     void sendPointcloud(QVector<pcl::PointXYZRGB> cloud);
};

class detection : public QThread
{
    Q_OBJECT
public:
    explicit detection(QObject *parent = nullptr);
    void recivepoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
    Eigen::Matrix4f rotate_matrix(Eigen::Vector3f angle_before, Eigen::Vector3f angle_after);

private:
    void run() override;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp;

private slots:


signals:
     void finish(QVector<pcl::PointXYZRGB> cloud_inn,QVector<pcl::PointXYZRGB> cloud_ex,QVector<pcl::PointXYZRGB> cloud_tar);
};



class compute : public QThread
{
    Q_OBJECT
public:
    explicit compute(QObject *parent = nullptr);
    void receivepoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);

private:
    void run() override;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp;

private slots:



signals:
     void finish(double volume);
};

#endif // READTHREAD_H
