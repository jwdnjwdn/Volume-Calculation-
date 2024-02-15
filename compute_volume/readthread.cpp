#include "readthread.h"
#include "widget.h"
#include "iostream"

using namespace std;
using namespace pcl;

readPointcloud::readPointcloud(QObject *parent) : QThread(parent)
{

}

void readPointcloud::recive(string PLYFileName){
    FileName=PLYFileName;
}

void readPointcloud::run(){

    QVector<pcl::PointXYZRGB> cloud;
    ifstream file;
    file.open(FileName.c_str(),ios::in);
    string line;
    int i=0;
    pcl::PointXYZRGB point;
    while (getline(file, line))
    {
        std::stringstream ss(line);
        if(i>=9){
            ss >> point.x;
            ss >> point.y;
            ss >> point.z;
            ss >> point.r;
            ss >> point.g;
            ss >> point.b;
            cloud.push_back(point);
        }
        if (file.fail())
            break;
        i++;
    }
    qDebug()<<FileName.c_str();
    file.close();
    emit sendPointcloud(cloud);
}

detection::detection(QObject *parent) : QThread(parent)
{

}
void detection::recivepoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in){
    cloud_temp=cloud_in;
}


Eigen::Matrix4f detection::rotate_matrix(Eigen::Vector3f angle_before, Eigen::Vector3f angle_after) {
    angle_before.normalize();
    angle_after.normalize();
    float angle = acos(angle_before.dot(angle_after));//点积，得到两向量的夹角
    Eigen::Vector3f p_rotate = angle_before.cross(angle_after);//叉积，得到的还是向量
    p_rotate.normalize();//该向量的单位向量，即旋转轴的单位向量
    Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
    rotationMatrix(0, 0) = cos(angle) + p_rotate[0] * p_rotate[0] * (1 - cos(angle));
    rotationMatrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle) - p_rotate[2] * sin(angle));//这里跟公式比多了一个括号，但是看实验结果它是对的。
    rotationMatrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));

    rotationMatrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));

    rotationMatrix(2, 0) = -p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));

    return rotationMatrix;
}


void detection::run(){


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outer(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_trans(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud_temp);
    seg.segment(*inliers,*coefficients);

    pcl::transformPointCloud(*cloud_temp, *cloud_trans,
    detection::rotate_matrix(Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]),Eigen::Vector3f(0, 0, -1)));
    qDebug()<<inliers->indices.size();
    if (inliers->indices.size() == 0) {
        PCL_ERROR("cloud not estimate a planar model for the given dataset.");
    }
    qDebug()<<"3";



    cloud_outer->width = cloud_trans->size() - inliers->indices.size();
    cloud_outer->height = 1;
    cloud_outer->resize(cloud_outer->width* cloud_outer->height);


    QVector<pcl::PointXYZRGB> cloud_inn;
    std::vector<int> flag(cloud_trans->size());

    for (int i = 0; i < inliers->indices.size(); i++) {
         flag[inliers->indices[i]]=1;
    }

    for (size_t i = 0, j = 0,k=0; i < cloud_trans->size(); i++) {
        if (flag[i] == 1) {
            cloud_inn.push_back(cloud_trans->points[i]);
            ++k;
        }
        if (flag[i] == 0) {
            cloud_outer->points[j].x = cloud_trans->points[i].x;
            cloud_outer->points[j].y = cloud_trans->points[i].y;
            cloud_outer->points[j].z = cloud_trans->points[i].z;
            cloud_outer->points[j].r = cloud_trans->points[i].r;
            cloud_outer->points[j].g = cloud_trans->points[i].g;
            cloud_outer->points[j].b = cloud_trans->points[i].b;
            ++j;
        }
    }
    qDebug()<<"4";
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_outer);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.01);
    ec.setMinClusterSize(5000);
    ec.setMaxClusterSize(10500000);
    ec.setInputCloud(cloud_outer);
    ec.extract(cluster_indices);

    int j = 0;
    qDebug()<<cluster_indices.size();
    std::vector<std::vector<pcl::PointXYZRGB>> cluster(cluster_indices.size());
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
            cluster[j].push_back(cloud_outer->points[*pit]);
        j++;
    }



    double maxheight = -999999, minheight = 999999;
    int key = -1;
    pcl::PointXYZ point;

    std::vector<bool> record(cluster.size());
    for (size_t i = 0; i < cluster_indices.size(); i++) {
        maxheight = -999999, minheight = 999999;

        for (size_t j = 0; j < cluster[i].size(); j++) {
            if (cluster[i][j].z > maxheight) {
                maxheight = cluster[i][j].z;
            }
            if (cluster[i][j].z < minheight) {
                minheight = cluster[i][j].z;
            }
        }
        double height = maxheight - minheight;
        if (height > 0.1) {
            record[i] = true;
            double minheightpoint = 99999;
            for (int k = 0; k < cluster[i].size(); k++) {
                if (cluster[i][k].z < minheightpoint) {
                    minheightpoint = cluster[i][k].z;
                    key = k;
                }
            }
            point.x = cluster[i][key].x;
            point.y = cluster[i][key].y;
            point.z = cluster[i][key].z;

        }
        else {
            record[i] = false;
        }
    }
    QVector<pcl::PointXYZRGB> cloud_extra;
    QVector<pcl::PointXYZRGB> cloud_target;

    qDebug()<<"4";
    for (int i = 0; i < cluster.size(); i++) {
        if (record[i] == false)
            for (int j = 0; j < cluster[i].size(); j++) {
                cloud_extra.push_back(pcl::PointXYZRGB(cluster[i][j].x, cluster[i][j].y, cluster[i][j].z, cluster[i][j].r,
                    cluster[i][j].g, cluster[i][j].b));
            }
        if (record[i] == true) {
            for (int j = 0; j < cluster[i].size(); j++) {
                cloud_target.push_back(pcl::PointXYZRGB(cluster[i][j].x, cluster[i][j].y, cluster[i][j].z, cluster[i][j].r,
                    cluster[i][j].g, cluster[i][j].b));
            }
        }
    }

    emit finish(cloud_inn,cloud_extra,cloud_target);
}

compute::compute(QObject *parent) : QThread(parent)
{

}
void compute::receivepoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in){
    cloud_temp=cloud_in;
}


void compute::run(){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    double min_height = 999999;
    for (int i = 0; i < cloud_temp->size(); i++) {
        cloud->push_back(pcl::PointXYZ(cloud_temp->points[i].x, cloud_temp->points[i].y, cloud_temp->points[i].z));
        if (cloud_temp->points[i].y < min_height) {
            min_height = cloud_temp->points[i].y;
        }
    }
    qDebug()<<"s1";
    double step = 0.01;
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    int cell_x = int((maxPt.x - minPt.x) / step) + 1;
    int cell_z = int((maxPt.z - minPt.z) / step) + 1;


    int** cell = new int* [cell_x];
    double** height = new double* [cell_x];

    double avg_height = 0.0;
    for (int i = 0; i < cloud->size(); i++) {
        avg_height += abs(cloud->points[i].z- min_height);
    }
    avg_height = avg_height / cloud->size();

    for (int i = 0; i < cell_x; i++)
    {
        cell[i] = new int[cell_z];
        height[i] = new double[cell_z];
        for (int j = 0; j < cell_z; j++)
        {
            cell[i][j] = 0;
            height[i][j] = 0.0;
        }
    }

    for (int i = 0; i < cloud->size(); i++) {
        int x = (cloud->points[i].x - minPt.x) / step;
        int z = (cloud->points[i].z - minPt.z) / step;
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
    volume=round(volume*1000)/1000;

    emit finish(volume);

}



