#include "utility.h"
#include "lio_sam/cloud_info.h"

using namespace std;
typedef pcl::PointXYZI PointType;

/*
template <typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}
*/
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY; // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

typedef PointXYZIRPYT PointTypePose;

class dense 
{
public:
    std::mutex path_mtx;
    std::mutex cloud_mtx;
    ros::NodeHandle nh;
    ros::Subscriber subcloud;
    ros::Subscriber subOdometry;
    std::deque<nav_msgs::Odometry> Odometry_que;
    pcl::PointCloud<PointType>::Ptr extractedCloud;
    std::deque< lio_sam::cloud_infoConstPtr> cloudque;
    pcl::PointCloud<pcl::PointXYZI> laserCloudFromMap;
    PointTypePose thisPose6D;
    bool is;

    dense():is(true)
    {
        std::cout <<"5" << std::endl;
        subcloud = nh.subscribe<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 5, &dense::cloudHandler, this, ros::TransportHints().tcpNoDelay());
       std::cout <<"6" << std::endl;
        subOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5, &dense::pathHandler, this, ros::TransportHints().tcpNoDelay());
        allocateMemory();
    }

    void allocateMemory()
    {
        
        //laserCloudFromMap.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());
        
    }

    void pathHandler(const nav_msgs::Odometry::ConstPtr &odomMsg)
    {
        std::cout <<"4" << std::endl;
        std::lock_guard<std::mutex> lock(path_mtx);

        Odometry_que.push_back(*odomMsg);
    }

    // lidar odom
    void cloudHandler(const lio_sam::cloud_infoConstPtr &msgIn)
    {
       std::cout <<" here" << std::endl;
        // new cloud header
        //std::lock_guard<std::mutex> lock(cloud_mtx);
        //pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud);
        cloudque.push_back(msgIn);
        if(Odometry_que.empty())
        {
            return;
        }
        cout<<"1" << endl;
        while(Odometry_que.front().header.stamp.toSec() <ROS_TIME(cloudque.front()))
        {
            Odometry_que.pop_front();
        }
        cout<<"2" << endl;
        while(Odometry_que.front().header.stamp.toSec()>ROS_TIME(cloudque.front()))
        {
            cloudque.pop_front();
        }
        cout<<"3" << endl;
        if (Odometry_que.front().header.stamp.toSec() ==ROS_TIME(cloudque.front()))
        {
            pcl::fromROSMsg(cloudque.front()->cloud_deskewed, *extractedCloud);
            const nav_msgs::Odometry odom = Odometry_que.front();
            cout<<"5" << endl;
            thisPose6D.x = odom.pose.pose.position.x;
            thisPose6D.y = odom.pose.pose.position.y;
            thisPose6D.z = odom.pose.pose.position.z;
            double Roll, Pitch, Yaw;
            tf::Quaternion orientation;
            tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
            tf::Matrix3x3(orientation).getRPY(Roll, Pitch, Yaw);
            thisPose6D.roll = Roll;
            thisPose6D.pitch = Pitch;
            thisPose6D.yaw = Yaw;
            //cout<<"6" << endl;
             //*laserCloudFromMap+=
             pcl::PointCloud<pcl::PointXYZI> laserCloudCornerTemp= *transformPointCloud(extractedCloud, &thisPose6D);
            
            laserCloudFromMap += laserCloudCornerTemp;
           
            cout << "8" << endl;
        }

        //cout<<"4" << endl;
        Odometry_que.pop_front();
        cloudque.pop_front();
    }
    void visualizeGlobalMapThread()
    {
        std::cout <<"7" << std::endl;
        ros::Rate rate(0.5);
        while (ros::ok())
        {
            rate.sleep();
        }

        cout << "Saving map to pcd files ..." << endl;
        // create directory and remove old files;
        //savePCDDirectory = std::getenv("HOME") + savePCDDirectory;
        //int unused = system((std::string("exec rm -r ") + savePCDDirectory).c_str());
        //unused = system((std::string("mkdir ") + savePCDDirectory).c_str());

        pcl::io::savePLYFileASCII("dense_map.ply", laserCloudFromMap);
        cout << "complete"<<endl;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        //PointType *pointFrom;
        std::cout <<"9" <<endl;
        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        std::cout <<"10" <<endl;
#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
            cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
            cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
            cloudOut->points[i].intensity = pointFrom.intensity;
            //std::cout <<"11" <<endl;
            /*
            PointType thisPose3D;
            thisPose3D.x = cloudOut->points[i].x;
            thisPose3D.y = cloudOut->points[i].y;
            thisPose3D.z = cloudOut->points[i].z;
            std::cout <<"12" <<endl;
            thisPose3D.intensity = cloudOut->points[i].intensity; // this can be used as index
            std::cout <<"13" <<endl;
            laserCloudFromMap->push_back(thisPose3D);
            std::cout <<"14" <<endl;
            */
        }

        return cloudOut;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_sam");

    dense dens;

    ROS_INFO("\033[1;32m----> dense_map Started.\033[0m");

    std::thread visualizeMapThread(&dense::visualizeGlobalMapThread, &dens);
    ros::spin();
    visualizeMapThread.join();

    return 0;
}
