#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

ros::Publisher pubLaserCloudFullRes;


struct RSPointXYZIRT {
  PCL_ADD_POINT4D
  uint8_t ring;
  uint8_t intensity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RSPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (uint8_t, ring, ring) (uint8_t, intensity, intensity)
)

void intensityAssociateRGB(pcl::PointXYZI const *const pi, pcl::PointXYZRGB  *const po)
{
    po->x = pi->x;
    po->y = pi->y;
    po->z = pi->z;
    if (pi->intensity < 30) {
        int green = (pi->intensity * 255 / 30);
        po->r = 0;
        po->g = green & 0xff;
        po->b = 0xff;
    } else if (pi->intensity < 90) {
        int red = (((90 - pi->intensity) * 255) / 60);
        po->r = 0;
        po->g = 0xff;
        po->b = red & 0xff;
    } else if (pi->intensity < 150) {
        int blue = ((pi->intensity - 90) * 255 / 60);
        po->r = blue & 0xff;
        po->g = 0xff;
        po->b = 0;
    } else {
        int green = (((255 - pi->intensity) * 255) / (255 - 150));
        po->r = 0xff;
        po->g = green & 0xff;
        po->b = 0;
    }
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    pcl::PointCloud<RSPointXYZIRT> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZRGB>());
    int cloudSize = laserCloudIn.points.size();
    for (int i = 0; i < cloudSize; i++)
    {
        pcl::PointXYZI temp_point;
        pcl::PointXYZRGB temp_colorpoint;
        temp_point.x = laserCloudIn.points[i].x;
        temp_point.y = laserCloudIn.points[i].y;
        temp_point.z = laserCloudIn.points[i].z;
        temp_point.intensity = laserCloudIn.points[i].intensity;
        intensityAssociateRGB(&temp_point, &temp_colorpoint);
        laserCloudFullRes->push_back(temp_colorpoint);
    }

    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = laserCloudMsg->header.stamp;
    laserCloudFullRes3.header.frame_id = "/rslidar";
    pubLaserCloudFullRes.publish(laserCloudFullRes3);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "repub");
    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 100, laserCloudHandler);
    pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("color_cloud",100);

    ros::spin();

    return 0;
}