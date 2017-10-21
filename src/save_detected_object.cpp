#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include <vector>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//globle variables
int counts = 0; //counts for objects
int temp_count;
float fx = 0, fy = 0, fz = 0;
vector<float> co_x; //coordinate x
vector<float> co_y; //coordinate y
vector<float> co_z; //coordinate z
vector<int> pixel_x;
vector<int> pixel_y;
visualization_msgs::Marker points;
image_geometry::PinholeCameraModel model1;

class object
{
  public:
    int ID;
    tf::Transform transform;
    object(int nID, tf::Transform ntransform)
    {
        ID = nID;
        transform = ntransform;
    }
};

vector<object> objects;

void yolo_callback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
{
    pixel_x.clear(); //column
    pixel_y.clear(); //row
    temp_count = 0;
    int size = msg->boundingBoxes.size();
    for (int i = 0; i < size; i++)
    {
        if (msg->boundingBoxes[i].Class.compare("chair") == 0)
        {
            pixel_x.push_back((msg->boundingBoxes[i].xmin + msg->boundingBoxes[i].xmax) / 2);
            pixel_y.push_back((msg->boundingBoxes[i].ymin + msg->boundingBoxes[i].ymax) / 2);
            temp_count = temp_count + 1;
        }
    }
    //ROS_INFO ("\t(%d, %d)\n", pixel_x[0], pixel_y[0]);  //test use
}

// point cloud option // slow

// void pcd_callback(const PointCloud::ConstPtr &msg)
// {
//     co_x.resize(temp_count);
//     co_y.resize(temp_count);
//     co_z.resize(temp_count);
//     for (int i = 0; i < temp_count; i++)
//     {
//         int row = pixel_y[i];
//         int cloumn = pixel_x[i];
//         int index = row * 640 + cloumn;
//         co_x[i] = (msg->points[index].x);
//         co_y[i] = (msg->points[index].y);
//         co_z[i] = (msg->points[index].z);
//     }
//     //printf ("Cloud: width = %d, height = %d", msg->width, msg->height);
//     //BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
//     //printf ("\t(%f, %f, %f)\n", pt.x, msg->points(100,200).y, msg->points(100,200).z);
//     //int row = 150;
//     //int column = 200;
//     //int index = row*640+column;
//     //printf ("\t(%f, %f, %f)\n", msg->points[index].x, msg->points[index].y, msg->points[index].z);
//     ROS_INFO("\t(%f, %f, %f)\n", co_x[0], co_y[0], co_z[0]); //test use
// }

// depth option // fast

void info_callback(const sensor_msgs::CameraInfoConstPtr &msg)
{
    model1.fromCameraInfo(msg);
}

void depthcb(const sensor_msgs::ImageConstPtr &msg)
{
    co_x.resize(temp_count);
    co_y.resize(temp_count);
    co_z.resize(temp_count);
    cv_bridge::CvImageConstPtr depth_img_cv;
    depth_img_cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    for (int i = 0; i < temp_count; i++)
    {
        cv::Point2d pixel_point(pixel_x[i], pixel_y[i]);
        float depth = depth_img_cv->image.at<short int>(pixel_point);
        cv::Point3d xyz = model1.projectPixelTo3dRay(pixel_point);
        cv::Point3d coordinate = xyz * depth;
        if (depth > 0.01)
        {
            co_x[i] = coordinate.x/1000;
            co_y[i] = coordinate.y/1000;
            co_z[i] = coordinate.z/1000;
        }
    }

    //ROS_INFO("\t(%f, %f, %f)\n", co_x[0], co_y[0], co_z[0]); //test use
}

//struct position
//{
//    float x;
//    float y;
//};

tf::Transform tf_cal(float x, float y, float z)
{
    //calculate transform from /map to /object
    tf::TransformListener listener;
    tf::StampedTransform stampedtransform;
    //ros::Rate rate(5.0);
    //rate.sleep();
    listener.waitForTransform("/map", "/camera_rgb_optical_frame", ros::Time::now(), ros::Duration(3.0));
    listener.lookupTransform("/map", "/camera_rgb_optical_frame", ros::Time(0), stampedtransform);
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf::Transform ntransform;
    ntransform = stampedtransform * transform;

    //float object_x = transform.getOrigin().x();
    //float object_y = transform.getOrigin().y(); //position of object in 2d map //to be saved
    //float result = {object_x, object_y};
    //return result;
    return ntransform;
}

void save_callback(const std_msgs::Int32ConstPtr &msg)
{
    geometry_msgs::Point p;
    int lock_count = temp_count;
    for (int i = 0; i < lock_count; i++)
    {
        fx = co_x[i];
        fy = co_y[i];
        fz = co_z[i];
        tf::Transform transform = tf_cal(co_x[i], co_y[i], co_z[i]);
        object new_object(objects.size() + 1, transform);
        objects.push_back(new_object);
        // an array[ID, x, y] to be save
        float object_pose[4] = {objects.back().ID, objects.back().transform.getOrigin().x(), objects.back().transform.getOrigin().y(), objects.back().transform.getOrigin().z()};
        ofstream out;
        out.open("/home/mzwang/Desktop/objects.txt", ios_base::app | ios_base::out);
        for (int i = 0; i < 4; i++)
        {
            out << object_pose[i] << " ";
        }
        out << "\n";
        out.close();

        p.x = object_pose[1];
        p.y = object_pose[2];
        p.z = 0;
        points.points.push_back(p);

        counts++;
    }

    cout << "Detected Object Saved\n";
    cout << "New saved " << lock_count << " objects, total saved " << counts <<" objects.\n" ;
}

int main(int argc, char **argv)
{
    cout << "System Staring...\n";
    ros::init(argc, argv, "save_detected");
    ros::NodeHandle nh;
    //ros::Subscriber pcd_sub = nh.subscribe<PointCloud>("camera/depth_registered/points", 1, pcd_callback); //point cloud option
    ros::Subscriber info_sub = nh.subscribe("camera/depth_registered/camera_info", 10, info_callback); //depth option
    ros::Subscriber image_sub = nh.subscribe("camera/depth_registered/image_raw", 10, depthcb);        //depth option
    ros::Subscriber yolo_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("darknet_ros/bounding_boxes", 1, yolo_callback);
    ros::Subscriber save_sub = nh.subscribe<std_msgs::Int32>("save_status", 1, save_callback);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("object_marker", 10);

    ros::Rate r(10);

    points.header.frame_id = "/map";
    points.header.stamp = ros::Time();
    points.ns = "Points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    points.id = 0;

    points.type = visualization_msgs::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    tf::TransformBroadcaster br;
    tf::Transform transform;
    
    cout << "System Ready!\n";
    while (ros::ok())
    {
        //create new frame for detected object
        transform.setOrigin(tf::Vector3(fx, fy, fz));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_rgb_optical_frame", "object"));
        //publish object markers
        marker_pub.publish(points);
        ros::spinOnce();
        r.sleep();
    }
}