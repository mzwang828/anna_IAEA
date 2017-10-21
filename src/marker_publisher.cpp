#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_marker");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("object_marker", 10);

    ros::Rate r(30);

    ifstream infile("/home/mzwang/Desktop/objects.txt", ios::in | ios::out | ios::binary);

    vector<vector<float> > objects;
    vector<float> one_object;
    string line;
    int row = 0;
    while (getline(infile, line))
    {
        stringstream ss;
        ss << line;
        float a;
        while (ss >> a) //write as ss>>a as while condition to avoid repeated last element
        {
            one_object.push_back(a);
        }
        objects.push_back(one_object);
        one_object.clear();
        row++;
    }

    infile.close();

    visualization_msgs::Marker points;
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

    // Create the vertices for the points and lines
    for (int i = 0; i < row; ++i)
    {

        geometry_msgs::Point p;
        p.x = objects[i][1];
        p.y = objects[i][2];
        p.z = 0; //set z axis translation as 0

        points.points.push_back(p);
    }

    while (ros::ok())
    {
        marker_pub.publish(points);
        r.sleep();
    }
}