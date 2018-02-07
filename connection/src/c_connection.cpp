#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <functional>
#include <signal.h>
#include <algorithm>
#include <math.h>
#include <cmath>
#include <math.h>
#include <functional>
#include <iostream>
#include <list>
#include <numeric>
#include <vector>
#include <string>
#include <message_filters/subscriber.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <curl/curl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <connection/c_connection.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>

#define PI 3.141592
using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace XmlRpc;

/**
 * @brief cConnection::cConnection constructor of class cConnection, initialization of parameters.
 * @param nh
 * @param nh_private
 */

cConnection::cConnection (const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
:nh_(nh),nh_private_(nh_private)
{

    //Initializing TCP and UDP sockets
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("192.168.1.101");
    addr.sin_port=htons(14004);

    this->socket_tcp=socket(AF_INET,SOCK_STREAM,0);
    this->socket_=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);

    //connecting sockets
    int result=connect(this->socket_tcp, (const struct sockaddr *) &addr, sizeof(this->addr));
    //Initializing TCP and UDP sockets
    addr_udp.sin_family = AF_INET;
    addr_udp.sin_addr.s_addr = INADDR_ANY;
    int port_udp = 15040;
    addr_udp.sin_port=htons(port_udp);
    int connection=bind(this->socket_, (struct sockaddr *) &addr_udp,  sizeof(this->addr_udp));
    usleep(1000*1000);

    //Handshake
    unsigned char buffer[40];
    memset(buffer, 0, 40);
    char port[10];
    sprintf(port, "%d", port_udp);
    int l=strlen(port);
    l++;
    buffer[0]=57;
    buffer[1]=48;
    buffer[2]=l%256;
    buffer[3]=0;
    buffer[4]=l/256;
    buffer[5]=0x7f;


    memcpy(buffer + 6, port, l);
    //sending message
    int ret=send(socket_tcp, buffer, l + 6, 0);
    cout<<ret<<endl;

    //Publishing laser_data
    this->laser_pub=nh_.advertise<sensor_msgs::LaserScan>("Doris/scan",1,true);
    this->odom_pub=nh_.advertise<geometry_msgs::PoseStamped>("Doris/odom",1,true);

    //Publishing image
    image_transport::ImageTransport it(nh_);
    this->image_pub=it.advertise("Doris/camera",1,true);

    //Publishing tf to laser_link
        this->static_laserTransform.header.frame_id="Doris/cuerpo";
        this->static_laserTransform.child_frame_id="Doris/laser_link";
        this->static_laserTransform.transform.translation.x=0.0;
        this->static_laserTransform.transform.translation.y=0.0;
        this->static_laserTransform.transform.translation.z=0.296;
        this->static_laserTransform.transform.rotation.x=0.0;
        this->static_laserTransform.transform.rotation.y=0.0;
        this->static_laserTransform.transform.rotation.z=0.0;
        this->static_laserTransform.transform.rotation.w=1.0;
        this->static_laserTransform.header.stamp=ros::Time::now();
        this->static_broadcaster.sendTransform(static_laserTransform);

    //Publishing transform to camera_link
    this->static_cameraTransform.header.frame_id="Doris/cuerpo";
    this->static_cameraTransform.child_frame_id="Doris/cam1_link";
    this->static_cameraTransform.transform.translation.x=-0.26;
    this->static_cameraTransform.transform.translation.y=0.0;
    this->static_cameraTransform.transform.translation.z=1.46;
    this->static_cameraTransform.transform.rotation.x=0.0;
    this->static_cameraTransform.transform.rotation.y=0.0;
    this->static_cameraTransform.transform.rotation.z=0.0;
    this->static_cameraTransform.transform.rotation.w=1.0;
    this->static_cameraTransform.header.stamp=ros::Time::now();
    this->static_broadcaster.sendTransform(static_cameraTransform);


    //Camera via ethernet
    static const string cameraUrl= "http://192.168.0.19/record/current.jpg";
    curl = curl_easy_init();
    cout<<curl<<endl;
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 1);
    if(curl){
    int code2 =curl_easy_setopt(curl, CURLOPT_URL, "http://192.168.0.19/record/current.jpg");
    int code=curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &cConnection::write_data);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &cameraStream);
    }

}

size_t cConnection::write_data(char *ptr, size_t size, size_t nmemb, void *userdata) {
        std::ostringstream *stream = (std::ostringstream*)userdata;
        cout<<stream<<endl;
        size_t count = size * nmemb;
        cout<<count<<endl;
        stream->write(ptr, count);
        return count;
}


/**
 * @brief cConnection::readImage: receive image from camera via ethernet
 */
void cConnection::readImage(void){
    data.clear();
    cameraStream.str("");
    cameraStream.clear();
    res=curl_easy_perform(curl);
    std::string strCameraStream = cameraStream.str();
    std::copy(strCameraStream.begin(), strCameraStream.end(), std::back_inserter(data));
    if(data.size() > 0){
                    cv::Mat data_mat = cv::Mat(data);
                    frame = cv::Mat(cv::imdecode(data_mat, 1));
                    data_mat.release();
                    imshow("Camara",frame);
                    waitKey(30);
                    this->image_msg=cv_bridge::CvImage(std_msgs::Header(),"bgr8",frame).toImageMsg();
                    this->image_pub.publish(this->image_msg);

    }


}

/**
 * @brief cConnection::readFromSocket reading characters from socket
 */

void cConnection::readFromSocket(void){
    long valread;
    char message [4096];
    memset(message, 0, 4096);
    struct sockaddr_in inAddr;
    unsigned int sourceSize= sizeof(inAddr);
    valread=recvfrom(this->socket_,message, 4096,0,  (struct sockaddr *) &inAddr, &sourceSize);
    if (valread<0){
        perror("recvfrom()");

    }else{
        this->decoMessage(message,valread);
    }
}
/**
 * @brief cConnection::decoMessage decodification if odometry and laser scan messages.
 * @param message
 * @param size
 */
void cConnection::decoMessage(char message [], int size){
    string str(message);
    std::string encabezado_odom="POSE_VEL";
    std::string encabezado_laser="LASER";
    std::vector<float> vec;
    string comp= "|";
    string encabezado=str.substr(1,str.find(comp)-1);
    if(encabezado == encabezado_odom){
        string variable = str.substr(str.find(comp)+1, str.length());
        std::stringstream ss(variable);
        int i=0;
        while (ss.good()){
           string substr;
           getline(ss,substr,',');
           vec.push_back(stof(substr));
        }
        //odom_base
        odom.header.frame_id="Doris/odom";
        odom.child_frame_id="Doris/cuerpo";
        odom.transform.translation.x=vec.at(0);
        odom.transform.translation.y=vec.at(1);
        odom.transform.translation.z=0.0;
        tf2::Quaternion quat;
        quat.setRPY(0,0,vec.at(2));
        odom.transform.rotation.x=quat.x();
        odom.transform.rotation.y=quat.y();
        odom.transform.rotation.z=quat.z();
        odom.transform.rotation.w=quat.w();
        odom.header.stamp=ros::Time::now();
        this->br.sendTransform(odom);

        //topic
        geometry_msgs::PoseStamped odom_msgs;
        odom_msgs.header.frame_id="Doris/odom";
        odom_msgs.pose.position.x=vec.at(0);
        odom_msgs.pose.position.y=vec.at(1);
        odom_msgs.pose.position.z=0.0;
        odom_msgs.pose.orientation.x=quat.x();
        odom_msgs.pose.orientation.y=quat.y();
        odom_msgs.pose.orientation.z=quat.z();
        odom_msgs.pose.orientation.w=quat.w();
        odom_msgs.header.stamp=ros::Time::now();
        odom_pub.publish(odom_msgs);





    }
    //If we receive a laser message
    if (encabezado==encabezado_laser){
        string variable = str.substr(str.find(comp)+1, str.length());
        std::stringstream ss(variable);
        int i=0;
        while (ss.good()){
           string substr;
           getline(ss,substr,',');
           vec.push_back(stof(substr));
        }
      scan.header.stamp=ros::Time::now();
      scan.header.frame_id="Doris/laser_link";
      scan.angle_min=PI/2;
      scan.angle_max=-PI/2;
      scan.angle_increment=(-PI)/(vec.size());
      scan.range_min=0.0;
      scan.range_max=80;
      scan.ranges=vec;
      this->laser_pub.publish(scan);


    }

}
