

#include <iostream>
#include <fstream>
#include <stdint.h>
#include <boost/thread/thread.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/Imu.h"
#include "optical_flow_rad.h"
#include"include/MavConnPx4Flow.h"
#include"include/SerialPort.h"
#include "time.h"
#include "stdlib.h"
#include "stdio.h"

using namespace std;

void qtoEuler(double& _roll,double& _pitch,double& _yaw,double q[]);
void eulertoRotation(float_t roll,float_t pitch,float_t yaw,float R[]);

ofstream result("/home/liuxiaobu/record/flow_test.txt", ios::out);
double viconRoll=0;
double viconPitch=0;
double viconYaw=0;
double poseVicon[3]={0,0,0};

struct OpticalFlowData{
    //optical flow global path and velocity
    float opticalGlobal_x_m;
    float opticalGlobal_y_m;
    float opticalGlobal_z_m;
    float opticalGlobal_vx;
    float opticalGlobal_vy;
    float opticalGlobal_vz;
    float opticalLocal_vx;
    float opticalLocal_vy;
    float opticalLocal_vz;
    float distance;
    float distanceRaw;
    int quality;
    uint32_t timeStamp;
public:
    OpticalFlowData()
    {
        opticalGlobal_x_m=0;
        opticalGlobal_y_m=0;
        opticalGlobal_z_m=0;
        opticalGlobal_vx=0;
        opticalGlobal_vy=0;
        opticalGlobal_vz=0;
        opticalLocal_vx=0;
        opticalLocal_vy=0;
        opticalLocal_vz=0;
        distance=0;
        distanceRaw=0;
        quality=0;
        timeStamp=0;
    }
};

void viconDataCallback(const geometry_msgs::TransformStamped::ConstPtr &msg)
{
    double pose[3],qbv[4];
    qbv[0]=msg->transform.rotation.w;
    qbv[1]=msg->transform.rotation.x;
    qbv[2]=msg->transform.rotation.y;
    qbv[3]=msg->transform.rotation.z;

    poseVicon[0]=msg->transform.translation.x;
    poseVicon[1]=msg->transform.translation.y;
    poseVicon[2]=msg->transform.translation.z;

    qtoEuler(viconRoll,viconPitch,viconYaw,qbv);
    //cout<<"vicon "<<" roll:"<<viconRoll<<" pitch:"<<viconPitch<<" yaw:"<<viconYaw<<endl;


}

void px4flowCallback(const opticalFlowTest::optical_flow_rad &px4flow_msg){
    uint32_t integrationTime_us = px4flow_msg.integration_time_us;
    float integratedLocal_x_pixel = px4flow_msg.integrated_x;
    float integratedLocal_y_pixel = px4flow_msg.integrated_y;
    uint32_t timeDeltaDistance_us = px4flow_msg.time_delta_distance_us;

    static OpticalFlowData opticalFlowData;
    static int flowUpdateCount=0;
    static float opticalFlowLastDis=0;
    static float opticalFlowLastLastDis=0;
    static int distanceGitchNum=0;
    static bool isFirstUpdate=true;

    opticalFlowData.quality = px4flow_msg.quality;
    opticalFlowData.timeStamp += integrationTime_us;
    opticalFlowData.distanceRaw = px4flow_msg.distance;

    //***************throw out the outlier of sonar data************
//    double distanceGitchNumThresh=1;
    if(flowUpdateCount==0){
        opticalFlowData.distance = px4flow_msg.distance;
        opticalFlowLastDis = px4flow_msg.distance;
        opticalFlowLastLastDis = px4flow_msg.distance;
        flowUpdateCount+=1;
    }
    else if(flowUpdateCount==1){
        opticalFlowData.distance = px4flow_msg.distance;
        opticalFlowLastDis = px4flow_msg.distance;
        flowUpdateCount+=1;
    }
    else{
//        if((msg.distance==0 || abs(msg.distance-opticalFlowLastDis)>0.5) && distanceGitchNum<distanceGitchNumThresh){
        if((px4flow_msg.distance==0 || abs(px4flow_msg.distance-opticalFlowLastDis)>0.5)){
            opticalFlowData.distance = 2*opticalFlowLastDis-opticalFlowLastLastDis;
            distanceGitchNum++;
        }
        else{
            opticalFlowData.distance = px4flow_msg.distance;
            distanceGitchNum=0;
        }
    }

    //*******************calculate the translation in inertial frame***********
    double yaw = viconYaw;
//    double roll = viconRoll;
//    double pitch = viconPitch;
    double roll=0;
    double pitch=0;
    double distance = opticalFlowData.distanceRaw;

    //px4flow y is forward; px4flow is SWU, so transform it to NWU(vicon)

    integratedLocal_x_pixel =- integratedLocal_x_pixel;
    integratedLocal_y_pixel = integratedLocal_y_pixel;

    //optical flow velocity in world NWU
    if(integrationTime_us>0){
        opticalFlowData.opticalLocal_vx = integratedLocal_x_pixel * distance / integrationTime_us * 1000000;
        opticalFlowData.opticalLocal_vy = integratedLocal_y_pixel * distance / integrationTime_us * 1000000;
        //optical_flow from mavros, distance is in world frame
         opticalFlowData.opticalGlobal_vz=0;
//        opticalFlowData.opticalGlobal_vz = (distance-opticalFlowLastDis)/integrationTime_us * 1000000;
//        opticalFlowData.opticalLocal_vz = -(distance-opticalFlowLastDis)/integrationTime_us * 1000000;

        //calculate R_NWU
        float R_NWU[9];
        eulertoRotation((float)roll,(float)pitch,(float)yaw,R_NWU);
        double bodyVx = opticalFlowData.opticalLocal_vx;
        double bodyVy = opticalFlowData.opticalLocal_vy;
        double worldVz = opticalFlowData.opticalGlobal_vz;
        opticalFlowData.opticalLocal_vz = (worldVz-R_NWU[6]*bodyVx-R_NWU[7]*bodyVy)/R_NWU[8];
        //calculate world velocity
        double NWU_vel[3],bodyVel[3];
        bodyVel[0]=opticalFlowData.opticalLocal_vx;
        bodyVel[1]=opticalFlowData.opticalLocal_vy;
        bodyVel[2]=opticalFlowData.opticalLocal_vz;
        NWU_vel[0]=0;NWU_vel[1]=0;NWU_vel[2]=0;
        for(int i=0;i<3;++i){
            for(int j=0;j<3;++j){
                NWU_vel[i]+=R_NWU[3*i+j]*bodyVel[j];
            }
        }
        opticalFlowData.opticalGlobal_vx = NWU_vel[0];
        opticalFlowData.opticalGlobal_vy = NWU_vel[1];
        opticalFlowData.opticalGlobal_vz = NWU_vel[2];

        opticalFlowLastLastDis=opticalFlowLastDis;
        opticalFlowLastDis = distance;

        //optical flow integration path in world NED
        opticalFlowData.opticalGlobal_x_m += opticalFlowData.opticalGlobal_vx * integrationTime_us/1000000;
        opticalFlowData.opticalGlobal_y_m += opticalFlowData.opticalGlobal_vy * integrationTime_us/1000000;

        if(isFirstUpdate){
            result<<"timeStamp "<<"flowLocalVx "<<"flowLocalVy "<<"flowLocalVz "<<"flowWorldVx "<<"flowWorldVy "<<"flowWorldVz "<<"flowWorldX "\
              <<"flowWorldY "<<"distanceRaw "<<"distance "<<"viconX "<<"viconY "<<"viconZ "<<"viconRoll "<<"viconPitch "<<"viconYaw"<<endl;
            isFirstUpdate=false;
        }
        else{
            result<<opticalFlowData.timeStamp<<" "<<opticalFlowData.opticalLocal_vx<<" "<<opticalFlowData.opticalLocal_vy<<" "<<opticalFlowData.opticalLocal_vz<<" "\
              <<opticalFlowData.opticalGlobal_vx<<" "<<opticalFlowData.opticalGlobal_vy<<" "<<opticalFlowData.opticalGlobal_vz<<" "<<opticalFlowData.opticalGlobal_x_m<<" "\
             <<opticalFlowData.opticalGlobal_y_m<<" "<<opticalFlowData.distanceRaw<<" "<<opticalFlowData.distance<<" "<<poseVicon[0]<<" "<<poseVicon[1]<<" "<<poseVicon[2]<<" "\
            <<viconRoll<<" "<<viconPitch<<" "<<viconYaw<<endl;
        }

        cout <<"dis: "<<opticalFlowData.distanceRaw<<" x: "<<opticalFlowData.opticalGlobal_x_m<<" y:"<<opticalFlowData.opticalGlobal_y_m<<"roll: "<<viconRoll<<" pitch:"<<viconPitch\
                            <<" yaw:"<<viconYaw<<" viconX:"<<poseVicon[0]<<" viconY:"<<poseVicon[1]<<" viconZ:"<<poseVicon[2]<<endl;


    }
}

int main(int argc, char *argv[]){
    ROS_INFO("Starting px4flow_node...");
    if(!result){
        cerr<<"files open failed!"<<endl;
        return -1;
    }
    ros::init(argc, argv, "px4flowTestNode");
    ros::NodeHandle nh;

    ros::Subscriber viconData_sub=nh.subscribe("/vicon/intel_aero/intel_aero", 1,&viconDataCallback);
    ros::Subscriber px4flow_sub=nh.subscribe("/px4flow/px4flow/raw/optical_flow_rad",10,&px4flowCallback);

    ros::Publisher opticalFLow_pub;
    ros::Publisher flowVel_pub;
    ros::Publisher flowPos_pub;

    opticalFlowTest::optical_flow_rad px4flow_msg;
    opticalFLow_pub = nh.advertise<opticalFlowTest::optical_flow_rad>("/px4flowTestNode/px4flow", 1);
    flowVel_pub = nh.advertise<geometry_msgs::Vector3>("/px4flowTestNode/flow_vel",1);
    flowPos_pub=nh.advertise<geometry_msgs::Vector3>("/px4flowTestNode/flow_pose",1);

    OpticalFlowData opticalFlowData;
    int flowUpdateCount=0;
    float opticalFlowLastDis;
    float opticalFlowLastLastDis;
    int distanceGitchNum=0;
    bool isFirstUpdate=true;

    char *uart_name = (char*)"/dev/ttyACM0";
    int baudrate = 115200;
    MAVCONNPx4Flow mav_conn_px4flow(uart_name, baudrate);
   mav_conn_px4flow.open();


    ros::Rate rosRate(20);
//    while(ros::ok()){
//        rosRate.sleep();
//        ros::spinOnce();
//    }
    double start,finish;
    int outLoop=0;
    int inLoop=0;
    int ininLoop=0;
    sleep(10);
    while(true){
        mavlink_message_t msg;

        int nMsg = mav_conn_px4flow.sp.read_message(msg);

        outLoop++;

        if( nMsg > 0){
             inLoop++;
            if( msg.msgid == MAVLINK_MSG_ID_OPTICAL_FLOW_RAD){
                ininLoop++;

                std::cout << "sysid:" << (int) msg.sysid << " " << (int) msg.compid << " msgid:" << (int) msg.msgid << std::endl;
                mavlink_optical_flow_rad_t optical_flow_rad;
//                start=ros::Time::now().toSec();
                mavlink_msg_optical_flow_rad_decode(&msg,&optical_flow_rad);
//                finish=ros::Time::now().toSec();
//                cout<<"time: "<<(double)( finish-start)<<endl;
                //mavlink_msg_optical_flow_decode(&msg,&optical_flow_rad);

                px4flow_msg.distance=optical_flow_rad.distance;
                uint32_t sec=optical_flow_rad.time_usec/1000000;
                uint32_t nsec=(optical_flow_rad.time_usec-sec*1000000)*1000;
                px4flow_msg.header.stamp=ros::Time(sec,nsec);
                px4flow_msg.integrated_x=optical_flow_rad.integrated_x;
                px4flow_msg.integrated_y=optical_flow_rad.integrated_y;
                px4flow_msg.integrated_xgyro=optical_flow_rad.integrated_xgyro;
                px4flow_msg.integrated_ygyro=optical_flow_rad.integrated_ygyro;
                px4flow_msg.integrated_zgyro=optical_flow_rad.integrated_zgyro;
                px4flow_msg.integration_time_us=optical_flow_rad.integration_time_us;
                px4flow_msg.quality=optical_flow_rad.quality;
                px4flow_msg.temperature=optical_flow_rad.temperature;
                px4flow_msg.time_delta_distance_us=optical_flow_rad.time_delta_distance_us;

                uint32_t integrationTime_us = px4flow_msg.integration_time_us;
                float integratedLocal_x_pixel = px4flow_msg.integrated_x;
                float integratedLocal_y_pixel = px4flow_msg.integrated_y;
                uint32_t timeDeltaDistance_us = px4flow_msg.time_delta_distance_us;

                opticalFlowData.quality = px4flow_msg.quality;
                opticalFlowData.timeStamp += integrationTime_us;
                opticalFlowData.distanceRaw = px4flow_msg.distance;

                //***************throw out the outlier of sonar data************
            //    double distanceGitchNumThresh=1;
                if(flowUpdateCount==0){
                    opticalFlowData.distance = px4flow_msg.distance;
                    opticalFlowLastDis = px4flow_msg.distance;
                    opticalFlowLastLastDis = px4flow_msg.distance;
                    flowUpdateCount+=1;
                }
                else if(flowUpdateCount==1){
                    opticalFlowData.distance = px4flow_msg.distance;
                    opticalFlowLastDis = px4flow_msg.distance;
                    flowUpdateCount+=1;
                }
                else{
            //        if((msg.distance==0 || abs(msg.distance-opticalFlowLastDis)>0.5) && distanceGitchNum<distanceGitchNumThresh){
                    if((px4flow_msg.distance==0 || abs(px4flow_msg.distance-opticalFlowLastDis)>0.5)){
                        opticalFlowData.distance = 2*opticalFlowLastDis-opticalFlowLastLastDis;
                        distanceGitchNum++;
                    }
                    else{
                        opticalFlowData.distance = px4flow_msg.distance;
                        distanceGitchNum=0;
                    }
                }

                //*******************calculate the translation in inertial frame***********
                double yaw = viconYaw;
//                double roll = viconRoll;
//                double pitch = viconPitch;
                double roll=0;
                double pitch=0;
                double distance = opticalFlowData.distanceRaw;

                //px4flow y is forward; px4flow is SWU, so transform it to NWU(vicon)

                integratedLocal_x_pixel =- integratedLocal_x_pixel;
                integratedLocal_y_pixel = integratedLocal_y_pixel;

                //optical flow velocity in world NWU
                if(integrationTime_us>0){
                    opticalFlowData.opticalLocal_vx = integratedLocal_x_pixel * distance / integrationTime_us * 1000000;
                    opticalFlowData.opticalLocal_vy = integratedLocal_y_pixel * distance / integrationTime_us * 1000000;
                    //optical_flow from mavros, distance is in world frame
                    opticalFlowData.opticalGlobal_vz=0;
//                    opticalFlowData.opticalGlobal_vz = (distance-opticalFlowLastDis)/integrationTime_us * 1000000;
            //        opticalFlowData.opticalLocal_vz = -(distance-opticalFlowLastDis)/integrationTime_us * 1000000;

                    //calculate R_NWU
                    float R_NWU[9];
                    eulertoRotation((float)roll,(float)pitch,(float)yaw,R_NWU);
                    double bodyVx = opticalFlowData.opticalLocal_vx;
                    double bodyVy = opticalFlowData.opticalLocal_vy;
                    double worldVz = opticalFlowData.opticalGlobal_vz;
                    opticalFlowData.opticalLocal_vz = (worldVz-R_NWU[6]*bodyVx-R_NWU[7]*bodyVy)/R_NWU[8];
                    //cout<<"localVz:"<< opticalFlowData.opticalLocal_vz<<endl;
                    //calculate world velocity
                    double NWU_vel[3],bodyVel[3];
                    bodyVel[0]=opticalFlowData.opticalLocal_vx;
                    bodyVel[1]=opticalFlowData.opticalLocal_vy;
                    bodyVel[2]=opticalFlowData.opticalLocal_vz;
                    NWU_vel[0]=0;NWU_vel[1]=0;NWU_vel[2]=0;
                    for(int i=0;i<3;++i){
                        for(int j=0;j<3;++j){
                            NWU_vel[i]+=R_NWU[3*i+j]*bodyVel[j];
                        }
                    }
                    opticalFlowData.opticalGlobal_vx = NWU_vel[0];
                    opticalFlowData.opticalGlobal_vy = NWU_vel[1];
                    opticalFlowData.opticalGlobal_vz = NWU_vel[2];
                    cout<<"globalVz:"<< opticalFlowData.opticalGlobal_vz<<endl;

                    opticalFlowLastLastDis=opticalFlowLastDis;
                    opticalFlowLastDis = distance;

                    //optical flow integration path in world NED
                    opticalFlowData.opticalGlobal_x_m += opticalFlowData.opticalGlobal_vx * integrationTime_us/1000000;
                    opticalFlowData.opticalGlobal_y_m += opticalFlowData.opticalGlobal_vy * integrationTime_us/1000000;

                    if(isFirstUpdate){
                        result<<"timeStamp "<<"flowLocalVx "<<"flowLocalVy "<<"flowLocalVz "<<"flowWorldVx "<<"flowWorldVy "<<"flowWorldVz "<<"flowWorldX "\
                          <<"flowWorldY "<<"distanceRaw "<<"distance "<<"viconX "<<"viconY "<<"viconZ "<<"viconRoll "<<"viconPitch "<<"viconYaw"<<endl;
                        isFirstUpdate=false;
                    }
                    else{
                        result<<optical_flow_rad.time_usec<<" "<<opticalFlowData.opticalLocal_vx<<" "<<opticalFlowData.opticalLocal_vy<<" "<<opticalFlowData.opticalLocal_vz<<" "\
                          <<opticalFlowData.opticalGlobal_vx<<" "<<opticalFlowData.opticalGlobal_vy<<" "<<opticalFlowData.opticalGlobal_vz<<" "<<opticalFlowData.opticalGlobal_x_m<<" "\
                         <<opticalFlowData.opticalGlobal_y_m<<" "<<opticalFlowData.distanceRaw<<" "<<opticalFlowData.distance<<" "<<poseVicon[0]<<" "<<poseVicon[1]<<" "<<poseVicon[2]<<" "\
                        <<viconRoll<<" "<<viconPitch<<" "<<viconYaw<<endl;
                    }


                }
                //***************************************************************************



                geometry_msgs::Vector3 flowVel;
                geometry_msgs::Vector3 flowPose;
                flowVel.x=opticalFlowData.opticalLocal_vx;
                flowVel.y=opticalFlowData.opticalLocal_vy;
                flowVel.z=opticalFlowData.opticalLocal_vz;
                flowPose.x=opticalFlowData.opticalGlobal_x_m;
                flowPose.y=opticalFlowData.opticalGlobal_y_m;
                flowPose.z=opticalFlowData.distance;
                flowVel_pub.publish(flowVel);
                flowPos_pub.publish(flowPose);
                opticalFLow_pub.publish(px4flow_msg);


                //cout<<"time: "<<(double)( ros::Time::now().toSec()-start)<<endl;
                //start=ros::Time::now().toSec();
                //rosRate.sleep();
                ros::spinOnce();
                cout <<"dis: "<<opticalFlowData.distanceRaw<<" x: "<<opticalFlowData.opticalGlobal_x_m<<" y:"<<opticalFlowData.opticalGlobal_y_m<<"roll: "<<viconRoll<<" pitch:"<<viconPitch\
                    <<" yaw:"<<viconYaw<<" viconX:"<<poseVicon[0]<<" viconY:"<<poseVicon[1]<<" viconZ:"<<poseVicon[2]<<endl;
            }

        }
       //cout<<"out:"<<outLoop<<" in:"<<inLoop<<" inin:"<<ininLoop<<endl;


    }
    //mav_conn_px4flow.startReadingLoop();
    //mav_conn_px4flow._readingLoop();
    //sleep(2);
    return 0;
}

void qtoEuler(double& _roll,double& _pitch,double& _yaw,double q[]){
    float_t w,x,y,z;
    //diffrent coordinates
    w=q[0];
    x=q[1];
    y=q[2];
    z=q[3];
    //cout<<"q:"<<w<<"    "<<x<<"    "<<y<<"    "<<z<<endl;
    _roll  = atan2(2 * (w * x + y* z) , 1 - 2 * (x * x + y * y));
    if(2 * (w * y - z * x)>1)_pitch =asin(1.0) ;
    else if(2 * (w * y - z * x)<-1.0)_pitch = asin(-1.0);
    else _pitch = asin(2 * (w * y - z * x));
    _yaw   = atan2(2 * (w * z + x * y) , 1 - 2 * (y * y + z * z));
}

void eulertoRotation(float_t roll,float_t pitch,float_t yaw,float R[]){
    R[0] = cos(yaw) * cos(pitch);
    R[1] = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw)*cos(roll);
    R[2] = cos(yaw) * sin(pitch) * cos(roll)+sin(yaw)*sin(roll);
    R[3] = sin(yaw) * cos(pitch);
    R[4] = sin(yaw) * sin(pitch) * sin(roll)+cos(yaw)*cos(roll);
    R[5] = sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
    R[6] = -sin(pitch);
    R[7] = cos(pitch)*sin(roll);
    R[8] = cos(pitch)*cos(roll);
}
