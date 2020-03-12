#include <iostream>
#include <ros/ros.h>

//#include <sensor_msgs/PointCloud2.h>
#include "radar_detection/ultrasonic.h"
#include <ICANCmd.h>

using namespace std;

#define height 0
DWORD dwDeviceHandle;
CAN_InitConfig config;
int main(int argc, char **argv)
{
	ros::init(argc, argv, "ultrasonic");
	ros::NodeHandle nh;
	//ros::Publisher pub_1 = nh.advertise<perception_sensor_msgs::ObjectList>("/radar", 2);
	ros::Publisher pub_2 = nh.advertise<radar_detection::ultrasonic>("ultrasonic_msg", 2);
	
	if(dwDeviceHandle = CAN_DeviceOpen(ACUSB_132B, 0, 0) == 1)
	{
		ROS_INFO_STREAM(" >>open device success!");
	}
	else
	{
		ROS_ERROR_STREAM(" >>open device error!");
		return 0;
        exit(1);
	}

	CAN_InitConfig config;
   	config.dwAccCode = 0;
   	config.dwAccMask = 0xffffffff;
   	config.nFilter  = 0;       // 滤波方式(0表示未设置滤波功能,1表示双滤波,2表示单滤波)
   	config.bMode    = 0;             // 工作模式(0表示正常模式,1表示只听模式)
   	config.nBtrType = 1;      // 位定时参数模式(1表示SJA1000,0表示LPC21XX)
   	config.dwBtr[0] = 0x01;   // BTR0   0014 -1M 0016-800K 001C-500K 011C-250K 031C-12K 041C-100K 091C-50K 181C-20K 311C-10K BFFF-5K
   	config.dwBtr[1] = 0x1c;   // BTR1
   	config.dwBtr[2] = 0;
   	config.dwBtr[3] = 0;
    
	if( CAN_ChannelStart(dwDeviceHandle, 0, &config) != CAN_RESULT_OK )
	{
		ROS_ERROR_STREAM(" >>Init CAN0 error!");

	    return 0;
	}

	if( CAN_ChannelStart(dwDeviceHandle, 1, &config) != CAN_RESULT_OK )
	{
		ROS_ERROR_STREAM(" >>Init CAN1 error!");

	    return 0;
	}
  
	int reclen = 0;
	CAN_DataFrame rec[3000]; //buffer
	int CANInd = 0; //CAN1=0, CAN2=1

	vector<CAN_DataFrame> frame_obj;

	while(ros::ok())
	{
		radar_detection::ultrasonic msg;
		if ((reclen = CAN_ChannelReceive(dwDeviceHandle, 0, rec, 3000, 100)) > 0) //调用接收函数,得到数据
		{

			
			for(int i=0; i<reclen; i++)
			{
				

				// release the objects of previous frame
			

					for(int j = 0; j < frame_obj.size(); j++)
					{
						if(frame_obj[j].uID == 0x525)
						{
							cout << "***************" << endl;
							switch (frame_obj[j].arryData[0])
							{
							case 0x40:
								msg.FL = 0.01;
								break;
							case 0x41:
								msg.FL = 0.01;
								break;
							case 0x42:
								msg.FL = 0.01;
								break;
							case 0x43:
								msg.FL = 0.40;
								break;
							case 0x44:
								msg.FL = 0.45;
								break;	
							case 0x45:
								msg.FL = 0.50;
								break;
							case 0x46:
								msg.FL = 0.55;
								break;	
							case 0x47:
								msg.FL = 0.60;
								break;
							case 0x48:
								msg.FL = 0.65;
								break;
							case 0x49:
								msg.FL = 0.70;
								break;
							case 0x4a:
								msg.FL = 0.75;
								break;
							case 0x4b:
								msg.FL = 0.80;
								break;
							case 0x4c:
								msg.FL = 0.85;
								break;
							case 0x4d:
								msg.FL = 0.90;
								break;
							case 0x4e:
								msg.FL = 0.95;
								break;
							case 0x4f:
								msg.FL = 1.00;
								break;
							case 0x50:
								msg.FL = 1.05;
								break;
							case 0x51:
								msg.FL = 1.10;
								break;
					        case 0x52:
								msg.FL = 1.15;
								break;
							case 0x53:
								msg.FL = 1.20;
								break;
							case 0x54:
								msg.FL = 1.25;
								break;
							case 0x55:
								msg.FL = 1.30;
								break;
							case 0x56:
								msg.FL = 1.35;
								break;
							case 0x57:
								msg.FL = 1.40;
								break;
							case 0x58:
								msg.FL = 1.45;
								break;
							case 0x59:
								msg.FL = 1.50;
								break;
							case 0x5e:
								msg.FL = 100;
								break;
							default:
								break;
							}
							switch (frame_obj[j].arryData[1])
							{
							case 0x20:
								msg.FCL = 0.01;
								break;
							case 0x21:
								msg.FCL = 0.01;
								break;
							case 0x22:
								msg.FCL = 0.01;
								break;
							case 0x23:
								msg.FCL = 0.40;
								break;
							case 0x24:
								msg.FCL = 0.45;
								break;	
							case 0x25:
								msg.FCL = 0.50;
								break;
							case 0x26:
								msg.FCL = 0.55;
								break;	
							case 0x27:
								msg.FCL = 0.60;
								break;
							case 0x28:
								msg.FCL = 0.65;
								break;
							case 0x29:
								msg.FCL = 0.70;
								break;
							case 0x2a:
								msg.FCL = 0.75;
								break;
							case 0x2b:
								msg.FCL = 0.80;
								break;
							case 0x2c:
								msg.FCL = 0.85;
								break;
							case 0x2d:
								msg.FCL = 0.90;
								break;
							case 0x2e:
								msg.FCL = 0.95;
								break;
							case 0x2f:
								msg.FCL = 1.00;
								break;
							case 0x30:
								msg.FCL = 1.05;
								break;
							case 0x31:
								msg.FCL = 1.10;
								break;
					        case 0x32:
								msg.FCL = 1.15;
								break;
							case 0x33:
								msg.FCL = 1.20;
								break;
							case 0x34:
								msg.FCL = 1.25;
								break;
							case 0x35:
								msg.FCL = 1.30;
								break;
							case 0x36:
								msg.FCL = 1.35;
								break;
							case 0x37:
								msg.FCL = 1.40;
								break;
							case 0x38:
								msg.FCL = 1.45;
								break;
							case 0x39:
								msg.FCL = 1.50;
								break;
							case 0x3e:
								msg.FCL = 100;
								break;
							default:
								break;
							}
						/*switch (frame_obj[j].arryData[2])
							{
							case 0x00:
								msg.FCR = 0.01;
								break;
							case 0x01:
								msg.FCR = 0.01;
								break;
							case 0x02:
								msg.FCR = 0.01;
								break;
							case 0x03:
								msg.FCR = 0.40;
								break;
							case 0x04:
								msg.FCR = 0.45;
								break;	
							case 0x05:
								msg.FCR = 0.50;
								break;
							case 0x06:
								msg.FCR = 0.55;
								break;	
							case 0x07:
								msg.FCR = 0.60;
								break;
							case 0x08:
								msg.FCR = 0.65;
								break;
							case 0x09:
								msg.FCR = 0.70;
								break;
							case 0x0a:
								msg.FCR = 0.75;
								break;
							case 0x0b:
								msg.FCR = 0.80;
								break;
							case 0x0c:
								msg.FCR = 0.85;
								break;
							case 0x0d:
								msg.FCR = 0.90;
								break;
							case 0x0e:
								msg.FCR = 0.95;
								break;
							case 0x0f:
								msg.FCR = 1.00;
								break;
							case 0x10:
								msg.FCR = 1.05;
								break;
							case 0x11:
								msg.FCR = 1.10;
								break;
					        case 0x12:
								msg.FCR = 1.15;
								break;
							case 0x13:
								msg.FCR = 1.20;
								break;
							case 0x14:
								msg.FCR = 1.25;
								break;
							case 0x15:
								msg.FCR = 1.30;
								break;
							case 0x16:
								msg.FCR = 1.35;
								break;
							case 0x17:
								msg.FCR = 1.40;
								break;
							case 0x18:
								msg.FCR = 1.45;
								break;
							case 0x19:
								msg.FCR = 1.50;
								break;
							case 0x1e:
								msg.FCR = 100;
								break;
							default:
								break;
							}*/
							switch (frame_obj[j].arryData[3])
							{
							case 0x40:
								msg.FR = 0.01;
								break;
							case 0x41:
								msg.FR = 0.01;
								break;
							case 0x42:
								msg.FR = 0.01;
								break;
							case 0x43:
								msg.FR = 0.40;
								break;
							case 0x44:
								msg.FR = 0.45;
								break;	
							case 0x45:
								msg.FR = 0.50;
								break;
							case 0x46:
								msg.FR = 0.55;
								break;	
							case 0x47:
								msg.FR = 0.60;
								break;
							case 0x48:
								msg.FR = 0.65;
								break;
							case 0x49:
								msg.FR = 0.70;
								break;
							case 0x4a:
								msg.FR = 0.75;
								break;
							case 0x4b:
								msg.FR = 0.80;
								break;
							case 0x4c:
								msg.FR = 0.85;
								break;
							case 0x4d:
								msg.FR = 0.90;
								break;
							case 0x4e:
								msg.FR = 0.95;
								break;
							case 0x4f:
								msg.FR = 1.00;
								break;
							case 0x50:
								msg.FR = 1.05;
								break;
							case 0x51:
								msg.FR = 1.10;
								break;
					        case 0x52:
								msg.FR = 1.15;
								break;
							case 0x53:
								msg.FR = 1.20;
								break;
							case 0x54:
								msg.FR = 1.25;
								break;
							case 0x55:
								msg.FR = 1.30;
								break;
							case 0x56:
								msg.FR = 1.35;
								break;
							case 0x57:
								msg.FR = 1.40;
								break;
							case 0x58:
								msg.FR = 1.45;
								break;
							case 0x59:
								msg.FR = 1.50;
								break;
							case 0x5e:
								msg.FR = 100;
								break;
							default:
								break;
							}

						}
					
					}
					cout<<"the distance is:"<<msg.FL<<endl;
									
					pub_2.publish(msg);
					frame_obj.clear();
                }
            }
		}
	
	 CAN_DeviceClose(dwDeviceHandle);
	return 0;
}
