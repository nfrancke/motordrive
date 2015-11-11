#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

#include "ros/ros.h"

using namespace std;
using namespace LibSerial;

//
// Init the serial port.
//
SerialStream serial_port5;
SerialStream serial_port7;
SerialStream serial_port8;

class Subscribe
{
public:
	int iSpeed[10];

	Subscribe(ros::NodeHandle nh)
	{
		sub = nh.subscribe("mcWheelVelocityMps",1000, &Subscribe::commandRpmReceived, this);
	}

	void commandRpmReceived(const std_msgs::Float32MultiArray::ConstPtr& msg)
	{
		ROS_INFO("Float32MultiArray received");

		float dstride0 = msg->layout.dim[0].stride;

		ROS_INFO("data 0 = %f" , msg->data[0]);
		ROS_INFO("data 1 = %f" , msg->data[1]);
		ROS_INFO("data 2 = %f" , msg->data[2]);

		iSpeed[5] = msg->data[0 + dstride0*0];
		iSpeed[7] = msg->data[0 + dstride0*0];
		iSpeed[8] = msg->data[0 + dstride0*0];

		char out_buf[10];
		
		//Define data
		out_buf[0] = 0x5a;	//start of frame
		out_buf[1] = 0xaa;	//type
		out_buf[2] = 0x03;	//cmd
		out_buf[3] = (iSpeed[5] & 0xff);		//speed	
		out_buf[4] = (iSpeed[5] >> 8) & 0xff;	//speed
		out_buf[5] = 0x00;	//????
		out_buf[6] = 0x00;	//????
		out_buf[7] = 0x00;	//End of frame

		//write data to serial port.
       	serial_port5.write(out_buf, 8);

		//Define data
		out_buf[0] = 0x5a;	//start of frame
		out_buf[1] = 0xaa;	//type
		out_buf[2] = 0x03;	//cmd
		out_buf[3] = (iSpeed[7] & 0xff);		//speed	
		out_buf[4] = (iSpeed[7] >> 8) & 0xff;	//speed
		out_buf[5] = 0x00;	//????
		out_buf[6] = 0x00;	//????
		out_buf[7] = 0x00;	//End of frame

       	serial_port7.write(out_buf, 8);

		//Define data
		out_buf[0] = 0x5a;	//start of frame
		out_buf[1] = 0xaa;	//type
		out_buf[2] = 0x03;	//cmd
		out_buf[3] = (iSpeed[8] & 0xff);		//speed	
		out_buf[4] = (iSpeed[8] >> 8) & 0xff;	//speed
		out_buf[5] = 0x00;	//????
		out_buf[6] = 0x00;	//????
		out_buf[7] = 0x00;	//End of frame

       	serial_port8.write(out_buf, 8);
	}

private:
	ros::Subscriber sub;
};//end of class Subscribe

bool initSerialPort(SerialStream& serial_port, string sSerialPort);


int main(int argc, char **argv  )
{
	//initialize ROS
	ros::init(argc, argv, "mcMotorDriver");
	
	ros::NodeHandle nh;

	//check if init serial went good.
	if((initSerialPort(serial_port5,"/dev/ttyS5") 
		&& initSerialPort(serial_port7,"/dev/ttyS7") 
		&& initSerialPort(serial_port8,"/dev/ttyS8")) 
		!= 1){
	return 1;
	}
	ROS_INFO("All serial ports are initialized");	

	//create class
	Subscribe Sobject(nh);
//	Sobject.iSpeed[5] = 0x0020;
//	Sobject.iSpeed[7] = 0x0020;
//	Sobject.iSpeed[8] = 0x0020;

	//	ros::Subscriber sub = nh.subscribe("mcWheelVelocityMps",1000, commandRpmReceived);	

	ros::spin();

	return 0;
}

bool initSerialPort(SerialStream& serialPort,string sSerialPort){
	//
	//Open the serial port
	//
	ROS_DEBUG("Serial Port will be opened");
	//	serialPort.Open("/dev/ttyS8") ;
	serialPort.Open(sSerialPort);
	//check if serial port opens correctly
	if ( ! serialPort.good() )
    {
		ROS_ERROR("Error: Could not open serial port.");
		return 0;
	}

	ROS_INFO("Serial port is opened");

	//
	// Set the baud rate of the serial port.
	//
	serialPort.SetBaudRate( SerialStreamBuf::BAUD_115200 ) ;
	if ( ! serialPort.good() )
	{
		ROS_ERROR("Error: Could not set the baud rate.");
		return 0;
	}
	ROS_INFO("baud rate is setted");

	//
	// Set the number of data bits.
	//
	serialPort.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
	if ( ! serialPort.good() )
	{
		ROS_ERROR("Error: Could not set the character size.");
		return 0;
	}
	ROS_INFO("char size is setted");

	//
	// Disable parity.
	//
	serialPort.SetParity( SerialStreamBuf::PARITY_NONE ) ;
	if ( ! serialPort.good() )
	{
		ROS_ERROR("Error: Could not disable the parity.");
		return 0;
	}
	ROS_INFO("parity is setted");

	//
	// Set the number of stop bits.
	//
	serialPort.SetNumOfStopBits( 2 ) ;
	if ( ! serialPort.good() )
	{
		ROS_ERROR("Error: Could not set the number of stop bits.");
		return 0;
	}
	ROS_INFO("stop bits are setted");

	//
	// Turn off hardware flow control.
	//
	serialPort.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;
	if ( ! serialPort.good() )
	{
		ROS_ERROR("Error: Could not use hardware flow control.");
		return 0;
	} 
	ROS_INFO("Disabled flow control");

return 1;
}
