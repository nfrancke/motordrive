#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <geometry_msgs/Twist.h>

#include "ros/ros.h"

using namespace std;
using namespace LibSerial;

bool initSerialPort(SerialStream& serial_port, string sSerialPort);

//void commandRPMReceived(){
//	ROS_DEBUG("command received");
//}

int main(int argc, char **argv  )
{
	//initialize ROS
	ros::init(argc, argv, "mcMotorDriver");

	//define a nodehandle and define the rate for the "ros while loop"
	ros::NodeHandle nh;
	ros::Rate rate(100);
	
	//	ros::Subscriber sub = nh.subscribe("turtle1/cmd_vel",1000,&commandRPMReceived);
	
	ROS_INFO("Ros is initialized");	

	//
	// Init the serial port.
	//
	SerialStream serial_port5;
	SerialStream serial_port7;
	SerialStream serial_port8;


	bool bInitSerialOk = initSerialPort(serial_port5,"/dev/ttyS5");
	
	//check if init serial went good.
	if(!bInitSerialOk){
		return 1;
	}

	bInitSerialOk = initSerialPort(serial_port7,"/dev/ttyS7");
	
	//check if init serial went good.
	if(!bInitSerialOk){
		return 1;
	}

	bInitSerialOk = initSerialPort(serial_port8,"/dev/ttyS8");
	
	//check if init serial went good.
	if(!bInitSerialOk){
		return 1;
	}

	//
	// Do not skip whitespace characters while reading from the
	// serial port.
	//
	// serial_port.unsetf( std::ios_base::skipws ) ;
	//
	// Wait for some data to be available at the serial port.
	//
	//
	// Wait for some data to be available at the serial port.
	// Keep reading data from serial port and print it to the screen.	
	//
   

	//
	//    while( serial_port.rdbuf()->in_avail() > 0 )

	int iSpeed[3];	
	iSpeed[0] = 0x0020;
	iSpeed[1] = 0x0020;
	iSpeed[2] = 0x0020;

	while(ros::ok())
	{
		ROS_INFO_ONCE("ROS WHILE IS RUNNING");

		char out_buf[8];
		
		//Define data
		out_buf[0] = 0x5a;	//start of frame
		out_buf[1] = 0xaa;	//type
		out_buf[2] = 0x03;	//cmd
		out_buf[3] = (iSpeed[0] & 0xff);		//speed	
		out_buf[4] = (iSpeed[0] >> 8) & 0xff;	//speed
		out_buf[5] = 0x00;	//????
		out_buf[6] = 0x00;	//????
		out_buf[7] = 0x00;	//End of frame

		//write data to serial port.
       	serial_port5.write(out_buf, 8);

		//Define data
		out_buf[0] = 0x5a;	//start of frame
		out_buf[1] = 0xaa;	//type
		out_buf[2] = 0x03;	//cmd
		out_buf[3] = (iSpeed[1] & 0xff);		//speed	
		out_buf[4] = (iSpeed[1] >> 8) & 0xff;	//speed
		out_buf[5] = 0x00;	//????
		out_buf[6] = 0x00;	//????
		out_buf[7] = 0x00;	//End of frame

       	serial_port7.write(out_buf, 8);

		//Define data
		out_buf[0] = 0x5a;	//start of frame
		out_buf[1] = 0xaa;	//type
		out_buf[2] = 0x03;	//cmd
		out_buf[3] = (iSpeed[2] & 0xff);		//speed	
		out_buf[4] = (iSpeed[2] >> 8) & 0xff;	//speed
		out_buf[5] = 0x00;	//????
		out_buf[6] = 0x00;	//????
		out_buf[7] = 0x00;	//End of frame

       	serial_port8.write(out_buf, 8);

		rate.sleep();
	}

//     while(1)
//     {
//         char next_byte;
//         serial_port.get(next_byte);  HERE I RECEIVE THE FIRST ANSWER
//         std::cerr << next_byte;
//
//     }

     return EXIT_SUCCESS ;
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
