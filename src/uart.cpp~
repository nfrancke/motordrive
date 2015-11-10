#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>

#include "ros/ros.h"

using namespace std;
using namespace LibSerial;

int main(int argc, char **argv  )
{
	//initialize ROS
	ros::init(argc, argv, "uart");

	//define a nodehandle and define the rate for the "ros while loop"
	ros::NodeHandle nh;
	ros::Rate rate(100);

	ROS_INFO("Ros is initialized");


	//
     // Open the serial port.
	//
     SerialStream serial_port;
     char c;
     ROS_DEBUG("Serial Port will be opened");
     serial_port.Open("/dev/ttyS8") ;
	//check if serial port opens correctly
     if ( ! serial_port.good() )
     {
		ROS_ERROR("Error: Could not open serial port.");
     	return(1);
     }

	ROS_INFO("Serial port is opened");

     //
     // Set the baud rate of the serial port.
     //
     serial_port.SetBaudRate( SerialStreamBuf::BAUD_115200 ) ;
     if ( ! serial_port.good() )
     {
		ROS_ERROR("Error: Could not set the baud rate.");
     	return(1) ;
     }
	ROS_INFO("baud rate is setted");

     //
     // Set the number of data bits.
     //
     serial_port.SetCharSize( SerialStreamBuf::CHAR_SIZE_8 ) ;
     if ( ! serial_port.good() )
     {
		ROS_ERROR("Error: Could not set the character size.");
     	return(1) ;
     }
	ROS_INFO("char size is setted");

     //
     // Disable parity.
     //
     serial_port.SetParity( SerialStreamBuf::PARITY_NONE ) ;
     if ( ! serial_port.good() )
     {
		ROS_ERROR("Error: Could not disable the parity.");
     	return(1) ;
     }

	ROS_INFO("parity is setted");
     //
     // Set the number of stop bits.
     //
     serial_port.SetNumOfStopBits( 2 ) ;
     if ( ! serial_port.good() )
     {
		ROS_ERROR("Error: Could not set the number of stop bits.");
		return(1) ;
     }
	ROS_INFO("stop bits are setted");

     //
     // Turn off hardware flow control.
     //
     serial_port.SetFlowControl( SerialStreamBuf::FLOW_CONTROL_NONE ) ;
     if ( ! serial_port.good() )
     {
		ROS_ERROR("Error: Could not use hardware flow control.");
     	return(1) ;
     } 
	ROS_INFO("Disabled flow control");

     //
     // Do not skip whitespace characters while reading from the
     // serial port.
     //
     // serial_port.unsetf( std::ios_base::skipws ) ;
     //
     // Wait for some data to be available at the serial port.
     //
     //
     // Keep reading data from serial port and print it to the screen.
     //
     // Wait for some data to be available at the serial port.
   

  //
  //    while( serial_port.rdbuf()->in_avail() > 0 )

	while(ros::ok())
     {
		char out_buf[8];
		
		//Define data
		out_buf[0] = 0x5a;
		out_buf[1] = 0xaa;
		out_buf[2] = 0x03;
		out_buf[3] = 0x20;
		out_buf[4] = 0x00;
		out_buf[5] = 0x00;
		out_buf[6] = 0x00;
		out_buf[7] = 0x00;

		//write data to serial port.
       	serial_port.write(out_buf, 8);
		
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
