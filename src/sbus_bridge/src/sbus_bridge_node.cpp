/*
* Copyright 2018 Jens Willy Johannsen <jens@jwrobotics.com>, JW Robotics
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*
* SBUS serial node
*
* Publishes:
*	/sbus
*/

#include "rclcpp/rclcpp.hpp"
#include "sbus_bridge/sbus_serial_driver.h"
#include "sbus_interface/msg/sbus.hpp"
#include "rcutils/logging.h"
#include <algorithm>
#include <chrono>
#include <boost/algorithm/clamp.hpp>

using namespace std::chrono_literals;


int main( int argc, char **argv )
{
	rclcpp::init(argc, argv);
	auto nh = rclcpp::Node::make_shared("sbus_serial_node");

	// Read/set parameters
	std::string port;
	int refresh_rate_hr;
	int rxMinValue;
	int rxMaxValue;
	int outMinValue;
	int outMaxValue;
	bool silentOnFailsafe;
	int enableChannelNum;
	double enableChannelProportionalMin;
	double enableChannelProportionalMax;

	port = nh->declare_parameter<std::string>("port", "/dev/ttyTHS2");
	refresh_rate_hr = nh->declare_parameter<int>("refresh_rate_hz", 5);
	rxMinValue = nh->declare_parameter<int>("rxMinValue", 172);
	rxMaxValue = nh->declare_parameter<int>("rxMaxValue", 1811);
	outMinValue = nh->declare_parameter<int>("outMinValue", 0);
	outMaxValue = nh->declare_parameter<int>("outMaxValue", 255);
	silentOnFailsafe = nh->declare_parameter<bool>("silentOnFailsafe", false);
	// Parameters for "enable channel". If channel number is -1, no enable channel is used.
	enableChannelNum = nh->declare_parameter<int>("enableChannelNum", -1);
	enableChannelProportionalMin = nh->declare_parameter<double>("enableChannelProportionalMin", -1.0);
	enableChannelProportionalMax = nh->declare_parameter<double>("enableChannelProportionalMax", -1.0);

	// Used for mapping raw values
	float rawSpan = static_cast<float>(rxMaxValue-rxMinValue);
	float outSpan = static_cast<float>(outMaxValue-outMinValue);

//	ros::Publisher pub = nh.advertise<sbus_serial::Sbus>( "sbus", 100 );
	auto pub = nh->create_publisher<sbus_interface::msg::Sbus>("sbus", 100);
	auto loop_rate = rclcpp::Rate((1.0 / refresh_rate_hr) * 1e9);

	// Initialize SBUS port (using pointer to have only the initialization in the try-catch block)
	sbus_serial::SBusSerialPort *sbusPort;
	try {
		sbusPort = new sbus_serial::SBusSerialPort( port, true );
	}
	catch( ... ) {
		// TODO: add error message in exception and report
		RCLCPP_ERROR(nh->get_logger(), "Unable to initalize SBUS port");
		return 1;
	}

	// Create Sbus message instance and set invariant properties. Other properties will be set in the callback lambda
	sbus_interface::msg::Sbus sbus;
	sbus.header.stamp = rclcpp::Time(0);//  ros::Time( 0 );

	// Callback (auto-capture by reference)
	auto callback = [&]( const sbus_serial::SBusMsg sbusMsg ) {
		// First check if we should be silent on failsafe and failsafe is set. If so, do nothing
		if( silentOnFailsafe && sbusMsg.failsafe ){
			RCLCPP_INFO(nh->get_logger(), "SBUS fail safe detected.");
			return;
		}

		// Next check if we have an "enable channel" specified. If so, return immediately if the value of the specified channel is outside of the specified min/max
		if( enableChannelNum >= 1 && enableChannelNum <= 16 ) {
			double enableChannelProportionalValue = (sbusMsg.channels[ enableChannelNum-1 ] - rxMinValue) / rawSpan;
			if( enableChannelProportionalValue < enableChannelProportionalMin || enableChannelProportionalValue > enableChannelProportionalMax )
				return;
		}

		sbus.header.stamp = nh->now();
		sbus.frame_lost = sbusMsg.frame_lost;
		sbus.failsafe = sbusMsg.failsafe;

		// Assign raw channels
		std::transform( sbusMsg.channels.begin(), sbusMsg.channels.end(), sbus.raw_channels.begin(), [&]( uint16_t rawChannel ) {
			return boost::algorithm::clamp( rawChannel, rxMinValue, rxMaxValue );   // Clamp to min/max raw values
		} );

		// Map to min/max values
		std::transform( sbusMsg.channels.begin(), sbusMsg.channels.end(), sbus.mapped_channels.begin(), [&]( uint16_t rawChannel ) {
			int16_t mappedValue = (rawChannel - rxMinValue) / rawSpan * outSpan + outMinValue;
			return boost::algorithm::clamp( mappedValue, outMinValue, outMaxValue );        // Clamp to min/max output values
		} );
	};
	sbusPort->setCallback( callback );

	RCLCPP_INFO(nh->get_logger(), "SBUS node started...");

	rclcpp::Time lastPublishedTimestamp = nh->now();
	while(rclcpp::ok())
	{
		// Only publish if we have a new sample
		if( lastPublishedTimestamp != sbus.header.stamp ) {
//			RCLCPP_INFO(nh->get_logger(), "PUB");
			pub->publish( sbus );
			lastPublishedTimestamp = sbus.header.stamp;
		}
		//RCLCPP_INFO(nh->get_logger(), "SPIN");

		rclcpp::spin_some(nh);
		loop_rate.sleep();
	}

	delete sbusPort;        // Cleanup
	return 0;
}
