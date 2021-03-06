/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_control.h
 *
 * @brief An example offboard control process via mavlink, definition
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <cmath>
#include <string.h>
#include <inttypes.h>
#include <fstream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

using std::string;
using namespace std;

#include "Mavlink/common/mavlink.h"

#include "autopilot_interface.h"
#include "serial_port.h"
#include "UAV_Database.h"

// ------------------------------------------------------------------------------
//   Payload Object
// ------------------------------------------------------------------------------

class Payload_Drop{ //Made by Alex Winger and Zach Cheben
private:
    double time;			//The time it takes to get to the point we need to release the payload
    double PLong;			//The longitude of the plane
    double PLat;			//The latitude of the plane
    double velocity;		//The velocity of the plane
    double altitude;		//The alt of the plane
    double targetDistance;	//The distance from the plane to the target
    double dropDistance;	//The distance the payload drops once it is dropped
    double distance;		//The distance from the plane to the point the payload should be dropped

public:

    double TLat;			//The latitude of the target
    double TLong;			//The longitude of the target

    Payload_Drop(double tlat, double tlong, double plat, double plong, double a);

    void setGPSinfo(double plat, double plong, double a);		//Sets the GPS information of the plane
    void setVelocity(int vx, int vy, int vz);

    double timeToDrop(); //Returns the time it will take to reach the drop point

    bool near_target(Autopilot_Interface &api, const int &radius); //Rudimentary code to drop payload. Use as last resort


    /*
     * Helper Functions
     */
    vector<Waypoint> payload_waypoints(const int &first_distance, const int &second_distance,
                                       const double &angle); //Returns 3 waypoints
    Waypoint meterDisplacement(const double & deltaX, const double & deltaY, const Waypoint & b);

    double gpsDistance(const double &target_lat, const double &target_long, const double &current_lat,
                       const double &current_long); //Function to convert latitude and longitude into a deistance (Measured in meters)

};



// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------

int main(int argc, char **argv);
int top(int argc, char **argv);

void commands(Autopilot_Interface &autopilot_interface);
void parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate);

// quit handler
Autopilot_Interface *autopilot_interface_quit;
Serial_Port *serial_port_quit;
void quit_handler( int sig );

struct aircraftinfo {

	//Aircraft positions
	// 0: current position, 1: last position (one second in the past), 2: very last position (two seconds in the past)
	float lat [3];
	float lon [3];
	float alt [3];

	//Aircraft velocity and accelerations
	//0: current velocity/acceleration, 1: last velocity (one second in the past)
	float VelocityX [2];
	float VelocityY [2];
	float xAcc;
	float yAcc;

	//Information for the predict function
	//0: Predicted point
	//1: Point that is .01 second in front of the predicted point
	//2: Distance between predicted point 0 and 1
	float futureDistx [3];
	float futureDisty [3];
	//Predicted heading
	float Hdg;
	
	uint8_t priority; 

};


