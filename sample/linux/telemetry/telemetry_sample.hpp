/*! @file telemetry_sample.hpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Telemetry API usage in a Linux environment.
 *  Shows example usage of the new data subscription API.
 *
 *  @Copyright (c) 2017 DJI
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
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef DJIOSDK_TELEMETRYSAMPLE_HPP
#define DJIOSDK_TELEMETRYSAMPLE_HPP

// System Includes
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <cmath>
#include <time.h>
#include <signal.h>
#include <vector>
// DJI OSDK includes
#include <dji_status.hpp>
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

#define C_EARTH (double)6378137.0
#define DEG2RAD 0.01745329252

bool subscribeToData(DJI::OSDK::Vehicle* vehiclePtr, int responseTimeout, int sockfd);

// Broadcast data implementation for Matrice 100
bool getBroadcastData(DJI::OSDK::Vehicle* vehicle, int responseTimeout, int sockfd);
bool getHeartBeat(int sockfd);

//!@note: All the default timeout parameters are for acknowledgement packets
//! from the aircraft.

/*! Monitored Takeoff
This implementation of takeoff  with monitoring makes sure your aircraft
actually took off and only returns when takeoff is complete.
Use unless you want to do other stuff during takeoff - this will block
the main thread.
!*/
bool monitoredTakeoff(DJI::OSDK::Vehicle* vehiclePtr, int timeout = 1);

// Examples of commonly used Flight Mode APIs

/*! Position Control. Allows you to set an offset from your current
location. The aircraft will move to that position and stay there.
Typical use would be as a building block in an outer loop that does not
require many fast changes, perhaps a few-waypoint trajectory. For smoother
transition and response you should convert your trajectory to attitude
setpoints and use attitude control or convert to velocity setpoints
and use velocity control.
!*/
bool moveByPositionOffset(DJI::OSDK::Vehicle *vehicle, float xOffsetDesired,
	float yOffsetDesired, float zOffsetDesired,
	float yawDesired, float posThresholdInM = 0.2,
	float yawThresholdInDeg = 1.0);

/*! Monitored Landing (Blocking API call). Return status as well as ack.
This version of takeoff makes sure your aircraft actually took off
and only returns when takeoff is complete.

!*/
bool monitoredLanding(DJI::OSDK::Vehicle* vehiclePtr, int timeout = 1);

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
* coordinates.
*
* Accurate when distances are small.
!*/
void localOffsetFromGpsOffset(DJI::OSDK::Vehicle*             vehicle,
	DJI::OSDK::Telemetry::Vector3f& deltaNed,
	void* target, void* origin);

DJI::OSDK::Telemetry::Vector3f toEulerAngle(void* quaternionData);
bool startGlobalPositionBroadcast(DJI::OSDK::Vehicle* vehicle);



// Subscription not supported in Matrice 100
bool setUpSubscription(DJI::OSDK::Vehicle* vehicle, int responseTimeout);
bool teardownSubscription(DJI::OSDK::Vehicle* vehicle, const int pkgIndex,
	int responseTimeout);

bool runWaypointMission(DJI::OSDK::Vehicle* vehicle, uint8_t numWaypoints, 
	int responseTimeout, DJI::OSDK::float32_t lat, DJI::OSDK::float32_t lon, DJI::OSDK::float32_t alt);

void setWaypointDefaults(DJI::OSDK::WayPointSettings* wp);
void setWaypointInitDefaults(DJI::OSDK::WayPointInitSettings* fdata);

std::vector<DJI::OSDK::WayPointSettings> createWaypoints(
	DJI::OSDK::Vehicle* vehicle, DJI::OSDK::float32_t lat, DJI::OSDK::float32_t lon, DJI::OSDK::float32_t alt);

std::vector<DJI::OSDK::WayPointSettings> generateWaypointsPolygon(
	DJI::OSDK::WayPointSettings* start_data, int num_wp, DJI::OSDK::float32_t lat, DJI::OSDK::float32_t lon, DJI::OSDK::float32_t alt);

void uploadWaypoints(DJI::OSDK::Vehicle*                       vehicle,
	std::vector<DJI::OSDK::WayPointSettings>& wp_list,
	int                                       responseTimeout);

bool runHotpointMission(DJI::OSDK::Vehicle* vehicle, int responseTimeout, DJI::OSDK::float32_t lat, DJI::OSDK::float32_t lon, DJI::OSDK::float32_t alt);

const int DEFAULT_PACKAGE_INDEX = 0;


bool
monitoredGoHome(DJI::OSDK::Vehicle* vehiclePtr, int timeout = 1);



#endif // DJIOSDK_TELEMETRYSAMPLE_HPP

