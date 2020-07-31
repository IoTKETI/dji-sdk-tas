/*! @file telemetry_sample.cpp
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

#include "telemetry_sample.hpp"
#include <typeinfo>

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
using namespace std;

float_t cur_longi = 0.0;
float_t cur_lati = 0.0;
float_t cur_alt = 0.0;
float_t cur_height = 0.0;
float_t cur_velx = 0.0;
float_t cur_vely = 0.0;
float_t cur_velz = 0.0;

//string  data;

uint8_t mav_gp[512];
uint8_t mav_heartbeat[512];

void telemetry_delay_loop_ms(unsigned int timeout_ms) {
	unsigned long timeout_count = (timeout_ms * 30000);
	unsigned long i= 0;
	
	for(i = 0; i < timeout_count; i++) {
	}
}


unsigned short crc_16(const unsigned char* data_p, unsigned char length){
    unsigned char x;
    unsigned short crc = 0xFFFF;

    while (length--){
        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x <<5)) ^ ((unsigned short)x);
    }
    return crc;
}

bool getBroadcastData(DJI::OSDK::Vehicle* vehicle, int responseTimeout, int sockfd)
{
	// Counters
	unsigned long elapsedTimeInMs = 0;
	unsigned long timeToPrintInMs = 30000000; // 1sec

	// We will listen to five broadcast data sets:
	// 1. Flight Status
	// 2. Global Position
	// 3. RC Channels
	// 4. Velocity
	// 5. Quaternion

	// Please make sure your drone is in simulation mode. You can fly the drone with your RC to get different values.
	Telemetry::TimeStamp      timestamp;
	Telemetry::Status         status;
	Telemetry::GlobalPosition globalPosition;
	Telemetry::RC             rc;
	Telemetry::Vector3f       velocity;
	Telemetry::Quaternion     quaternion;
	Telemetry::Battery        battery;
	const int TIMEOUT = 20;

	// Re-set Broadcast frequencies to their default values
	ACK::ErrorCode ack = vehicle->broadcast->setBroadcastFreqDefaults(TIMEOUT);

  // Print in a loop for 2 seconds
	while (elapsedTimeInMs < timeToPrintInMs)
	{
		elapsedTimeInMs++;
		if(elapsedTimeInMs >= timeToPrintInMs) {
			// Matrice 100 broadcasts only flight status
			timestamp      = vehicle->broadcast->getTimeStamp();
			status         = vehicle->broadcast->getStatus();
			globalPosition = vehicle->broadcast->getGlobalPosition();
			rc             = vehicle->broadcast->getRC();
			velocity       = vehicle->broadcast->getVelocity();
			quaternion     = vehicle->broadcast->getQuaternion();
			battery		   = vehicle->broadcast->getBatteryInfo();

			string  data;
			data = "[";
			data += to_string((unsigned)status.flight);
			data += ",";
			data += to_string(timestamp.time_ms);
			data += ",";
			cur_lati = globalPosition.latitude;
			data += to_string((cur_lati*57.295779513082320876798154814));
			data += ",";
			cur_longi = globalPosition.longitude;
			data += to_string((cur_longi*57.295779513082320876798154814));
			data += ",";
			data += to_string(globalPosition.altitude);
			data += ",";
			data += to_string(globalPosition.height);
			data += ",";
			data += to_string(rc.roll);
			data += ",";
			data += to_string(rc.pitch);
			data += ",";
			data += to_string(rc.yaw);
			data += ",";
			//data += to_string(rc.throttle);
			//data += ",";
			data += to_string(velocity.x);
			data += ",";
			data += to_string(velocity.y);
			data += ",";
			data += to_string(velocity.z);
			//data += "\n";
			data += ",";
			data += to_string(battery.percentage);
			data += "]";
			// data += to_string(quaternion.q0);
			// data += ",";
			// data += to_string(quaternion.q1);
			// data += ",";
			// data += to_string(quaternion.q2);
			// data += ",";
			// data += to_string(quaternion.q3);
		
			//std::cout << data << "\n";

			char *buffer = NULL;
			buffer = (char*)malloc(strlen(data.c_str())+1);
			strcpy(buffer,data.c_str());

			send(sockfd, buffer, strlen(buffer), MSG_DONTWAIT);
			free(buffer);

			//std::cout << "-----send ---------------------------------" << "\n";
		
		//    std::cout << "Counter = " << elapsedTimeInMs << ":\n";
			//std::cout << "-------\n";
			//std::cout << "Flight Status                         = " << (unsigned)status.flight << "\n";
			//std::cout << "Time Stamp                            = " << (unsigned)timestamp.time_ms << "\n";
			//std::cout << "Position              (LLA)           = "
				//<< (globalPosition.latitude*57.295779513082320876798154814) << ", " << (globalPosition.longitude*57.295779513082320876798154814)
					  //<< ", " << globalPosition.altitude << ", " << globalPosition.height << "\n";
			//std::cout << "RC Commands           (r/p/y/thr)     = " << rc.roll << ", "
					  //<< rc.pitch << ", " << rc.yaw << ", " << rc.throttle << "\n";
			//std::cout << "Velocity              (vx,vy,vz)      = " << velocity.x
					  //<< ", " << velocity.y << ", " << velocity.z << "\n";
			//std::cout << "Battery                               = "
					  //<< (unsigned)battery.percentage << "\n";
			//std::cout << "Attitude Quaternion   (w,x,y,z)       = " << quaternion.q0
			//		  << ", " << quaternion.q1 << ", " << quaternion.q2 << ", "
			//		 << quaternion.q3 << "\n";
			//std::cout << "-------\n\n";
		
	//#endif
			//delay_loop(500);
			//sleep(500000);
			//elapsedTimeInMs += 125;
		}
	}
	
	
 
//  std::cout << "Done printing!\n";
	return true;
}


// bool getBroadcastData(DJI::OSDK::Vehicle* vehicle, int responseTimeout, int sockfd)
// {
// 	// Counters
// 	int elapsedTimeInMs = 0;
// 	int timeToPrintInMs = 125;

// 	// We will listen to five broadcast data sets:
// 	// 1. Flight Status
// 	// 2. Global Position
// 	// 3. RC Channels
// 	// 4. Velocity
// 	// 5. Quaternion

// 	// Please make sure your drone is in simulation mode. You can
// 	// fly the drone with your RC to get different values.
// 	Telemetry::TimeStamp      timestamp;
// 	Telemetry::Status         status;
// 	Telemetry::GlobalPosition globalPosition;
// 	Telemetry::RC             rc;
// 	Telemetry::Vector3f       velocity;
// 	Telemetry::Quaternion     quaternion;
// 	Telemetry::Battery        battery;
// 	const int TIMEOUT = 20;

// 	// Re-set Broadcast frequencies to their default values
// 	ACK::ErrorCode ack = vehicle->broadcast->setBroadcastFreqDefaults(TIMEOUT);

//   // Print in a loop for 2 seconds
// 	while ( tt >= timeToPrintInMs)
// 	{
// 		//std::cout << "======================Send DATA ==============================\n";
// 		// Matrice 100 broadcasts only flight status
// 		timestamp      = vehicle->broadcast->getTimeStamp();
// 		status         = vehicle->broadcast->getStatus();
// 		globalPosition = vehicle->broadcast->getGlobalPosition();
// 		rc             = vehicle->broadcast->getRC();
// 		velocity       = vehicle->broadcast->getVelocity();
// 		quaternion     = vehicle->broadcast->getQuaternion();
// 		battery		   = vehicle->broadcast->getBatteryInfo();

// 	//	bzero(buffer, 256);
// 	//	buffer += (unsigned)status.flight;
// 	//	buffer +=  "\n";
// 	//        buffer +=  globalPosition.latitude ;
// 	//        buffer +=  ", " ;
// 	//        buffer +=  globalPosition.longitude + ", " + globalPosition.altitude + "\n"+
// 	//    rc.roll + ", " + rc.pitch + ", " + rc.yaw + ", " + rc.throttle + "\n" +
// 	//    velocity.x + ", " + velocity.y + ", " + velocity.z + "\n" +
// 	//    quaternion.q0 + ", " + quaternion.q1 + ", " + quaternion.q2  + ", " + quaternion.q3 + "\n";

		
// 	   // int abc = (unsigned)status.flight;
// 	//    std::cout << abc << "\n";


// 	//  string data[2];
// 	//  data[0] = to_string(globalPosition.latitude);
// 	//  data[1] = to_string(globalPosition.longitude);
// 		/*
// 		cur_lati = ((globalPosition.latitude)*57.295779513082320876798154814)*100000000;
// 		cur_longi = ((globalPosition.longitude)*57.295779513082320876798154814)*10000000;	
// 		cur_alt = (globalPosition.altitude)*1000;
// 		cur_height = (globalPosition.height)*1000;
// 		cur_velx = (velocity.x)*100;
// 		cur_vely = (velocity.y)*100;
// 		cur_velz = (velocity.z)*100;
		
// 		// mav version 1
// 		// Global Position
// 		uint16_t idx = 0;
// 		static uint8_t seq = 0;
// 		mav_gp[idx++] = 0xfe;
// 	  	mav_gp[idx++] = 28; // length
// 		mav_gp[idx++] = seq++; // seq
// 		mav_gp[idx++] = 0x65; // sys_id
// 	  	mav_gp[idx++] = 0x01; // comp_id
// 		mav_gp[idx++] = 0x21; // msg_id
		
// 		printf("data : %f,\t%f,\t%f,\t%f\n%f,\t%f,\t%f,\t\r\n", cur_lati, cur_longi, cur_alt, cur_height, cur_velx, cur_vely, cur_velz);
// 		std::cout << "Position              (LLAH)           = "
// 			<< (globalPosition.latitude*57.295779513082320876798154814) << ", " << (globalPosition.longitude*57.295779513082320876798154814)
// 				  << ", " << globalPosition.altitude << ", " << globalPosition.height << "\n";
// 		std::cout << "Velocity              (vx,vy,vz)      = " << velocity.x
// 				  << ", " << velocity.y << ", " << velocity.z << "\n";
		
// 		// payload
// 		memcpy(&mav_gp[idx], (uint8_t *)&timestamp.time_ms, 4);
// 		idx += 4;
// 		memcpy(&mav_gp[idx], (uint8_t *)&cur_lati, 4);
// 		idx += 4;
// 		memcpy(&mav_gp[idx], (uint8_t *)&cur_longi, 4);
// 		idx += 4;
// 		memcpy(&mav_gp[idx], (uint8_t *)&cur_alt, 4);
// 		idx += 4;
// 		memcpy(&mav_gp[idx], (uint8_t *)&cur_height, 4);
// 		idx += 4;
// 		memcpy(&mav_gp[idx], (uint8_t *)&cur_velx, 2);
// 		idx += 2;
// 		memcpy(&mav_gp[idx], (uint8_t *)&cur_vely, 2);
// 		idx += 2;
// 		memcpy(&mav_gp[idx], (uint8_t *)&cur_velz, 2);
// 		idx += 2;

// 		mav_gp[idx++] = 0x00; // heading low
// 		mav_gp[idx++] = 0x00; // heading high

// 		unsigned short crc16_val = 0;
// 		crc16_val = crc_16(&mav_gp[1], idx-1);

// 		mav_gp[idx++] = (uint8_t) ((crc16_val & 0xff00) >> 8);
// 		mav_gp[idx++] = (uint8_t) ((crc16_val & 0x00ff));
		
// 		send(sockfd, mav_gp, idx, MSG_DONTWAIT);
		
// 		// HeartBeat
// 		uint16_t idx_2 = 0;
// 		static uint8_t seq_2 = 0;
// 		mav_heartbeat[idx_2++] = 0xfe;
// 	  	mav_heartbeat[idx_2++] = 9; // length
// 		mav_heartbeat[idx_2++] = seq++; // seq
// 		mav_heartbeat[idx_2++] = 0x65; // sys_id
// 	  	mav_heartbeat[idx_2++] = 0x01; // comp_id
// 		mav_heartbeat[idx_2++] = 0x00; // msg_id
		
// 		// payload
// 		mav_heartbeat[idx_2++] = 0x00; 
// 		mav_heartbeat[idx_2++] = 0x00; 
// 		mav_heartbeat[idx_2++] = 0x00; 
// 		mav_heartbeat[idx_2++] = 0x00; 
// 		mav_heartbeat[idx_2++] = 0x02; 
// 		mav_heartbeat[idx_2++] = 0x03; 
// 		mav_heartbeat[idx_2++] = 0xd1; 
// 		mav_heartbeat[idx_2++] = 0x03; 
// 		mav_heartbeat[idx_2++] = 0x03; 
		
// 		unsigned short crc16_val_2 = 0;
// 		crc16_val_2 = crc_16(&mav_heartbeat[1], idx_2-1);

// 		mav_heartbeat[idx_2++] = (uint8_t) ((crc16_val_2 & 0xff00) >> 8);
// 		mav_heartbeat[idx_2++] = (uint8_t) ((crc16_val_2 & 0x00ff));
		
// 		send(sockfd, mav_heartbeat, idx_2, MSG_DONTWAIT);
// */
// //#if 0

// 		string  data;
// 		data = "[";
// 		data += to_string((unsigned)status.flight);
// 		data += ",";
// 		data += to_string(timestamp.time_ms);
// 		data += ",";
// 		cur_lati = globalPosition.latitude;
// 		data += to_string((cur_lati*57.295779513082320876798154814));
// 		data += ",";
// 		cur_longi = globalPosition.longitude;
// 		data += to_string((cur_longi*57.295779513082320876798154814));
// 		data += ",";
// 		data += to_string(globalPosition.altitude);
// 		data += ",";
// 		data += to_string(globalPosition.height);
// 		data += ",";
// 		data += to_string(rc.roll);
// 		data += ",";
// 		data += to_string(rc.pitch);
// 		data += ",";
// 		data += to_string(rc.yaw);
// 		data += ",";
// 		//data += to_string(rc.throttle);
// 		//data += ",";
// 		data += to_string(velocity.x);
// 		data += ",";
// 		data += to_string(velocity.y);
// 		data += ",";
// 		data += to_string(velocity.z);
// 		//data += "\n";
// 		data += ",";
// 		data += to_string(battery.percentage);
// 		data += "]";
// 		// data += to_string(quaternion.q0);
// 		// data += ",";
// 		// data += to_string(quaternion.q1);
// 		// data += ",";
// 		// data += to_string(quaternion.q2);
// 		// data += ",";
// 		// data += to_string(quaternion.q3);
	
// 		//std::cout << data << "\n";

// 		char *buffer = NULL;
// 		buffer = (char*)malloc(strlen(data.c_str())+1);
// 		strcpy(buffer,data.c_str());

// 	//	std::cout <<  buffer << "\n";
// 	//	data += to_string(globalPosition.latitude);
// 	//	data += to_string(globalPosition.longitude);
// 	//	data += to_string(globalPosition.altitude);
// 	//	data += to_string(rc.roll);
// 		/*
// 		data[5] = to_string(rc.pitch);
// 		data[6] = to_string(rc.yaw);
// 		data[7] = to_string(rc.throttle);
// 		data[8] = to_string(velocity.x);
// 		data[9] = to_string(velocity.y);
// 		data[10] = to_string(velocity.z);
// 		data[11] = to_string(quaternion.q0);
// 		data[12] = to_string(quaternion.q1);
// 		data[13] = to_string(quaternion.q2);
// 		data[14] = to_string(quaternion.q3);

// 	*/

// 	/*
// 		string str = to_string((unsigned)status.flight);
// 		std::cout << str << ":\n";
// 		string str1 = to_string(globalPosition.latitude);
// 		std::cout << str1 << ":\n";
// 		string str2 = to_string(globalPosition.altitude);
// 		std::cout << str2 << ":\n";
// 		string str3 = to_string(velocity.z);
// 		std::cout << str3 << ":\n";
// 		string str4 = to_string(quaternion.q0);
// 		std::cout << str4 << ":\n";
// 	*/

// 		//std::cout << buffer << "\n";

// 		send(sockfd, buffer, strlen(buffer), MSG_DONTWAIT);
// 		free(buffer);

// 	    std::cout << "-----send ---------------------------------" << "\n";
	
// 	//    std::cout << "Counter = " << elapsedTimeInMs << ":\n";
// 		//std::cout << "-------\n";
// 		std::cout << "Flight Status                         = "
// 				  << (unsigned)status.flight << "\n";
// 		std::cout << "Time Stamp                            = "
// 				  << (unsigned)timestamp.time_ms << "\n";
// 		std::cout << "Position              (LLA)           = "
// 			<< (globalPosition.latitude*57.295779513082320876798154814) << ", " << (globalPosition.longitude*57.295779513082320876798154814)
// 				  << ", " << globalPosition.altitude << ", " << globalPosition.height << "\n";
// 		std::cout << "RC Commands           (r/p/y/thr)     = " << rc.roll << ", "
// 				  << rc.pitch << ", " << rc.yaw << ", " << rc.throttle << "\n";
// 		std::cout << "Velocity              (vx,vy,vz)      = " << velocity.x
// 				  << ", " << velocity.y << ", " << velocity.z << "\n";
// 		std::cout << "Battery                               = "
// 				  << (unsigned)battery.percentage << "\n";
// 		//std::cout << "Attitude Quaternion   (w,x,y,z)       = " << quaternion.q0
// 		//		  << ", " << quaternion.q1 << ", " << quaternion.q2 << ", "
// 		//		 << quaternion.q3 << "\n";
// 		//std::cout << "-------\n\n";
	  	
// //#endif
// 	//    usleep(10000);
// 		//    elapsedTimeInMs += 5;
// 		tt = 0;
// 	}
	
// 	tt += 10;   
	
	
 
// //  std::cout << "Done printing!\n";
// 	return true;
// }
/*
bool
getHeartBeat(int sockfd)
{	
	// Counters
	int elapsedTimeInMs = 0;
	int timeToPrintInMs = 500;

  // Print in a loop for 2 seconds
	while ( tt >= timeToPrintInMs)
	{
		// HeartBeat
		uint16_t idx_2 = 0;
		static uint8_t seq_2 = 0;
		mav_heartbeat[idx_2++] = 0xfe;
	  	mav_heartbeat[idx_2++] = 9; // length
		mav_heartbeat[idx_2++] = seq_2++; // seq
		mav_heartbeat[idx_2++] = 0x65; // sys_id
	  	mav_heartbeat[idx_2++] = 0x01; // comp_id
		mav_heartbeat[idx_2++] = 0x00; // msg_id
		
		// payload
		mav_heartbeat[idx_2++] = 0x00; 
		mav_heartbeat[idx_2++] = 0x00; 
		mav_heartbeat[idx_2++] = 0x00; 
		mav_heartbeat[idx_2++] = 0x00; 
		mav_heartbeat[idx_2++] = 0x02; 
		mav_heartbeat[idx_2++] = 0x03; 
		mav_heartbeat[idx_2++] = 0xd1; 
		mav_heartbeat[idx_2++] = 0x03; 
		mav_heartbeat[idx_2++] = 0x03; 
		
		unsigned short crc16_val_2 = 0;
		crc16_val_2 = crc_16(&mav_heartbeat[1], idx_2-1);

		mav_heartbeat[idx_2++] = (uint8_t) ((crc16_val_2 & 0xff00) >> 8);
		mav_heartbeat[idx_2++] = (uint8_t) ((crc16_val_2 & 0x00ff));
		
		send(sockfd, mav_heartbeat, idx_2, MSG_DONTWAIT);

		tt = 0;
	}
	
	tt += 10;   

	return true;
}
*/
bool subscribeToData(Vehicle* vehicle, int responseTimeout , int sockfd)
{
	// RTK can be detected as unavailable only for Flight controllers that don't support RTK
	bool rtkAvailable = false;
	// Counters
	int elapsedTimeInMs = 0;
	int timeToPrintInMs = 2000;

	// We will subscribe to six kinds of data:
	// 1. Flight Status at 1 Hz
	// 2. Fused Lat/Lon at 10Hz
	// 3. Fused Altitude at 10Hz
	// 4. RC Channels at 50 Hz
	// 5. Velocity at 50 Hz
	// 6. Quaternion at 200 Hz

	// Please make sure your drone is in simulation mode. You can fly the drone
	// with your RC to
	// get different values.

	// Telemetry: Verify the subscription
	ACK::ErrorCode subscribeStatus;
	subscribeStatus = vehicle->subscribe->verify(responseTimeout);
	if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
		ACK::getErrorCodeMessage(subscribeStatus, __func__);
		return false;
	}

	// Package 0: Subscribe to flight status at freq 1 Hz
	int       pkgIndex        = 0;
	int       freq            = 1;
	TopicName topicList1Hz[]  = { TOPIC_STATUS_FLIGHT };
	int       numTopic        = sizeof(topicList1Hz) / sizeof(topicList1Hz[0]);
	bool      enableTimestamp = false;

	bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
	pkgIndex, numTopic, topicList1Hz, enableTimestamp, freq);
	if (!(pkgStatus)) {
		return pkgStatus;
	}
	
	subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
	if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
		ACK::getErrorCodeMessage(subscribeStatus, __func__);
		// Cleanup before return
		vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
		return false;
	}

	// Package 1: Subscribe to Lat/Lon, and Alt at freq 10 Hz
	pkgIndex                  = 1;
	freq                      = 10;
	TopicName topicList10Hz[] = { TOPIC_GPS_FUSED, TOPIC_ALTITUDE_FUSIONED};
	numTopic                  = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
	enableTimestamp           = false;

	pkgStatus = vehicle->subscribe->initPackageFromTopicList(
	pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
	if (!(pkgStatus)) {
		return pkgStatus;
	}
	
	subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
	if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
		ACK::getErrorCodeMessage(subscribeStatus, __func__);
		// Cleanup before return
		vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
		return false;
	}

	// Package 2: Subscribe to RC Channel and Velocity at freq 50 Hz
	pkgIndex                  = 2;
	freq                      = 50;
	TopicName topicList50Hz[] = { TOPIC_RC, TOPIC_VELOCITY };
	numTopic                  = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
	enableTimestamp           = false;

	pkgStatus = vehicle->subscribe->initPackageFromTopicList(pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
	if (!(pkgStatus)) {
		return pkgStatus;
	}
	
	subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
	if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
		ACK::getErrorCodeMessage(subscribeStatus, __func__);
		// Cleanup before return
		vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
		return false;
	}

	// Package 3: Subscribe to Quaternion at freq 200 Hz.
	pkgIndex                   = 3;
	freq                       = 200;
	TopicName topicList200Hz[] = { TOPIC_QUATERNION };
	numTopic        = sizeof(topicList200Hz) / sizeof(topicList200Hz[0]);
	enableTimestamp = false;

	pkgStatus = vehicle->subscribe->initPackageFromTopicList(
	pkgIndex, numTopic, topicList200Hz, enableTimestamp, freq);
	if (!(pkgStatus))
	{
		return pkgStatus;
	}
	
	subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
	if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
	{
		ACK::getErrorCodeMessage(subscribeStatus, __func__);
		// Cleanup before return
		vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
		return false;
	}

	// Package 4: Subscribe to RTK at freq 5 Hz.
	pkgIndex                   = 4;
	freq                       = 5;
	TopicName topicListRTK5Hz[] = {TOPIC_RTK_POSITION, TOPIC_RTK_YAW_INFO,
								  TOPIC_RTK_POSITION_INFO, TOPIC_RTK_VELOCITY,
								  TOPIC_RTK_YAW};
	numTopic        = sizeof(topicListRTK5Hz) / sizeof(topicListRTK5Hz[0]);
	enableTimestamp = false;

	pkgStatus = vehicle->subscribe->initPackageFromTopicList(pkgIndex, numTopic, topicListRTK5Hz, enableTimestamp, freq);
	if (!(pkgStatus)) {
		return pkgStatus;
	}
	else {
		subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
		if(subscribeStatus.data == ErrorCode::SubscribeACK::SOURCE_DEVICE_OFFLINE) {
			std::cout << "RTK Not Available" << "\n";
			rtkAvailable = false;
		}
		else {
			rtkAvailable = true;
			if (ACK::getError(subscribeStatus) != ACK::SUCCESS) {
				ACK::getErrorCodeMessage(subscribeStatus, __func__);
				// Cleanup before return
				vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
				return false;
			}
		}
	}

	// Wait for the data to start coming in.
	//sleep(1);
	telemetry_delay_loop_ms(1000);

	// Get all the data once before the loop to initialize vars
	TypeMap<TOPIC_STATUS_FLIGHT>::type     flightStatus;
	TypeMap<TOPIC_GPS_FUSED>::type         latLon;
	TypeMap<TOPIC_ALTITUDE_FUSIONED>::type altitude;
	TypeMap<TOPIC_RC>::type                rc;
	TypeMap<TOPIC_VELOCITY>::type          velocity;
	TypeMap<TOPIC_QUATERNION>::type        quaternion;
	TypeMap<TOPIC_RTK_POSITION>::type      rtk;
	TypeMap<TOPIC_RTK_POSITION_INFO>::type rtk_pos_info;
	TypeMap<TOPIC_RTK_VELOCITY>::type      rtk_velocity;
	TypeMap<TOPIC_RTK_YAW>::type           rtk_yaw;
	TypeMap<TOPIC_RTK_YAW_INFO>::type      rtk_yaw_info;


	// Print in a loop for 2 sec
	while (elapsedTimeInMs < timeToPrintInMs)
	{
		flightStatus = vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>();
		latLon       = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
		altitude     = vehicle->subscribe->getValue<TOPIC_ALTITUDE_FUSIONED>();
		rc           = vehicle->subscribe->getValue<TOPIC_RC>();
		velocity     = vehicle->subscribe->getValue<TOPIC_VELOCITY>();
		quaternion   = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
		if(rtkAvailable) {
			rtk = vehicle->subscribe->getValue<TOPIC_RTK_POSITION>();
			rtk_pos_info = vehicle->subscribe->getValue<TOPIC_RTK_POSITION_INFO>();
			rtk_velocity = vehicle->subscribe->getValue<TOPIC_RTK_VELOCITY>();
			rtk_yaw = vehicle->subscribe->getValue<TOPIC_RTK_YAW>();
			rtk_yaw_info = vehicle->subscribe->getValue<TOPIC_RTK_YAW_INFO>();
		}
		
		std::cout << "Counter = " << elapsedTimeInMs << ":\n";
		std::cout << "-------\n";
		std::cout << "Flight Status                         = " << (int)flightStatus
				  << "\n";
		std::cout << "Position              (LLA)           = " << latLon.latitude
				  << ", " << latLon.longitude << ", " << altitude << "\n";
		std::cout << "RC Commands           (r/p/y/thr)     = " << rc.roll << ", "
				  << rc.pitch << ", " << rc.yaw << ", " << rc.throttle << "\n";
		std::cout << "Velocity              (vx,vy,vz)      = " << velocity.data.x
				  << ", " << velocity.data.y << ", " << velocity.data.z << "\n";
		std::cout << "Attitude Quaternion   (w,x,y,z)       = " << quaternion.q0
				  << ", " << quaternion.q1 << ", " << quaternion.q2 << ", "
				  << quaternion.q3 << "\n";
		if(rtkAvailable) {
		  std::cout << "RTK if available   (lat/long/alt/velocity_x/velocity_y/velocity_z/yaw/yaw_info/pos_info) ="
					<< rtk.latitude << "," << rtk.longitude << "," << rtk.HFSL << "," << rtk_velocity.x << ","
					<< rtk_velocity.y
					<< "," << rtk_velocity.z << "," << rtk_yaw << "," << rtk_yaw_info << rtk_pos_info << "\n";
		}
		std::cout << "-------\n\n";
		//usleep(5000);
		telemetry_delay_loop_ms(5);
		elapsedTimeInMs += 5;
	}

	std::cout << "Done printing!\n";
	vehicle->subscribe->removePackage(0, responseTimeout);
	vehicle->subscribe->removePackage(1, responseTimeout);
	vehicle->subscribe->removePackage(2, responseTimeout);
	vehicle->subscribe->removePackage(3, responseTimeout);
	vehicle->subscribe->removePackage(4, responseTimeout);

	return true;
}

