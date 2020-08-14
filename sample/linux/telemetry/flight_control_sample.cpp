/*! @file flight_control_sample.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Flight Control API usage in a Linux environment.
 *  Provides a number of helpful additions to core API calls,
 *  especially for position control, attitude control, takeoff,
 *  landing.
 *
 *  @copyright
 *  2016-17 DJI. All rights reserved.
 * */

#include "telemetry_sample.hpp"

uint32_t loop_count = 0;

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

void flight_delay_loop_ms(unsigned int timeout_ms) {
	unsigned long timeout_count = (timeout_ms * 30000);
	unsigned long i= 0;
	
	for(i = 0; i < timeout_count; i++) {
	}
}


/*! Monitored Takeoff (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/
bool monitoredTakeoff(Vehicle *vehicle, int timeout)
{
    //@todo: remove this once the getErrorCode function signature changes
    char func[50];
    int pkgIndex;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        // Telemetry: Verify the subscription
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = vehicle->subscribe->verify(timeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            return false;
        }

        // Telemetry: Subscribe to flight status and mode at freq 10 Hz
        pkgIndex = 0;
        int freq = 10;
        TopicName topicList10Hz[] = {TOPIC_STATUS_FLIGHT,
                                     TOPIC_STATUS_DISPLAYMODE};
        int numTopic = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
        bool enableTimestamp = false;

        bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
            pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
        if (!(pkgStatus))
        {
            return pkgStatus;
        }
        subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, timeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            // Cleanup before return
            vehicle->subscribe->removePackage(pkgIndex, timeout);
            return false;
        }
    }

    // Start takeoff
    ACK::ErrorCode takeoffStatus = vehicle->control->takeoff(timeout);
    if (ACK::getError(takeoffStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(takeoffStatus, func);
        return false;
    }

    // First check: Motors started
    int motorsNotStarted = 0;
    int timeoutCycles = 20;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        loop_count = 0;
        while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() != VehicleStatus::FlightStatus::ON_GROUND &&
               vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_ENGINE_START &&
               motorsNotStarted < timeoutCycles)
        {
            motorsNotStarted++;
            usleep(10000);
            //flight_delay_loop_ms(10);
            printf("GGGGG- %ld", loop_count);
        }

        if (motorsNotStarted >= timeoutCycles)
        {
            std::cout << "Takeoff failed. Motors are not spinning." << std::endl;
            // Cleanup
            if (!vehicle->isM100() && !vehicle->isLegacyM600())
            {
                vehicle->subscribe->removePackage(0, timeout);
            }
            return false;
        }
        else
        {
            std::cout << "Motors spinning...\n";
        }
    }
    else if (vehicle->isLegacyM600())
    {
        loop_count = 0;
        while ((vehicle->broadcast->getStatus().flight < DJI::OSDK::VehicleStatus::FlightStatus::ON_GROUND) &&
               motorsNotStarted < timeoutCycles)
        {
            motorsNotStarted++;
            usleep(10 * 1000);
            //flight_delay_loop_ms(100);
            printf("FFFFF- %ld\n", loop_count);
        }

        if (motorsNotStarted < timeoutCycles)
        {
            std::cout << "Successful TakeOff!" << std::endl;
        }
    }
    else // M100
    {
        loop_count = 0;
        while ((vehicle->broadcast->getStatus().flight < DJI::OSDK::VehicleStatus::M100FlightStatus::TAKEOFF) &&
               motorsNotStarted < timeoutCycles)
        {
            motorsNotStarted++;
            usleep(10 * 1000);
            //flight_delay_loop_ms(100);
            //loop_count++;
            //if(loop_count > 16384) {
            //loop_count = 0;
            //break;
            //}
            printf("HHHHH- %ld\n", loop_count);
        }

        if (motorsNotStarted < timeoutCycles)
        {
            std::cout << "Successful TakeOff!" << std::endl;
        }
    }

    // Second check: In air
    int stillOnGround = 0;
    timeoutCycles = 110;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        loop_count = 0;
        while (vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() != VehicleStatus::FlightStatus::IN_AIR &&
               (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
                vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
               stillOnGround < timeoutCycles)
        {
            stillOnGround++;
            usleep(10 * 1000);
            //flight_delay_loop_ms(100);
            //loop_count++;
            //if(loop_count > 16384) {
            //loop_count = 0;
            //break;
            //}
            printf("IIIII- %ld\n", loop_count);
        }

        if (stillOnGround >= timeoutCycles)
        {
            std::cout << "Takeoff failed. Aircraft is still on the ground, but the motors are spinning." << std::endl;

            // Cleanup
            if (!vehicle->isM100() && !vehicle->isLegacyM600())
            {
                vehicle->subscribe->removePackage(0, timeout);
            }
            return false;
        }
        else
        {
            std::cout << "Ascending...\n";
        }
    }
    else if (vehicle->isLegacyM600())
    {
        loop_count = 0;
        while ((vehicle->broadcast->getStatus().flight < DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR) &&
               stillOnGround < timeoutCycles)
        {
            stillOnGround++;
            usleep(100*1000);
            //flight_delay_loop_ms(100);
            //loop_count++;
            //if(loop_count > 16384) {
            //loop_count = 0;
            //break;
            //}
            printf("JJJJJ- %ld\n", loop_count);
        }

        if (stillOnGround < timeoutCycles)
        {
            std::cout << "Aircraft in air!" << std::endl;
        }
    }
    else // M100
    {
        stillOnGround = 0;
        loop_count = 0;
        while ((vehicle->broadcast->getStatus().flight != DJI::OSDK::VehicleStatus::M100FlightStatus::IN_AIR_STANDBY) &&
               stillOnGround < timeoutCycles)
        {
            stillOnGround++;
            usleep(100*1000);
            //flight_delay_loop_ms(100);
            //loop_count++;
            //if(loop_count > 16384) {
            //loop_count = 0;
            //break;
            //}
            printf("KKKKK- %ld\n", loop_count);
        }

        if (stillOnGround < timeoutCycles)
        {
            printf("Aircraft in air!\n");
        }
    }

    // Final check: Finished takeoff
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        timeoutCycles = 50;
        loop_count = 0;
        while ((vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() == VehicleStatus::DisplayMode::MODE_ASSISTED_TAKEOFF ||
               vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() == VehicleStatus::DisplayMode::MODE_AUTO_TAKEOFF) &&
               loop_count < timeoutCycles)
        {
            usleep(1*1000);
            //flight_delay_loop_ms(100);
            loop_count++;
            printf("LLLLL- %ld\n", loop_count);
        }

        // if (loop_count >= timeoutCycles)
        // {
        //     std::cout << "Takeoff failed. mode is MODE_ASSISTED_TAKEOFF, MODE_AUTO_TAKEOFF yet." << std::endl;

        //     // Cleanup
        //     if (!vehicle->isM100() && !vehicle->isLegacyM600())
        //     {
        //         vehicle->subscribe->removePackage(0, timeout);
        //     }
        //     return false;
        // }
        // else
        // {
        //     std::cout << "Aircraft in air!\n";
        // }

        if (!vehicle->isM100() && !vehicle->isLegacyM600())
        {
            if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_P_GPS ||
                vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_ATTITUDE)
            {
                std::cout << "Successful takeoff!\n";
            }
            else
            {
                std::cout << "Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.\n";
                vehicle->subscribe->removePackage(0, timeout);
                return false;
            }
        }
    }
    else
    {
        float32_t delta;
        Telemetry::GlobalPosition currentHeight;
        Telemetry::GlobalPosition deltaHeight = vehicle->broadcast->getGlobalPosition();

        timeoutCycles = 100;
        loop_count = 0;
        do
        {
            usleep(4*1000);
            //flight_delay_loop_ms(300);
            currentHeight = vehicle->broadcast->getGlobalPosition();
            delta = fabs(currentHeight.altitude - deltaHeight.altitude);
            deltaHeight.altitude = currentHeight.altitude;

            loop_count++;
            printf("MMMMM- %ld\n", loop_count);

        } while (delta >= 0.009); // && loop_count < timeoutCycles);

        // if (loop_count >= timeoutCycles)
        // {
        //     std::cout << "Takeoff failed. The aircraft has not reached the target altitude.\n" << std::endl;
        // }
        // else
        // {
        //     std::cout << "Aircraft hovering at " << currentHeight.altitude << "m!\n";
        // }

        std::cout << "Aircraft hovering at " << currentHeight.altitude << "m!\n";
    }

    // Cleanup
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
        if (ACK::getError(ack))
        {
            std::cout << "Error unsubscribing; please restart the drone/FC to get back to a clean state.\n";
        }
    }

    return true;
}

/*! Position Control. Allows you to set an offset from your current
    location. The aircraft will move to that position and stay there.
    Typical use would be as a building block in an outer loop that does not
    require many fast changes, perhaps a few-waypoint trajectory. For smoother
    transition and response you should convert your trajectory to attitude
    setpoints and use attitude control or convert to velocity setpoints
    and use velocity control.
!*/
bool moveByPositionOffset(Vehicle *vehicle, float xOffsetDesired, float yOffsetDesired, float zOffsetDesired, float yawDesired, float posThresholdInM, float yawThresholdInDeg)
{
    // Set timeout: this timeout is the time you allow the drone to take to finish
    // the
    // mission
    int responseTimeout = 1;
    int timeoutInMilSec = 100000; // 100000 msec
    int controlFreqInHz = 50; // Hz
    int cycleTimeInMs = 1000 / controlFreqInHz; // 20 msec
    int outOfControlBoundsTimeLimit = 10 * cycleTimeInMs;  // 10 cycles
    int withinControlBoundsTimeReqmt = 50 * cycleTimeInMs; // 50 cycles
    int pkgIndex;

    //@todo: remove this once the getErrorCode function signature changes
    char func[50];

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        // Telemetry: Verify the subscription
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = vehicle->subscribe->verify(responseTimeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            return false;
        }

        // Telemetry: Subscribe to quaternion, fused lat/lon and altitude at freq 50
        // Hz
        pkgIndex = 0;
        int freq = 50;
        TopicName topicList50Hz[] = {TOPIC_QUATERNION, TOPIC_GPS_FUSED};
        int numTopic = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
        bool enableTimestamp = false;

        bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
        if (!(pkgStatus))
        {
            return pkgStatus;
        }

        subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, responseTimeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            // Cleanup before return
            vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
            return false;
        }
    }

    // Wait for data to come in
    usleep(1*1000);
    //flight_delay_loop_ms(1000);

    // Get data

    // Global position retrieved via subscription
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type currentSubscriptionGPS;
    Telemetry::TypeMap<TOPIC_GPS_FUSED>::type originSubscriptionGPS;
    // Global position retrieved via broadcast
    Telemetry::GlobalPosition currentBroadcastGP;
    Telemetry::GlobalPosition originBroadcastGP;

    // Convert position offset from first position to local coordinates
    Telemetry::Vector3f localOffset;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
        originSubscriptionGPS = currentSubscriptionGPS;
        localOffsetFromGpsOffset(vehicle, localOffset, static_cast<void *>(&currentSubscriptionGPS), static_cast<void *>(&originSubscriptionGPS));
        
        // Get the broadcast GP since we need the height for zCmd
        currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
    }
    else
    {
        currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
        originBroadcastGP = currentBroadcastGP;
        localOffsetFromGpsOffset(vehicle, localOffset, static_cast<void *>(&currentBroadcastGP), static_cast<void *>(&originBroadcastGP));
    }

    // Get initial offset. We will update this in a loop later.
    double xOffsetRemaining = xOffsetDesired - localOffset.x;
    double yOffsetRemaining = yOffsetDesired - localOffset.y;
    double zOffsetRemaining = zOffsetDesired - localOffset.z;

    // Conversions
    double yawDesiredRad = DEG2RAD * yawDesired;
    double yawThresholdInRad = DEG2RAD * yawThresholdInDeg;

    //! Get Euler angle

    // Quaternion retrieved via subscription
    Telemetry::TypeMap<TOPIC_QUATERNION>::type subscriptionQ;
    // Quaternion retrieved via broadcast
    Telemetry::Quaternion broadcastQ;

    double yawInRad;
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
        yawInRad = toEulerAngle((static_cast<void *>(&subscriptionQ))).z / DEG2RAD;
    }
    else
    {
        broadcastQ = vehicle->broadcast->getQuaternion();
        yawInRad = toEulerAngle((static_cast<void *>(&broadcastQ))).z / DEG2RAD;
    }

    int elapsedTimeInMs = 0;
    int withinBoundsCounter = 0;
    int outOfBounds = 0;
    int brakeCounter = 0;
    int speedFactor = 2;
    float xCmd, yCmd, zCmd;
    // There is a deadband in position control
    // the z cmd is absolute height
    // while x and y are in relative
    float zDeadband = 0.12;

    if (vehicle->isM100() || vehicle->isLegacyM600())
    {
        zDeadband = 0.12 * 10;
    }

    /*! Calculate the inputs to send the position controller. We implement basic
   *  receding setpoint position control and the setpoint is always 1 m away
   *  from the current position - until we get within a threshold of the goal.
   *  From that point on, we send the remaining distance as the setpoint.
   */
    if (xOffsetDesired > 0) {
        xCmd = (xOffsetDesired < speedFactor) ? xOffsetDesired : speedFactor;
    }
    else if (xOffsetDesired < 0) {
        xCmd = (xOffsetDesired > -1 * speedFactor) ? xOffsetDesired : -1 * speedFactor;
    }
    else {
        xCmd = 0;
    }

    if (yOffsetDesired > 0) {
        yCmd = (yOffsetDesired < speedFactor) ? yOffsetDesired : speedFactor;
    }
    else if (yOffsetDesired < 0) {
        yCmd = (yOffsetDesired > -1 * speedFactor) ? yOffsetDesired : -1 * speedFactor;
    }
    else {
        yCmd = 0;
    }

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        zCmd = currentSubscriptionGPS.altitude + zOffsetDesired;
    }
    else
    {
        zCmd = currentBroadcastGP.altitude + zOffsetDesired;
    }

    //! Main closed-loop receding setpoint position control
    loop_count = 0;
    while (elapsedTimeInMs < timeoutInMilSec)
    {
        vehicle->control->positionAndYawCtrl(xCmd, yCmd, zCmd, yawDesiredRad / DEG2RAD);

        usleep(cycleTimeInMs*1000);
        //flight_delay_loop_ms(cycleTimeInMs);
        elapsedTimeInMs += cycleTimeInMs;
        //loop_count++;
        //if(loop_count > 16384) {
        //loop_count = 0;
        //break;
        //}
        printf("ZZZZZ- %ld\r\n", elapsedTimeInMs);

        //! Get current position in required coordinates and units
        if (!vehicle->isM100() && !vehicle->isLegacyM600())
        {
            subscriptionQ = vehicle->subscribe->getValue<TOPIC_QUATERNION>();
            yawInRad = toEulerAngle((static_cast<void *>(&subscriptionQ))).z;
            currentSubscriptionGPS = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
            localOffsetFromGpsOffset(vehicle, localOffset, static_cast<void *>(&currentSubscriptionGPS), static_cast<void *>(&originSubscriptionGPS));

            // Get the broadcast GP since we need the height for zCmd
            currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
        }
        else
        {
            broadcastQ = vehicle->broadcast->getQuaternion();
            yawInRad = toEulerAngle((static_cast<void *>(&broadcastQ))).z;
            currentBroadcastGP = vehicle->broadcast->getGlobalPosition();
            localOffsetFromGpsOffset(vehicle, localOffset, static_cast<void *>(&currentBroadcastGP), static_cast<void *>(&originBroadcastGP));
        }

        //! See how much farther we have to go
        xOffsetRemaining = xOffsetDesired - localOffset.x;
        yOffsetRemaining = yOffsetDesired - localOffset.y;
        zOffsetRemaining = zOffsetDesired - localOffset.z;

        printf("[0] localOffset.z = %f\r\n", localOffset.z);
        printf("[0] zOffsetDesired = %f\r\n", zOffsetDesired);

        printf("[0] xOffsetRemaining = %f\r\n", xOffsetRemaining);
        printf("[0] yOffsetRemaining = %f\r\n", yOffsetRemaining);
        printf("[0] zOffsetRemaining = %f\r\n", zOffsetRemaining);

        printf("[0] std::abs(xOffsetRemaining) = %f\r\n", std::abs(xOffsetRemaining));
        printf("[0] std::abs(yOffsetRemaining) = %f\r\n", std::abs(yOffsetRemaining));
        printf("[0] std::abs(zOffsetRemaining) = %f\r\n", std::abs(zOffsetRemaining));
        printf("[0] std::abs(yawInRad - yawDesiredRad) = %f\r\n", std::abs(yawInRad - yawDesiredRad));

        printf("[0] posThresholdInM = %f\r\n", posThresholdInM);
        printf("[0] zDeadband = %f\r\n", zDeadband);
        printf("[0] yawThresholdInRad = %f\r\n", yawThresholdInRad);

        float pth = std::abs(posThresholdInM);
        float xoffr = std::abs(xOffsetRemaining);
        printf("[0] xoffr <= posThresholdInM = %d\r\n", (xoffr <= pth));
        float yoffr = std::abs(yOffsetRemaining);
        printf("[0] yoffr <= posThresholdInM = %d\r\n", (yoffr <= pth));
        float zoffr = std::abs(zOffsetRemaining);
        printf("[0] zoffr <= zDeadband = %d\r\n", (zoffr <= zDeadband));

        //! See if we need to modify the setpoint
        if (std::abs(xOffsetRemaining) < speedFactor) {
            xCmd = xOffsetRemaining;
        }
        
        if (std::abs(yOffsetRemaining) < speedFactor) {
            yCmd = yOffsetRemaining;
        }

        if (vehicle->isM100() &&
            std::abs(xOffsetRemaining) < posThresholdInM &&
            std::abs(yOffsetRemaining) < posThresholdInM &&
            std::abs(zOffsetRemaining) < zDeadband &&
            std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
        {
            //! 1. We are within bounds; start incrementing our in-bound counter
            withinBoundsCounter += cycleTimeInMs;
            printf("[1] withinBoundsCounter = %ld\r\n", withinBoundsCounter);
        }
        else if (std::abs(xOffsetRemaining) < posThresholdInM &&
                 std::abs(yOffsetRemaining) < posThresholdInM &&
                 std::abs(zOffsetRemaining) < zDeadband &&
                 std::abs(yawInRad - yawDesiredRad) < yawThresholdInRad)
        {
            //! 1. We are within bounds; start incrementing our in-bound counter
            withinBoundsCounter += cycleTimeInMs;
            printf("[2] withinBoundsCounter = %ld\r\n", withinBoundsCounter);
        }
        else
        {
            printf("[3] withinBoundsCounter = %ld\r\n", withinBoundsCounter);
            if (withinBoundsCounter != 0)
            {
                //! 2. Start incrementing an out-of-bounds counter
                outOfBounds += cycleTimeInMs;
            }
        }

        //! 3. Reset withinBoundsCounter if necessary
        if (outOfBounds > outOfControlBoundsTimeLimit)
        {
            withinBoundsCounter = 0;
            outOfBounds = 0;
        }

        //! 4. If within bounds, set flag and break
        if (withinBoundsCounter >= withinControlBoundsTimeReqmt)
        {
            break;
        }
    }

    //! Set velocity to zero, to prevent any residual velocity from position
    //! command
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        loop_count = 0;
        while (brakeCounter < withinControlBoundsTimeReqmt)
        {
            vehicle->control->emergencyBrake();
            usleep(cycleTimeInMs * 10);
            //flight_delay_loop_ms(cycleTimeInMs);
            brakeCounter += cycleTimeInMs;
            //loop_count++;
            //if(loop_count > 16384) {
            //loop_count = 0;
            //break;
            //}
            printf("OOOOO- %ld", loop_count);
        }
    }

    if (elapsedTimeInMs >= timeoutInMilSec)
    {
        std::cout << "Task timeout!\n";
        if (!vehicle->isM100() && !vehicle->isLegacyM600())
        {
            ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
            if (ACK::getError(ack))
            {
                std::cout << "Error unsubscribing; please restart the drone/FC to get back to a clean state.\n";
            }
        }
        return ACK::FAIL;
    }

    std::cout << "move to Position SUCCESS!\n";
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
        if (ACK::getError(ack))
        {
            std::cout << "Error unsubscribing; please restart the drone/FC to get back to a clean state.\n";
        }
    }

    return ACK::SUCCESS;
}

/*! Monitored Takeoff (Blocking API call). Return status as well as ack.
    This version of takeoff makes sure your aircraft actually took off
    and only returns when takeoff is complete.
    Use unless you want to do other stuff during takeoff - this will block
    the main thread.
!*/

bool monitoredLanding(Vehicle *vehicle, int timeout)
{
    //@todo: remove this once the getErrorCode function signature changes
    char func[50];
    int pkgIndex;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        // Telemetry: Verify the subscription
        ACK::ErrorCode subscribeStatus;
        subscribeStatus = vehicle->subscribe->verify(timeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            return false;
        }

        // Telemetry: Subscribe to flight status and mode at freq 10 Hz
        pkgIndex = 0;
        int freq = 10;
        TopicName topicList10Hz[] = {TOPIC_STATUS_FLIGHT,
                                     TOPIC_STATUS_DISPLAYMODE};
        int numTopic = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
        bool enableTimestamp = false;

        bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
        if (!(pkgStatus))
        {
            return pkgStatus;
        }
        subscribeStatus = vehicle->subscribe->startPackage(pkgIndex, timeout);
        if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
        {
            ACK::getErrorCodeMessage(subscribeStatus, func);
            // Cleanup before return
            vehicle->subscribe->removePackage(pkgIndex, timeout);
            return false;
        }
    }

    // Start landing
    ACK::ErrorCode landingStatus = vehicle->control->land(timeout);
    if (ACK::getError(landingStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(landingStatus, func);
        return false;
    }

    // First check: Landing started
    int landingNotStarted = 0;
    int timeoutCycles = 20;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        loop_count = 0;
        while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
               landingNotStarted < timeoutCycles)
        {
            landingNotStarted++;
            //usleep(10000);
            flight_delay_loop_ms(10);
            
            printf("NNNNN- %ld", loop_count);
        }

        if (landingNotStarted >= timeoutCycles)
        {
            std::cout << "Landing failed. Aircraft is still in the air." << std::endl;
            
            // Cleanup before return
            ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
            if (ACK::getError(ack))
            {
                std::cout << "Error unsubscribing; please restart the drone/FC to get back to a clean state.\n";
            }

            return false;
        }
        else
        {
            std::cout << "Landing...\n";
        }
    }
    else if (vehicle->isM100())
    {
        loop_count = 0;
        while (vehicle->broadcast->getStatus().flight != DJI::OSDK::VehicleStatus::M100FlightStatus::LANDING &&
               landingNotStarted < timeoutCycles)
        {
            landingNotStarted++;
            //usleep(10000);
            flight_delay_loop_ms(10);
            printf("PPPPP- %ld", loop_count);
        }

        if (landingNotStarted >= timeoutCycles)
        {
            std::cout << "Landing failed. Aircraft is still in the air." << std::endl;
            return false;
        }
        else
        {
            std::cout << "Landing...\n";
        }
    }

    // Second check: Finished landing
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        timeoutCycles = 20;
        loop_count = 0;
        while (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() == VehicleStatus::DisplayMode::MODE_AUTO_LANDING &&
               vehicle->subscribe->getValue<TOPIC_STATUS_FLIGHT>() == VehicleStatus::FlightStatus::IN_AIR &&
               loop_count < timeoutCycles)
        {
            //usleep(100000);
            flight_delay_loop_ms(100);
            loop_count++;
            printf("QQQQQ- %ld", loop_count);
        }

        if (vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_P_GPS ||
            vehicle->subscribe->getValue<TOPIC_STATUS_DISPLAYMODE>() != VehicleStatus::DisplayMode::MODE_ATTITUDE)
        {
            std::cout << "Successful landing!\n";
        }
        else
        {
            std::cout << "Landing finished, but the aircraft is in an unexpected mode. Please connect DJI GO.\n";
            ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
            if (ACK::getError(ack))
            {
                std::cout << "Error unsubscribing; please restart the drone/FC to get back to a clean state.\n";
            }
            return false;
        }
    }
    else if (vehicle->isLegacyM600())
    {
        timeoutCycles = 20;
        loop_count = 0;
        while (vehicle->broadcast->getStatus().flight >DJI::OSDK::VehicleStatus::FlightStatus::STOPED &&
               loop_count < timeoutCycles)
        {
            //usleep(100000);
            flight_delay_loop_ms(100);
            loop_count++;
            printf("AAAAA - %ld", loop_count);
        }

        Telemetry::GlobalPosition gp;
        loop_count = 0;
        do
        {
            //usleep(100000);
            flight_delay_loop_ms(100);
            loop_count++;
            gp = vehicle->broadcast->getGlobalPosition();
            printf("gp.alttitude %ld", loop_count);
        } while (gp.altitude != 0 && loop_count < timeoutCycles);

        if (gp.altitude != 0)
        {
            std::cout << "Landing finished, but the aircraft is in an unexpected mode. Please connect DJI GO.\n";
            return false;
        }
        else
        {
            std::cout << "Successful landing!\n";
        }
    }
    else // M100
    {
        timeoutCycles = 20;
        loop_count = 0;
        while (vehicle->broadcast->getStatus().flight == DJI::OSDK::VehicleStatus::M100FlightStatus::FINISHING_LANDING &&
               loop_count < timeoutCycles)
        {
            //usleep(100000);
            flight_delay_loop_ms(100);
            loop_count++;
            printf("BBBBB- %ld", loop_count);
        }

        Telemetry::GlobalPosition gp;
        loop_count = 0;
        do
        {
            //usleep(100000);
            flight_delay_loop_ms(100);
            loop_count++;
            gp = vehicle->broadcast->getGlobalPosition();
            printf("gp.alttitude %ld", loop_count);
        } while (gp.altitude != 0 && loop_count < timeoutCycles);

        if (gp.altitude != 0)
        {
            std::cout << "Landing finished, but the aircraft is in an unexpected mode. Please connect DJI GO.\n";
            return false;
        }
        else
        {
            std::cout << "Successful landing!\n";
        }
    }

    // Cleanup
    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
        if (ACK::getError(ack))
        {
            std::cout << "Error unsubscribing; please restart the drone/FC to get back to a clean state.\n";
        }
    }

    return true;
}

// Helper Functions

/*! Very simple calculation of local NED offset between two pairs of GPS
/coordinates.
    Accurate when distances are small.
!*/
void localOffsetFromGpsOffset(Vehicle *vehicle, Telemetry::Vector3f &deltaNed, void *target, void *origin)
{
    Telemetry::GPSFused *subscriptionTarget;
    Telemetry::GPSFused *subscriptionOrigin;
    Telemetry::GlobalPosition *broadcastTarget;
    Telemetry::GlobalPosition *broadcastOrigin;
    double deltaLon;
    double deltaLat;

    if (!vehicle->isM100() && !vehicle->isLegacyM600())
    {
        subscriptionTarget = (Telemetry::GPSFused *)target;
        subscriptionOrigin = (Telemetry::GPSFused *)origin;
        deltaLon = subscriptionTarget->longitude - subscriptionOrigin->longitude;
        deltaLat = subscriptionTarget->latitude - subscriptionOrigin->latitude;
        deltaNed.x = deltaLat * C_EARTH;
        deltaNed.y = deltaLon * C_EARTH * cos(subscriptionTarget->latitude);
        deltaNed.z = subscriptionTarget->altitude - subscriptionOrigin->altitude;
    }
    else
    {
        broadcastTarget = (Telemetry::GlobalPosition *)target;
        broadcastOrigin = (Telemetry::GlobalPosition *)origin;
        deltaLon = broadcastTarget->longitude - broadcastOrigin->longitude;
        deltaLat = broadcastTarget->latitude - broadcastOrigin->latitude;
        deltaNed.x = deltaLat * C_EARTH;
        deltaNed.y = deltaLon * C_EARTH * cos(broadcastTarget->latitude);
        deltaNed.z = broadcastTarget->altitude - broadcastOrigin->altitude;
    }
}

Telemetry::Vector3f
toEulerAngle(void *quaternionData)
{
    Telemetry::Vector3f ans;
    Telemetry::Quaternion *quaternion = (Telemetry::Quaternion *)quaternionData;

    double q2sqr = quaternion->q2 * quaternion->q2;
    double t0 = -2.0 * (q2sqr + quaternion->q3 * quaternion->q3) + 1.0;
    double t1 =
        +2.0 * (quaternion->q1 * quaternion->q2 + quaternion->q0 * quaternion->q3);
    double t2 =
        -2.0 * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
    double t3 =
        +2.0 * (quaternion->q2 * quaternion->q3 + quaternion->q0 * quaternion->q1);
    double t4 = -2.0 * (quaternion->q1 * quaternion->q1 + q2sqr) + 1.0;

    t2 = (t2 > 1.0) ? 1.0 : t2;
    t2 = (t2 < -1.0) ? -1.0 : t2;

    ans.x = asin(t2);
    ans.y = atan2(t3, t4);
    ans.z = atan2(t1, t0);

    return ans;
}

bool monitoredGoHome(Vehicle *vehicle, int timeout)
{
    //@todo: remove this once the getErrorCode function signature changes
    char func[50];
    int pkgIndex;

    // Start GoHome
    ACK::ErrorCode goHomeStatus = vehicle->control->goHome(timeout);
    if (ACK::getError(goHomeStatus) != ACK::SUCCESS)
    {
        ACK::getErrorCodeMessage(goHomeStatus, func);
        return false;
    }

    // First check: Landing started
    int goHomeNotStarted = 0;
    int timeoutCycles = 20;

    if (goHomeNotStarted == timeoutCycles)
    {
        std::cout << "GoHome failed. Aircraft is still in the air." << std::endl;
        // Cleanup before return
        ACK::ErrorCode ack = vehicle->subscribe->removePackage(pkgIndex, timeout);
        if (ACK::getError(ack))
        {
            std::cout << "Error unsubscribing; please restart the drone/FC to get back to a clean state.\n";
        }
        return false;
    }
    else
    {
        std::cout << "GoHome...\n";
    }

    // Second check: Finished landing

    timeoutCycles = 10;
    loop_count = 0;
    while (vehicle->broadcast->getStatus().flight == DJI::OSDK::VehicleStatus::M100FlightStatus::FINISHING_LANDING && 
           loop_count < timeoutCycles)
    {
        //usleep(100000);
        flight_delay_loop_ms(100);
        loop_count++;
        printf("DDDDD- %ld", loop_count);
    }

    Telemetry::GlobalPosition gp;
    timeoutCycles = 20;
    loop_count = 0;
    do
    {
        //usleep(100000);
        flight_delay_loop_ms(100);
        loop_count++;
        gp = vehicle->broadcast->getGlobalPosition();
        printf("EEEEE- %ld", loop_count);
    } while (gp.altitude != 0 && loop_count < timeoutCycles);

    if (gp.altitude != 0)
    {
        std::cout << "Landing finished, but the aircraft is in an unexpected mode. Please connect DJI GO.\n";
        return false;
    }
    else
    {
        std::cout << "Successful GoHome!\n";
    }

    return true;
}
