/**
 ******************************************************************************
 * @file            listener.cpp
 * @brief           Methods of the Listener class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 11/2024
 *****************************************************************************
 */

// Standard libraries
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <cmath>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <cstring>
#include <net/if.h> // network devices
#include <sstream> // Convert dec-hex

#include "listener.hpp"

using namespace std;

namespace KMR::CBM
{

/**
 * @brief       Start the CAN bus listener
 * @details		The listener works in its own thread, created on constructor call. 
 * 				The thread is safely stopped on destructor call
 * @param[in]	motors List of CubeMars motors models
 * @param[in]	ids IDs of the motors
 * @param[in]   s Socket
 */
Listener::Listener(vector<Motor*> motors, vector<int> ids, int s)
{
	m_motors = motors;
	m_nbrMotors = motors.size();
    m_stopThread = false;
    m_thread = thread(&Listener::listenerLoop, this, s);
	m_ids = ids;

    cout << "Creating the CAN listener's thread..." << endl;
	usleep(50000);  
}

/**
 * @brief	Class destructor. Takes care of safely stopping the thread
 */
Listener::~Listener()
{
	// Change the internal boolean to get the thread out of its loop function
	{
		scoped_lock lock(m_mutex);
		m_stopThread = true;	
	}	
    
	// Block the main thread until the Listener thread finishes
    if (m_thread.joinable()) 
        m_thread.join();

	cout << "Listener thread safely stopped" << endl;
}

/**
 * @brief       Function executed by the Listener thread
 * @param[in]   s Socket
 * @retval      1 when the thread is finished
 */
int Listener::listenerLoop(int s)
{
	bool stopThread = 0;

	cout << "Starting CAN bus monitoring" << endl;
    while(!stopThread) {

        struct can_frame frame;
        int nbytes = read(s, &frame, sizeof(can_frame));  // Usually takes ~4us, rarely jumping to 26 us

        if (nbytes > 0)
			parseFrame(frame);

		// Thread sleep for scheduling
		std::this_thread::sleep_for(chrono::microseconds(10));  // 50 at start
		{
			scoped_lock lock(m_mutex);
			stopThread = m_stopThread;	
		}
    }

    return 0;
}

/**
 * 	@brief 		Convert a parameter received from motors to SI units
 * 	@param[in] 	x Value to be converted to SI
 * 	@param[in] 	xMin Minimum value that the converted parameter can take 
 * 	@param[in] 	xMax Maximum value that the converted parameter can take 
 * 	@param[in] 	bitSize Number of bits encoding the parameter
 * 	@return 	Parameter converted to SI units
 */ 
float Listener::convertParameter_to_SI(int x, float xMin, float xMax, int bitSize)
{
	float span = xMax - xMin;

	float value = x * span/(float)((1<<bitSize)-1) + xMin;
	return value;
}


/**
 * 	@brief 		Parse a received frame
 * 	@param[in] 	frame CAN frame received on the bus
 */ 
void Listener::parseFrame(can_frame frame)
{
	// Extract the motor ID from the received frame
	int id = frame.data[0];
	int idx = getIndex(m_ids, id);	

	// Parse data
	int positionParameter = (frame.data[1] << 8) | frame.data[2];
	int speedParameter = (frame.data[3] << 4) | (frame.data[4] >> 4);
	int torqueParameter = ((frame.data[4]&0xF) << 8) | frame.data[5];
	int temperatureParameter = frame.data[6];

    // Extract min/max parameters from the query moto
    float minPosition = m_motors[idx]->minPosition;
    float maxPosition = m_motors[idx]->maxPosition;
    float minSpeed = m_motors[idx]->minSpeed;
    float maxSpeed = m_motors[idx]->maxSpeed;
    float minTorque = m_motors[idx]->minTorque;
    float maxTorque = m_motors[idx]->maxTorque; 

	// Convert parameters to SI
	float position = convertParameter_to_SI(positionParameter, minPosition, maxPosition, 16);
	float speed = convertParameter_to_SI(speedParameter, minSpeed, maxSpeed, 12);
	float torque = convertParameter_to_SI(torqueParameter, minTorque, maxTorque, 12);
	int temperature = temperatureParameter;

	// Adjust signs for our reference
	position = -position;
	speed = -speed;
	torque = -torque;

	// Save to internal structure
	scoped_lock lock(m_mutex);
	m_motors[idx]->fbckPosition = position;
	m_motors[idx]->fbckSpeed = speed;
	m_motors[idx]->fbckTorque = torque;
	m_motors[idx]->fbckTemperature = temperature;

	m_motors[idx]->fr_fbckReady = 1;
	m_cv.notify_all(); //  NEWNotify the main thread that the feedback is ready
}

/**
 * @brief       Check if the query motor sent the response to a previous command
 * @param[in]   id ID of the query motor
 * @retval      1 if the motor answered, 0 if not (timeout)
 *
 * RT-safe rewrite: instead of busy-waiting on the mutex, the caller sleeps
 * on m_cv until either the listener thread signals fr_fbckReady=1 (see
 * parseFrame) or RESPONSE_TIMEOUT microseconds elapse. This eliminates
 * the priority-inversion trap where a SCHED_FIFO caller would starve a
 * SCHED_OTHER listener via mutex contention. [4]
 */
bool Listener::fbckReceived(int id)
{
    int idx = getIndex(m_ids, id);

    std::unique_lock<std::mutex> lock(m_mutex);
    bool ok = m_cv.wait_for(
        lock,
        std::chrono::microseconds(RESPONSE_TIMEOUT),
        [&]{ return m_motors[idx]->fr_fbckReady == 1; }
    );

    if (!ok) {
        std::cout << "[TIMEOUT] Receiving feedback for motor " << id
                  << " timed out!" << std::endl;
        return false;
    }

    // Consume the flag so the next call waits for a fresh reply
    m_motors[idx]->fr_fbckReady = 0;
    return true;
}
	

/**
 * @brief       Get the feedbacks from a query motor, if it was received
 * @param[in]   id ID of the query motor
 * @param[out]  fbckPosition Feedback motor position [rad]
 * @param[out]  fbckSpeed Feedback motor speed [rad/s]
 * @param[out]  fbckTorque Feedback torque [Nm]
 * @param[out]  fbckTemperature Feedback temperature [°C]
 * @retval      1 if the motor answered (= valid fbck), 0 if not (timeout)
 *
 * RT-safe rewrite: same CV-based wait as fbckReceived(). While holding the
 * lock after wakeup, we copy the four feedback values out and then clear
 * the ready flag atomically w.r.t. the listener's parseFrame() writes.
 */
bool Listener::getFeedbacks(int id, float& fbckPosition, float& fbckSpeed,
                            float& fbckTorque, int& fbckTemperature)
{
    int idx = getIndex(m_ids, id);

    std::unique_lock<std::mutex> lock(m_mutex);
    bool ok = m_cv.wait_for(
        lock,
        std::chrono::microseconds(RESPONSE_TIMEOUT),
        [&]{ return m_motors[idx]->fr_fbckReady == 1; }
    );

    if (!ok) {
        std::cout << "[TIMEOUT] Receiving feedback for motor " << id
                  << " timed out!" << std::endl;
        return false;
    }

    // Copy feedback values out while still holding the lock
    fbckPosition    = m_motors[idx]->fbckPosition;
    fbckSpeed       = m_motors[idx]->fbckSpeed;
    fbckTorque      = m_motors[idx]->fbckTorque;
    fbckTemperature = m_motors[idx]->fbckTemperature;

    // Consume the flag
    m_motors[idx]->fr_fbckReady = 0;
    return true;
}

}