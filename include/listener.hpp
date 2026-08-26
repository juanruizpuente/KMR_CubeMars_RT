/**
 ******************************************************************************
 * @file            listener.hpp
 * @brief           Definition of the Listener class
 ******************************************************************************
 * @copyright
 * Copyright 2021-2024 Kamilo Melo        \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors katarina.lichardova@km-robota.com, 11/2024
 *****************************************************************************
 */

#ifndef KMR_CUBEMARS_LISTENER_HPP
#define KMR_CUBEMARS_LISTENER_HPP

#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <linux/can.h>

#include "../config/structures.hpp"
#include "utils.hpp"
#include <condition_variable>

namespace KMR::CBM
{

/**
 * @brief   CAN bus listener
 * @details The main goal of this class is to handle the listening of the CAN bus. \n
 *          On construction, an object of this class creates a new thread that constantly listens 
 *          to the CAN bus, and saves any information it reads there. On information receiving, it 
 *          also raises internal flags to indicate new information has been received. \n
 *          This class provides public methods that check if new information was received, by checking
 *          said internal flags. If it's the case, these methods pull the flags down and return the 
 *          receive confirmation, as well as any feedback values when applicable. In the case where 
 *          the internal flags were not raised in a given time interval, the methods time out and 
 *          return a motor response fail. 
 */
class Listener {
public:
    Listener(std::vector<Motor*> motors, std::vector<int> ids, int s);
    ~Listener();

    bool fbckReceived(int id);
    bool getFeedbacks(int id, float& fbckPosition, float& fbckSpeed,
                      float& fbckTorque, int& fbckTemperature);
    bool setListenerPriority(int sched_policy, int priority);

private:
    // Thread
    bool m_stopThread = 0;
    std::thread m_thread;
    std::mutex m_mutex;
    std::condition_variable m_cv;

    std::vector<Motor*> m_motors;
    int m_nbrMotors;
    std::vector<int> m_ids;

    int listenerLoop(int s);
    void parseFrame(can_frame frame);
    float convertParameter_to_SI(int x, float xMin, float xMax, int bitSize);

};

}

#endif