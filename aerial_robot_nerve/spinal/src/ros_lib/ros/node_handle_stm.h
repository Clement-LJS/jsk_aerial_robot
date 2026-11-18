/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_NODE_HANDLE_STM_H_
#define ROS_NODE_HANDLE_STM_H_

#include "STM32Hardware.h"
#include "node_handle.h"

namespace ros {

const int SPIN_UNAVAILABLE = -3;

  /* Node Handle */
  template<int MAX_SUBSCRIBERS=25,
           int MAX_PUBLISHERS=25,
           int INPUT_SIZE=512,
           int OUTPUT_SIZE=512>
  class NodeHandleStm_ : public NodeHandle_<STM32Hardware,MAX_SUBSCRIBERS, MAX_PUBLISHERS, INPUT_SIZE, OUTPUT_SIZE>
  {
    typedef NodeHandle_<STM32Hardware, MAX_SUBSCRIBERS, MAX_PUBLISHERS, INPUT_SIZE, OUTPUT_SIZE> parent;

#if SUPPORT_RTOS
    // Separate from STM32Hardware's TX queue mutex. initNode runs once in the
    // core task; publication stays disabled until both transport and lock exist.
    osMutexId publication_mutex_ = nullptr;

    void initPublicationMutex()
    {
      if (publication_mutex_ == nullptr)
      {
        osMutexDef(rosSerialization);
        publication_mutex_ = osRecursiveMutexCreate(osMutex(rosSerialization));
      }
    }
#endif

  public:

    NodeHandleStm_(): parent::NodeHandle_()
    {
    }

    /* Start a named seiral port */
#if SUPPORT_RTOS
    void initNode(UART_HandleTypeDef* port,
                  osMutexId* mutex,
                  osSemaphoreId* semaphore){
      parent::initNode();
      parent::hardware_.init(port, mutex, semaphore);
      initPublicationMutex();
    };
#endif

    /* Start a UDP port */
#if SUPPORT_LWIP
    void initNode(ip4_addr_t dst_addr, uint16_t src_port, uint16_t dst_port)
    {
      parent::initNode();
      parent::hardware_.init(dst_addr, src_port, dst_port);
#if SUPPORT_RTOS
      initPublicationMutex();
#endif
    }
#endif

    // All Publisher::publish(), negotiation and log messages dispatch here.
    // Lock the shared message_out before serialization, through the queue copy.
    int publish(int id, const Msg* msg) override
    {
#if SUPPORT_RTOS
      // Blocking mutex APIs cannot be used in an interrupt. Fail closed on early
      // publication or allocation failure; logging here would recurse.
      if (__get_IPSR() != 0 || publication_mutex_ == nullptr)
        return -1;
      if (osRecursiveMutexWait(publication_mutex_, osWaitForever) != osOK)
        return -1;

      // The parent's oversize-message error path publishes a log recursively.
      // A recursive lock handles that without reusing the TX queue mutex.
      const int result = parent::publish(id, msg);
      osRecursiveMutexRelease(publication_mutex_);
      return result;
#else
      return parent::publish(id, msg);
#endif
    }

    /* This function goes in your loop() function, it handles
     *  serial tx data and ouput for publishers.
     */
    virtual int publish()
    {
      return  parent::hardware_.publish();
    }

    int spinOnce() override
    {
      bool has_data = parent::hardware_.rx_available();

      int status =  parent::spinOnce();

      if(has_data) return status;
      else return SPIN_UNAVAILABLE;
    }
  };
}

#endif
