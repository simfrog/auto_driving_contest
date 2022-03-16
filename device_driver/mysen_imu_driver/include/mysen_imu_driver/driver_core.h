// Software License Agreement (BSD License)
//
// Copyright (c) 2020, Taewook Park <sjrnfu12@naver.com>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the names of the authors nor the names of their
//    affiliated organizations may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <serial/serial.h>
#include <sensor_msgs/Imu.h>

class MySenIMU{// almost all of this class function can throw error related to serial   
public:
    static void createInstance(std::string port, int mode);
    static MySenIMU* getInstancePtr();
    bool ok();
    void destroy();
    sensor_msgs::Imu read_and_parse(); //when error occures, it returns previous msg
    ~MySenIMU();
private:
    MySenIMU();
    inline bool isValidChecksum(uint8_t* begin, uint8_t* end, uint8_t checksum);
    size_t getDataLength(uint8_t *begin, size_t read_sz);
    static MySenIMU *ptr; //single tone. To destroy the object, i use static method
    void setupImu(std::string port, int mode);
    bool serial_read_ok;

    serial::Serial imu_serial;

    static constexpr int BUF_LEN = 4096;
    uint8_t buf[BUF_LEN];
    sensor_msgs::Imu imu_msg;
};