#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <deque>
#include <ros/ros.h>

static constexpr int BUF_LEN = 4096;

class SerialReader{

public:
    SerialReader() : isStarted(false) {}

    void arrangeData(uint8_t* buf, size_t length){
        if (isStarted == false){ //initial point
            uint8_t *buf_start = nullptr;
            size_t i = 0; 
            for(; i < length; ++i){
                if (buf[i] == '$') {
                    buf_start = buf + i;
                    isStarted = true;
                    break;
                }
            }

            if (buf_start == nullptr) return;
            else {
                len_buf_sentence = length - i;
                memcpy(buf_sentence, buf_start, len_buf_sentence);
                return;
            }
        }

        memcpy(buf_sentence + len_buf_sentence, buf, length);
        len_buf_sentence += length;

        while(true){ //find end of sentence
            size_t len_sentence;
            size_t i = 0;
            for(; i < len_buf_sentence; ++i){
                if (buf_sentence[i] == '\n') {
                    len_sentence = i + 1;
                    if (len_sentence > len_buf_sentence) return; //sentence not done
                    break;
                }
            }
            if (i == len_buf_sentence) return; 

            std::string str((const char*)buf_sentence, len_sentence - 1);
            
            for(size_t i = 0; i < len_buf_sentence - len_sentence; ++i)
                buf_sentence[i] = buf_sentence[len_sentence + i];
            len_buf_sentence -= len_sentence;
            sentenceQueue.push_back(str);
        }
    }
    std::string getSentence(){
        std::string str;
        if (sentenceQueue.size() == 0)
            return "";
        str = sentenceQueue.front();
        sentenceQueue.clear();
        return str;
    } 
private:
    uint8_t buf_sentence[BUF_LEN];
    size_t len_buf_sentence;
    std::deque<std::string> sentenceQueue;
    bool isStarted;
};


class VN100IMU{// almost all of this class function can throw error related to serial   
public:
    static void createInstance(std::string port, double baudrate);
    static VN100IMU* getInstancePtr();
    bool ok();
    void destroy();
    sensor_msgs::Imu read_and_parse(); 
    ~VN100IMU();
private:
    VN100IMU();
    inline unsigned char calculateChecksum(unsigned char data[], unsigned int length);
    size_t getDataLength(uint8_t *begin, size_t read_sz);
    static VN100IMU *ptr; 
    void setupImu(std::string port, double baudrate);
    bool serial_read_ok;

    serial::Serial imu_serial;

    uint8_t buf[BUF_LEN];
    sensor_msgs::Imu imu_msg;
    SerialReader serialReader;
};