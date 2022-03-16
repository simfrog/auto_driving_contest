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


#include <ros/ros.h>

#include <serial/serial.h>
#include <sensor_msgs/Imu.h>
#include <mutex>
#include <string>
#include <endian.h>
#include <erp42_msgs/erp42_to_pc.h>
#include <erp42_msgs/pc_to_erp42.h>
#include <signal.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <algorithm>
#include <thread>
#include <errno.h>
#include <std_msgs/Bool.h>
#include <autoware_msgs/VehicleStatus.h>
//for controller---------------------------------------------------
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
//-----------------------------------------------------------------

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (1.0 / DEG2RAD)
#define KPH2MPS (1000.0 / 3600.0)

//for controller---------------------------------------------------
#define NO_ACCEL 0
#define MAX_ACCEL 200
#define NO_BRAKE 1
#define NO_SLIP_BRAKE 70
#define MAX_BRAKE 200
#define MAX_STEER 2000
#define GEAR_FORWARD 0
#define GEAR_BACKWARD 2
#define SWITHCING_GEAR_SPEED_TOLERANCE 0.2
    
#define PI 3.141592
#define RAD2SERIAL (180.0/PI)*71.0 // rad -> serial *100 -> *71
#define M_S2SERIAL 1.0/0.0598 // m/s -> serial (@ steady-state)
#define EPSILON 0.03   // log(negative) 방지
//----------------------------------------------------------------

typedef struct{
    uint8_t             S           ;
    uint8_t             T           ;
    uint8_t             X           ;
    uint8_t             AorM        ; //0 : manual, 1 : auto
    uint8_t             estop       ; //0 : Off, 1: on
    uint8_t             gear        ; //0 : forward, 1 : neutral, 2 : backward
    uint16_t            speed       ; //actual speed * 10 * kph
    int16_t             steer       ; //actual steering degree * 71 with 4% error. negative is left steer
    uint8_t             brake       ; // 0(no brake) ~ 200(full brake)
    uint8_t             alive       ; //increasing at one step
    uint8_t             ETX0        ;
    uint8_t             ETX1        ;
}__attribute__((packed)) PC_TO_ERP42; //must be written as big endian

typedef struct{
    uint8_t             S           ;
    uint8_t             T           ;
    uint8_t             X           ;
    uint8_t             AorM        ;
    uint8_t             estop       ;
    uint8_t             gear        ;
    uint16_t            speed       ;
    int16_t             steer       ;
    uint8_t             brake       ;
    int32_t             encoder     ;
    uint8_t             alive       ;
    uint8_t             ETX0        ;
    uint8_t             ETX1        ;
}__attribute__((packed)) ERP42_TO_PC;//must be interpreted as little endian

constexpr static size_t PC_TO_ERP42_LEN = 14; //14 bytes
constexpr static size_t ERP42_TO_PC_LEN = 18; //18 bytes

class CarlikeOdometry{
public:
    double encoderVelocity(int32_t encoder, double dt){
	    static bool start = true;
	    static int32_t pre;
	    static constexpr int32_t ONECYCLE_ENCODER = 100;
	    static double REVOLUTION = 1.57079;
	    if (start){
		pre = encoder;
		start = false;
		return 0.0;
	    }

	    int32_t diff_encoder = encoder - pre;
	    double diff_dist = (1.0 * diff_encoder) / ONECYCLE_ENCODER * REVOLUTION;
	    double v = diff_dist / dt;
	    pre = encoder;

	    return v;
    }

    void create(){
        odom_pub = nh.advertise<nav_msgs::Odometry>("/vehicle/odom", 100);
        if (!nh.getParam("erp42_main/wheelbase", wheelbase))        
            throw std::runtime_error("set erp42_main/wheelbase!");
        if (!nh.getParam("erp42_main/minimum_turning_radius", minimum_turning_radius))        
            throw std::runtime_error("set erp42_main/minimum_turning_radius!");
        if (!nh.getParam("erp42_main/maximum_steering_angle", maximum_steering_angle))        
            throw std::runtime_error("set erp42_main/maximum_steering_angle!");
        isStart = true;
        x = y = th = 0;
    }
    void updateAndPublish(ERP42_TO_PC erp42_to_pc){
        if (isStart){
            isStart = false;
            pre_time = ros::Time::now();
 	        encoderVelocity(erp42_to_pc.encoder, 0.01);
	        return;
        }
	    /* calc dt */
        ros::Time cur_time = ros::Time::now();
        double dt = (cur_time - pre_time).toSec();
        pre_time = cur_time;

        /* calc v */
 	    //double v = encoderVelocity(erp42_to_pc.encoder, dt); //encoder based calculation
        double v = erp42_to_pc.speed / 10.0 / 3.6;
        if (erp42_to_pc.gear == 2) v *= -1; //backward gear  

        /* calc w */
        double maximumTireAngle = asin(wheelbase / (minimum_turning_radius)) * RAD2DEG; //deg
        double currentTireAngle = erp42_to_pc.steer / maximum_steering_angle * maximumTireAngle; //deg.
        //since erp's roration is inverse, we need to use negative steering angle value
        double w = tan(currentTireAngle * DEG2RAD) * v / wheelbase;
	    
        /* update time, position, theta */
        ros::Duration d(dt);
        x += v * cos(th) * dt;
        y += v * sin(th) * dt;
        th+= w * dt;

        /* update and publish odometry msg*/
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        odom_msg.header.seq++;
        odom_msg.header.stamp = cur_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.orientation = odom_quat;
        odom_msg.twist.twist.linear.x = v;
        odom_msg.twist.twist.angular.z = w;
        odom_pub.publish(odom_msg);

        /*update and publish tf msg */
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = cur_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.rotation = odom_quat;
        //odom_broadcaster_.sendTransform(odom_trans);
    }
private:
    	ros::NodeHandle nh;
    	ros::Publisher odom_pub;
    	tf::TransformBroadcaster odom_broadcaster_;
    	double wheelbase, minimum_turning_radius, maximum_steering_angle;
	bool isStart;
    	nav_msgs::Odometry odom_msg;
    	ros::Time pre_time;
    	double x, y, th;
};

class PlatformController;

class ERP42{
    //PlatformController----------------------------------------------------------------------
    class PlatformController{
    public:
        PlatformController(ERP42* p)
        : ref_speed(0.0), current_speed(0.0), err_speed(0.0)
        , ref_steer(0.0), current_steer(0.0), err_steer(0.0)
        , cmd_speed(0), cmd_steer(0), cmd_brake(0), cmd_gear(0)
        , kp_steer(1.0), ki_steer(0.0), kd_steer(0.0)
        , kp_brake(30.0), ki_brake(0.0), kd_brake(0.0)
        , settling_time(0.8), brake_max(200)
        , dt(0.0)
        , ss_speed(0), ss_speed_weight(1.35), ss_speed_shift(20.0)
        , target_accel(0.0), current_gear(GEAR_FORWARD), target_gear(GEAR_FORWARD)
        , wheelbase(1.0),C0(0.0),C1(0.0)
        , erp42_ptr(p), error_accel_tot(0.0), error_accel_prev(0.0) //tw
		, error_speed_tot(0.0), error_speed_prev(0.0), isEstop(false)  
        {
            nh1.param<double>("erp42_main/wheelbase", wheelbase, 1.0);
            ROS_INFO("--------------------------------IN Platform controller----------------------------------");
			ROS_INFO("1. Parameter check----------------------------------------------------------------------");
			ROS_INFO("wheelbase = %f", wheelbase);
            nh1.param<double>("erp42_main/settling_time", settling_time, 0.8);
            nh1.param<double>("erp42_main/weight", ss_speed_weight, 1.1);
            nh1.param<double>("erp42_main/shift", ss_speed_shift, 0.0);

            nh1.param<double>("/erp42_main/steer_kp", kp_steer, 1.0);
            nh1.param<double>("/erp42_main/steer_ki", ki_steer, 0.0);
            nh1.param<double>("/erp42_main/steer_kd", kd_steer, 0.0);

			nh1.param<double>("/erp42_main/accel_kp", accel_kp, 1.0);
			nh1.param<double>("/erp42_main/accel_ki", accel_ki, 0.0);
			nh1.param<double>("/erp42_main/accel_kd", accel_kd, 0.0);

            nh1.param<double>("/erp42_main/brake_kp", kp_brake, 50.0);
			nh1.param<double>("/erp42_main/brake_kp", ki_brake, 0.0);
            nh1.param<double>("/erp42_main/brake_kd", kd_brake, 0.0);
			
			nh1.param<double>("/erp42_main/C0", C0, 0.108); //acceleration [m/s^2] = 0.108*exp(0.0184*platform) 
			nh1.param<double>("/erp42_main/C1", C1, 0.0184);//acceleration [m/s^2] = 0.108*exp(0.0184*platform)
			ROS_INFO("C0 = %lf , C1 = %lf ", C0,C1);

            if (!nh1.getParam("/erp42_main/max_estop_time", MAX_ESTOP_TIME)) throw std::runtime_error("set erp42_main/max_estop_time!");

			
            sub_cmd = nh2.subscribe("twist_cmd_erp42", 100, &PlatformController::Cmd_Callback, this);
            estop_sub = nh2.subscribe("erp42_estop", 100, &PlatformController::estop_cb, this);
            accel_pub = nh3.advertise<std_msgs::Int32>("accel_state" ,10);
			last_time = ros::Time::now();
        	}
    
        	void Calc_PID(void){ //ref 와 cur 값, PID 변수를 이용하여 cmd_steer,speed,gear,brake 계산 
            		ros::Time current_time = ros::Time::now();
            		dt = (current_time - last_time).toSec();
            		last_time = current_time;
			
            		Calc_longitudinal(); // SPEED CONTROL
			Calc_lateral(); // STEER CONTROL
        	}

        	//erp42 current_speed and current_steer 
        	void SetState(ERP42_TO_PC current_){
			rx_erp_lock.lock();
            current_speed = current_.speed/10.0/3.6; //serial -> m/s
            if (current_.gear == 2 ) // backward
                current_speed *= -1;
            current_steer = current_.steer/71.0*DEG2RAD; //serial -> radian
			isAuto = current_.AorM;
            rx_erp_lock.unlock();
        }

        void estop_cb(const std_msgs::BoolConstPtr& ptr){
            if (ptr->data == true) isEstop = true;
            else isEstop = false;
        }

        void uncheckOldEstop(){
            //this function removes estop when it maintained too long
            static bool estop_prev = false;
            static ros::Time estop_start_time;
                
            if (isAuto == true && isEstop == true && estop_prev == false) estop_start_time = ros::Time::now();

            else if (isAuto == true && isEstop == true && 
                estop_prev == true && ((ros::Time::now() - estop_start_time).toSec() >= MAX_ESTOP_TIME)){
                ROS_ERROR("estop has maintained too long. uncheck estop");
                cmd_estop = 0;
                isEstop = false;
            }

            estop_prev = isEstop;
        }
    
        //update ref_speed and ref_steer 
        void Cmd_Callback(const geometry_msgs::TwistStampedConstPtr& twist){
			/* detect Manual->Auto pulse */
			if (isAuto == false) {
                isAutoStart = false;
                return;
            }
            else if (isAutoStart == false) { //pulse
                isAutoStart = true;
                error_accel_tot = 0;
                error_accel_prev = 0;
				error_speed_tot = 0;
				error_speed_prev =0;
            }

            if (isEstop == false){
                cmd_estop = 0;
            }
            else {
                error_accel_tot = 0;
                error_accel_prev = 0;
				error_speed_tot = 0;
				error_speed_prev =0;
                cmd_estop = 1;
            }

            //uncheckOldEstop();

			/* detect logic end */
			const double MY_EPSILON = 0.000001;
			double curvature = 0;
            
            ref_speed = twist->twist.linear.x; // m/s
            if(std::fabs(twist->twist.linear.x) < MY_EPSILON ||  std::fabs(twist->twist.angular.z) < MY_EPSILON){
                ref_steer = 0;
            }
            else{
                curvature = twist->twist.linear.x / twist->twist.angular.z;
                ref_steer = atan(wheelbase / curvature); // radian
                // jw 실험해보기
                // if(ref_steer <= STEER_EPSILON_FOR_PUREPURSUIT){
                //     ref_speed = 1.5;
                // }
            }

			rx_erp_lock.lock();
		    	if (!nh1.getParam("/erp42_main/C0", C0))
              			throw std::runtime_error("set c0");
            		if (!nh1.getParam("/erp42_main/C1", C1))
              			throw std::runtime_error("set c1");
  			if (!nh1.getParam("/erp42_main/accel_kp", accel_kp))
   			        throw std::runtime_error("set accel_kp");
			if (!nh1.getParam("/erp42_main/accel_ki", accel_ki))
              			throw std::runtime_error("set accel_ki");
			if (!nh1.getParam("/erp42_main/accel_kd", accel_kd))
              			throw std::runtime_error("set accel_kd");
	        	if (!nh1.getParam("/erp42_main/brake_kp", kp_brake))
              			throw std::runtime_error("set kp_brake ");
           		//For set param while test  
			Calc_PID();
			rx_erp_lock.unlock();
			erp42_ptr->changeERP42WriteBuf(cmd_gear, cmd_speed, cmd_steer, cmd_brake, cmd_estop);
			//cmd_speed, cmd_steer, cmd_gear, cmd_brake를 buffer에 써주는 함수 호출
        }
private:
    //function for calc pid 
    void Calc_longitudinal(){
    //---------calc GEAR & SPEED & BRAKE--------------------------------------------
	target_gear = (ref_speed >= 0.0) ? GEAR_FORWARD : GEAR_BACKWARD;

	if(target_gear != current_gear){  //1. 기어변환이 있다면
		if(fabs(current_speed) >  SWITHCING_GEAR_SPEED_TOLERANCE){ //1-1. 아직 속력이 높을 때  
          cmd_speed = NO_ACCEL;
          cmd_brake = NO_SLIP_BRAKE;
          cmd_gear = current_gear;
          return ;
        }  
        else{ //1-2. 일정 속도보다 낮아졌을 때 기어를 변화시켜준다
          cmd_gear = target_gear;
          current_gear = target_gear;
        }
        //---------cmd_gear확정------------------------------------------------------
    }
    double dir = (current_gear == GEAR_FORWARD) ? 1.0 : -1.0;
	err_speed = dir * (ref_speed - current_speed);//m/s 
	target_accel = err_speed / settling_time; //가속도
    ss_speed = (int)(ss_speed_shift + ss_speed_weight * fabs(ref_speed) * M_S2SERIAL);
		
		
	std_msgs::Int32 acc_or_dacc_msg;

    error_accel_tot += target_accel * dt;
    double error_accel_d = (target_accel - error_accel_prev) / dt ;
    error_accel_prev = target_accel;

	if(target_accel > (C0 + EPSILON)){ // 가속(Acceleration)
			double min_accel =  C0 * exp(C1 * ref_speed*M_S2SERIAL);
			acc_or_dacc_msg.data = 1;

            /* calc pid for accel */
            double target_accel_pid =
                target_accel * accel_kp +
                error_accel_tot * accel_ki +
                error_accel_d * accel_kd;
            cmd_speed = (target_accel > min_accel) ? (uint16_t)(log(target_accel_pid/C0) / C1) : ss_speed;
            cmd_brake = NO_BRAKE;

   }
   else{ // 가속 필요 X or 감속(Deceleration)
		acc_or_dacc_msg.data = 0;
		cmd_speed = ss_speed; // 현재 내야하는 가속도
        
		cmd_brake = fabs(err_speed) * (kp_brake + ki_brake*dt + kd_brake/dt);
            
    	if(cmd_brake <= NO_BRAKE){
           	cmd_brake = NO_BRAKE;
        }
        else{
            cmd_brake = (cmd_brake <= (uint8_t)brake_max) ? cmd_brake : (uint8_t)brake_max ;
        }
        //--------cmd_brake, cmd_brake확정---------------------------------------    
    }
    accel_pub.publish(acc_or_dacc_msg);
  }
  void Calc_lateral(){
  //---------calc STEER------------------------------------------------------   
  int16_t ref_steer_serial = RAD2SERIAL * ref_steer;
  cmd_steer = (fabs(ref_steer_serial) <= MAX_STEER) ? ref_steer_serial : MAX_STEER*((ref_steer_serial)/fabs(ref_steer_serial));
  cmd_steer *= -1;
  //---------cmd_steer확정---------------------------------------------------
  }
    private:
        ros::NodeHandle nh1;
        ros::NodeHandle nh2;
		ros::NodeHandle nh3;
        ros::Subscriber sub_cmd;
        ros::Subscriber estop_sub;
        ros::Publisher accel_pub;
		ros::Time last_time;

        std::mutex rx_erp_lock;
        double wheelbase;
        double dt;
        //for rx 
        double ref_speed, current_speed, err_speed;//m/s
        double ref_steer, current_steer, err_steer;//radian
        //for tx 
        uint8_t cmd_brake, cmd_gear;
        uint16_t cmd_speed;
        int16_t cmd_steer;
        uint8_t cmd_estop;
        //for PID_CALC
        double settling_time;
        double kp_steer, ki_steer, kd_steer;
        double kp_brake, ki_brake, kd_brake;
	double accel_kp, accel_ki, accel_kd, error_accel_tot, error_accel_prev;
	double error_speed_tot, error_speed_prev;
        double ss_speed_weight, ss_speed_shift;
        uint16_t ss_speed;
        double target_accel;
        uint8_t current_gear, target_gear; 
        int brake_max;
	double C0,C1;		
	bool isAuto, isAutoStart;
    bool isEstop;
    double MAX_ESTOP_TIME;
        //for serial write
        ERP42* erp42_ptr;
    };
//PlatformController END---------------------------------------------------------------------------------------
public:
    void create(){ //it can throw serial exceptions
        /* get ros parameters */
        std::string port;
        if (!nh.getParam("/erp42_main/frequency", hz))       throw std::runtime_error("set erp42_main/frequency!");
        if (!nh.getParam("/erp42_main/port", port))          throw std::runtime_error("set erp42_main/port!");
        bool initWriteThread;
        if (!nh.getParam("/erp42_main/initWriteThread", initWriteThread))	throw std::runtime_error("set erp42_main/initWriteThread!");
        
        /* initialize tx msg */
        PC_TO_ERP42 *p = (PC_TO_ERP42*) pc_to_erp42_buf;
        p->S            = 'S';
        p->T            = 'T';
        p->X            = 'X';
        p->AorM         = 1;//
        p->estop        = 0;//
        p->gear         = 0;//
        p->alive        = 0;
        p->ETX0         = 0x0D;
        p->ETX1         = 0x0A;        

        /* iniialize serial port */
        serial.setPort(port);
        serial.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(50);
    	serial.setTimeout(to);
        serial.open();
        if (false == serial.isOpen()){
            ROS_ERROR("Serial.open() error with port %s", port.c_str());
            throw serial::PortNotOpenedException(port.c_str());
        }//since i didn't set the timeout, it's nonblocking file
        //find the trailor of the msg 
        uint8_t t;
        do{
		serial.read(&t, 1);
		ROS_INFO("syncing... %x", t);
        } while(t != 0x0A);
        ROS_INFO("open erp42 serial device success!");

        /* initialize ros members */
        erp42_to_pc_pub = nh.advertise<erp42_msgs::erp42_to_pc>("erp42_to_pc", 100);
        erp42_degree_publisher = nh.advertise<std_msgs::Float64>("steer_degree", 100);
        erp42_velocity_publisher = nh.advertise<std_msgs::Float64>("velocity_debug", 100); 
        erp42_vehicle_status_pub = nh.advertise<autoware_msgs::VehicleStatus>("vehicle_status_1", 1000);
	    erp42_steer_publisher = nh.advertise<std_msgs::Float64>("steer_debug", 100); 
        
        /* initialize odometry object */
        odom.create();
        
	/* generate erp42 controller */
	ROS_INFO("generate controller instance");
	controller_ptr = new PlatformController(this);

        /* init write thread */
        if (initWriteThread){
            std::thread(&ERP42::writeSerialThread, this).detach();
            ROS_INFO("init writeSerialThread!");
        }
        std::thread(&ERP42::readSerialThread, this).detach();
        ROS_INFO("init writeSerialThread!");
    }

    void read(){
        static uint8_t erp42_to_pc_tmp[ERP42_TO_PC_LEN];
        if (serial.available() >= ERP42_TO_PC_LEN){
            serial_mutex.lock();
            size_t read_sz = serial.read(erp42_to_pc_tmp, ERP42_TO_PC_LEN);
            serial_mutex.unlock();
        }
        else return;
        //if (read_sz != ERP42_TO_PC_LEN) return;

        
        /* check header and tailer */
        ERP42_TO_PC* erp42_to_pc_tmp_ptr = (ERP42_TO_PC*)erp42_to_pc_tmp;
        if (erp42_to_pc_tmp_ptr->S     != 'S')         return;
        if (erp42_to_pc_tmp_ptr->T     != 'T')         return;
        if (erp42_to_pc_tmp_ptr->X     != 'X')         return;
        if (erp42_to_pc_tmp_ptr->ETX0  != 0x0D)        return;
        if (erp42_to_pc_tmp_ptr->ETX1  != 0x0A)        return;

        /* copy */
        memcpy(erp42_to_pc_buf, erp42_to_pc_tmp, ERP42_TO_PC_LEN);

        /* publish rx msg */
        ERP42_TO_PC* erp42_to_pc_ptr = (ERP42_TO_PC*)erp42_to_pc_buf;
        
        erp42_msgs::erp42_to_pc msg;
		msg.header.stamp= ros::Time::now();
        msg.isAuto      = erp42_to_pc_ptr->AorM == 1        ? true : false;
        msg.isEstop     = erp42_to_pc_ptr->estop == 1       ? true : false;
        msg.isForward   = erp42_to_pc_ptr->gear == 0        ? true : false;
        msg.isBackward  = erp42_to_pc_ptr->gear == 2        ? true : false;
        msg.speed       = erp42_to_pc_ptr->speed / 10.0 / 3.6; // /10 is actual kph. divided by 3.6 to change kph to mps
        msg.steer       = erp42_to_pc_ptr->steer / 71.0 * DEG2RAD; //counter clockwise direction(left) is positive
	msg.brake       = erp42_to_pc_ptr->brake / 2;
        msg.encoder     = erp42_to_pc_ptr->encoder;
        msg.alive       = erp42_to_pc_ptr->alive;
        erp42_to_pc_pub.publish(msg);

        /* publish /vehicle_status for autoware's mpc follower */
        autoware_msgs::VehicleStatus vehicle_status_msg;
        vehicle_status_msg.header.stamp = ros::Time::now();
        vehicle_status_msg.angle = msg.steer;
        vehicle_status_msg.speed = msg.speed * 3.6; //mps->kph
        if (msg.isBackward) vehicle_status_msg.speed *= -1;
        //erp42_vehicle_status_pub.publish(vehicle_status_msg);


        /* test for steer degree */
	    std_msgs::Float64 degree_msgs;
	    degree_msgs.data = erp42_to_pc_ptr->steer / 71.0;	
        erp42_degree_publisher.publish(degree_msgs);

        /* update and publish odometry msg */
        odom.updateAndPublish(*erp42_to_pc_ptr);
	    
        /* update current speed and steer for controller */
	    controller_ptr->SetState(*erp42_to_pc_ptr);
    }

    void readSerialThread(){
        ros::Rate r(hz);
        while(ros::ok()){
            read();
            r.sleep();
        }
    }

    void destroy(){
        if (true == serial.isOpen())
            serial.close();
    }

    void changeERP42WriteBuf(uint8_t gear, /* 0:Forward, 1:Neutral, 2:Backward */
        uint16_t speed_le, /* 0 ~ 200 */
        int16_t steer_le, /* -2000 ~ 2000 */
        uint8_t brake, /* 0 ~ 200 */
        uint8_t estop) /* 0 or else. 0 is non-estop */
    {
        write_buf_mutex.lock();
        PC_TO_ERP42 *p = (PC_TO_ERP42*) pc_to_erp42_buf;
        
        if ((0 <= gear) && (gear <= 2)) p->gear = gear;
        else p->gear = 1; //neutral
        
        speed_le = std::max<uint16_t>(speed_le, 0); speed_le = std::min<uint16_t>(speed_le, 200); //min max bound
        uint16_t speed_be = htobe16(speed_le); // speed
        p->speed = speed_be;

        steer_le = std::max<int16_t>(steer_le, -2000); steer_le = std::min<int16_t>(steer_le, 2000);
        int16_t steer_be = htobe16(steer_le);
        p->steer = steer_be;

        p->brake = std::min<uint8_t>(brake, 200); //no need to use std::max
        
        p->estop = estop;
	    write_buf_mutex.unlock();

        std_msgs::Float64 msg;
	    msg.data = speed_le / 200.0;
        erp42_velocity_publisher.publish(msg);

        msg.data = steer_le / 2000.0;
        erp42_steer_publisher.publish(msg);
	    
    }

    void writeSerialThread(){
        ROS_INFO("start Write Thread");
        ros::Rate r(20); // 20hz, 50ms
        PC_TO_ERP42 *p = (PC_TO_ERP42*) pc_to_erp42_buf;
        while(ros::ok()){
            serial_mutex.lock();
            write_buf_mutex.lock();

            p->alive++;
            
            serial.write(pc_to_erp42_buf, PC_TO_ERP42_LEN);

            write_buf_mutex.unlock();
            serial_mutex.unlock();
            r.sleep();
        }
    }
private:
    /* erp42 related members */
    uint8_t pc_to_erp42_buf[PC_TO_ERP42_LEN];
    uint8_t erp42_to_pc_buf[ERP42_TO_PC_LEN];
    serial::Serial serial;
    std::mutex serial_mutex, write_buf_mutex;

    /*ros related members */
    ros::NodeHandle nh;
    ros::Publisher erp42_to_pc_pub;
    ros::Publisher erp42_degree_publisher; //steer test
    ros::Publisher erp42_velocity_publisher;
    ros::Publisher erp42_vehicle_status_pub;
    ros::Publisher erp42_steer_publisher;
    /* param */
    int hz;

    /* odometry */
    CarlikeOdometry odom;
	
    /* controller */
    PlatformController* controller_ptr;
};

ERP42 *erp42_ptr;

void mySigintHandler(int sig)
{
    erp42_ptr->destroy();
    ros::shutdown();
    exit(17);
}

                                          
int main(int argc, char *argv[]){
    ros::init(argc, argv, "erp42_main", ros::init_options::NoSigintHandler); 
    signal(SIGINT, mySigintHandler);

    erp42_ptr = new ERP42();
    try{
        erp42_ptr->create();
        ros::spin();
    }
    catch(std::runtime_error& e){
        ROS_ERROR("runtime : %s", e.what());
    }
    catch(serial::PortNotOpenedException){
        ROS_ERROR("erp42 driver : PortNotOpenedException");
    }
    catch(serial::SerialException){ 
        ROS_ERROR("erp42 driver : SerialExcepetion!");
    }
    catch(serial::IOException){ 
        ROS_ERROR("erp42 driver : Check Connection");
    }

    kill(getpid(), SIGINT);//send signal
}

