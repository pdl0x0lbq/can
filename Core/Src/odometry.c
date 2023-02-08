#include "odometry.h"
#include "can.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "usr_moto.h"


//机械结构部分  减速比 是 1:50
#define REDUCTION_RATIO	50
//wheel base(distance between shafts):678mm
#define WHELL_BASE 0.678
//wheel diameter : 220mm
#define WHEEL_DIAMETER	0.22
//thickness of crawler:23mm
#define CREWLER_THICKNESS	0.023
//wheel actual diameter:  220mm+23mm=243mm
#define ACTUAL_WHEEL_DIAMETER 0.243
//parameter
#define PARAMETER_K	1.876//1.721:0.066m/s(0.06m/s)    1.432:0.08m/s(0.06m/s)    1.176:0.036m/s(0.06m/s)  1.578:0.07m/s(0.06m/s)   idle parameter:1.96

float robot_wheel_circumfernce   = PI * ACTUAL_WHEEL_DIAMETER ;    // 轮子的周长  π*d=0.763m   单位m

// 右边速度  左轮速度  x y  角度 线速度 角速度

extern uint8_t speed_data[10], auto_speed_data[10]; //底盘接收的数据   左轮速度(4byte) + 右轮速度(4byte) +'\r'+'\n'
union   floatdata   vel_r, vel_l, left_speed, right_speed, positon_x, positon_y, oriention, vel_linear, vel_angular;
union   floatdata   report_vel_linear, report_vel_angular;
unsigned int ileft_speed = 0, iright_speed = 0;

extern uint8_t  usart2_flag, usart3_flag, usart5_flag;
extern unsigned short int wifi_ctl_data;

extern unsigned long int EncoderValues_L, EncoderValues_R;

int FloatToInt(float source)
{
    union IntFloat ret, bias;
    ret.f = source;
    bias.i = ((23 + 127) << 23);  //  ????????? 1.0 * 2^23
    ret.f += bias.f;
    ret.i -= bias.i;
    return ret.i;
}

int r_speed = 0, l_speed = 0;
void line_speed_angular_speed_2_motospeed(float line_speed, float angular_speed){ //Forward kinematics
    if(line_speed > 0)//线速度大于零
    {		//vr = v + (l*w)/2, vl = v - (l*w)/2; MotoSpeed = (writeSpeed / 8192) * 3000
        vel_r.d = ((2 * line_speed) + (angular_speed * WHELL_BASE)) / 2;
        vel_l.d = ((2 * line_speed) - (angular_speed * WHELL_BASE)) / 2;		//_vr = ((vr / (PI * R)) * N) * 8192 / 3000  (rmp/s), _vl = ((vl / (PI * R)) * N) * 8192 / 3000   (rmp/s)
        vel_r.d = (vel_r.d / robot_wheel_circumfernce) * REDUCTION_RATIO * 2.731 * 60 / PARAMETER_K;//1.176;//1.96
        vel_l.d = (vel_l.d / robot_wheel_circumfernce) * REDUCTION_RATIO * 2.731 * 60 / PARAMETER_K;//1.176

        vel_l.d = vel_l.d * PARAMETER_K;
        vel_r.d = vel_r.d * PARAMETER_K;
        if(angular_speed > 0 || angular_speed < 0) {//转弯
#ifdef MOTO_DIRECTION_F
            r_speed = FloatToInt(vel_r.d);
				l_speed = FloatToInt(vel_l.d);
#elif MOTO_DIRECTION_B
            r_speed = FloatToInt(vel_l.d);
				l_speed = FloatToInt(vel_r.d);
#endif
        }else{//直行
            r_speed = FloatToInt(vel_r.d);
            l_speed = FloatToInt(vel_l.d);
        }
    }
    else if(line_speed < 0)	{//线速度小于零
        line_speed = 0 - line_speed;
        vel_r.d = ((2 * line_speed) + (angular_speed * WHELL_BASE)) / 2;
        vel_l.d = ((2 * line_speed) - (angular_speed * WHELL_BASE)) / 2;		//_vr = ((vr / (PI * R)) * N) * 8192 / 3000  (rmp/s), _vl = ((vl / (PI * R)) * N) * 8192 / 3000   (rmp/s)
        vel_r.d = (vel_r.d / robot_wheel_circumfernce) * REDUCTION_RATIO * 2.731 * 60 / PARAMETER_K;//1.176;//1.96
        vel_l.d = (vel_l.d / robot_wheel_circumfernce) * REDUCTION_RATIO * 2.731 * 60 / PARAMETER_K;//1.176
        vel_l.d = vel_l.d * PARAMETER_K;
        vel_r.d = vel_r.d * PARAMETER_K;//		r_speed = 0 - FloatToInt(vel_r.d);//		l_speed = 0 - FloatToInt(vel_l.d);
        if(angular_speed > 0 || angular_speed < 0)//转弯
        {
#ifdef MOTO_DIRECTION_F
            r_speed = 0 - FloatToInt(vel_r.d);
				l_speed = 0 - FloatToInt(vel_l.d);
#elif MOTO_DIRECTION_B
            r_speed = 0 - FloatToInt(vel_l.d);
				l_speed = 0 - FloatToInt(vel_r.d);
#endif
        }else{//直行
            r_speed = 0 - FloatToInt(vel_r.d);
            l_speed = 0 - FloatToInt(vel_l.d);
        }
    }
    else if(angular_speed > 0 || angular_speed < 0)	{//原地掉头
        vel_r.d = ((angular_speed * WHELL_BASE)) / 2;
        vel_l.d = ((angular_speed * WHELL_BASE)) / 2;
        vel_r.d = (vel_r.d / robot_wheel_circumfernce) * REDUCTION_RATIO * 2.731 * 60 / PARAMETER_K;//1.176;//1.96
        vel_l.d = (vel_l.d / robot_wheel_circumfernce) * REDUCTION_RATIO * 2.731 * 60 / PARAMETER_K;//1.176

#ifdef MOTO_DIRECTION_F
        if(angular_speed > 0){//右原
			l_speed = FloatToInt(vel_l.d);
			r_speed = (0 - FloatToInt(vel_r.d));
		}
		else{//左原
			r_speed = (0 - FloatToInt(vel_l.d)) / 2;
			l_speed = FloatToInt(vel_r.d) / 2;
		}
#elif MOTO_DIRECTION_B
        r_speed =FloatToInt(vel_l.d);
			l_speed = 0 - FloatToInt(vel_r.d);
#endif
    }
    else	{		r_speed = l_speed = 0;	}
}

float velR, velL;
extern short int right_rpm, left_rpm,right_get_flag, left_get_flag;
void motospeed_2_line_speed_angular_speed(){ //inverse kinematics
    unsigned char AutoDriverTxData[10] = {0};
    float right_rps ,left_rps ;
    unsigned char count_i= 0;
//	left_rpm  *= 1000;  // 转每分钟*1000 ; 扩大1000倍
//	right_rpm *= 1000;
//////	left_rpm  = left_rpm  * 0.366;//3000 / 8192;
//////	right_rpm = right_rpm  * 0.366;//3000 / 8192;//电机转速读出值转变为实机值

//////	left_speed.d = (left_rpm / REDUCTION_RATIO)  *	robot_wheel_circumfernce; //左轮履带转速
//////	right_speed.d = (right_rpm / REDUCTION_RATIO) *	robot_wheel_circumfernce; //右轮履带转速
//////	left_speed.d = 0 - left_speed.d;//电机一个正转一个反转，切换方向
//////
//////	vel_linear.d =  (left_speed.d + right_speed.d) / 2 ;   //线速度
//////	vel_angular.d = (left_speed.d - right_speed.d) / WHELL_BASE ; //角速度
//////	report_vel_linear.d= (vel_linear.d) / 60; // rps
//////	report_vel_angular.d  = (vel_angular.d) / 60;	//rps
    left_rpm = 0 - left_rpm;
    right_rpm = 0 - right_rpm;
    report_vel_linear.d = 0.0000;
    report_vel_angular.d = 0.0000;
    report_vel_linear.d =  ((left_rpm + right_rpm) / 22800.000);   //线速度
    report_vel_angular.d = ((left_rpm - right_rpm) / 22800.000) * 2.95 * 2; //角速度
    report_vel_linear.d =  0 - report_vel_linear.d;
    report_vel_angular.d = 0 - report_vel_angular.d;

//	 report_vel_linear.d = report_vel_linear.d * report_vel_linear.d;
//	 report_vel_angular.d = report_vel_angular.d * report_vel_angular.d;

    AutoDriverTxData[0] = 0xcc;
    for(count_i = 0; count_i < 4; count_i++){
        AutoDriverTxData[count_i + 1] = report_vel_linear.data[count_i];
        AutoDriverTxData[count_i + 5] = report_vel_angular.data[count_i];
    }
    AutoDriverTxData[9] = 0xdd;

}

//void get_inc(void)  // 得到里程计
//{//		float right_rpm ,left_rpm ;//		float right_rps ,left_rps ;//	  double inc ; // 单位 m//		uint32_t  last_time = 0 , current_count = 0 ;//		// current_count   =  time5_count   ;//		if(time5_count - current_count > 50) { // 10ms *50 = 500ms;		//			//time5_count = 0 ;
//			left_rpm = read_motor1_speed();   //得到左右轮子的速度
//			right_rpm = read_motor2_speed();
//			//			left_rpm*= 1000;  // 转每分钟*1000 ; 扩大1000倍//			right_rpm*=1000;

//			left_speed.d = left_rpm/REDUCTION_RATIO *	robot_wheel_circumfernce ;   //左轮线速度
//			right_speed.d = right_rpm/REDUCTION_RATIO *	robot_wheel_circumfernce ; //右轮线速度
//
//			vel_linear.d =  (left_speed.d +right_speed.d)/2 ;   //线速度
//			vel_angular.d = (left_speed.d - right_speed.d)/2*robot_wheel_witdth ; //角速度
//			oriention.d = (left_speed.d - right_speed.d)/robot_wheel_witdth ; //角度方向
//
//			inc +=  vel_linear.d*robot_wheel_circumfernce*0.5 ; // 线速度 * 直径
//
//			right_rps= right_rpm/60; // rps
//			left_rps = left_rpm/60 ;	//rps
//		}
//}

unsigned int stop_i = 0;
float line_speed = 0.0, angular_speed = 0.0;
void speed_transformation_send(void)    // 速度转换并发送
{

        robot_base_control(100, 100);  // 发送速度值给轮子


}

void robot_base_control(int l_speed, int r_speed)   //发送左轮速度、再发送右轮速度
{
    MotoSpeedSet(LEFT, l_speed);
    HAL_Delay(5);
    MotoSpeedSet(RIGHT, r_speed);
    HAL_Delay(5);
}


//float  get_speed(void)  //得到线速度
//{
//		float right_rpm ,left_rpm,speed_tmp ;

//		left_rpm = read_motor1_speed();   //得到左右轮子的速度
//	  vTaskDelay(50);
//		right_rpm = read_motor2_speed();
//
//	  printf("left_rpm :%.3f\n",left_rpm);
//    printf("right_rpm :%.3f\n",right_rpm);
//		left_rpm*= 1000;  // 转每分钟*1000 ; 扩大1000倍
//		right_rpm*=1000;

//		left_speed.d = left_rpm/REDUCTION_RATIO *	robot_wheel_circumfernce ;   //左轮线速度
//		right_speed.d = right_rpm/REDUCTION_RATIO *	robot_wheel_circumfernce ; //右轮线速度
//
//		speed_tmp =  (left_speed.d +right_speed.d)/2/1000/60 ;   //线速度  m/s
//
//	  return speed_tmp ;
//}
