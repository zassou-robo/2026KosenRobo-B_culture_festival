#include <mbed.h>
#include "controler.hpp"
#include "C620.hpp"
#include "PID_new.hpp"
#include "omuni.hpp"

dji::C620 c620(PB_12,PB_13);
CAN arduino(PA_11,PA_12, (int)1e6);

constexpr int c610_min = -10000;
constexpr int c620_min = -10000;
constexpr int c610_max = 10000;
constexpr int c620_max = 10000;

constexpr int drive_const = 3000;
constexpr int turn_const = 1500;

PidGain gain_c610{1.0, 0.1, 0.0};
PidGain gain_c620{2.0, 0.5, 0.0};
constexpr int NUM_MOTORS = 8;
std::array<Pid, NUM_MOTORS> pids = {{
    Pid({gain_c620, c620_min, c620_max}),
    Pid({gain_c620, c620_min, c620_max}),
    Pid({gain_c620, c620_min, c620_max}),
    Pid({gain_c620, c620_min, c620_max}),
    Pid({gain_c610, c610_min, c610_max}),
    Pid({gain_c610, c610_min, c610_max}),
    Pid({gain_c610, c610_min, c610_max}),
    Pid({gain_c610, c610_min, c610_max})
}};
BufferedSerial pc(USBTX,USBRX,115200);
CANMessage msg;
Ps5 ps5;
omuni_param param{drive_const,turn_const};
Omuni omuni({param});


constexpr int roller_send_id = 200;
constexpr int motor_id_one = 35;
constexpr int servo_id = 140;

DigitalIn arc_u(PC_1,PullUp);
DigitalIn arc_d(PC_3,PullUp);
DigitalIn neo_t(PC_0,PullUp);//ネオPの射出
DigitalIn neo_r(PC_2,PullUp);//ネオPの装填

DigitalIn meI(PC_8,PullUp);
DigitalIn meI_2(PC_9,PullUp);

constexpr int arc_power = 5000;
constexpr int neopino_power_h = 8000;
constexpr int neopino_power_m = 7000;
constexpr int neopino_power_l = 6000;
constexpr int mi_power = 15000;
constexpr int mi_ud_power = 5000;

int16_t bl_goal_rpm[8] = {0};
int16_t bl_actu_rpm[8] = {0};
int16_t bl_goal_pos[8] = {0};
int16_t bl_actu_pos[8] = {0};
int16_t dc_out_one[4] = {0};
int16_t bl_pre_pos[8] = {0};
uint8_t servo_out[8] = {0};
uint8_t roller_signal[8] = {0};

bool robot_mode = 0;
bool reload = 1;
int reload_time = 0;

//各機構出力初期化
int neopino_out = 0;


int main(){
    c620.read_data();
    while(1){
        if(ps5.read(arduino)){
            static bool pre_option = ps5.option;
            if (ps5.option && !pre_option)
            {
                robot_mode = !robot_mode;
            }
            pre_option = ps5.option;
            omuni.omuni_calc(ps5.lstick_x,ps5.lstick_y,ps5.rstick_x,bl_goal_rpm,0);
            if(robot_mode){
                roller_signal[1] = 1;
                roller_signal[0] = 0;
                if(ps5.cross){
                    roller_signal[0] = 1;
                }
            }else{
                roller_signal[0] = 0;
                roller_signal[1] = 0;
                if(ps5.circle && !reload){
                    neopino_out = -neopino_power_m;
                    if(neo_t == 0){
                        neopino_out += neopino_power_m;
                        reload = 1;
                    }
                }else if(ps5.triangle && !reload){
                    neopino_out = -neopino_power_l;
                    if(neo_t == 0){
                        neopino_out += neopino_power_l;
                        reload = 1;
                    }
                }else if(ps5.square && !reload){
                    neopino_out = -neopino_power_h;
                    if(neo_t == 0){
                        neopino_out += neopino_power_h;
                        reload = 1;
                    }
                }else{
                    if(reload){
                        neopino_out = 2500;
                        reload_time ++ ;
                        if(reload_time > 150){
                            reload = 0;
                            reload_time = 0;
                            neopino_out = 0;
                        }
                    }else{
                        neopino_out = 0;
                    }
                }
            }
        }
    }
}