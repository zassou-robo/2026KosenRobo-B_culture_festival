#ifndef OMUNI_HPP
#define OMUNI_HPP

#include <mbed.h> 

struct omuni_param{
    int const_output_drive;
    int const_output_turn;
};

class Omuni{
    public:
    Omuni(const omuni_param const_output_): const_output_(const_output_){};
    void omuni_calc(int8_t lstick_x,int8_t lstick_y,int8_t rstick_x,int16_t goal[8],bool straight){
        int16_t drive_goal[4] = {0};
        int16_t turn_goal[4] = {0};
        if(straight == 0){//横向きでないとき(普通のやつ)
            if(abs(lstick_x) > 20 ||abs(lstick_y) > 20){
                for(int i = 0;i < 4;i++){
                    drive_goal[i] = sin(atan2(lstick_y,lstick_x) + (i*2 + 1) * M_PI / 4) * hypot(lstick_x,lstick_y) / 127 * const_output_.const_output_drive;
                }
            }else{
                for(int i = 0;i < 4;i ++){
                    drive_goal[i] = 0;
                }
            }
            if(abs(rstick_x) > 20){
                for(int i = 0;i < 4;i ++){
                    turn_goal[i] = rstick_x / 127.0 * const_output_.const_output_turn;
                }
            }else{
                for(int i = 0;i < 4; i++){
                    turn_goal[i] = 0;
                }
            }
        }else{
            if(lstick_x > 40){
                drive_goal[0] = hypot(lstick_x,lstick_y) / 127 * const_output_.const_output_drive;
                drive_goal[1] = hypot(lstick_x,lstick_y) / 127 * const_output_.const_output_drive;
                drive_goal[2] = hypot(lstick_x,lstick_y) / 127 * const_output_.const_output_drive * -1;
                drive_goal[3] = hypot(lstick_x,lstick_y) / 127 * const_output_.const_output_drive * -1;
            }else if(lstick_x < -40){
                drive_goal[0] = hypot(lstick_x,lstick_y) / 127 * const_output_.const_output_drive * -1;
                drive_goal[1] = hypot(lstick_x,lstick_y) / 127 * const_output_.const_output_drive * -1;
                drive_goal[2] = hypot(lstick_x,lstick_y) / 127 * const_output_.const_output_drive;
                drive_goal[3] = hypot(lstick_x,lstick_y) / 127 * const_output_.const_output_drive;
            }else if(lstick_y < -40){
                drive_goal[0] = hypot(lstick_x,lstick_y) / 127 * const_output_.const_output_drive * -1;
                drive_goal[1] = hypot(lstick_x,lstick_y) / 127 * const_output_.const_output_drive;
                drive_goal[2] = hypot(lstick_x,lstick_y) / 127 * const_output_.const_output_drive * -1;
                drive_goal[3] = hypot(lstick_x,lstick_y) / 127 * const_output_.const_output_drive;
            }else if(lstick_y > 40){
                drive_goal[0] = hypot(lstick_x,lstick_y) / 127 * const_output_.const_output_drive;
                drive_goal[1] = hypot(lstick_x,lstick_y) / 127 * const_output_.const_output_drive * -1;
                drive_goal[2] = hypot(lstick_x,lstick_y) / 127 * const_output_.const_output_drive;
                drive_goal[3] = hypot(lstick_x,lstick_y) / 127 * const_output_.const_output_drive * -1;
            }else{
                for(int i = 0; i < 4 ; i++){
                    drive_goal[i] = 0;
                }
            }
            if(abs(rstick_x) > 20){
                for(int i = 0;i < 4;i ++){
                    turn_goal[i] = rstick_x / 127.0 * const_output_.const_output_turn;
                }
            }else{
                for(int i = 0;i < 4; i++){
                    turn_goal[i] = 0;
                }
            }
        }
            for(int i = 0; i < 4;i ++){
                goal[i] = drive_goal[i] + turn_goal[i];
            }
        }
    private:
    omuni_param const_output_;
};
#endif 