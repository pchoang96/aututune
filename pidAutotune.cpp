#include "iostream"
#include "stdlib.h"
#include </home/rageki/Downloads/arduino-1.8.8/hardware/arduino/avr/cores/arduino/Arduino.h>
//turning parametters
int sample = 0;
double sample_period = 0.02; //s depend on how frequenly you use pid
double kI=0.0 ,kP=0.0,kD = 0.0;
double topOut,botOUT,OUT_range[2]={0,225}; 
double triggre_mid,triggre_top,triggre_bot,trig_distance;
double react_max,react_min,cur_react; //reaction signal peek
double kU,pU,outDis,reactDis;
double t1,t2,save1,save2;
bool pikachu,react;
const double pi=3.141592654;
int cycle=0;
double kP[cycle]={0.0},kI[cycle]={0.0},kD[cycle]={0.0};
double kPavg,kIavg,kDavg;

void setup() {
/*preset the output jump distance*/
    topOut = OUT_range[0]+(OUT_range[1]-OUT_range[0])*3/4;
    botOUT = OUT_range[0]+(OUT_range[1]+OUT_range[0])*1/4;
}
void loop() {
    //
}

void tunePID() {
    if (sample<1){
        //write the top Ouput to device
        //pwmOUT(topOUT,topOUT,c_clkw,c_clkw);
        //---------------------------------------
        pikachu=true;
        react=true;
        sample ++;
    }
    else if (pikachu=true && cur_react>triggre_top) {
        if (react == true) {
            t1 = micros();
            pikachu = false;
            //write the bot Ouput to device
            //pwmOUT(botOUT,botOUT,c_clkw,c_clkw);
            //---------------------------------------
        } 
        else (react == false) {
            t2=micros();
            save1= t2-t1; //this is pU
            outDis=topOut-botOUT;
            reactDis = react_max -react_min;
            calTune(save1,reactDis,outDis);
            pikachu = false;
            //write the bot Ouput to device
            //pwmOUT(botOUT,botOUT,c_clkw,c_clkw);
            //---------------------------------
            t1=micros();
            sample ++;
        }
    }
    else if (pikachu = false && cur_react<triggre_bot) {
        react=false;
        //write the top Ouput to device
        //pwmOUT(topOUT,topOUT,c_clkw,c_clkw);
        //---------------------------------------
    }

}
void calTune(double pU, double A, double D){
    double kU=4*D/(A*pi);
    kP[sample-2] = 0.6*kU;
    kI[sample-2] = 1.2*kU/pU;
    kD[sample-2] = 0.075*kU*pU;
}
void end {
    for(int i=0,i<=cycle,i++){
        kPavg+=kP[i];
        kIavg+=kI[i];
        kDavg+=kD[i];
    }
    kPavg=kPavg/cycle;
    kIavg=kIavg/cycle;
    kDavg=kDavg/cycle;
}