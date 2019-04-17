#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h> //x,y,theta float 64
#include <geometry_msgs/Point.h> //x,y,z loat 64


#define clkw        0
#define c_clkw      1

#define encodPinA1  3
#define M1_p        6
#define M1_l        7
#define encodPinA2  2
#define M2_p        5
#define M2_l        4
#define DEBUG       1

/**----------------pid position calculator------------------------------------------**/
/* 
 * pid for motor:
 * period time: 20 ms
 * peek time: 400 ms
 * max speed (setting): value: 25/0.02 = 1250 pulses/s = 981.75 mm/s  <vm = vp*65*pi/wheels_encoder>
 * smallest speed (runable speed): value: 5/0.02= 250 pulses/s = 196.35mm/s
*/

/*-----------------------------------------------------------------------------------------------*/

//turning parametters
const int cycle=24, wait= 300;//cycles, ms;
double topOUT,botOUT,OUT_range[2]={0,225}; 
double triggre_mid,triggre_top,triggre_bot;
const double trig_distance=2;
double react_max,react_min,cur_react; //reaction signal peek
double kU,pU,outDis,reactDis;
double t1,t2,save1;
volatile int sample = 0,i=0;
double kP[cycle]={},kI[cycle]={},kD[cycle]={};
double kPavg,kIavg,kDavg;
bool pikachu,ending=false;
/*-----------------------------------------------------------------------------------------------*/
/**--------------------Control signal variable-----------------------------------------------------------------------------------**/
volatile double ang_vel=0,lin_vel=0;
double l_v,l_vt;
double r_v,r_vt; // pwm: pwm output. lv: mm/sec. lvt: tic/delta_t l:lert, r: right 
bool l_dir=clkw, r_dir=clkw;
/**-----------------------pid velocity calculation-------------------------------------------**/
volatile double  l_error=0.0,l_pre_error=0.0,l_integral=0.0,l_derivative=0.0,l_Ppart=0.0,l_Ipart=0.0,l_Dpart=0.0,l_out,l_set,l_ms,l_pre_out=0;
//double const l_kP = 0.72, l_kI=25.205 ,l_kD = 0.005;
double l_kP = 0.81, l_kI=20.06,l_kD = 0.01;
volatile double  r_error=0.0,r_pre_error=0.0,r_integral=0.0,r_derivative=0.0,r_Ppart=0.0,r_Ipart=0.0,r_Dpart=0.0,r_out,r_set,r_ms,r_pre_out=0;
double r_kP = 0.81, r_kI=20.06,r_kD = 0.01;
//double const r_kP = 0.72, r_kI=25.205,r_kD = 0.005;
/**--------------------------car parameter-----------------------------------------------**/
const double pi=3.141592654;
const double sampletime = 0.02, inv_sampletime = 1/sampletime,timer_set=65535-sampletime*250000;
const double wheels_distance = 207, wheels_radius = 31, wheels_diameter=62,wheels_encoder = 440 ;// mm
const double wheel_ticLength = wheels_diameter*pi/wheels_encoder;
/*--------------------------position calculation----------------------------*/
double p_org[]={0.0,0.0,0.0}, p_now[]= {0.0,0.0,0.0}; //{x,y,phi}
int l_p=0,r_p=0;
double l_d=0,r_d=0;

//time period for connecting
int pos_sampleTime=50;
long int setting_millis=millis()+pos_sampleTime;
ros::NodeHandle  nh;
/*----------------------Subscriber define------------------------------------------------*/
void messageCb(const geometry_msgs::Twist& vel)
{
  lin_vel=vel.linear.x*1000;
  ang_vel=vel.angular.z;//*180/pi;
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", messageCb );

void messageCb2(const geometry_msgs::Twist& velo)
{
  lin_vel=velo.linear.x*1000;
  ang_vel=velo.angular.z;//*180/pi;
}

ros::Subscriber<geometry_msgs::Twist> suby("/turtle1/cmd_vel", messageCb2 );
/*----------------------Publisher define------------------------------------------------*/
geometry_msgs::Pose2D postef;
ros::Publisher pos_temp("position", &postef);

geometry_msgs::Point lin_ang;
ros::Publisher test_temp("velocity", &lin_ang);
/*
  rosrun rosserial_python serial_node.py /dev/ttyACM0 (aka /dev/tty<-arduino port->)
 * using rostopic to manage and checking topic
 * set baud rate for connection by these lines:
 *  nh.getHardware()->setBaud(250000);
 * rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=1000000
 * .
*/

void setup()
{
  nh.getHardware()->setBaud(1000000);
 // rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=250000
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(suby);
  nh.advertise(pos_temp);
  nh.advertise(test_temp);
  pinMode(M1_l,OUTPUT);
  pinMode(M2_l,OUTPUT);
  pinMode(encodPinA1, INPUT_PULLUP);                  // encoder input pin
  pinMode(encodPinA2, INPUT_PULLUP);
  attachInterrupt(0, encoder_1 , FALLING);               // update encoder position
  attachInterrupt(1, encoder_2 , FALLING);
  //--------------setup timer------------------------------------------ 
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;
  TCCR1B |= (1 << CS11) | (1 << CS10);    // prescale = 64 4us per pulse
  TCNT1 = timer_set; //(12500*4)=50ms
  TIMSK1 |= (1 << TOIE1);                  // Overflow interrupt enable 
      /*preset the output jump distance*/
    topOUT = OUT_range[0]+(OUT_range[1]-OUT_range[0])*3/4;
    botOUT = OUT_range[0]+(OUT_range[1]+OUT_range[0])*1/4;
  sei();                                  // enable all interrupt
}

void loop()
{
   while(!ending){
        tunePID();
        if (sample>=cycle){
            ending = end_();
            l_kP = r_kP = kPavg;
            l_kI = r_kI = kIavg;
            l_kD = r_kD = kDavg;
        }
    }
  if (millis()>=setting_millis)
    {
      postef.x     = p_now[0];
      postef.y     = p_now[1];
      postef.theta = p_now[2];
      pos_temp.publish(&postef);
      
      lin_ang.x = r_d*inv_sampletime/1000;
      lin_ang.y = l_d*inv_sampletime/1000;
      test_temp.publish(&lin_ang);
       
      setting_millis=millis() + pos_sampleTime;
    }
  nh.spinOnce();
  delay(1);
}
/*-------------------encoder interrupt 1 ---------------------------------------*/
void encoder_1()
{
  if(!l_dir) l_p ++;
  else l_p--;
}
/*----------------------encoder interrupt 2 ------------------------------------*/
void encoder_2()
{  
  if(!r_dir) r_p ++;
  else r_p--;
}
/*--------------------generarte pwm-----------------------------------*/
void pwmOut(int Lpwm, int Rpwm, bool Ldir, bool Rdir)
{
  if(Lpwm==0 && Rpwm==0)
  { 
    analogWrite(M1_p,0); digitalWrite(M1_l,0);
    analogWrite(M2_p,0); digitalWrite(M2_l,0);
  }
  else if(Ldir==c_clkw && Rdir==c_clkw)
  {
    analogWrite(M1_p,0-Rpwm); digitalWrite(M1_l,1);
    analogWrite(M2_p,0-Lpwm); digitalWrite(M2_l,1);
  }

  else if(Ldir==clkw && Rdir==clkw)
  {
    analogWrite(M1_p,Rpwm); digitalWrite(M1_l,0);
    analogWrite(M2_p,Lpwm); digitalWrite(M2_l,0) ;  
  }

  else if(Ldir==clkw && Rdir==c_clkw)
  {
    analogWrite(M1_p,0-Rpwm); digitalWrite(M1_l,1);
    analogWrite(M2_p,Lpwm); digitalWrite(M2_l,0);  
  }

  else if(Ldir==c_clkw && Rdir==clkw)
  {
    analogWrite(M1_p,Rpwm); digitalWrite(M1_l,0);
    analogWrite(M2_p,0-Lpwm); digitalWrite(M2_l,1);
  }  
}
/*-------------------calculate_position------------------------------------*/
void calculate_position(double xt,double yt, double pht)
{
  l_d = l_p*wheel_ticLength;
  r_d = r_p*wheel_ticLength;
  double c_d = (l_d + r_d )/2;

  xt += c_d*cos(pht/180*pi);
  yt += c_d*sin(pht/180*pi);
  pht += (atan2((r_d-l_d),wheels_distance))*180/pi;
 // while (pht<0) {pht = 360-abs(pht);}
 // while (pht>=360) {pht = pht-360;}
  //update position
  p_now[0]=xt;
  p_now[1]=yt;
  p_now[2]=pht;
}
/*----------Calculate from angle and linear to motion of 2 wheels--------------------------------------*/
void motion(double lin, double phi )
{
  r_v = (2*lin - phi*wheels_distance)/(2.0); //speed of right wheels  
  l_v = (2*lin + phi*wheels_distance)/(2.0);  //speed of left wheels
  //to l_vt and r_vt
  l_vt = l_v*(wheels_encoder/(pi*wheels_diameter))*sampletime;
  r_vt = r_v*(wheels_encoder/(pi*wheels_diameter))*sampletime;
  
  if (l_vt>=0) l_dir = clkw;//go ahead
  else l_dir =c_clkw;       //backhead
  if (r_vt>=0) r_dir = clkw;
  else r_dir =c_clkw;
  
  l_set=abs(l_vt);
  r_set=abs(r_vt);
 // if (l_set>30) l_set=30;
  //if (l_set<5 && l_set>0.5) l_set=5 ;
  //if (r_set>30) r_set=30;
  //if (r_set<5 && r_set>0.5) r_set =5;
}
/**---------------------------------------------------------------------------------------------------------------**/
//PID_cal(l_error,l_pre_error,l_integral,l_derivative,l_Ppart,l_Ipart,l_Dpart,l_kP,l_kI,l_kD);
//PID_cal(r_error,r_pre_error,r_integral,r_derivative,r_Ppart,r_Ipart,r_Dpart,r_kP,r_kI,r_kD);
//PID_cal(ang_error,ang_pre_error,ang_integral,ang_derivative,ang_Ppart,ang_Ipart,ang_Dpart,p_kP,p_kI,p_kD);
//PID_cal(lin_error,lin_pre_error,lin_integral,lin_derivative,lin_Ppart,lin_Ipart,lin_Dpart,p_kP,p_kI,p_kD);
double PID_cal(double error,double pre_error,double _integral,double _derivative,double Ppart,double Ipart,double Dpart,double Kp,double Ki, double Kd)
{
  double PID_output;
    Ppart = Kp*error;

    _integral += error * sampletime;
    Ipart = Ki*_integral;

    _derivative  = (error  - pre_error )*inv_sampletime;
    Dpart  = Kd *_derivative ;

    PID_output = Ppart  + Ipart  + Dpart  ;
    pre_error  = error ;
    
    return PID_output;
}
/**-------------------------------------------------------------**/
void tunePID() {
    if (sample<1){
        pwmOut(botOUT,botOUT,c_clkw,clkw);
        delay(wait);        
        react_min = cur_react;     
        pwmOut(topOUT,topOUT,c_clkw,clkw);
        delay(wait);      
        triggre_mid = (react_max+react_min)/2;
        triggre_top = triggre_mid + trig_distance/2;
        triggre_bot = triggre_mid - trig_distance/2;
        Serial.println((String) "top = " + triggre_top + " bot: " + triggre_bot);
        pikachu=true;
        sample ++;
    }
    if (pikachu==true && cur_react>=triggre_top) {
        if (sample == 1&&i==0) {
           // Serial.println((String) "im going in 1");
            t1 = micros();
            pikachu = false;
            pwmOut(botOUT,botOUT,c_clkw,clkw);
            i++;
            //---------------------------------------
        } 
        else {
            react_max=cur_react;
          //  Serial.println((String) "im going in 2");
            t2=micros();
            save1= (t2-t1)/1000000;
            outDis=(topOUT-botOUT)/2;
            reactDis = (react_max -react_min)/2;
            calTune(save1,reactDis,outDis);
            pwmOut(botOUT,botOUT,c_clkw,clkw);
            //---------------------------------
            pikachu = false;
            t1=micros();
            sample ++;
        }
    }
    else if (pikachu == false && cur_react<triggre_bot) {
        //Serial.println((String) "im going in 3");
        react_min=cur_react;
        pikachu=true;
        //write the top Ouput to device
        pwmOut(topOUT,topOUT,c_clkw,clkw);
        //---------------------------------------
    }
}
void calTune(double pU, double A, double D){
    // Calculate Ku (ultimate gain)
    // Formula given is Ku = 4d / πa
    // d is the amplitude of the output signal
    // a is the amplitude of the input signal
    double kU= (4*D)/(A*pi);
    // How gains are calculated
    // PID control algorithm needs Kp, Ki, and Kd
    // Ziegler-Nichols tuning method gives Kp, Ti, and Td
    //
    // Kp = 0.6Ku = Kc
    // Ti = 0.5Tu = Kc/Ki
    // Td = 0.125Tu = Kd/Kc
    //
    // Solving these equations for Kp, Ki, and Kd gives this:
    //
    // Kp = 0.6Ku
    // Ki = 1.2*kU/pU;
    // Kd = 0.075*kU*pU;
    
    kP[sample-1] = 0.6*kU;
    kI[sample-1] = 2*kP[sample-1]/pU;
    kD[sample-1] = kP[sample-1]*pU/8;
}
bool end_() {
    for(int i=0;i<=cycle-2;i++){
        kPavg+=kP[i];
        kIavg+=kI[i];
        kDavg+=kD[i];
    }
    kPavg=(kPavg/(cycle-1));
    kIavg=(kIavg/(cycle-1));
    kDavg=(kDavg/(cycle-1));
    pwmOut(0,0,0,0);
    return true;
}
/*--------------------------------------------------------*/
ISR(TIMER1_OVF_vect) {
  if(ending==false){
      cur_react=((r_p+l_p)/2)/sampletime;
      if (sample<1) {
      if (cur_react>react_max) react_max=cur_react;
     }
  }
  else if(ending==true) {
    motion(lin_vel,ang_vel);  
    calculate_position(p_now[0],p_now[1],p_now[2]);
    l_error= l_set-abs(l_p);
    r_error= r_set-abs(r_p);
    l_out += PID_cal(l_error,l_pre_error,l_integral,l_derivative,l_Ppart,l_Ipart,l_Dpart,l_kP,l_kI,l_kD);
    r_out += PID_cal(r_error,r_pre_error,r_integral,r_derivative,r_Ppart,r_Ipart,r_Dpart,r_kP,r_kI,r_kD);
  
    if (l_set==0) {l_out=0; l_dir=c_clkw;}
    if (r_set==0) {r_out=0; r_dir=c_clkw;}
    
    if (l_out>= 255) l_out = 255;
    if (r_out>= 255) r_out = 255;
    pwmOut(l_out,r_out,l_dir,r_dir);
  }
  l_p=0;
  r_p=0;
  TCNT1 =timer_set;
}
/*-----------------------------------------------------------------*/
