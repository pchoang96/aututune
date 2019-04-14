#define clkw        0
#define c_clkw      1
#define encodPinA1  3
#define M1_p        6
#define M1_l        7
#define encodPinA2  2
#define M2_p        5
#define M2_l        4
#define DEBUG       1
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
//control parameters
const double sampletime = 0.02, inv_sampletime = 1/sampletime,timer_set=65535-sampletime*250000;
int l_p=0,r_p=0;
double l_v;
double r_v; // pwm: pwm output. lv: mm/sec. lvt: tic/delta_t l:lert, r: right 
bool l_dir = clkw, r_dir = clkw;
const double pi=3.141592654;

void setup(){
    Serial.begin(9600);
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
    sei();    
}
/*-----------------------------------------------------------------*/
void loop() {
    while(!ending){
        tunePID();
        if (sample>=cycle){
            ending = end_();
            Serial.println((String) kPavg +" "+ kIavg +" " + kDavg );
        }
    }
}
/*-----------------------------------------------------------------*/
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
            Serial.println((String) "im going in 1");
            t1 = micros();
            pikachu = false;
            pwmOut(botOUT,botOUT,c_clkw,clkw);
            i++;
            //---------------------------------------
        } 
        else {
            react_max=cur_react;
            Serial.println((String) "im going in 2");
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
        Serial.println((String) "im going in 3");
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
/*-------------------encoder interrupt 1 ---------------------------------------*/
void encoder_1() {
  if(!l_dir) l_p ++;
  else l_p--;
}
/*----------------------encoder interrupt 2 ------------------------------------*/
void encoder_2() {  
  if(!r_dir) r_p ++;
  else r_p--;
}
/*---------------------------generate control signal-------------------------------------------------*/
void pwmOut(int Lpwm, int Rpwm, bool Ldir, bool Rdir){
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
/*-----------------------------------------------------------------*/
ISR(TIMER1_OVF_vect) {
     cur_react=((r_p+l_p)/2)/sampletime;
     if (sample<1) {
        if (cur_react>react_max) react_max=cur_react;
     }
   // Serial.println(cur_react);
    l_p=0;
    r_p=0;
    TCNT1 =timer_set;
}
/*-----------------------------------------------------------------*/