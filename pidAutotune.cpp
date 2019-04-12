#define clkw        0
#define c_clkw      1
#define encodPinA1  3
#define M1_p        6
#define M1_l        7
#define encodPinA2  2
#define M2_p        5
#define M2_l        4
#define DEBUG       1
//turning parametters
int sample = 0;

double topOUT,botOUT,OUT_range[2]={0,225}; 
double triggre_mid,triggre_top,triggre_bot;
const double trig_distance=2;
double react_max,react_min,cur_react; //reaction signal peek
double kU,pU,outDis,reactDis;
double t1,t2,save1;
const int cycle=5, wait= 100;//cycles, ms;
double kP[cycle]={},kI[cycle]={},kD[cycle]={};
double kPavg,kIavg,kDavg;
bool pikachu,ending=false;
//control parameters
const double sampletime = 0.02, inv_sampletime = 1/sampletime,timer_set=65535-sampletime*250000;
int l_p=0,r_p=0;
double l_v;
double r_v; // pwm: pwm output. lv: mm/sec. lvt: tic/delta_t l:lert, r: right 
bool l_dir = clkw, r_dir = clkw;
const double pi=3.141592654;

void setup(){
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
    sei();                                  // enable all interrupt
    /*preset the output jump distance*/
    topOUT = OUT_range[0]+(OUT_range[1]-OUT_range[0])*3/4;
    botOUT = OUT_range[0]+(OUT_range[1]+OUT_range[0])*1/4;
}
/*-----------------------------------------------------------------*/
void loop() {
    while(!ending){
        tunePID();
        if (sample>=cycle){
            ending = end_();
        }
    }
}
/*-----------------------------------------------------------------*/
void tunePID() {
    if (sample<1){
        pwmOut(botOUT,botOUT,c_clkw,clkw);
        delay(wait);
        pwmOut(topOUT,topOUT,c_clkw,clkw);
        delay(wait);
        triggre_mid = (react_max+react_min)/2;
        triggre_top = triggre_mid + trig_distance/2;
        triggre_bot = triggre_mid - trig_distance/2;
        pikachu=true;
        sample ++;
    }
    else if (pikachu=true && cur_react>=triggre_top) {
        if (sample == 1) {
            t1 = micros();
            pikachu = false;
            //write the bot Ouput to device
            pwmOut(botOUT,botOUT,c_clkw,clkw);
            //---------------------------------------
        } 
        else {
            t2=micros();
            save1= t2-t1; //this is pU
            outDis=topOUT-botOUT;
            reactDis = react_max -react_min;
            calTune(save1,reactDis,outDis);
            pikachu = false;
            //write the bot Ouput to device
            pwmOut(botOUT,botOUT,c_clkw,clkw);
            //---------------------------------
            t1=micros();
            sample ++;
        }
    }
    else if (pikachu = false && cur_react<=triggre_bot) {
        pikachu=true;
        //write the top Ouput to device
        pwmOut(topOUT,topOUT,c_clkw,clkw);
        //---------------------------------------
    }

}
void calTune(double pU, double A, double D){
    double kU=4*D/(A*pi);
    kP[sample-1] = 0.6*kU;
    kI[sample-1] = 1.2*kU/pU;
    kD[sample-1] = 0.075*kU*pU;
}
bool end_() {
    for(int i=0;i<=cycle-1;i++){
        kPavg+=kP[i];
        kIavg+=kI[i];
        kDavg+=kD[i];
    }
    kPavg=kPavg/cycle;
    kIavg=kIavg/cycle;
    kDavg=kDavg/cycle;
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
    if (sample<1){
      int _p;
      _p=(abs(r_p)+abs(l_p))/2;
      react_max= max(cur_react,_p);
      react_min= min(cur_react,_p);
      cur_react= _p ;
      }

    l_p=0;
    r_p=0;
    TCNT1 =timer_set;
}
/*-----------------------------------------------------------------*/