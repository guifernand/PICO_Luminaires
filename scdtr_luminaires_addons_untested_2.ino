#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#include <algorithm>
#include <vector>
#include <string>
#include "pid.h"
#include "buffer.hpp"
#include "metrics.hpp"

using namespace std;

Buffer lm_buffer;

const int LED_PIN {15}, DAC_RANGE {4095}, R10K {10000};
const float VCC {3.3};

float Gain {0}, scale {1};
int pwm {0}, period {2500}, step_sz {1};
char op_mode;
bool occupancy {false}, pid_cntrl {false}, stream[3] {false, false, false}, up {true};
tuple<int,float,float,float>metrics;

unsigned long prevTime {0};

//----------------PID controller-----------------

pid my_pid {0.01, 45, 1, 0.9, 0}; //Create a pid controller (h,K,b,Ti,Td,N,Tt) --> {0.01, 50, 1, 0.7, 0} --> the good
float ref_lux {0.0};


//-----------------Calculations------------------

inline float read_lux(){
  int median_read_adc = median_reads();
  return measure_LUX(median_read_adc);
}

inline float ohm_to_LUX(float LDR){
    return pow(10,(-(log10(LDR/1000)-3.15)/0.8));   
}

inline float adc_to_V(int adc){
    return (adc*VCC)/DAC_RANGE;
}

inline float R_LDR(float read_vo){ 
  return R10K*((VCC/read_vo)-1);
}

float measure_LUX(int read_adc){ 
  float read_vo {0},LDR {0};
  read_vo = adc_to_V(read_adc);
  LDR = R_LDR(read_vo);
  return ohm_to_LUX(LDR);
}

int median_reads(){

  vector<int> reads_adc(5,0);
  
  for(int i = 5;i--;){
      reads_adc[i] = analogRead(A0);
  }
  sort(reads_adc.begin(), reads_adc.end());

  return reads_adc.at(2);
  
}

float inst_power(){
    float read_vo {0},LDR {0}, curr {0};
    int median_read_adc = median_reads();
    read_vo = adc_to_V(median_read_adc);
    LDR = R_LDR(read_vo);
    curr = (VCC - read_vo)*LDR;
    return read_vo*curr;
}

float external_src(){
  float sensed_lux = read_lux();
  return sensed_lux - Gain*pwm;
}

//-------------------Interface-------------------

//https://forum.arduino.cc/t/serial-input-basics-updated/382007/2

const int numChars = 32;
std::string receivedChars(numChars ,' ');
boolean newData = false;

void recvCmmd() {
    static boolean recvInProgress = false;
    static int ndx = 0;      
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void GetRequest(string receivedChars){
  int nums = stoi(receivedChars.substr(2));
  
  Serial.print(receivedChars[1]); Serial.print(" "); 
  Serial.print(nums); Serial.print(" "); 

  switch(receivedChars[1]) {
          case 'd':
            {
              Serial.print(pwm/DAC_RANGE);
            }
            break;
          case 'r':
            {
              Serial.print(ref_lux);
            }
            break;
          case 'l':
            {
              Serial.print(read_lux());
            }
            break;
          case 'o':
            {
              if (occupancy){
                Serial.print("on");
              } else {
                Serial.print("off");
              }
            }
            break;
          case 'a':
            {
              if (my_pid.getAntiW()){
                Serial.print("on");
              } else {
                Serial.print("off");
              }
            }
            break;
          case 'k':
            {
              if (my_pid.getFeedB()){
                Serial.print("on");
              } else {
                Serial.print("off");
              }
            }
            break;
          case 'x':
            {
              Serial.print(external_src());
            }
            break;
          case 'p':
            {
              Serial.print(inst_power());
            }
            break;
          case 't':
            {
              Serial.print(millis()/1000);
            }
            break;
          case 'b':
            {
              Serial.print("EU SOU O JOSÉ O LARI LOLLÉ");   
              const char* print_buff = lm_buffer.showData(receivedChars[2],receivedChars[3]);
              Serial.print(print_buff);             
            }
            break;
          case 'e':
            {
              Serial.print(get<1>(metrics));
            }
            break;
          case 'v':
            {
              Serial.print(get<2>(metrics)/get<0>(metrics));
            }
            break;
          case 'f':
            {
              Serial.print(get<3>(metrics)/get<0>(metrics));
            }
            break;
          default:
            Serial.print("err");
            break;
        }
  Serial.println(" ");
}

void newCmmd(){
    if (newData == true) {
     
        newData = false;

        switch(receivedChars[0]) {
          case 'd':
            {
              int nums = stoi(receivedChars.substr(2));
              Serial.print(receivedChars[0]); Serial.print(" "); 
              Serial.print(nums); Serial.print(" "); 
              if (!(0 <= nums && nums <= 100))  {
                Serial.println("err");
              } else {
                Serial.println("ack");
                pwm = nums*DAC_RANGE;
                op_mode = 'd';
              }  
            }
            break;
          case 'g':
            {
              GetRequest(receivedChars);
            }
            break;
          case 'r':
            {
              int nums = stoi(receivedChars.substr(2));
              Serial.print(receivedChars[0]); Serial.print(" "); 
              Serial.print(nums); Serial.print(" "); 
              if (nums < 0)  {
                Serial.println("err");
              } else {
                ref_lux = nums;
                scale = ref_lux/100;
                Serial.println("ack");
              }  
            }
            break;
          case 'o':
            {
             int nums = stoi(receivedChars.substr(2));
             Serial.print(receivedChars[0]); Serial.print(" "); 
             Serial.print(nums); Serial.print(" "); 
             if (nums != 0 && nums != 1 )  {
                Serial.println("err");
             } else {
                occupancy = nums;
                op_mode = 'n';
                Serial.println("ack");
             }  
            }
            break;
          case 'a':
            {
             int nums = stoi(receivedChars.substr(2));
             Serial.print(receivedChars[0]); Serial.print(" "); 
             Serial.print(nums); Serial.print(" "); 
             if (nums != 0 && nums != 1 )  {
                Serial.println("err");
             } else {
                my_pid.setAntiW(nums);
                Serial.println("ack");
             }  
            }
            break;
          case 'k':
            {
             int nums = stoi(receivedChars.substr(2));
             Serial.print(receivedChars[0]); Serial.print(" "); 
             Serial.print(nums); Serial.print(" "); 
             if (nums != 0 && nums != 1 )  {
                Serial.println("err");
             } else {
                my_pid.setFeedB(nums);
                op_mode = 'n';
                Serial.println("ack");
             }  
            }
            break;
          case 's':
            {
              if (receivedChars[1] == 'l'){
                stream[0] = true;
              }else if(receivedChars[1] == 'd'){
                stream[1] = true;
              }else if(receivedChars[1] == 'p'){
                stream[2] = true;
              }
            }
            break;
          case 'S':
            if (receivedChars[1] == 'l'){
                stream[0] = false;
                Serial.println("ack");
              }else if(receivedChars[1] == 'd'){
                stream[1] = false;
                Serial.println("ack");
              }else if(receivedChars[1] == 'p'){
                stream[2] = false;
              }else{
                Serial.println("err");
              }
            break;
          case 'c':
            {
              int nums = stoi(receivedChars.substr(2));
              Serial.print(receivedChars[0]); Serial.print(" "); 
              Serial.print(nums); Serial.print(" "); 
              if (!(0 < nums))  {
                Serial.println("err");
              } else {
                pwm == 0;
                op_mode = 'c';
                period = nums;
                Serial.println("ack");
              }               
            }
            break;
          case 'p':
            {
              int nums = stoi(receivedChars.substr(2));
              Serial.print(receivedChars[0]); Serial.print(" "); 
              Serial.print(nums); Serial.print(" "); 
              if (!(0 < nums))  {
                Serial.println("err");
              } else {
                pwm == 0;
                op_mode = 'p';
                step_sz = nums;
                Serial.println("ack");
              }  
            }
            break;
          default:
            Serial.println("err");
            break;
        }
    }
}

//----------------Repeating alarm----------------

struct repeating_timer timer;

bool execute_controler( struct repeating_timer *t ){
    float sensed_lux = read_lux();
    
    if (my_pid.getFeedB()){
      pair<float, float> outs = my_pid.compute_control(ref_lux, sensed_lux);
      pwm = (int)outs.first;
  
      analogWrite(LED_PIN, pwm);
      float err = my_pid.housekeep( ref_lux , sensed_lux, outs.first, outs.second );
      metrics = compute_metrics(ref_lux,sensed_lux,pwm,millis()/1000);

    }
    lm_buffer.addData(sensed_lux, pwm*100/DAC_RANGE);
    if (stream[0]){
         Serial.print("sl<i> "); Serial.print(sensed_lux);  Serial.print(" "); Serial.print(millis());
      }
    if (stream[1]){
       Serial.print("sd<i> "); Serial.print(pwm*100/DAC_RANGE); Serial.print(" "); Serial.print(millis());
       Serial.println();
    }
    if (stream[2]){
       Serial.print("Duty: "); Serial.print((pwm*100/DAC_RANGE)*scale); Serial.print(" ");
       Serial.print("Ref: "); Serial.print(ref_lux); Serial.print(" "); 
       Serial.print("LUX: "); Serial.println(sensed_lux); 
    }


    
  return true;
}

//---------------Pre Calculations (G,b)--------------

float pre_computations(){
  double G {0};
  int count {0};
  analogWrite(LED_PIN,0);
  float o = read_lux();
  delay(500);
  for(int i=1; i<=10; i++){ 
    count++;
    analogWrite(LED_PIN, i*0.1*4095);
    delay(2500);

    float a = read_lux();
    G += (a - o)/ (i*0.1*4095);

    Serial.print(i*0.1*4095); Serial.print(" "); Serial.println(a);
  }
  analogWrite(LED_PIN, 0);
  delay(1000);
  return G/count;
}

//---------------------------------------------------

void operationMode(){
  
  switch(op_mode) {
          case 'd':
            {
              my_pid.setFeedB(0);
              analogWrite(LED_PIN, pwm);
            }
            break;
          case 'c':
            {
              my_pid.setFeedB(0);
              if (pwm == 0){
                pwm = DAC_RANGE;
              } else if (pwm == DAC_RANGE){
                pwm = 0;
              }
              analogWrite(LED_PIN, pwm);
              delay(period);
            }
            break;
          case 'p':
            {
              my_pid.setFeedB(0);
              if (pwm <= 0){
                up = true;
              } else if (pwm >= DAC_RANGE){
                up = false;
              }
              if (up){
                pwm+= step_sz;
              }else{
                pwm-= step_sz;
              }
              delay(50);
              analogWrite(LED_PIN, pwm);
            }
            break;
          default:
            break;
        }
}

void blink(){
  const long interval = 2000;
  unsigned long start_t = millis();

  do{
    digitalWrite(LED_BUILTIN,HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN,LOW);
    delay(200);
  } while(millis() - start_t <= interval);
  
}

//---------------------------------------------------

void setup(){
  Serial.println("Starting setup");

  Serial.begin();

  analogReadResolution(12);
  analogWriteFreq(60000); //30KHz
  analogWriteRange(4096); //Max PWM

  pinMode(LED_BUILTIN,OUTPUT);
  blink();
  Gain = pre_computations();
  Serial.print("Gain*1000:"); Serial.println(Gain*1000);
  digitalWrite(LED_BUILTIN, HIGH);

  my_pid.set_point(Gain);
  
  add_repeating_timer_ms( -10,
    execute_controler,
    NULL, &timer); //100 Hz

  Serial.println("Ended setup");
}

void loop() {
  recvCmmd();
  newCmmd();
  operationMode();
  
}
