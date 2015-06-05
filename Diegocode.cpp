//Este codigo esta basado en las librerías del APMP ver3.1
//Con licencia GNU


#include <APM_RC.h> 
#include <FastSerial.h>
#include <SPI.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_InertialSensor.h>
#include <AP_IMU.h>
#include <AP_ADC.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <DataFlash.h>
#include <AP_Compass.h> 
#include <I2C.h>
//DaraFlash
#define HEAD_BYTE1 0xA3
#define HEAD_BYTE2 0x95
#define END_BYTE1 0xA2
#define END_BYTE2 0x4E
#define DF_SLAVESELECT 53 
#define ToRad(x) (x*0.01745329252)  
#define ToDeg(x) (x*57.2957795131) 
#define A_LED_PIN
#define C_LED_PIN
#define LED_ON
#define LED_OFF 
#define MIN 1235        
#define MAX 1500   
#define CORTE 1200

Vector3f accel;   
Vector3f gyro;
FastSerialPort(Serial, 0);
AP_Compass_HMC5843 compass;


Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess  adc_scheduler;
APM_RC_APM1 APM_RC;
DataFlash_APM1 DataFlash;

int valor1=0;
float throttle=MIN;
float valor[7],valorF[7];
float pitch,roll;
int aux=0,aux2=0;
byte recived;
boolean estado=false,datos=true;
int8_t m_opp[7]={0,2,1,4,3,6,5};

float KpP=50,KdP=400,KiP=0.1;    
float KpR=50,KdR=300,KiR=0.1;                
float pitchSum=0,lastPitch=0,lastPitchFilter=0,pitchFilter=0;
float giroPitch=0,lastGiroPitch =0,giroPitchFilter=0,lastGiroPitchFilter=0;
float rollSum=0,lastRoll=0,lastRollFilter=0,rollFilter=0;
float giroRoll=0,lastGiroRoll=0,giroRollFilter=0,lastGiroRollFilter=0,giroRoll2=0;   
float errorPitch=0,errorRoll=0;
float compas=0;  
float xref=0,yref=0,yaw=0,yawref=0;

int kyaw=5; 
int yawi=0;    
int counter=0;

long timer=0,timer1=0;


AP_ADC_ADS7844 adc;
AP_InertialSensor_Oilpan oilpan_ins(&adc);
AP_IMU_INS imu(&oilpan_ins,0);

static void flash_leds(bool on)
{
    digitalWrite(A_LED_PIN, on?LED_OFF:LED_ON);
    digitalWrite(C_LED_PIN, on?LED_ON:LED_OFF);
}

static uint32_t      fastTimer; 
static uint32_t      fast_loopTimer;    
static byte    medium_loopCounter;  
static uint32_t         fiftyhz_loopTimer;

static byte    slow_loopCounter;
static int16_t   superslow_loopCounter;
static byte    counter_one_herz;


void setup(void) {
 Serial.begin(115200);
  menu();
  I2c.begin();
  I2c.timeOut(20);
  
   if (!compass.init()) {
    Serial.println("compass initialisation failed!");

   while (1) ;
  }
  
  isr_registry.init();
  adc_scheduler.init(&isr_registry);
  APM_RC.Init(&isr_registry);  
  APM_RC.enable_out(CH_1);
  APM_RC.enable_out(CH_2);
  APM_RC.enable_out(CH_3);
  APM_RC.enable_out(CH_4);
  APM_RC.enable_out(CH_5);
  APM_RC.enable_out(CH_6);
  APM_RC.enable_out(CH_7);
  APM_RC.enable_out(CH_8);
  APM_RC.OutputCh(1, valor1);
  APM_RC.OutputCh(2, valor1);
  APM_RC.OutputCh(3, valor1);
  APM_RC.OutputCh(4, valor1);
  APM_RC.OutputCh(5, valor1);
  APM_RC.OutputCh(6, valor1);

DataFlash.Init();
DataFlash.StartWrite(1); 

 imu.init(IMU::COLD_START, delay, flash_leds, &adc_scheduler);
 imu.init_accel(delay, flash_leds);Serial.print("\n");

funcion_serial();
fast_loopTimer=micros();timer1=micros();
  compass.set_orientation(AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD); 
  compass.set_offsets(0,0,0);
  compass.set_declination(ToRad(0.23));
}


void loop(void) {
int32_t timer = micros();

  if ((timer - fast_loopTimer) >= 10000) {
    fast_loop();
    fast_loopTimer=micros();
    }
  if ((timer - fiftyhz_loopTimer) >= 20000) {
    fiftyhz_loopTimer = timer;
    medium_loop();    
    fiftyhz_loop();
    counter_one_herz++;

  if(counter_one_herz == 50){
    super_slow_loop();    //1 Hz
    counter_one_herz = 0;
  }
}
}

static void fast_loop() {
  if(estado) {
    imu.update();
    accel = imu.get_accel();
    gyro = imu.get_gyro();
    pitch = accel.y; 
    roll =  accel.x; 
    giroPitch=  gyro.x;
    giroRoll = gyro.y; 
  
  pitchFilter=lastPitchFilter*0.9048 + 0.09516*lastPitch;
  rollFilter=lastRollFilter*0.9048 + 0.09516*lastRoll;
  giroRollFilter=lastGiroRollFilter*0.00253 + 0.9975  *lastGiroRoll;    
  giroPitchFilter=giroPitch;
  pitchSum +=errorPitch;  
  rollSum += errorRoll;
  errorPitch=yref-pitchFilter;
  errorRoll=xref-rollFilter;   

  valor[1] = throttle + 1.0* (KpR*errorRoll   - KdR*giroRollFilter   + KiR*rollSum);
  valor[5] = throttle + 0.5* (KpR*errorRoll   - KdR*giroRollFilter   + KiR*rollSum) -0.866* (KpP*errorPitch +KdP*giroPitchFilter + KiP*pitchSum);
  valor[4] = throttle + 0.5* (KpR*errorRoll   - KdR*giroRollFilter   + KiR*rollSum) +0.866* (KpP*errorPitch +KdP*giroPitchFilter + KiP*pitchSum);
  valor[2] = throttle - 1.0* (KpR*errorRoll   - KdR*giroRollFilter   + KiR*rollSum);
  valor[3] = throttle - 0.5* (KpR*errorRoll   - KdR*giroRollFilter   + KiR*rollSum)  -0.866* (KpP*errorPitch +KdP*giroPitchFilter + KiP*pitchSum);
  valor[6] = throttle - 0.5* (KpR*errorRoll   - KdR*giroRollFilter   + KiR*rollSum)  +0.866* (KpP*errorPitch +KdP*giroPitchFilter + KiP*pitchSum);                              
  
  escribe_data();    
  
  for(int8_t m=1;m<=6;m++) { 
    if(valor[m]>MAX) {
      valor[m_opp[m]]-=valor[m]-MAX;
      valor[m]=MAX;
    }
  }
  
  for(int8_t i=1;i<=6;i++) {                 
    valor[i] = constrain(valor[i],MIN,MAX);
  }
  
  actualiza_motores();

  lastPitch=pitch;
  lastPitchFilter=pitchFilter;
  lastRoll=roll;
  lastGiroRoll=giroRoll;
  lastRollFilter=rollFilter;
  lastGiroRollFilter=giroRollFilter;                                      
  
}
  else {   
    funcion_serial();
    for(int8_t i=1;i<=6;i++) {   
      APM_RC.OutputCh(i, 0);
    }
  }

}

static void medium_loop() {
  switch(medium_loopCounter) {
    case 0:
      medium_loopCounter++;
      compass.read();
      compass.calculate(0,0);
      compas=ToDeg(compass.heading);
      Serial.print(compas);Serial.print(",   ");Serial.print(yawref);Serial.print(",   ");Serial.println(yaw);
      break;
    
    case 1:
      medium_loopCounter++;
      yref=(APM_RC.InputCh(CH_2)-1050);yref/=210;yref-=2;yref=constrain(yref,-1,1);
      break;

    case 2:
      medium_loopCounter++;
      break;

    case 3:
      medium_loopCounter++;
      break;

    case 4:
      medium_loopCounter = 0;
      break;
    
    default:
      medium_loopCounter = 0;
      break;
 }
}

static void fiftyhz_loop() {

  if(APM_RC.InputCh(CH_3)>CORTE) {  
    throttle=1280;
    estado=true;
  }
  else {
    estado=false;timer=millis();
  }
}


void actualiza_motores() {
   for(int8_t i=1;i<=6;i++) { 
    APM_RC.OutputCh(i,valor[i]);
    }
}

static void escribe_data()
{
  DataFlash.WriteByte(HEAD_BYTE1);
  DataFlash.WriteByte(HEAD_BYTE2);
  DataFlash.WriteInt(rollFilter*10000);
  DataFlash.WriteInt(pitchFilter*10000);
  DataFlash.WriteInt(giroRollFilter*10000);
  DataFlash.WriteInt(gyro.y*10000);
  DataFlash.WriteLong((long)(millis()-timer));
  DataFlash.WriteInt(valor[1]);
  DataFlash.WriteInt(valor[2]);
  DataFlash.WriteInt(valor[4]);
  DataFlash.WriteInt(valor[5]);    
  DataFlash.WriteInt(xref*100);
  DataFlash.WriteInt(yref*100);
  DataFlash.WriteInt(compas*100);
  DataFlash.WriteInt(throttle);
  DataFlash.WriteByte(END_BYTE1);
  DataFlash.WriteByte(END_BYTE2);   
}

static void funcion_serial() {
  if(Serial.available()>0) {
    byte recived=Serial.read();

    if(recived==76||recived==108) {
      int  i, tmp_int;
      byte  tmp_byte1, tmp_byte2;
      long  tmp_long;
      Serial.println("empezando a leer...");
      DataFlash.StartRead(1);
      
      for(int i;i<10000;i++) {
        tmp_byte1 = DataFlash.ReadByte();
        tmp_byte2 = DataFlash.ReadByte();
        if ((tmp_byte1 == HEAD_BYTE1) && (tmp_byte1 == HEAD_BYTE1)) {
          tmp_int = DataFlash.ReadInt();
        if(isSpace(tmp_int)) {
          tmp_int = DataFlash.ReadInt(); 
          if(isSpace(tmp_int)) {
          Serial.println("espacio encontrado");
          break;
    }
  }

  Serial.print(tmp_int);
  Serial.print(",");
  tmp_int = DataFlash.ReadInt();

  Serial.print(tmp_int);
  Serial.print(",");
  tmp_int = DataFlash.ReadInt();
  
  Serial.print(tmp_int);
  Serial.print(",");
  tmp_int = DataFlash.ReadInt();
  
  Serial.print(tmp_int);
  Serial.print(",");
  tmp_long = DataFlash.ReadLong();
  
  Serial.print(tmp_long);
  Serial.print(",");
  tmp_int = DataFlash.ReadInt();
  
  Serial.print(tmp_int);
  Serial.print(",");
  tmp_int = DataFlash.ReadInt();
  
  Serial.print(tmp_int);
  Serial.print(",");
  tmp_int = DataFlash.ReadInt();
  
  Serial.print(tmp_int);
  Serial.print(",");
  tmp_int = DataFlash.ReadInt();
  
  Serial.print(tmp_int);
  Serial.print(",");
  tmp_int = DataFlash.ReadInt();
  
  Serial.print(tmp_int);
  Serial.print(",");
  tmp_int = DataFlash.ReadInt();
  
  Serial.print(tmp_int);
  Serial.print(",");
  tmp_int = DataFlash.ReadInt();  

  Serial.print(tmp_int);
  Serial.print(",");
  tmp_int = DataFlash.ReadInt();
  
  Serial.print(tmp_int);
  tmp_byte1 = DataFlash.ReadByte();
  tmp_byte2 = DataFlash.ReadByte();

  }  
  Serial.println();
}
  Serial.println("");
  Serial.println("lectura completada ");
  Serial.println("");
  
  while(1); 
}
  else if(recived==98||recived==66) {
    borrar();
    while(1);
    } 
  }
}

void dataflash_CS_inactivee() {
  digitalWrite(DF_SLAVESELECT,HIGH); //disable device
}

void dataflash_CS_activee() {
  digitalWrite(DF_SLAVESELECT,LOW); //enable device
}

static void borrar() {
  dataflash_CS_activee();   
  SPI.transfer(0xC7);
  SPI.transfer(0x94);
  SPI.transfer(0x80);
  SPI.transfer(0x9A);
  dataflash_CS_inactivee();   
  Serial.println("Borrado de memoria Flash completado");
}
static void menu() {
  Serial.println("Bienvenido;\n \t\t\t pulse.... \n 'L' para Leer datos \n 'B' para Borrar datos\n \t\t\t\t\t y despues INTRO");
}
  //

