// This #include statement was automatically added by the Particle IDE.

#include "Particle.h"
#include "Arduino.h"
#include "application.h"
//#undef min
//#undef max
#include "Pixy2I2C.h"
#include "PIDLoop.h"
#include "TPixy2.h"
#include "Pixy2CCC.h"
#include "Pixy2Line.h"
#include "Pixy2Video.h"



SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);

Pixy2I2C pixy;
PIDLoop panLoop(200, 0, 400, true);
PIDLoop tiltLoop(800, 100, 400, true);


unsigned long lastPrintSample = 0;
unsigned long prevLEDMillis=0;
unsigned long prevSyncMillis=0;
unsigned long prevPingMillis=0;
unsigned long prevPingFMillis=0;
unsigned long prevPingLMillis=0;
unsigned long prevPingRMillis=0;
unsigned long prevPingBMillis=0;
unsigned long startMillis = 0;
unsigned long pixyMillis = 0;
unsigned long prevConnection = -1;
unsigned long pulseStartF = -1;
unsigned long pulseStartL = -1;
unsigned long pulseStartR = -1;
unsigned long pulseStartB = -1;
double distance = 5;
double distanceF = 0;
double distanceR = 0;
double distanceL = 0;
double distanceB = 0;
double smoothServo=40;

long millis24Hr=1000*60*60*24;

bool lightStatus=false;
bool target = false;
bool LmotorOn = false;
bool RmotorOn = false;

const int brightness = 2;
int pingPin = DAC;
int leftSpeed =0; //0
int rightSpeed =0; //0
int leftSpeedD =0;
int rightSpeedD=0;
double motorProportionD=0;
int motorProportionP=0;
int pLevel = 100;
double frontUS=0;
Servo frontServo;


void setup()
{
    
    Particle.variable("frontUS",distanceF);
    Particle.variable("backUS",distanceB);
    Particle.variable("rightUS",distanceR);
    Particle.variable("leftUS",distanceL);
	//strip.begin();
    //strip.show();
	
	RGB.control(true);
	Particle.connect();
	pingLED(0,255,255);
    prevConnection=-1;
    
    Serial.begin(9600);
    Serial.print("Starting...\n");
    pixy.init();
    pixy.changeProg("color_connected_components"); 

    pinMode(D2,OUTPUT);
    pinMode(D3,OUTPUT);
    pinMode(D4,OUTPUT);
    pinMode(D5,OUTPUT);
    
    frontServo.attach(WKP);
    startMillis = millis();

}

void loop()
{
	unsigned long currentMillis = millis();
	if(currentMillis-startMillis>5000)
	{
	    pixyRead(currentMillis);
	}
	checkTimer(currentMillis);
	LED(currentMillis);
	pingPin=DAC;
	pingUltrasonic(currentMillis,0); 
	pingPin = A0;
	pingUltrasonic(currentMillis,3); 
	pingPin = A2;
	pingUltrasonic(currentMillis,2); 
	pingPin = A4;
	pingUltrasonic(currentMillis,1); 
	
	if(leftSpeed>0)
	    digitalWrite(D4,HIGH);
	else if(leftSpeed<0)
	    digitalWrite(D4,LOW);
	analogWrite(D2,abs(leftSpeed),4096);
	
	
	if(rightSpeed>0)
	    digitalWrite(D5,HIGH);
	else if(rightSpeed<0)
	    digitalWrite(D5,LOW);
	    
	analogWrite(D3,abs(rightSpeed),4096);

	//leftWheel.write(leftSpeed);
	//rightWheel.write(rightSpeed);
}



void checkTimer(unsigned long currentMillis)
{
	if(currentMillis-prevSyncMillis>millis24Hr)
	{
	    Particle.syncTime();
	}
	
}


void pixyRead(unsigned long currentMillis)
{
      static int i = 0;
      int j;
      char buf[64]; 
      int32_t panOffset;
      int32_t tiltOffset;
      
      // get active blocks from Pixy
      pixy.ccc.getBlocks();
      
        // frontUS = 0.5-distanceF;
        double rightUS = 0;
        double leftUS = 0;
        double backUS = 0;
        int forwardSpeed;

        if(distanceF<0.7)
            frontUS = 60/distanceF;
        else
            frontUS = 0;
            
        if(distanceB<0.7)
            backUS = 60/distanceB;
        else
            backUS = 0;
            
        if(distanceL<0.5)
            leftUS = 10/distanceL;
        else
            leftUS = 0;
            
        if(distanceR<0.5)
            rightUS = 10/distanceR;
        else
            rightUS = 0;
   
            
      noInterrupts();
      if (pixy.ccc.numBlocks)
      {        
        pixyMillis = currentMillis;
        i++;
       
        //if (i%60==0)
        //  Serial.println(i);   
        
        // calculate pan and tilt "errors" with respect to first object (blocks[0]), 
        // which is the biggest object (they are sorted by size).  
        panOffset = (int32_t)pixy.frameWidth/2 - (int32_t)pixy.ccc.blocks[0].m_x;
        tiltOffset = (int32_t)pixy.frameHeight/2 - (int32_t)pixy.ccc.blocks[0].m_y;
        // update loops
        panLoop.update(panOffset);
        tiltLoop.update(tiltOffset);
        Serial.println(String(pixy.ccc.blocks[0].m_height));  
        // set pan and tilt servos  
        pixy.setServos(panLoop.m_command,tiltLoop.m_command);
        
        interrupts();
        int motorProportion=610-panLoop.m_command;
        smoothServo = 0.8*smoothServo+0.2*(panLoop.m_command/12+40);
        frontServo.write(smoothServo);
        
        int alignment =0;
        if(motorProportion>10)
        {
            alignment = 1;
        }
        else if (motorProportion<-10)
        {
            alignment = -1;
        }
        
        
            
        int forwardSpeed=(pixy.ccc.blocks[0].m_height-60);
        //leftSpeed=motorProportion/100+alignment+forwardSpeed;//abs(motorProportion)/motorProportion;
        //rightSpeed=-motorProportion/100-alignment-forwardSpeed;//abs(motorProportion)/motorProportion;
        int trim = 0;
        //leftSpeedD=(5*forwardSpeed+motorProportion/3+trim+alignment)*0.05+leftSpeedD*0.95;
        int gain = 10;
        motorProportionD=(0.5*motorProportion+0.6*motorProportionD-gain*(motorProportionP-motorProportion));
        
        leftSpeedD=(forwardSpeed*5)*0.05+leftSpeedD*0.96;
        leftSpeed = int(leftSpeedD+motorProportionD/2+frontUS-backUS-leftUS/5+rightUS/5);
        if(leftSpeed>255)
            leftSpeed = 255;
        if(leftSpeed<-255)
            leftSpeed = -255;
        
       // rightSpeedD = (-5*forwardSpeed-motorProportion/3-trim-alignment)*0.05+rightSpeedD*0.95;
       rightSpeedD=(-forwardSpeed*5)*0.05+rightSpeedD*0.96;
        rightSpeed=int(rightSpeedD+motorProportionD/2-frontUS+backUS-leftUS/5+rightUS/5);
        if(rightSpeed>255)
            rightSpeed = 255;
        if(rightSpeed<-255)
            rightSpeed = -255;
        
        motorProportionP=motorProportion;
        
        pixy.setLED(0,170,0);
      }  
      else
      {
           interrupts();
          motorProportionP=motorProportionD;
          if(currentMillis-pixyMillis<2000)
          {
              rightSpeedD=(-frontUS/5+backUS/5)*0.04+rightSpeedD*0.98;
              leftSpeedD=(frontUS/5-backUS/5)*0.04+leftSpeedD*0.98;
          }
          else
          {
              rightSpeedD=0;
              leftSpeedD=0;
          }
           rightSpeed=int(rightSpeedD);
            if(rightSpeed>255)
                rightSpeed = 255;
            if(rightSpeed<-255)
                rightSpeed = -255;
                
          
          leftSpeed = int(leftSpeedD);
            if(leftSpeed>255)
                leftSpeed = 255;
            if(leftSpeed<-255)
                leftSpeed = -255;
          motorProportionD=0.98*motorProportionD;
          
          
            
           // rightSpeedD = (-5*forwardSpeed-motorProportion/3-trim-alignment)*0.05+rightSpeedD*0.95;
           
                
          if(!lightStatus)
          {
              pixy.setLED(170,0,0);
          }
          else
          {
            pixy.setLED(0,0,0);
          }
      }
}

void LED(unsigned long currentMillis) 
{
	if(currentMillis-prevLEDMillis>1000)
	{
	    int additive = 0;
	    int additive2 =0;
	    if(target)
	    {
	        additive2=170;
	    }
	    
	    if(Particle.connected())
	    {
	        additive = 255;
	    }
	    

		if(!lightStatus)
		{
			RGB.color(170,0,additive);
		}
		else
		{
			RGB.color(0,0,0);
		}
		
		lightStatus=!lightStatus;
		prevLEDMillis=millis();
	}
	
}

int leftWrite(String command)
{
    leftSpeed = atoi(command);
    return leftSpeed;
}

int rightWrite(String command)
{
    rightSpeed = atoi(command);
    return rightSpeed;
}

void pingLED(int R, int G, int B)
{
    lightStatus = true;
    RGB.color(R,G,B);
    prevLEDMillis=millis();
}


void pingLowF()
{
    distanceF = double(micros()-pulseStartF)*343/2/949500-0.1;

}
void pingLowL()
{
    distanceL = double(micros()-pulseStartL)*343/2/949500-0.1;

}
void pingLowR()
{
    distanceR = double(micros()-pulseStartR)*343/2/949500-0.1;

}
void pingLowB()
{
    distanceB = double(micros()-pulseStartB)*343/2/949500-0.1;

}

void pingUltrasonic(unsigned long currentMillis,int i)
{
    if(i==0)
        prevPingMillis = prevPingFMillis;
    else  if(i==1)
        prevPingMillis = prevPingLMillis;
    else  if(i==2)
        prevPingMillis = prevPingRMillis;
    else if(i==3)
        prevPingMillis = prevPingBMillis;
        
    if(currentMillis-prevPingMillis>50)
    {
        pinMode(pingPin, OUTPUT);          // Set pin to OUTPUT
        digitalWrite(pingPin, LOW);        // Ensure pin is low
        delayMicroseconds(2);
        digitalWrite(pingPin, HIGH);       // Start ranging
        delayMicroseconds(5);              //   with 5 microsecond burst
        digitalWrite(pingPin, LOW);        // End ranging
        pinMode(pingPin, INPUT);           // Set pin to INPUT
        //interrupts();
        
        if(i==0)
        {
            prevPingMillis = prevPingFMillis;
            pulseStartF = micros();
            attachInterrupt(pingPin,pingLowF,FALLING);
        }
         else  if(i==1)
        {
            prevPingMillis = prevPingFMillis;
            pulseStartL = micros();
            attachInterrupt(pingPin,pingLowL,FALLING);
        }
         else  if(i==2)
        {
            prevPingMillis = prevPingFMillis;
            pulseStartR = micros();
            attachInterrupt(pingPin,pingLowR,FALLING);
        }
        else if(i==3)
        {
            prevPingMillis = prevPingFMillis;
            pulseStartB = micros();
            attachInterrupt(pingPin,pingLowB,FALLING);
        }     // Short delay
    }
}
