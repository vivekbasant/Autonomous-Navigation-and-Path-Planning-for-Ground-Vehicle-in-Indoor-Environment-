
int dist;
int rx=90;
int ry=90;
int angle;
String angleTemp = ""; 
char rec = ' ';
int angle1=0;

int read_flag = 0;


// create servo object to control a servo

// twelve servo objects can be created on most boards




//int pos = 0;    // variable to store the servo position

int i = 0;
int j = 0;
int motor,distance,angleLength,distanceLength,cm;

#define enr1   6
#define enr11   7
#define enr12   8
#define enr2 24
#define enr21 26
#define enr22 25
#define enr3   53
#define enr31   50
#define enr32   52
#define enl1   14
#define enl11   15
#define enl12   16
#define enl2   27
#define enl21   28
#define enl22   29
#define enl3   49
#define enl31   51
#define enl32   46
///int i = 0;
//float x[15],y[15];
float z=0,t=0,k=0;
int highSpeed = 160;  //150
int highSpeed1 = 200;//128 //150

int counts=0;
void code()
{
  counts++;
//Serial.println(counts);


}
#include<Wire.h>
#include<MPU6050.h>
const int MPU6050_addr=0x68; 
int16_t AccX,AccY,AccZ,Temp,GyroX,GyroY,GyroZ;
float dps;
long long unsigned int loop_timer;
float anglempu=0;
void mpuangle()
{ 
  loop_timer = millis();
  
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr,14,true);  // REQUESTING 14 BYTES FROM DEVICE ADDRESSED AS MPU6050_addr
  AccX=Wire.read()<<8|Wire.read();         // Wire.read() READS ONE BYTE WHERE AS DATA IS OF 2 BYTE SO, READ THE FIRST BYTE , <<8 , OR IT WITH NEXT BYTE TO MAKE 16 BIT(2 BYTE) DATA
  AccY=Wire.read()<<8|Wire.read();
  AccZ=Wire.read()<<8|Wire.read();
  Temp=Wire.read()<<8|Wire.read();
  GyroX=Wire.read()<<8|Wire.read();
  GyroY=Wire.read()<<8|Wire.read();
  GyroZ=Wire.read()<<8|Wire.read();
  dps = GyroZ/65.5;
  anglempu += dps*0.004;
//  anglemp/u = anglempu;
//  Serial.println(anglempu*2); 

   
  while((millis()-loop_timer)<4);
}
void forward()
{
  digitalWrite(enr12, LOW);
  digitalWrite(enr11, HIGH);
  digitalWrite(enr22, HIGH);
  digitalWrite(enr21, LOW);
  digitalWrite(enr31, HIGH);
  digitalWrite(enr32, LOW);
  digitalWrite(enl11, HIGH);
  digitalWrite(enl12, LOW);
  digitalWrite(enl21, HIGH);
  digitalWrite(enl22, LOW);    //240 counts = 720
  digitalWrite(enl31, LOW);
  digitalWrite(enl32, HIGH);

  analogWrite(enr1, highSpeed1);
  analogWrite(enr2, highSpeed1);
  analogWrite(enr3, highSpeed);
  analogWrite(enl1, highSpeed);
  analogWrite(enl2, highSpeed);
  analogWrite(enl3, highSpeed);
}

void right()
{
  digitalWrite(enr12, HIGH);
  digitalWrite(enr11, LOW);
  digitalWrite(enr22, LOW);
  digitalWrite(enr21, HIGH);
  digitalWrite(enr31, LOW);
  digitalWrite(enr32, HIGH);
  digitalWrite(enl11, HIGH);
  digitalWrite(enl12, LOW);
  digitalWrite(enl21, HIGH);
  digitalWrite(enl22, LOW);    //240 counts = 720
  digitalWrite(enl31, LOW);
  digitalWrite(enl32, HIGH);

  analogWrite(enr1, highSpeed1);
  analogWrite(enr2, highSpeed1);
  analogWrite(enr3, highSpeed);
  analogWrite(enl1, highSpeed);
  analogWrite(enl2, highSpeed);
  analogWrite(enl3, highSpeed);
}


void left()
{
  digitalWrite(enr12, LOW);
  digitalWrite(enr11, HIGH);
 digitalWrite(enr22, HIGH);
  digitalWrite(enr21, LOW);
  digitalWrite(enr31, HIGH);
  digitalWrite(enr32, LOW);
  digitalWrite(enl11, LOW);
  digitalWrite(enl12, HIGH);
  digitalWrite(enl21, LOW);
  digitalWrite(enl22, HIGH);    //240 counts = 720
  digitalWrite(enl31, HIGH);
  digitalWrite(enl32, LOW);

  analogWrite(enr1, highSpeed1);
  analogWrite(enr2, highSpeed1);
  analogWrite(enr3, highSpeed);
  analogWrite(enl1, highSpeed);
  analogWrite(enl2, highSpeed);
  analogWrite(enl3, highSpeed);
}

void pause()
{
  digitalWrite(enr12, LOW);
  digitalWrite(enr11, LOW);
  digitalWrite(enr22, LOW);
  digitalWrite(enr21, LOW);
  digitalWrite(enr31, LOW);
  digitalWrite(enr32, LOW);
  digitalWrite(enl11, LOW);
  digitalWrite(enl12, LOW);
  digitalWrite(enl21, LOW);
  digitalWrite(enl22, LOW);
  digitalWrite(enl31, LOW);
  digitalWrite(enl32, LOW);

  analogWrite(enr1, 0);
  analogWrite(enr2, 0);
  analogWrite(enr3, 0);
  analogWrite(enl1, 0);
  analogWrite(enl2, 0);
  analogWrite(enl3, 0);
}

void encdist(){
  attachInterrupt(digitalPinToInterrupt(2),code,RISING);
  cm = 0.18*counts;
//  Serial.println(cm);
}

void control(){

  
     while(anglempu*2.54<angle)  //1.3
  {
    left();
    mpuangle();
  }
  
  pause();
  delay(500);

  
  while(anglempu*2.54>angle)   //1.3
  {
    right();
    mpuangle();
  }
  
  
  pause();
  delay(500);
  /*if(angle1 == 45 and angle == 0)
  {
    right();
    delay(600);
    
  }
   if(angle1 == 90 and angle == 0)
  {
    right();
    delay(1000);
    
  }
   if(angle1 == -45 and angle == 0)
  {
    left();
    delay(600);
    
  }
   if(angle1 == -90 and angle == 0)
  {
    left();
    delay(1000);
    
  }
  if(angle==45)
  {
    left();
    delay(600);
  }
   if(angle==95)
  {
    left();
    delay(100);
  }

   if(angle==-45)
  {
    right();
    delay(600);
  }
   if(angle==-90)
  {
    right();
    delay(1000);
  }*/

  if(angle == 90 || angle == -90 || angle==45 || angle==-45)
  {
    while(cm<dist*30)
    {
      forward();
    encdist();
    detachInterrupt(digitalPinToInterrupt(2));
    }
  }
  else{
  while(cm<dist*6)
  {
    forward();
    encdist();
    detachInterrupt(digitalPinToInterrupt(2));
  }
  }
  counts=0;
  cm=0;

pause();
  delay(500);
  angle1=angle;
  
///Serial.println('2');
}

void setup() {

  Serial.begin(9600);

  
pinMode(13,OUTPUT);
//digitalWrite(13,HIGH);
 Wire.begin();
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  

  Wire.beginTransmission(MPU6050_addr);                                       
  Wire.write(0x1B);                                                   
  Wire.write(0x08);                                                   
  Wire.endTransmission(true); 

  MPU6050 accelgyro(MPU6050_addr);
  accelgyro.setXAccelOffset(1248); //-729
  accelgyro.setYAccelOffset(-1563); //-1384
  accelgyro.setZAccelOffset(1287); //1518
  accelgyro.setXGyroOffset(46);//23
  accelgyro.setZGyroOffset(-37); 

  pinMode(enr1, OUTPUT);
  pinMode(enr11, OUTPUT);
  pinMode(enr12, OUTPUT);
  pinMode(enr2, OUTPUT);
  pinMode(enr21, OUTPUT);
  pinMode(enr22, OUTPUT);
  pinMode(enr3, OUTPUT);
  pinMode(enr31, OUTPUT);
  pinMode(enr32, OUTPUT);
  pinMode(enl1, OUTPUT);
  pinMode(enl12, OUTPUT);
  pinMode(enl11, OUTPUT);
  pinMode(enl2, OUTPUT);
  pinMode(enl21, OUTPUT);
  pinMode(enl22, OUTPUT);
  pinMode(enl3, OUTPUT);
  pinMode(enl31, OUTPUT);
  pinMode(enl32, OUTPUT);

 //initialization of motors

  digitalWrite(enr12, LOW);
  digitalWrite(enr11, LOW);
  digitalWrite(enr22, LOW);
  digitalWrite(enr21, LOW);
  digitalWrite(enr31, LOW);
  digitalWrite(enr32, LOW);
  digitalWrite(enl11, LOW);
  digitalWrite(enl12, LOW);
  digitalWrite(enl21, LOW);
  digitalWrite(enl22, LOW);
  digitalWrite(enl31, LOW);
  digitalWrite(enl32, LOW);

  analogWrite(enr1, 0);
  analogWrite(enr2, 0);
  analogWrite(enr3, 0);
  analogWrite(enl1, 0);
  analogWrite(enl2, 0);
  analogWrite(enl3, 0);

 /* while(anglempu*1.1<90)
{
  left();
  mpuangle();
  
}
pause();*/
/*left();
delay(600);
pause();*/

}





void loop()
{
  
// myservo_x.write(anglex);

//  myservo_y.write(angley);


    
if(Serial.available()>0)
{

rec = Serial.read();

if(rec == 'e')
{
  read_flag = 2;
}

if(read_flag == 1)
{
  angleTemp += rec;
}

if(rec == 's')
{
  read_flag = 1;
  angleTemp = "";
  
}



//delay(500);
  //Serial.flush();

if(read_flag == 2)
{
  
 for(i=2;i>=0;i--)
{
  if(i==2)
  dist=(angleTemp[i]-'0');
  if(i==1)
  dist=dist+((angleTemp[i]-'0')*10);
  if(i==0)
  dist=dist+((angleTemp[i]-'0')*100);
    //int rx=map(anglex,180,0,108,62);
  
    
  
 }

 for(i=5;i>=3;i--)
{
  if(i==5)
  angle=(angleTemp[i]-'0');
  if(i==4)
  angle=angle+((angleTemp[i]-'0')*10);
  if(i==3)
  angle=angle+((angleTemp[i]-'0')*100);
   // int ry=map(angley,180,0,100,71);
  
 }
 angle = angle - 90;

 /*if(dist==6)
{
  digitalWrite(13,LOW); 
  delay(1000);

}
 if(dist==12)
{
  digitalWrite(13,LOW); 
  delay(1000);

}
if(angle==90)
{
   digitalWrite(13,HIGH); 
delay(1000);
  
  }
  if(angle==45)
{
   digitalWrite(13,HIGH); 
delay(1000);*/
 control();

  }

/*if(angle==45)
{
    digitalWrite(13,LOW); 
delay(1000);
}*/
  
}

/*while(anglempu>-90)
{
  right();
  mpuangle();
  
}
pause();
delay(1000);
while(anglempu<90)
{
  left();
  mpuangle();
  
}
pause();
delay(1000);*/

    
  


//mpuangle();
}

 
//Serial.flush();
