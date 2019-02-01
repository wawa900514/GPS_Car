#include <Wire.h>
#include <HMC5883L.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2); 
HMC5883L compass;
#include "TinyGPS++.h"
#include "SoftwareSerial.h"
SoftwareSerial serial_connection(10, 11); //RX=pin 10, TX=pin 11
TinyGPSPlus gps;
float Aj;
float Aw;
float Bj;
float Bw;
float j[]={120.835733,120.835659,120.835763};  //經度
float w[]={24.239366,24.239330,24.239374};   //緯度
byte total=sizeof(j);
float x;
float y;
float declinationAngle;
float heading;
int angle;            //兩點間角度
int headingDegrees;   //羅盤角度
int i;
float(earthRadius); //地球半徑
float(factor);
float(dlat);
float(dlon);
float(a);
float(c);
float(d);
byte position_led;
const byte LED = 22;   // LED 的腳位
const byte SW = 3;     // 開關的腳位
boolean lastState = LOW; // 記錄上次的開關狀態，預設為「低電位」
boolean toggle = LOW;    // 輸出給 LED 的訊號，預設為「低電位」
byte click = 0;        // 開關訊號的改變次數，預設為 0 
byte sonar_count;
#include <NewPing.h>
#define TRIGGER_PIN  A1  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     A0  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup()
{
  Serial.begin(9600);
  // Initialize Initialize HMC5883L
  Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(68,-316);
 Serial.begin(9600);//This opens up communications to the Serial monitor in the Arduino IDE
  serial_connection.begin(9600);//This opens up communications to the GPS
  Serial.println("GPS Start");
Bj=j[0];
Bw=w[0];  
pinMode(4,OUTPUT);
pinMode(5,OUTPUT);
pinMode(6,OUTPUT);
pinMode(7,OUTPUT);
pinMode(LED, OUTPUT);
pinMode(24,OUTPUT);
pinMode(26,OUTPUT);
pinMode(28,OUTPUT);
pinMode(SW, INPUT);
lastState = digitalRead(SW);   // 讀取開關的初始值
lcd.begin();// Print a message to the LCD.
lcd.backlight();
}

void loop()
{
 sw();
 while((toggle==0)||(d>10000)){
  motor_stop();
  delay(100);                       
  break;
 }
//GPS
 while(serial_connection.available())//While there are characters to come from the GPS
  {
    gps.encode(serial_connection.read());//This feeds the serial NMEA data into the library one char at a time
  }
  if(gps.location.isUpdated())//This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
  {
    //Get the latest info from the gps object which it derived from the data sent by the GPS unit
  //  Serial.println("Satellite Count:");
  //  Serial.println(gps.satellites.value());
    Serial.println("Latitude:");
    Serial.println(gps.location.lat(), 6);
    Serial.println("Longitude:");
    Serial.println(gps.location.lng(), 6);
  }
    //compass
  Vector norm = compass.readNormalize();
  // Calculate heading
heading = atan2(norm.YAxis, norm.XAxis);
declinationAngle = (-4.0 + (14.0 / 60.0)) / (180 / M_PI);
heading += declinationAngle;
degree_correct();
  // Output
 // Serial.print(" Heading = ");
  //Serial.print(heading);
  Serial.print(" Degress = ");
  Serial.print(headingDegrees );
  Serial.println();

 angle_calculate();
 Distance_calculation();
 sonar_d();
 
if(angle>(headingDegrees+30)){

 turn_left();
 
}
if(angle< (headingDegrees-30)){

turn_right();

}
/*if(d>10000){
  motor_stop();
  delay(1000);
}*/

if ((d>2) && (toggle==1) && (d<10000)){
  if((angle<= (headingDegrees+30)) && (angle>=(headingDegrees-30))){
straight();
lcd_print();
delay(100);
sonar_count=0;
}
}
if(d<=2){
motor_stop();
lcd_print();
delay(3000);
i=i+1;
position_led=(2*i)+22;
digitalWrite(position_led,HIGH);
if(i>total ){
  motor_stop();
}else{
Bj=j[i];
Bw=w[i];
}
}

  

}



//副程式
void motor_stop (){
  digitalWrite(4,LOW);
   digitalWrite(5,LOW);
   digitalWrite(6,LOW);
    digitalWrite(7,LOW);
    
}
void straight (){
  digitalWrite(4,HIGH);
   digitalWrite(5,LOW);
   digitalWrite(6,HIGH);
    digitalWrite(7,LOW);
}
void turn_right (){
  digitalWrite(4,HIGH);
   digitalWrite(5,LOW);
   digitalWrite(6,LOW);
    digitalWrite(7,HIGH);
}
void turn_left (){
  digitalWrite(4,LOW);
   digitalWrite(5,HIGH);
   digitalWrite(6,HIGH);
    digitalWrite(7,LOW);
}
void degree_correct(){
 if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
 headingDegrees = heading * 180/M_PI;     
}

void angle_calculate(){
 Aj =gps.location.lng();
 Aw=gps.location.lat();

x=(Bj-Aj)*cos(Bw);
y=Bw-Aw;
angle = atan2(x,y)*180/PI;
if (Bw>Aw && Bj>Aj){
 angle +=360; 
  }
 if (Bw>Aw && Bj<Aj){
 angle +=360; 
  }
  if(angle<0){
    angle=angle+360;
  }
  if (angle>360){
    angle=angle-360;
  }
//Serial.print("angle:");
//Serial.print(angle);
   
}
void Distance_calculation(){
factor=PI/180;
earthRadius=6371;
dlat=(Bw-Aw)*factor;
dlon=(Bj-Aj)*factor;
a=sin(dlat/2)*sin(dlat/2)+cos(Aw*factor)*cos(Bw*factor)*sin(dlon/2)*sin(dlon/2);
c=2*atan2(sqrt(a),sqrt(1-a));
d=earthRadius*c*1000;
//Serial.print("   distance:");
//Serial.print(d);
}

void lcd_print(){
 lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("deg:");
  lcd.setCursor(4,0);
  lcd.print(angle);
   lcd.setCursor(8,0);
  lcd.print("cps:");
   lcd.setCursor(12,0);
  lcd.print(headingDegrees);
lcd.setCursor(0,1);
  lcd.print("dts:");
  lcd.setCursor(4,1);
  lcd.print(d);
}

void sw(){
 boolean b1 = digitalRead(SW);

  if (b1 != lastState) {     // 如果和之前的開關值不同...
    delay(20);               // 等待 20 毫秒
    boolean b2 = digitalRead(SW);   // 再讀取一次開關值

    if (b1 == b2) {    // 確認兩次開關值是否一致
      lastState = b1; // 儲存開關的狀態
      click ++;       // 增加訊號變化次數
    }
  }

  if (click == 2) {    // 如果開關狀態改變兩次
    click = 0;         // 狀態次數歸零
    toggle = !toggle;            // 取相反值
    digitalWrite(LED, toggle);   // 輸出
  }
  
}
void sonar_d(){
   delay(50);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  Serial.print("Ping: ");
  Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
  Serial.println(sonar_count);
 if((sonar_count >= 3) && (sonar.ping_cm()>10) && (sonar.ping_cm()<70)){
     motor_stop();
  delay(500);
  turn_right();
  delay(1500);
  straight();
  delay(500);
  
}
if((sonar.ping_cm()>10) && (sonar.ping_cm()<70) && (sonar_count <= 3)){
  sonar_count++;
   motor_stop();
  delay(500);
  turn_left();
  delay(500);
  straight();
  delay(500);
}
/*if(sonar_count>2){
  motor_stop();
  delay(500);
  turn_left();
  delay(500);
  straight();
  delay(500);
  sonar_count = 0;
}*/
  
}
