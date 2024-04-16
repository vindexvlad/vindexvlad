#include <Wire.h>                                       // Подключаем библиотеку Wire
#include <Adafruit_Sensor.h>                            // Подключаем библиотеку Adafruit_Sensor
#include <Adafruit_BME280.h>                            // Подключаем библиотеку Adafruit_BME280

#include <MQ135.h>
#include <DHT.h>
#define SEALEVELPRESSURE_HPA (1013.25)                  // Задаем высоту
 Adafruit_BME280 bme;

#define PIN_MQ135 A7
MQ135 mq135_sensor(PIN_MQ135);

//#include<string.h>
byte buff[2];
int pin = 8;//DSM501A input D8
unsigned long duration;
unsigned long starttime;
unsigned long endtime;
unsigned long sampletime_ms = 9999;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

#include <iarduino_GPS_NMEA.h>                    //  Подключаем библиотеку для расшифровки строк протокола NMEA получаемых по UART.
                                                  
iarduino_GPS_NMEA gps;


#include <SPI.h>
#include <SD.h>
const int PIN_CHIP_SELECT = 53;







 void setup() {
   //Wire.begin();
   Serial.begin(9600);                                  // Открытие последовательного порта на скорости 9600  
   Serial1.begin(9600);
   gps.begin(Serial1);

   Serial.print("Initializing SD card...");
   pinMode(5, OUTPUT);                                  
 if (!SD.begin(PIN_CHIP_SELECT)) {
    Serial.println("Card failed, or not present");
    return;                                             
  }
  Serial.println("card initialized.");
 
 if (!bme.begin(0x76)) {                                // Инициализация датчика BME280
     Serial.println();                                  // Печать отступа
     Serial.println("Could not find a valid BME280!");  // Печать сообщения об ошибки
     while (1);
   }
   pinMode(8,INPUT);
   starttime = millis(); 
   loop();

  

 }

 void loop() {
  String logStringData = "";
  gps.read();                                     
     if(!gps.errPos){                             //
         Serial.print("http://maps.yandex.ru/");  //  Ссылка на yandex карты:
                                                  //
         Serial.print("?ll=");                    //  Координаты центра экрана:
         Serial.print(gps.longitude,5);           //  Долгота.
         Serial.print(",");                       //  ,
         Serial.print(gps.latitude,5);            //  Широта.
                                                  //
         Serial.print("&pt=");                    //  Координаты точки на карте:
         Serial.print(gps.longitude,5);           //  Долгота.
         Serial.print(",");                       //  ,
         Serial.print(gps.latitude,5);            //  Широта.
                                                  //
         Serial.print("&l=");                     //  Тип карты:
         Serial.print("map");                     //  "map"-схема (по умолчанию), "sat"-спутник, "skl"-гибрид.
                                                  //
         Serial.print("&z=");                     //  Приближение:
         Serial.print("18");                      //  от 2-мир, до 19-дом, по умолчанию 10-город.
                                                  //
         Serial.print("\r\n");                    //

        logStringData+="http://maps.yandex.ru/";
        logStringData+="?ll=";
        logStringData+=String(gps.longitude,5);
        logStringData+=",";
        logStringData+=String(gps.latitude,5);
        logStringData+="&pt=";
        logStringData+=String(gps.longitude,5);
        logStringData+=",";
        logStringData+=String(gps.latitude,5);
        logStringData+="&l=";
        logStringData+="map";
        logStringData+="&z=";
        logStringData+="18";
        logStringData+="\r\n";
        logStringData+="    ";

     }else{                                       
         Serial.println("Нет данных.");           
         logStringData+="Нет данных.";
         logStringData+="    ";
     }
  



   float temp = bme.readTemperature();
   float pres = bme.readPressure() / 100.0F;
   float alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
   float hum = bme.readHumidity();

  Serial.println();

  Serial.print("Temperature: ");                          // Печать текста
   Serial.print(temp);                                    // Печать температуры
   Serial.println("*C");                                  // Печать текста
   Serial.println();
   
   logStringData+="Temperature: ";
   logStringData+=String(temp);
   logStringData+="*C,   ";

   Serial.print("Pressure: ");                           // Печать текста       
   Serial.print(pres);                                   // Печать атмосферное давление
   Serial.println("hPa");                                // Печать текста
   Serial.println();

   logStringData+="Pressure: ";
   logStringData+=String(pres);
   logStringData+="hPa,   ";

   Serial.print("Approx. Altitude: ");                   // Печать текста
   Serial.print(alt);                                    // Вычисление высоты
   Serial.println("m");                                  // Печать текста
   Serial.println();

   logStringData+="Approx. Altitude: ";
   logStringData+=String(alt);
   logStringData+="m,   ";

   Serial.print("Humidity: ");                           // Печать текста
   Serial.print(hum);                                    // Печать влажности
   Serial.println("%");                                  // Печать текста

   logStringData+="Humidity: ";
   logStringData+=String(hum);
   logStringData+="%,   ";
   
   Serial.println();
   MQ135(temp,hum,logStringData);


 }

 void MQ135(float temp,float hum, String logStringData) {
  float humidity = hum;
  float temperature = temp;
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("Failed to read from BME sensor!"));
    return;
  }
  float ppm = mq135_sensor.getPPM();
  float correctedPPM = mq135_sensor.getCorrectedPPM(temperature, humidity);

  Serial.print("PPM: ");
  Serial.print(ppm);
  Serial.println("ppm");
  Serial.println();

   logStringData+="PPM: ";
   logStringData+=String(ppm);
   logStringData+="ppm,   ";

  Serial.print("Corrected PPM: ");
  Serial.print(correctedPPM);
  Serial.println("ppm");
  Serial.println();

   logStringData+="Corrected PPM: ";
   logStringData+=String(correctedPPM);
   logStringData+="ppm,   ";


  Serial.println("============================");
  DSM501A(logStringData);
  

}
 void DSM501A(String logStringData) {
  duration = pulseIn(pin, LOW);
  lowpulseoccupancy += duration;
  endtime = millis();
    float ratio = (lowpulseoccupancy-endtime+starttime + sampletime_ms)/(sampletime_ms*10.0);  
    float concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; 
    Serial.print("Lowpulseoccupancy: ");
    Serial.println(lowpulseoccupancy);
    Serial.println();
    logStringData+="Lowpulseoccupancy: ";
    logStringData+=String(lowpulseoccupancy);
    logStringData+=",   ";
    Serial.print("Ratio: ");
    Serial.println(ratio);
    Serial.println();
    logStringData+="Ratio: ";
    logStringData+=String(ratio);
    logStringData+=",   ";
    Serial.print("DSM501A: ");
    Serial.println(concentration);
    Serial.println();
    logStringData+="DSM501A: ";
    logStringData+=String(concentration);
    logStringData+=",   ";

    lowpulseoccupancy = 0;
    starttime = millis();
  SD_card(logStringData);

  
  
 }

  
  
  

  void SD_card(String logStringData){

    File dataFile = SD.open("datalog.csv", FILE_WRITE);
    if (dataFile) {
      logStringData ="Time: " + String(millis()) + ",   " + logStringData;
    dataFile.println(logStringData);
    dataFile.println();
    dataFile.close();

    Serial.println(logStringData);
    logStringData = "";
    delay(5000);
  }
  else {

    Serial.println("error opening datalog.csv");
  }

  }
