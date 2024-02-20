#include <Wire.h>                                       // Подключаем библиотеку Wire
#include <Adafruit_Sensor.h>                            // Подключаем библиотеку Adafruit_Sensor
#include <Adafruit_BME280.h>                            // Подключаем библиотеку Adafruit_BME280

#include <MQ135.h>
#include <DHT.h>
#define SEALEVELPRESSURE_HPA (1013.25)                  // Задаем высоту
 Adafruit_BME280 bme;

#define PIN_MQ135 A2
MQ135 mq135_sensor(PIN_MQ135);

#include<string.h>
byte buff[2];
int pin = 8;//DSM501A input D8
unsigned long duration;
unsigned long starttime;
unsigned long endtime;
unsigned long sampletime_ms = 9999;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

/*const uint8_t pinRX = 5;                          //  Определяем вывод RX (программного UART) на плате Arduino к которому подключён вывод TX модуля. Номер вывода можно изменить.
const uint8_t pinTX = 6;                          //  Определяем вывод TX (программного UART) на плате Arduino к которому подключён вывод RX модуля. Номер вывода можно изменить.
                                                  //
#include <SoftwareSerial.h>                       //  Подключаем библиотеку для работы с программным UART, до подключения библиотеки iarduino_GPS_NMEA.
#include <iarduino_GPS_NMEA.h>                    //  Подключаем библиотеку для расшифровки строк протокола NMEA получаемых по UART.
                                                  //
SoftwareSerial    SerialGPS(pinRX, pinTX);        //  Объявляем объект SerialGPS для работы с функциями и методами библиотеки SoftwareSerial, указав выводы RX и TX Arduino.
iarduino_GPS_NMEA gps;                            //  Объявляем объект gps для работы с функциями и методами библиотеки iarduino_GPS_NMEA.
*/

// Инициализируем библиотеки
#include <Wire.h>
#include <CG_RadSens.h>


#define buz_pin 14 // Задаём значения пина для пищалки


CG_RadSens radSens(RS_DEFAULT_I2C_ADDRESS); // Инициализируем RadSens

int timer_cnt; // Таймер для измерений дозиметра
int timer_bat; // Таймер для измерения заряда батареи
int timer_imp; // Таймер опроса импульсов для пьезоизлучателя
int pulsesPrev; // Число импульсов за предыдущую итерацию

//Функция аудиоприветствия
void hello() {
  for (int i = 1; i < 5; i++) {
    tone(buz_pin, i * 1000);
    delay(100);
  }
  tone(buz_pin, 0);
  delay(100);
  Serial.print("Radsensor");
  delay(3000);
  Serial.println("----------------------------");
}

//Функция, которая создаёт "трески" пьезоизлучателя при появлении импульсов
void beep() {     // Функция, описывающая время и частоту пищания пьезоизлучателя
  tone(buz_pin, 3500);
  delay(13);
  tone(buz_pin, 0);
  delay(40);
}

//Функция предупреждения при превышении порога излучения
void warning() {
  for (int i = 0; i < 3; i++) {
    tone(buz_pin, 1500);
    delay(250);
    tone(buz_pin, 0);
    delay(250);
  }
}




 void setup() {
   Wire.begin();
   Serial.begin(9600);                                  // Открытие последовательного порта на скорости 9600  
 /*if (!bme.begin(0x76)) {                                // Инициализация датчика BME280
     Serial.println();                               // Печать отступа
     Serial.println("Could not find a valid BME280!");  // Печать сообщения об ошибки
     while (1);
   }*/
   pinMode(8,INPUT);
   starttime = millis(); 
   /*SerialGPS.begin(9600);                       //  Инициируем работу с программной шиной UART для получения данных от GPS модуля на скорости 9600 бит/сек.
   gps.begin(SerialGPS);                        //  Инициируем расшифровку строк NMEA указав объект используемой шины UART (вместо программной шины, можно указывать аппаратные: Serial, Serial1, Serial2, Serial3).*/
   hello();  // Приветствуем пищанием  
  pulsesPrev = radSens.getNumberOfPulses(); // Записываем значение для предотвращения серии тресков на старте


 }
 void loop() {
  BME280();
  delay(10000);
 }
 void BME280() {
   /*float temp = bme.readTemperature();
   float pres = bme.readPressure() / 100.0F;
   float alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
   float hum = bme.readHumidity();*/
   float temp = 25.2;
   float pres = 970.2;
   float alt = 70.3;
   float hum = 60.1;
  Serial.println();

  Serial.print("Temperature = ");                         // Печать текста
   Serial.print(temp);                                   // Печать температуры
   Serial.println("*C");                                 // Печать текста
   Serial.println();

   Serial.print("Pressure = ");                            // Печать текста       
   Serial.print(pres);                                   // Печать атмосферное давление
   Serial.println("hPa");                                // Печать текста
   Serial.println();

   Serial.print("Approx. Altitude = ");                    // Печать текста
   Serial.print(alt);                                    // Вычисление высоты
   Serial.println("m");                                  // Печать текста
   Serial.println();

   Serial.print("Humidity = ");                            // Печать текста
   Serial.print(hum);                                    // Печать влажности
   Serial.println("%");                                  // Печать текста
   
   Serial.println();
   MQ135(temp,hum);

 }

 void MQ135(float temp,float hum) {
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
  Serial.print("Corrected PPM: ");
  Serial.print(correctedPPM);
  Serial.println("ppm");
  Serial.println();
  DSM501A();
  Serial.println("============================");

 
}
 void DSM501A() {
  duration = pulseIn(pin, LOW);
  lowpulseoccupancy += duration;
  endtime = millis();
    float ratio = (lowpulseoccupancy-endtime+starttime + sampletime_ms)/(sampletime_ms*10.0);  // Integer percentage 0=>100
    float concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
    Serial.print("Lowpulseoccupancy: ");
    Serial.println(lowpulseoccupancy);
    Serial.println();
    Serial.print("Ratio: ");
    Serial.println(ratio);
    Serial.println();
    Serial.print("DSM501A: ");
    Serial.println(concentration);
    Serial.println();
    lowpulseoccupancy = 0;
    starttime = millis();
  //GPS_sensor();
  
  
 }

  /*void GPS_sensor() {
    gps.read();                                  //  Читаем данные (чтение может занимать больше 1 секунды). Функции можно указать массив для получения данных о спутниках.
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
     }else{                                       //
         Serial.println("Нет данных.");           //
         delay(2000);                             //
     }


  RadSensor();
  }*/
  
  void RadSensor() {
    timer_imp = millis();
    int pulses = radSens.getNumberOfPulses();
    if (pulses - pulsesPrev > 5 ) {
      pulsesPrev = pulses;
      warning();
    }
    if (pulses > pulsesPrev) {
      for (int i = 0; i < (pulses - pulsesPrev); i++) {
        beep();
      }
      pulsesPrev = pulses;
    }
  
  // Снимаем показания с дозиметра и выводим их на экран
  
    timer_cnt = millis();
    char buf1[50];
    char buf2[50];
    char buf3[50];
    sprintf(buf1, "%.1f мкр/ч", radSens.getRadIntensyDynamic()); // Собираем строку с показаниями динамической интенсивности
    sprintf(buf2, "Стат: %.1f мкр/ч ", radSens.getRadIntensyStatic()); // Собираем строку с показаниями средней интенсивности за период работы
    Serial.println(buf1);
    Serial.println();
    Serial.println(buf2);
    Serial.println();
  

  }