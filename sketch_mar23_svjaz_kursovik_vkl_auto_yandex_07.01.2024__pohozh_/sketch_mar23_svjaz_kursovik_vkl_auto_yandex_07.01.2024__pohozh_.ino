

#include <string.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "certs.h"
#include <Wire.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

const char *mqtt_server = "mqtt.cloud.yandex.net";  //сервер кому есп будет направлять запрос
const int mqtt_port = 8883;

/* change it with your ssid-password */
const char *ssid = "Alevtina";
const char *password = "A6003557";
//const char *ssid = "AndroidAP069D";
//const char *password = "ikmu9538";
/* this is the IP of PC/raspberry where you installed MQTT Server
on Wins use "ipconfig"
on Linux use "ifconfig" to get its IP address */
//const char *mqtt_server = "51.250.11.159";

/* create an instance of PubSubClient client */
WiFiClientSecure net_client_pub;  //wifisecure для подключения к сети. И через него подключается клиент mqtt.
WiFiClientSecure net_client_sub;  //в обратную сторону
PubSubClient mqtt_client_pub;     //подключается клиент mqtt для публикации уже на яндексе посде подключения
PubSubClient mqtt_client_sub;     //в обратную сторону
//WiFiClient espClient; //An object of WiFiClientSecure is created called ‘espClient.’
//PubSubClient client(espClient);

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)
uint16_t input_from_stm[4];
uint8_t output_from_esp[2];
long lastPublishTime;

int digit_sub = 0;
int digit_pub = 0;


/* topics */
#define DHT11_TOPIC_T "gb_iot/1863_KAO/smarthome/temp"
#define DHT11_TOPIC_HUM "gb_iot/1863_KAO/smarthome/hum"
#define TOPIC_GMP180_PR "gb_iot/1863_KAO/smarthome/Pres"
#define LED_TOPIC_BRIGHT "gb_iot/1863_KAO/smarthome/brigh"
#define LED_TOPIC "gb_iot/1863_KAO/smarthome/status_led"
#define LED_TOPIC_1 "gb_iot/1863_KAO/smarthome/led_on"
//#define AUTO_TOPIC "smarthome/auto"
#define LED_TYPE_BRIGHT "gb_iot/1863_KAO/smarthome/type_brigh"


#define TOPIC_GMP180_P "gb_iot/Challenge/50_MSK_КАО/Pres"
#define TOPIC_GMP180_T "gb_iot/Challenge/50_MSK_КАО/Temp"
#define TOPIC_GMP180_H "gb_iot/Challenge/50_MSK_КАО/Hum"

#define RECONNECTED_SUB "gb_iot/Challenge/50_MSK_КАО/Reconnected_SUB"
#define RECONNECTED_PUB "gb_iot/Challenge/50_MSK_КАО/Reconnected_PUB"

#define LED_PIN (GPIO_NUM_4)  // GPIO-контакт, к которому подключен светодиод



//сервер получил данные, запускается функция Callback по этому событию. Серевер отправляет данные всем подписчикам,
//Callback приостанавливаем смежные процессы и принимает данные
void receivedCallback(char *topic, byte *payload, unsigned int length)  // topic - имя топика, payload - содержимое типа байт, length - длина содержимого
{
  Serial.print("topic = ");
  Serial.println(topic);
  Serial.print("payload[0] = ");
  //чтобы прочитать набор byte как символы - делаем преобразование типов
  Serial.println(((char *)payload)[0]);  //чтобы вытащить значение в нужном виде
                                         //сначала преобразовываем адрес byte в адес char, поэтому ставим скобки, а потом только читаем первый элемент массива
  if (strcmp(topic, LED_TOPIC_1) == 0) {
    Serial.println("topic valid");
    output_from_esp[0] = payload[0];

    Serial.print("resieved - ");
    Serial.println(((char *)output_from_esp)[0]);
    Serial.print("topic = ");
    Serial.println(topic);
    Serial.print("lenght = ");
    Serial.println(length);
  } else if (strcmp(topic, LED_TYPE_BRIGHT) == 0) {
    Serial.println("topic valid");
    output_from_esp[1] = payload[0];

    Serial.print("resieved - ");
    Serial.println(((char *)output_from_esp)[1]);
  }

  //Serial2.write(0xAB);
  //uint8_t test = 5;
  //Serial2.write(&test, 1);
  Serial2.write(output_from_esp, 2);
}

void wifi_connect() {
  Serial.println("Connecting to " + String(ssid));
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("ip: ");
  Serial.println(WiFi.localIP());
  Serial.print("mac: ");
  Serial.println(WiFi.macAddress());
  Serial.println();
}

void mqtt_connect_pub() {
  while (!mqtt_client_pub.connected()) {
    Serial.print("MQTT publish connecting ");
    if (mqtt_client_pub.connect(mqtt_server)) {
      Serial.println("OK");

    } else {
      Serial.print("failed, status code = ");
      Serial.print(mqtt_client_pub.state());
      Serial.println(" try again in 5 seconds");
      /* Wait 5 seconds before retrying */
      delay(5000);
    }
  }
}
void mqtt_connect_sub() {
  while (!mqtt_client_sub.connected()) {
    Serial.print("MQTT subscribe connecting ");
    if (mqtt_client_sub.connect(mqtt_server)) {
      Serial.println("OK");
      /* subscribe topic with default QoS 0*/
      mqtt_client_sub.subscribe(LED_TOPIC_1);
      //client.subscribe(AUTO_TOPIC);
      mqtt_client_sub.subscribe(LED_TYPE_BRIGHT);


    } else {
      Serial.print("failed, status code = ");
      Serial.print(mqtt_client_sub.state());
      Serial.println(" try again in 5 seconds");
      /* Wait 5 seconds before retrying */
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  if (!bmp.begin()) {
    Serial.println("Not connected with BMP180/BMP085 sensor, check connections ");
    while (1) {}
  }
  
  // We start by connecting to a WiFi network - не возвращаем
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  /* configure the MQTT server with IPaddress and port */
  //client.setServer(mqtt_server, 8883);
  /* this receivedCallback function will be invoked
  when client received subscribed topic */
  // client.setCallback(receivedCallback);

  /* configure the MQTT server with IPaddress and port */
  // client.setServer(mqtt_server, 8883);
  mqtt_client_pub.setServer(mqtt_server, 8883);
  mqtt_client_sub.setServer(mqtt_server, 8883);

  Serial2.begin(115200, SERIAL_8N1, RXD_PIN, TXD_PIN);  // SERIAL_8N2 означает, что по последовательному порту ардуино передаст пакет
                                                        // длиной 8 бит без бита контроля четности (указано N) и с одним стоповым битом,                                                   // который будет добавлен после байта данных.

  //publish topic
  net_client_pub.setCACert(root_ca);
  net_client_pub.setCertificate(dev_cert_tx);
  net_client_pub.setPrivateKey(key_cert_tx);
  
  mqtt_client_pub.setClient(net_client_pub);
  mqtt_client_pub.setServer(mqtt_server, mqtt_port);
  mqtt_client_pub.setKeepAlive (70);

  //subscribe topic
  net_client_sub.setCACert(root_ca);
  net_client_sub.setCertificate(dev_cert_rx);
  net_client_sub.setPrivateKey(key_cert_rx);
  
  mqtt_client_sub.setClient(net_client_sub);
  mqtt_client_sub.setServer(mqtt_server, mqtt_port);
  mqtt_client_sub.setKeepAlive (70);
  mqtt_client_sub.setCallback(receivedCallback);
  // mqtt_client_sub.setCallback(callbackMqtt); //не помню откуда взяла и что это

  //подключение
  wifi_connect();
  delay(500);
  mqtt_connect_pub();
  mqtt_connect_sub();
  

  //  mqtt_client_sub.subscribe(lamp);
  //  mqtt_client_pub.publish(lamp, "off");
}

void loop() {

  /* if client was disconnected then try to reconnect again */
  if (!mqtt_client_sub.connected()) {
    mqtt_connect_sub();
    digit_sub++;
    Serial.print("digit_sub = ");
    Serial.println(digit_sub);
  }

  /* if client was disconnected then try to reconnect again */
  if (!mqtt_client_pub.connected()) {
    mqtt_connect_pub();
    digit_pub++;
    Serial.print("digit_pub = ");
    Serial.println(digit_pub);
  }


  /* this function will listen for incomming
  subscribed topic-process-invoke receivedCallback */
  mqtt_client_sub.loop();

  if (Serial2.available() > 0)  // если есть доступные данные
                                //  считываем 1 байта.
  {
    uint8_t start = 0;
    Serial2.readBytes(&start, 1);  //читаем 1 байта в переменную start
    Serial.print("read start = ");
    Serial.println(start, HEX);

    if (start == 0xFF)  // если это начало передачи, читаем остальное.
    {
      Serial2.readBytes((uint8_t *)input_from_stm, 8);
    }
  }

  //это та часть котрая отображает статус светика
  static uint16_t prev_led = 0xFF; // static сохраняет переменные между вызовами функции loop
  static char  led[4] = {0};
  //если предидущее показанее не совпадает с текущим, то в предидущее записываем текущее и публикуем

  if (prev_led != input_from_stm[3]) //сравниваем начальную переменную и то что пришло с стм32, если не равны проваливаемся в код
  {
    Serial.println("prev_led != led"); //печать в монитор порта
    if (input_from_stm[3] == 0) { //далее если пришел 0 то записываем off
      sprintf(led, "%s", "OFF");
    } else {
      sprintf(led, "%s", "ON"); // если пришел 0, то пишет в статус лед on
    }

    mqtt_client_pub.publish(LED_TOPIC, led); // публикуем в топик статус
  }
  prev_led = input_from_stm[3]; // помещаем полученное значение в переменную (котрая статик и не изменит свое содержание при вновь вызванной функции)
// и заново возвращаемся к if (prev_led != input_from_stm[3])  - теперь если не поступило нового сообщения, то перемнные равны
//и в тело функции мы не провалимся и вновь публиковать значение не будет


  if (millis() - lastPublishTime > 70000) {
    lastPublishTime = millis();

    char c[20];
    sprintf(c, "%d", input_from_stm[0]);
    Serial.print("t=");
    Serial.println(input_from_stm[0]);
    mqtt_client_pub.publish(DHT11_TOPIC_T, c, strlen(c));

    char a[20];
    sprintf(a, "%d", input_from_stm[1]);
    Serial.print("h=");
    Serial.println(input_from_stm[1]);
    mqtt_client_pub.publish(DHT11_TOPIC_HUM, a, strlen(a));
    mqtt_client_pub.publish(TOPIC_GMP180_H, a, strlen(a));


    char b[20];
    sprintf(b, "%d", input_from_stm[2]);
    Serial.print("br=");
    Serial.println(input_from_stm[2]);
    mqtt_client_pub.publish(LED_TOPIC_BRIGHT, b, strlen(b));

    //ниже данные опроса состояния транзистора подключенного к стм32. если 1, значит свет включен, если 0 - выключен
    //char d[20];
    //sprintf(d, "%d", input_from_stm[3]);
    //Serial.print("rez=");
    //Serial.println(input_from_stm[3]);

    //  mqtt_client_pub.publish(LED_TOPIC, d, strlen(d));//????????? тоже самое. сами писал не убирать из под миллс, иначе один раз будет публиковать и все


    char f[20];
    sprintf(f, " %0.0f", ((bmp.readPressure()) * 0.00750062));
    mqtt_client_pub.publish(TOPIC_GMP180_P, f);
    mqtt_client_pub.publish(TOPIC_GMP180_PR, f);

    //Serial.println(" Pa");

    char j[20];
    sprintf(j, " %0.0f", bmp.readTemperature());
    mqtt_client_pub.publish(TOPIC_GMP180_T, j);

    char w[20];
    sprintf(w, "%d", digit_pub);
    mqtt_client_pub.publish(RECONNECTED_PUB, w);

    char x[20];
    sprintf(x, "%d", digit_sub);
    mqtt_client_pub.publish(RECONNECTED_SUB, x);
  }
}
