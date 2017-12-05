#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
extern "C" {
#include "user_interface.h"
}
//////////////////////
// WiFi Definitions //
//////////////////////
const char WiFiAPPSK[] = "yoloswag";

/////////////////////
// Pin Definitions //
/////////////////////
const int VIB_PIN = D2; // gpio13 Thing's onboard, green LED
const int LEDPIN = 16;
ESP8266WebServer server(80);
static char data_websiteBuffer[10000];
const static uint8_t data_indexHTML[] PROGMEM = "<html><head><meta charset='utf-8'></head><body><h1>CHEATPEN</p></body></html>";
const char data_error404[] PROGMEM = "<html><head><meta charset='utf-8'></head><body><h1>SITE NOT FOUND</p></body></html>";
//VARS

void setup()
{
  initHardware();
  setupWiFi();
  server.begin();
  server.onNotFound(load404);
  server.on("/", loadIndex);
  server.on("/index.html", loadIndex);
  server.on("/vibrate", handleVibrateArgs); //Associate the handler function to the path
  server.begin();
}

void loop() {
  server.handleClient();

}
void handleVibrateArgs() { //Handler


  //respons msg
  String code = server.arg("code");
  int vibDelay = server.arg("delay").toInt();

  server.send(200, "text/html", "OK");       //Response to the HTTP request
  Serial.println("__________________________");
  Serial.print("code: ");
  Serial.print(code);
  Serial.println();
  for (int i = 0 ; i < code.length(); i++) {
    
    if (code.charAt(i) == '0') {
      analogWrite(VIB_PIN, 0);
      analogWrite(LEDPIN, 1023);
      Serial.println("STOP VIBRATE");
    }else{
      int codeToPWM = code.charAt(i);
      codeToPWM = map(codeToPWM, 0, 9, 0, 800); // from 0 to 9 map it to val between 0-1023 REF https://www.arduino.cc/reference/en/language/functions/math/map/
       int LEDToPWM = code.charAt(i);
      LEDToPWM = map(LEDToPWM, 0, 9, 1023, 0); // from 0 to 9 map it to val between 0-1023 REF https://www.arduino.cc/reference/en/language/functions/math/map/
      analogWrite(VIB_PIN, codeToPWM); // get the value from the req, and set it. REF http://henrysbench.capnfatz.com/henrys-bench/arduino-projects-tips-and-more/nodemcu-io-basics-pwm/
      
      analogWrite(LEDPIN, LEDToPWM);
      Serial.println("VIBRATE LEDPWM:");
      Serial.println(LEDToPWM);
      }
    Serial.println(code.charAt(i));
    delay(vibDelay);//fixed
    
  }
  analogWrite(LEDPIN, 1023);
  analogWrite(VIB_PIN, 0);
  Serial.println("__________________________");


  Serial.println("__________________________");
  Serial.print("delay: ");
  Serial.print(vibDelay);
  Serial.println();
  Serial.println("__________________________");


}
void load404() {
  server.send ( 200, "text/html", data_get404() );
}
void loadIndex() {
  server.send ( 200, "text/html", data_getIndexHTML());
}
char* data_get404() {
  int _size = sizeof(data_error404);
  for (int i = 0; i < sizeof(data_websiteBuffer); i++) {
    if (i < _size) data_websiteBuffer[i] = pgm_read_byte_near(data_error404 + i);
    else data_websiteBuffer[i] = 0x00;
  }
  return data_websiteBuffer;
}
char* data_getIndexHTML() {
  int _size = sizeof(data_indexHTML);
  for (int i = 0; i < sizeof(data_websiteBuffer); i++) {
    if (i < _size) data_websiteBuffer[i] = pgm_read_byte_near(data_indexHTML + i);
    else data_websiteBuffer[i] = 0x00;
  }
  return data_websiteBuffer;
}

void setupWiFi()
{
  WiFi.mode(WIFI_AP);

  // Do a little work to get a unique-ish name. Append the
  // last two bytes of the MAC (HEX'd) to "Thing-":
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
  String AP_NameString = "ExamCheatPen " + macID;

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, AP_NameString.length() + 1, 0);

  for (int i = 0; i < AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);

  WiFi.softAP(AP_NameChar, WiFiAPPSK);
  Serial.print("Use this URL to connect: ");
  Serial.print("http://");
  Serial.print(WiFi.localIP());
}

void initHardware()
{
  Serial.begin(115200);
  pinMode(LEDPIN, OUTPUT);
  pinMode(VIB_PIN, OUTPUT);
}
