
#include <SPI.h>
#include <Wire.h>
#include <SSD1306.h>
#include <LoRa.h>
#include "images.h"
//#include <SoftwareSerial.h>
#include <Adafruit_MCP4725.h>


// Crear un objeto MCP4725
Adafruit_MCP4725 dac;

//#define LORA_BAND    433
#define LORA_BAND    868
//#define LORA_BAND    915

#define OLED_SDA    21
#define OLED_SCL    22
#define OLED_RST    -1

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

SSD1306 display(0x3c, OLED_SDA, OLED_SCL);

float Sensibilidad = 0.0381;// ESP32 0.03935;
#define sense 4 //sensor de corriente
const int pwmPin = 25; // Pin PWM
const int pedal_PIN = 34; // Pin PWM

float maxI = 0;
float I;
int dutyCycle;
//---------GPS---------------//
// Configuración de los pines de comunicación serie

//--------------------------//

// Forward declarations
void displayLoraData(String countStr);
void showLogo();

void setup() {
  Serial.begin(115200);
  
  Wire.begin();
  dac.begin(0x60);
  //mySerial.begin(9600);  // Inicializa el puerto serial en los pines 19 y 23
  //GPS_SERIAL.begin(9600, SERIAL_8N1, 3, 1); // Configuración serie para GPS (9600 baudios)
  
  while (!Serial){};
  Serial.println();
  Serial.println("LoRa Transmitter");

  // Configure the LED an an output
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(sense, INPUT);
  pinMode(pedal_PIN, INPUT);
  // Configura el canal PWM
  //ledcSetup(0, 15000, 12); // Canal 0, 15kHz de frecuencia, 8 bits de resolución
  //ledcAttachPin(pwmPin, 0); // Adjunta el pin al canal 0

  // Configure OLED by setting the OLED Reset HIGH, LOW, and then back HIGH
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, HIGH);
  delay(100);
  digitalWrite(OLED_RST, LOW);
  delay(100);
  digitalWrite(OLED_RST, HIGH);

  display.init();
  display.flipScreenVertically();

  showLogo();
  delay(1000);

  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(display.getWidth() / 2, display.getHeight() / 2, "LoRa Transmitter");
  display.display();
  delay(1000);

  // Configure the LoRA radio
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(LORA_BAND * 1E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("init ok");
  dutyCycle = 0;
}

void loop() {
  //------------VELOCIODAD----------------//
  int vel = 1000; //  rpm motor
  
  //------------CORRIENTE------------------//
  I=get_corriente(200);//obtenemos la corriente promedio de 200 muestras
  maxI = max(abs(I),maxI);

  //-----------PEDAL--------------------//
  dutyCycle = get_voltage(100);
  if (dutyCycle<=15) dutyCycle = 0;
  if (dutyCycle>=3670) dutyCycle = 3670;
  //ledcWrite(0, dutyCycle); // Canal 0, valor del ciclo de trabajo
  //dacWrite(25,dutyCycle);
  dac.setVoltage(dutyCycle, false);
  float voltaje = dutyCycle*3.30/4096.0;//256.0;

  String countStr = String(voltaje, DEC);
  String adc = String(dutyCycle, DEC);
  //Serial.print("Voltaje: ");
  //Serial.println(voltaje);
  //Serial.print("Corriente: ");
  //Serial.println(-I,3);
  
  //----------SEND LORA-----------------
  LoRa.beginPacket();
  //LoRa.print("CORRIENTE ");
  LoRa.print(I);
  LoRa.print(",");
  LoRa.print(maxI);
  LoRa.print(",");
  LoRa.print(voltaje);
  LoRa.print(",");
  LoRa.print(vel);
  LoRa.endPacket();

    
    
  String IStr = String(I, DEC);
  String max = String(maxI, DEC);
  String rpm = String(vel, DEC);
  //String km = String(speedkph, DEC);
  //Serial.println(IStr);
  displayLoraData(IStr, max);
  displayPWM(countStr, adc);

  display.drawString(0, 44, "MOTOR: ");
  display.drawString(70, 44, rpm);
  display.display();
  //displayGPS(lat, longi, km);
  // toggle the led to give a visual indication the packet was sent
  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

float get_corriente(int n_muestras)
{
  float voltajeSensor;
  float corriente=0;
  for(int i=0;i<n_muestras;i++)
  {
    voltajeSensor = analogRead(4) * (3.30 / 4095.0);////lectura del sensor
    corriente=corriente+(voltajeSensor - 1.4444 )/Sensibilidad; //Ecuación  para obtener la corriente esp32 1.5
  }
  corriente=corriente/n_muestras;
  return(corriente);
}

void displayLoraData(String countStr, String max) {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  
  display.drawString(0, 0, "Corriente: ");
  display.drawString(70, 0, countStr);
  display.drawString(0, 11, "Máx Corriente: ");
  display.drawString(70, 11, max);
  display.display();
}

void showLogo() {
  uint8_t x_off = (display.getWidth() - logo_width) / 2;
  uint8_t y_off = (display.getHeight() - logo_height) / 2;

  display.clear();
  display.drawXbm(x_off, y_off, logo_width, logo_height, logo_bits);
  display.display();
}

float get_voltage(int muestras){
  
  int dutyCycle = 0;
  for(int i=0;i<muestras;i++)
  {
    dutyCycle = dutyCycle + analogRead(pedal_PIN);
  }
  
  dutyCycle = dutyCycle/ muestras;
  dutyCycle = map(dutyCycle, 833, 2860, 0, 4095); //256 //4095
  return(dutyCycle);
}
void displayPWM(String countStr, String adc) {
  //display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  
  display.drawString(0, 22, "Señal pedal: ");
  display.drawString(70, 22, adc);
  display.drawString(0, 33, "VOL PWM: ");
  display.drawString(70, 33, countStr);
  display.display();
}