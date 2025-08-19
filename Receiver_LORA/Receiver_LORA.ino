#include <SPI.h>
#include <Wire.h>
#include <SSD1306.h>
#include <LoRa.h>
#include "images.h"
//envio datos serial
String dataLa="-22.893495";
String dataLo="-43.187053";
String data1="0";
String data2="0";
String data3="0";
String data4="0";
String data5="0";
String data6="0";
String data7="0";
String data8="0";//pedal
//#define LORA_BAND    433
#define LORA_BAND    868
// #define LORA_BAND    915

#define OLED_SDA    21
#define OLED_SCL    22
#define OLED_RST    -1

#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     23//14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

SSD1306 display(0x3c, OLED_SDA, OLED_SCL);

// Forward declarations
void displayLoraData(int packetSize, String packet, String rssi);
void showLogo();

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println();
  Serial.println("LoRa Receiver");

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
  delay(2000);

  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(display.getWidth() / 2, display.getHeight() / 2, "LoRa Receiver");
  display.display();
  delay(2000);

  // Configure the LoRA radio
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(LORA_BAND * 1E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }


  // Set the radio into receive mode
  LoRa.receive();
  delay(1500);
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String packet = "";
    for (int i = 0; i < packetSize; i++) {
      packet += (char)LoRa.read();
    }
    String rssi = "RSSI " + String(LoRa.packetRssi(), DEC);

  int startIndex = 0;
  //int commaIndex = 0;
  String packet1;
  String packet2;
  String packet3;
  /*
  while (commaIndex >= 0) {
    commaIndex = packet.indexOf(',', startIndex);  // Encontrar la posición de la siguiente coma

    String word;
    if (commaIndex == -1) {
      packet2 = packet.substring(startIndex);  // Última palabra (sin coma al final)
    } else {
      packet1 = packet.substring(startIndex, commaIndex);  // Extraer la palabra entre comas
    }
    //Serial.println(word);  // Imprimir la palabra extraída

    startIndex = commaIndex + 1;  // Mover el índice de inicio al siguiente carácter después de la coma
  }
*/
/////////////

// Contador de tokens
  int tokenIndex = 0;

  // Buscar la primera coma
  int commaIndex = packet.indexOf(',');

  // Procesar cada parte
  while (commaIndex != -1) {
    // Extraer el token hasta la coma
    String token = packet.substring(0, commaIndex);

    // Eliminar espacios en blanco al inicio y fin del token
    token.trim();

    // Guardar el token en la variable correspondiente
    switch (tokenIndex) {
      case 0:
        packet1 = token;
        break;
      case 1:
        packet2 = token;
        break;
      case 2:
        packet3 = token;
        break;
    }

    // Incrementar el índice de token
    tokenIndex++;

    // Remover el token procesado de la cadena original
    packet = packet.substring(commaIndex + 1);

    // Buscar la siguiente coma
    commaIndex = packet.indexOf(',');
  }

  // Procesar el último token (después de la última coma)
  packet.trim();
  if (tokenIndex == 2) {
    packet3 = packet;
  } else if (tokenIndex == 1) {
    packet2 = packet;
  } else if (tokenIndex == 0) {
    packet1 = packet;
  }


/////////////////
  

    Serial.println(dataLa+" "+dataLo+" "+packet1+" "+data6+" "+data3+" "+data4+" "+packet2+" "+packet3+" "+data7+" "+packet3);

    displayLoraData(packet3, packet1, packet2);
  }
  delay(10);
}

void displayLoraData(String packet3, String packet1, String packet2) {
  //String packSize = String(packetSize, DEC);

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  //display.drawString(0 , 15 , "Received " + packSize + " bytes");
  display.drawString(0, 15, packet3);
  display.drawStringMaxWidth(0 , 26 , 128, packet2);
  display.drawString(0, 0, packet1);
  display.display();
}

void showLogo() {
  uint8_t x_off = (display.getWidth() - logo_width) / 2;
  uint8_t y_off = (display.getHeight() - logo_height) / 2;

  display.clear();
  display.drawXbm(x_off, y_off, logo_width, logo_height, logo_bits);
  display.display();
}
