#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <IRremote.hpp> // v4.x.x kütüphanesi kullanılmıştır

// --- Pin Tanımlamaları ---
#define LED_PIN 2       // Dahili Mavi LED
#define DHTPIN 4        // DHT11 Data
#define LDR_PIN 32      // LDR Analog
#define FIRE_PIN 33     // Yangın Dijital
#define SOIL_PIN 34     // Toprak Nemi Analog
#define IR_PIN 35       // IR Alıcı Sinyal

DHT dht(DHTPIN, DHT11);
WiFiClient espClient;
PubSubClient client(espClient);

// --- Değişkenler ---
int sendInterval = 5000; // Başlangıçta 5 saniye
bool sistemAcik = true;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(FIRE_PIN, INPUT);
  
  dht.begin();
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK); // Kumandayı başlat

  setup_wifi();
  client.setServer("broker.hivemq.com", 1883);
  client.setCallback(callback);
}

// Web'den gelen komutları dinle
void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];

  if (String(topic) == "Gaye/kontrol/guc") {
    sistemAcik = (msg == "1");
    client.publish("Gaye/ev/durum", sistemAcik ? "1" : "0");
    Serial.println(sistemAcik ? "Sistem Acildi (Web)" : "Sistem Kapandi (Web)");
  }
}

void setup_wifi() {
  delay(10);
  Serial.print("WiFi Baglaniyor: Gaye");
  WiFi.begin("Gaye", "GayeMakbule");

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Bağlanana kadar yanıp söner
    delay(250);
    Serial.print(".");
  }
  Serial.println("\nWiFi Baglandi!");
  digitalWrite(LED_PIN, HIGH); // Bağlanınca sabit mavi yanar
}

unsigned long sonTusZamani = 0; // Tuş kilidi için değişken

void handleIR() {
  if (IrReceiver.decode()) {
    uint16_t cmd = IrReceiver.decodedIRData.command;
    
    // Sinyal karmaşasını ve hızlı tekrarları önlemek için 500ms kilit
    if (millis() - sonTusZamani > 500) { 
      Serial.print("Basilan Tus: 0x"); Serial.println(cmd, HEX);

      if (cmd == 0x1C) { // Sadece OK tuşu (Sistemi açar/kapatır)
        sistemAcik = !sistemAcik;
        client.publish("Gaye/ev/durum", sistemAcik ? "1" : "0");
        Serial.println(sistemAcik ? "Sistem: CALISIYOR" : "Sistem: DURDURULDU");
        sonTusZamani = millis();
      }
      else if (cmd == 0x45) { // 1 tuşu
        sendInterval = 1000; // 1 saniye
        Serial.println("Hiz: 1s (1 tusu)");
        sonTusZamani = millis();
      }
      else if (cmd == 0x46) { // 2 tuşu
        sendInterval = 5000; // 5 saniye
        Serial.println("Hiz: 5s (2 tusu)");
        sonTusZamani = millis();
      }
      else if (cmd == 0x47) { // 3 tuşu
        sendInterval = 10000; // 10 saniye
        Serial.println("Hiz: 10s (3 tusu)");
        sonTusZamani = millis();
      }
    }

    IrReceiver.resume(); // Yeni sinyal alabilmek için sıfırla
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT Baglantisi deneniyor...");
    String clientId = "GayeStation-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("Baglandi!");
      digitalWrite(LED_PIN, HIGH); // Bağlantı başarılıysa mavi ışığı yak
      client.subscribe("Gaye/kontrol/#");
      client.publish("Gaye/ev/durum", "1");
    } else {
      digitalWrite(LED_PIN, LOW); // Bağlantı yoksa söndür
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();
  handleIR();

  static unsigned long lastMsg = 0;
  if (sistemAcik && (millis() - lastMsg > sendInterval)) {
    lastMsg = millis();
    
    // Sensör Verilerini Oku
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    int soilRaw = analogRead(SOIL_PIN);
    int soil = map(soilRaw, 4095, 1200, 0, 100);
    int lightRaw = analogRead(LDR_PIN);
    int light = map(lightRaw, 0, 4095, 0, 100);
    bool fire = (digitalRead(FIRE_PIN) == LOW);

    if (!isnan(t) && !isnan(h)) {
      // --- Serial Port Yazdırma Kısmı ---
      Serial.println("\n--- Yeni Veri Paketi ---");
      Serial.print("Sıcaklık: "); Serial.print(t); Serial.println(" °C");
      Serial.print("Hava Nemi: %"); Serial.println(h);
      Serial.print("Toprak Nemi: %"); Serial.print(constrain(soil, 0, 100)); Serial.print(" (Ham: "); Serial.print(soilRaw); Serial.println(")");
      Serial.print("Işık Şiddeti: %"); Serial.print(constrain(light, 0, 100)); Serial.print(" (Ham: "); Serial.print(lightRaw); Serial.println(")");
      Serial.print("Yangın Durumu: "); Serial.println(fire ? "TEHLIKE VAR!" : "Güvenli");
      Serial.println("------------------------");

      // Mavi LED Göz Kırpma (Veri gönderiliyor efekti)
      digitalWrite(LED_PIN, LOW);
      
      client.publish("Gaye/ev/bitki/sicaklik", String(t, 1).c_str());
      client.publish("Gaye/ev/bitki/havanemi", String(h, 1).c_str());
      client.publish("Gaye/ev/bitki/topraknemi", String(constrain(soil, 0, 100)).c_str());
      client.publish("Gaye/ev/bitki/isik", String(constrain(light, 0, 100)).c_str());
      client.publish("Gaye/ev/bitki/yangin", fire ? "1" : "0");

      // Yapay Zeka Önerisi (Serial'a da ekleyelim)
      if (light > 75 && t > 30) {
          client.publish("Gaye/ev/bitki/oneri", "Hava cok sicak ve aydinlik, bitkiyi sulayabilirsin!");
          Serial.println("Öneri: Sulama gerekebilir.");
      } else {
          client.publish("Gaye/ev/bitki/oneri", "Her sey normal gorunuyor.");
          Serial.println("Öneri: Her şey normal.");
      }

      delay(100); 
      digitalWrite(LED_PIN, HIGH);
    } else {
      Serial.println("Hata: DHT sensörü okunamıyor!");
    }
  }
}
