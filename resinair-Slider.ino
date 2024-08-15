#include <BLEDevice.h>            // BLE aygıt işlemleri için gerekli kütüphane
#include <BLEServer.h>            // BLE sunucu işlemleri için gerekli kütüphane
#include <BLEUtils.h>             // BLE yardımcı işlemleri için gerekli kütüphane
#include <BLE2902.h>              // BLE2902 karakteristiği için gerekli kütüphane
#include <BLEAdvertisedDevice.h>  // BLE reklam cihazları için gerekli kütüphane

#include <AccelStepper.h>         // Step motor kontrolü için kullanılan kütüphane

// Step motor pin tanımları ve motor yapılandırmaları
#define dirPin 2                  // Lineer eksen step motoru yön pini
#define stepPin 15                // Lineer eksen step motoru adım pini
#define motorInterfaceType 1      // Motor arayüz tipi
AccelStepper stepper_lin = AccelStepper(motorInterfaceType, stepPin, dirPin);  // Lineer eksen motoru tanımı

#define dirPin2 17                // Açısal eksen step motoru yön pini
#define stepPin2 5                // Açısal eksen step motoru adım pini
AccelStepper stepper_ang = AccelStepper(motorInterfaceType, stepPin2, dirPin2);// Açısal eksen motoru tanımı

// Ayarlar ///////////////////////////////////////////////////////

unsigned int max_stepper_speed = 5000;          // Step motorun maksimum hızı
float n_point = 100;                            // Hareket süresince hesaplanacak ara noktaların sayısı
float step_per_mm = 100.576;                    // Milimetre başına düşen step sayısı
float step_per_rad = 10 * 16 * 14.4 * 1.1;      // Radyan başına düşen step sayısı
bool back_at_end = 0;                           // Hareket sonunda geri dönme ayarı

//////////////////////////////////////////////////////////////////////

float step_per_deg =  step_per_rad / 57.296;    // Derece başına düşen step sayısı

// Task handle'ları (FreeRTOS görevleri için tanımlamalar)
//TaskHandle_t Task1;
TaskHandle_t Task2;                             // Task2 için handle tanımı

BLEServer *pServer = NULL;                      // BLE sunucusu için tanımlama
BLECharacteristic *pCharacteristic;             // BLE karakteristiği için tanımlama

bool deviceConnected = false;                   // Cihazın bağlı olup olmadığını tutan bayrak
bool oldDeviceConnected = false;                // Önceki cihaz bağlantı durumu
float txValue = 0;                              // İletilecek değer
bool volatile started  = 0;                     // Hareketin başlatılıp başlatılmadığını gösteren bayrak
bool volatile paused   = 0;                     // Hareketin duraklatılıp duraklatılmadığını gösteren bayrak
bool volatile man_lin  = 0;                     // Manuel lineer hareketin aktif olup olmadığını gösteren bayrak
bool volatile man_rot  = 0;                     // Manuel açısal hareketin aktif olup olmadığını gösteren bayrak

float curr_x = 0.0000;                          // Şu anki pozisyonun x değeri
int percentage;                                 // Yüzde ilerleme durumu

// Gelen verilerin saklanacağı string değişkenler
String Leng;
String X_obj;
String Y_obj;
String Tf;

String man_lin_dist;                            // Manuel lineer hareket için mesafe
String man_lin_vel;                             // Manuel lineer hareket için hız
String man_rot_deg;                             // Manuel açısal hareket için açı
String man_rot_vel;                             // Manuel açısal hareket için hız

// Gelen verilerin tam sayı olarak saklanacağı değişkenler
int man_lin_dist_int;
int man_lin_vel_int;
int man_rot_deg_int;
int man_rot_vel_int;

int Leng_int;
int X_obj_int;
int Y_obj_int;
unsigned int Tf_int;

std::string rxValue;                            // Gelen BLE verilerini saklayan değişken

// BLE UUID tanımları
#define SERVICE_UUID           "fc22b10e-a3ad-11ec-b909-0242ac120002" // UART servis UUID'si
#define CHARACTERISTIC_UUID_RX "e4a8b0a0-a3cb-11ec-b909-0242ac120002" // RX karakteristiği UUID'si
#define CHARACTERISTIC_UUID_TX "fc22b10e-a3ad-11ec-b909-0242ac120002" // TX karakteristiği UUID'si

// BLE sunucusu bağlantı durumu değişikliklerinde çalışacak callback sınıfı
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;   // Bağlantı kurulduğunda bayrağı true yap
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;  // Bağlantı koptuğunda bayrağı false yap
    }
};

// BLE karakteristiği yazıldığında çalışacak callback sınıfı
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      rxValue = pCharacteristic->getValue(); // Gelen veriyi al

      if (rxValue.length() > 0) { // Gelen veri varsa
        for (int h = 0; h<rxValue.length();h++){
          Serial.print(rxValue[h]);  // Gelen veriyi seri porttan yazdır
        }
        
        if (rxValue[0] == 'S') {  // Gelen veri 'S' ile başlıyorsa
          Leng = "";
          X_obj = "";
          Y_obj = "";
          Tf = "";
          int i = 2;

          // Kaydırma uzunluğunu al
          while (rxValue[i] != '*') {
            Leng = Leng + rxValue[i];
            i++;
          }
          Leng_int = Leng.toInt();
          i++;

          // X mesafesini al
          while (rxValue[i] != '*') {
            X_obj = X_obj + rxValue[i];
            i++;
          }
          X_obj_int = X_obj.toInt();
          i++;

          // Y mesafesini al
          while (rxValue[i] != '*') {
            Y_obj = Y_obj + rxValue[i];
            i++;
          }
          Y_obj_int = Y_obj.toInt();
          i++;

          // Toplam zamanı al
          while (rxValue[i] != '*') {
            Tf = Tf + rxValue[i];
            i++;
          }
          Tf_int = Tf.toInt();

          // Hareketi başlat
          started = 1;

          i++;

          // Hareket sonunda geri dönme ayarını al
          if (rxValue[i] == 'f') { 
            back_at_end = 0;
          } else {
            back_at_end = 1;
          }

        } else if (rxValue[0] == 'L') {  // Gelen veri 'L' ile başlıyorsa
          man_lin_dist = "";
          man_lin_vel = "";
          int i = 2;

          // Manuel lineer hareket mesafesini al
          while (rxValue[i] != '*') {
            man_lin_dist = man_lin_dist + rxValue[i];
            i++;
          }
          man_lin_dist_int = man_lin_dist.toInt();
          i++;

          // Manuel lineer hareket hızını al
          while (rxValue[i] != '*') {
            man_lin_vel = man_lin_vel + rxValue[i];
            i++;
          }
          man_lin_vel_int = man_lin_vel.toInt();

          if (rxValue[1] == 'm') {  // Geriye doğru hareket durumu
            man_lin_dist_int = -man_lin_dist_int;
          }

          // Manuel lineer hareketi başlat
          man_lin = 1;

        } else if (rxValue[0] == 'R') {  // Gelen veri 'R' ile başlıyorsa
          man_rot_deg = "";
          man_rot_vel = "";
          int i = 2;

          // Manuel açısal hareket açısını al
          while (rxValue[i] != '*') {
            man_rot_deg = man_rot_deg + rxValue[i];
            i++;
          }
          man_rot_deg_int = man_rot_deg.toInt();
          i++;

          // Manuel açısal hareket hızını al
          while (rxValue[i] != '*') {
            man_rot_vel = man_rot_vel + rxValue[i];
            i++;
          }
          man_rot_vel_int = man_rot_vel.toInt();
          if (rxValue[1] == 'm') {  // Ters yönde dönme durumu
            man_rot_deg_int = -man_rot_deg_int;
          }

          // Manuel açısal hareketi başlat
          man_rot = 1;

        } else if (rxValue[0] == 'T') {  // Gelen veri 'T' ile başlıyorsa
          // Hareketi durdur
          started = 0;
          paused  = 0;
        } else if (rxValue[0] == 'P') {  // Gelen veri 'P' ile başlıyorsa
          // Hareketi duraklat veya devam ettir
          paused = !paused;
        }

      }
    }
};

// Setup fonksiyonu, cihaz ilk başladığında bir kez çalışır
void setup() {
  Serial.begin(115200);   // Seri haberleşmeyi başlat
  
  stepper_lin.setMaxSpeed(max_stepper_speed);  // Lineer motorun maksimum hızını ayarla
  stepper_ang.setMaxSpeed(max_stepper_speed);  // Açısal motorun maksimum hızını ayarla

  // BLE cihazını başlat ve ayarla
  BLEDevice::init("ESP32_BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // BLE servis ve karakteristiklerini oluştur
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
                      
  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Servisi başlat ve BLE reklamlarını başlat
  pService->start();
  pServer->getAdvertising()->start();
}

// Loop fonksiyonu, sonsuz döngüde çalışır ve sürekli tekrar eder
void loop() {
  if (deviceConnected) {
    if (started) {
      if (!paused) {
        float dx = (X_obj_int - curr_x) / n_point;
        float dy = Y_obj_int / n_point;

        for (int i = 0; i <= n_point; i++) {
          stepper_lin.moveTo(curr_x + i * dx);
          stepper_ang.moveTo(i * dy);
          stepper_lin.run();
          stepper_ang.run();
        }

        // Hareket tamamlandığında geri döndürme işlemi
        if (back_at_end) {
          for (int i = 0; i <= n_point; i++) {
            stepper_lin.moveTo(X_obj_int - i * dx);
            stepper_ang.moveTo(Y_obj_int - i * dy);
            stepper_lin.run();
            stepper_ang.run();
          }
        }

        started = 0;
      }
    }

    if (man_lin) {
      stepper_lin.setSpeed(man_lin_vel_int);
      stepper_lin.move(man_lin_dist_int);
      stepper_lin.runToPosition();
      man_lin = 0;
    }

    if (man_rot) {
      stepper_ang.setSpeed(man_rot_vel_int);
      stepper_ang.move(man_rot_deg_int);
      stepper_ang.runToPosition();
      man_rot = 0;
    }

    if (!deviceConnected && oldDeviceConnected) {
      delay(500);
      pServer->startAdvertising();
      oldDeviceConnected = deviceConnected;
    }

    if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
    }
  }
}
