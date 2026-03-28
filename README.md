# BahceOtomasyonu
Robotik odevim için yaptığım proje
raporda ihtiyaç duyulan malzemeleri görebilirsiniz
# 🌿 Bahçe Otomasyonu Sistemi (Garden Automation System)

Bu proje, bitki bakımını ve tarım süreçlerini dijitalleştirmek, su tasarrufu sağlamak ve çevresel faktörleri uzaktan izlemek amacıyla geliştirilmiş **IoT (Nesnelerin İnterneti) tabanlı bir otomasyon sistemidir.** Donanım sensörlerinden alınan veriler (nem, sıcaklık vb.) anlık olarak işlenir ve kullanıcının arayüzüne aktarılır. Gerektiğinde sulama gibi fiziksel işlemler sistem tarafından otomatik olarak tetiklenir.

## 🚀 Özellikler
* **Gerçek Zamanlı İzleme:** Toprak nemi, ortam sıcaklığı ve hava nemi değerlerinin anlık takibi.
* **Akıllı Sulama Sistemi:** Toprak nemi belirli bir seviyenin altına düştüğünde su motorunun otomatik olarak çalıştırılması.
* **Uzaktan Kontrol:** [Web / Masaüstü / Mobil] arayüzü üzerinden manuel sistem kontrolü ve durum takibi.
* **Veri Loglama:** Geçmişe dönük sensör verilerinin veritabanında (SQL/Oracle) saklanması ve analiz edilmesi.

## 🛠️ Kullanılan Teknolojiler (Tech Stack)

### Donanım (Hardware)
* **Mikrodenetleyici:** [Örn: ESP32 / Arduino Uno]
* **Sensörler:** [Örn: DHT11 (Sıcaklık/Nem), Toprak Nemi Sensörü, Röle Modülü, Su Pompası]
## ⚙️ Kurulum ve Çalıştırma

1. **Depoyu Klonlayın:**
   ```bash
   git clone [https://github.com/GayesizCoder/BahceOtomasyonu.git](https://github.com/GayesizCoder/BahceOtomasyonu.git)
