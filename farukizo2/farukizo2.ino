// ==========================================
// EBC APEX MOTORSPORT - IZOLASYON IZLEME V9 (CAN KALDIRILDI, UART/D9 CIKISI)
// ==========================================
// NOT: Gercek cip ATmega168 (1KB RAM). Bu yuzden TUM sabit metinler F() ile
// Flash'ta tutuluyor, String siniflari (heap) kullanilmiyor - davranis/degerler
// AYNI, sadece bellek yerlesimi degisti (RAM tasmasi/cokme onlemi).
//
// V9 DEGISIKLIGI (26.08.2026): MCP2515/CAN BUS TAMAMEN KALDIRILDI (hem AKS'ye
// gonderim hem BMS'ten alim) - kararsiz/aralikli reset sorunlarinin kok nedeni
// olarak CAN donanimi supheliydi. Yerine D9 pininde SoftwareSerial ile TX-ONLY
// (tek yonlu, sadece gonderim) bir UART cikisi geldi. Gonderilen veri AYNI
// (pozitif/negatif direnc kOhm + hata bayragi), sadece tasima katmani degisti.
// V_BAT artik BMS'ten canli okunmuyor (CAN gittigi icin bu yol da gitti),
// tekrar SABIT bir deger (85.0V, gercek arac nominaline yakin).
//
// V8 GUNCELLEMESI (25.08.2026, 34 noktali kapsamli saha taramasi): eski 9/12
// noktali kalibrasyon tablolarinin yerine, 8.28k...2.5M araligini kapsayan
// 34 noktali tablolar geldi (bkz. asagidaki TABLO_ARTI/EKSI tanimlari).
//
// V7 DEGISIKLIGI: Eski TEK/CIFT sabit-carpan kalibrasyonu kaldirilip yerine
// tablo+enterpolasyon tabanli kalibrasyon getirildi (bkz. tabloIleKalibreEt()).

#include <Wire.h>
#include <avr/pgmspace.h>
#include <Adafruit_ADS1X15.h>
#include <SoftwareSerial.h>

Adafruit_ADS1115 ads;

// --- UART ÇIKIŞI (D9, TX-ONLY) - AKS'ye izolasyon verisi gönderimi ---
// Eskiden CAN ID 0x180 ile gidiyordu, artik D9 uzerinden UART ile gidiyor.
// RX parametresi (D8) SoftwareSerial kütüphanesinin zorunlu tuttuğu ama hiç
// KULLANILMAYAN bir pin - biz sadece yaziyoruz (write/print), hic okumuyoruz.
const int UART_TX_PIN = 9;
const int UART_RX_PIN_KULLANILMIYOR = 8;
// TERS MANTIK - optokuplor cikisi sinyali TERSYUZ ediyor, verici de ters
// gondermek ZORUNDA ki iki inversiyon birbirini goturup AKS'de sinyal NORMAL olsun.
// izo_deneme_tx.ino'daki calisan kurulumun aynisi ("optokuplor cikisi tersyuz
// ettigi icin ZORUNLU").
//
// TELAFI NEDEN BURADA: AKS tarafi izolasyon hattini DONANIM UART'i ile
// (Serial2, RX2 = pin 17) okuyor. Donanim UART'inda ters mantik secenegi YOKTUR -
// o secenek yalnizca SoftwareSerial'de var. Dolayisiyla inversiyonun telafisi
// mecburen verici tarafinda, yani burada yapiliyor.
//
// BELIRTI (bu satir olmadan): AKS 0xAA basligini 0xAB/0x95 gibi bozuk aliyor,
// senkron hic kurulamiyor, [IZO-DURUM] satirinda cerceve=0 kaliyor.
const bool INVERSE_LOGIC = true;
SoftwareSerial aksUart(UART_RX_PIN_KULLANILMIYOR, UART_TX_PIN, INVERSE_LOGIC);

// UART_DEBUG: 1 -> seri porta (USB) her UART gonderiminde gonderilen veriyi
//             yazdirir. 0 -> bu satirlari derlemeye hic dahil etmez (flash tasarrufu).
#define UART_DEBUG 1

// --- UART ÇERÇEVE FORMATI (AKS tarafinin da bu formata gore guncellenmesi gerekir) ---
// [0]=0xAA [1]=0x55 (baslik/sync) [2]=pozitif_kOhm MSB [3]=pozitif_kOhm LSB
// [4]=negatif_kOhm MSB [5]=negatif_kOhm LSB [6]=hata_bayragi (0x00/0x01)
// [7]=checksum (bayt 2..6'nin XOR'u - AKS.ino'daki LoRa paketiyle ayni stil)
const uint8_t UART_BASLIK_1 = 0xAA;
const uint8_t UART_BASLIK_2 = 0x55;

// --- PIN TANIMLAMALARI ---
const int roleArtiPin = 4;
const int roleEksiPin = 5;
const int sirenPin = 6;

// Bir onceki dongudeki hata durumu - "yeni hata basladi" (guvenli->tehlike
// gecisi) anini yakalayip o an ekstra tekrar gonderim yapmak icin (bkz. loop()
// sonundaki UART cikisi).
bool oncekiIzolasyonHatasi = false;

// --- GERÇEK DONANIM DEĞERLERİ ---
// V_BAT ARTIK SABIT (CAN/BMS canli okuma kaldirildiktan sonra tekrar sabit
// deger - gercek arac nominal geriliminin yakini secildi).
// ONEMLI: Kalibrasyon tablolari (TABLO_ARTI/EKSI) 32.2V bench testinde
// cikarilmisti - gercek arac geriliminde (85V) tablolarin yeniden
// dogrulanmasi/kalibre edilmesi gerekir (bkz. onceki saha notlari).
const float V_BAT = 85.0;
const float R_ARTI_SERI = 1488.0;
const float R_EKSI_SERI = 1485.93;
const float R_BIAS_UST = 46.20;
const float R_BIAS_ALT = 46.24;
const float R_ORTA_ESDEGER = (R_BIAS_UST * R_BIAS_ALT) / (R_BIAS_UST + R_BIAS_ALT);

const float CARPAN_ARTI = (R_ARTI_SERI + R_ORTA_ESDEGER) / R_ORTA_ESDEGER;
const float CARPAN_EKSI = (R_EKSI_SERI + R_ORTA_ESDEGER) / R_ORTA_ESDEGER;

const float HATA_SINIRI_KOHM = 100.0; // 100 kOhm altı alarm eşiği

// "Olculemeyecek kadar yuksek direnc" ISARET (sentinel) degeri. Bunun ustundeki
// her sey sonsuz sayilir - ariza esigi 100 kOhm iken 10 MOhm zaten tamamen guvenli.
// AKS.ino ve dashboard.py de AYNI degeri kullanir; ucunun de esitlenmesi sart,
// aksi halde ayni olcum iki ekranda farkli gorunur.
const float SONSUZ_KOHM = 9999.0;

// --- SAHA KALİBRASYON TABLOLARI (PROGMEM - flash'ta, RAM harcamaz) ---
// Her çift: (ham_kOhm ölçüm, o hamme karşılık gelen GERÇEK direnç kOhm).
// ham_kOhm sütunu KESİNLİKLE artan sırada olmalı (enterpolasyon bunu varsayar).
// (+) kanal: batarya(+)-şase arası.
const float TABLO_ARTI_HAM[] PROGMEM = {
  23.55,   25.33,   27.27,   30.39,   33.51,   36.64,   42.29,   48.85,
  68.92,   77.98,   89.42,   97.92,  112.84,  115.06,  137.22,  158.68,
 214.96,  268.49,  315.39,  381.77,  442.29,  485.12,  546.58,  617.52,
 663.91,  691.50,  776.23,  842.61,  927.91, 1059.61, 1170.20, 1512.02,
2024.75, 2567.64
};
const float TABLO_ARTI_GERCEK[] PROGMEM = {
   8.28,    9.95,   12.01,   15.41,   18.67,   22.68,   27.37,   33.07,
  51.82,   60.0,    74.9,    82.7,    96.4,    98.9,   120.0,   140.0,
 197.2,   250.02,  295.2,   360.5,   420.6,   462.1,   520.5,   601.0,
 641.0,   666.0,   757.0,   820.0,   900.0,  1030.0,  1140.0,  1470.0,
2000.0,  2500.0
};
const int TABLO_ARTI_N = 34;

// (-) kanal: batarya(-)-şase arası. Gerçek direnç değerleri (+) kanalla
// AYNI 34 test noktası (aynı oturumda, aynı dirençlerle ölçüldü).
const float TABLO_EKSI_HAM[] PROGMEM = {
  26.73,   27.94,   30.35,   32.76,   37.59,   40.60,   45.06,   51.04,
  67.88,   75.68,   89.11,   95.64,  106.42,  108.81,  126.59,  142.19,
 184.51,  219.47,  247.46,  286.12,  319.11,  338.77,  365.46,  396.93,
 414.12,  421.77,  454.15,  473.74,  496.14,  529.24,  552.01,  624.91,
 691.00,  741.12
};
const float TABLO_EKSI_GERCEK[] PROGMEM = {
   8.28,    9.95,   12.01,   15.41,   18.67,   22.68,   27.37,   33.07,
  51.82,   60.0,    74.9,    82.7,    96.4,    98.9,   120.0,   140.0,
 197.2,   250.02,  295.2,   360.5,   420.6,   462.1,   520.5,   601.0,
 641.0,   666.0,   757.0,   820.0,   900.0,  1030.0,  1140.0,  1470.0,
2000.0,  2500.0
};
const int TABLO_EKSI_N = 34;

void setup() {
  Serial.begin(9600);
  pinMode(roleArtiPin, OUTPUT);
  pinMode(roleEksiPin, OUTPUT);
  pinMode(sirenPin, OUTPUT);

  digitalWrite(roleArtiPin, LOW);
  digitalWrite(roleEksiPin, LOW);
  digitalWrite(sirenPin, LOW);

  aksUart.begin(9600);

  Serial.println(F("========================================"));
  Serial.println(F(" EBC APEX MOTORSPORT - İZOLASYON HAZIR  "));
  Serial.println(F("========================================"));

  if (!ads.begin()) {
    Serial.println(F("HATA: ADS1115 Bulunamadi!"));
    while (1);
  }

  ads.setGain(GAIN_TWOTHIRDS);
  delay(1000);

  // --- [FAZ0-DEBUG] Baslangic ozet bilgisi (davranisi degistirmez, sadece log) ---
  Serial.println(F("--- FAZ0 DEBUG BILGISI ---"));
  Serial.print(F("V_BAT (kod, sabit): ")); Serial.println(V_BAT, 2);
  Serial.print(F("HATA_SINIRI_KOHM (kod, sabit): ")); Serial.println(HATA_SINIRI_KOHM, 2);
  Serial.print(F("Regulasyon tabani (100 ohm/V * V_BAT), birim kod ile AYNI olmali: "));
  Serial.println((100.0 * V_BAT) / 1000.0, 3);
  Serial.print(F("R_ARTI_SERI (kod): ")); Serial.println(R_ARTI_SERI, 2);
  Serial.print(F("R_EKSI_SERI (kod): ")); Serial.println(R_EKSI_SERI, 2);
  Serial.print(F("R_BIAS_UST (kod): ")); Serial.println(R_BIAS_UST, 2);
  Serial.print(F("R_BIAS_ALT (kod): ")); Serial.println(R_BIAS_ALT, 2);
  Serial.println(F(">> Bu 4 direnci multimetre ile olcup kOhm mertebesinde (~46 kOhm, ~1488 kOhm) oldugunu teyit edin."));
  Serial.println(F("--------------------------"));
  Serial.println(F("UART cikisi: D9 (TX-only), 9600 baud. AKS.ino'nun bu formati okuyacak sekilde guncellenmesi gerekiyor."));
}

void loop() {
  // --- ADIM 1: ARTI HAT ÖLÇÜMÜ ---
  float vBase1 = voltajOku();

  digitalWrite(roleEksiPin, LOW);
  digitalWrite(roleArtiPin, HIGH);
  delay(1200);

  float vOlcumArti = voltajOku();
  digitalWrite(roleArtiPin, LOW);

  delay(500);

  // --- ADIM 2: EKSİ HAT ÖLÇÜMÜ ---
  float vBase2 = voltajOku();

  digitalWrite(roleEksiPin, HIGH);
  delay(1200);

  float vOlcumEksi = voltajOku();
  digitalWrite(roleEksiPin, LOW);

  // --- [FAZ0-DEBUG] Ham voltaj okumalari (davranisi degistirmez) ---
  Serial.print(F("[FAZ0] vBase1=")); Serial.print(vBase1, 4);
  Serial.print(F(" vOlcumArti=")); Serial.print(vOlcumArti, 4);
  Serial.print(F(" vBase2=")); Serial.print(vBase2, 4);
  Serial.print(F(" vOlcumEksi=")); Serial.println(vOlcumEksi, 4);

  // --- CRAMER MATRİS MATEMATİĞİ ---
  float V_p1 = vBase1 + (vOlcumArti - vBase1) * CARPAN_ARTI;
  float V_n2 = vBase2 + (vOlcumEksi - vBase2) * CARPAN_EKSI;

  float b1 = (vBase1 - vOlcumArti) / R_ORTA_ESDEGER;
  float b2 = (vBase2 - vOlcumEksi) / R_ORTA_ESDEGER;

  float D = V_BAT * (V_BAT + V_n2 - V_p1);

  float G_p = 0.0;
  float G_n = 0.0;

  if (D > 1.0) {
    float D_Gp = (b1 * V_n2) - (b2 * (V_p1 - V_BAT));
    float D_Gn = (V_p1 * b2) - ((V_n2 + V_BAT) * b1);

    G_p = D_Gp / D;
    G_n = D_Gn / D;
  }

  // --- HAM DİRENÇ HESAPLAMA ---
  float R_arti_ham = 9999.0;
  float R_eksi_ham = 9999.0;

  if (G_p > 0.000001) R_arti_ham = (1.0 / G_p);
  if (G_n > 0.000001) R_eksi_ham = (1.0 / G_n);

  // --- [FAZ0-DEBUG] Kalibrasyon-oncesi HAM direnc (davranisi degistirmez) ---
  Serial.print(F("[FAZ0] D=")); Serial.print(D, 6);
  Serial.print(F(" R_arti_ham=")); Serial.print(R_arti_ham, 2);
  Serial.print(F(" R_eksi_ham=")); Serial.println(R_eksi_ham, 2);

  // --- TABLO TABANLI KALİBRASYON (saha testiyle ölçülmüş ham->gerçek egrisi) ---
  float R_arti_kOhm = tabloIleKalibreEt(R_arti_ham, TABLO_ARTI_HAM, TABLO_ARTI_GERCEK, TABLO_ARTI_N);
  float R_eksi_kOhm = tabloIleKalibreEt(R_eksi_ham, TABLO_EKSI_HAM, TABLO_EKSI_GERCEK, TABLO_EKSI_N);

  // --- ALARM VE YAZDIRMA ---
  bool izolasyonHatasiVar = (R_arti_kOhm < HATA_SINIRI_KOHM || R_eksi_kOhm < HATA_SINIRI_KOHM);

  Serial.println(F("----------------------------------------"));
  Serial.print(F("DURUM: "));

  if (izolasyonHatasiVar) {
    digitalWrite(sirenPin, HIGH);
    Serial.println(F("!!! TEHLİKE - İZOLASYON HATASI (< 100 kOhm) !!!"));
  } else {
    digitalWrite(sirenPin, LOW);
    Serial.println(F("GÜVENLİ"));
  }

  Serial.print(F("R_ARTI: ")); yazdirDirenc(R_arti_kOhm);
  Serial.print(F("R_EKSİ: ")); yazdirDirenc(R_eksi_kOhm);

  // --- UART ÇIKIŞI (D9, SoftwareSerial TX-only) - eskiden CAN 0x180 ile giden veri ---
  uint16_t pozitif_direnc_kohm = (uint16_t) constrain(R_arti_kOhm, 0.0, 65535.0);
  uint16_t negatif_direnc_kohm = (uint16_t) constrain(R_eksi_kOhm, 0.0, 65535.0);
  uint8_t  hata_durumu = izolasyonHatasiVar ? 0x01 : 0x00;

  uint8_t artiMSB = (pozitif_direnc_kohm >> 8) & 0xFF;
  uint8_t artiLSB = pozitif_direnc_kohm & 0xFF;
  uint8_t eksiMSB = (negatif_direnc_kohm >> 8) & 0xFF;
  uint8_t eksiLSB = negatif_direnc_kohm & 0xFF;
  uint8_t checksum = artiMSB ^ artiLSB ^ eksiMSB ^ eksiLSB ^ hata_durumu;

  gonderUartCercevesi(artiMSB, artiLSB, eksiMSB, eksiLSB, hata_durumu, checksum);

#if UART_DEBUG
  Serial.print(F("[UART-D9] ARTI=")); Serial.print(pozitif_direnc_kohm);
  Serial.print(F(" EKSI=")); Serial.print(negatif_direnc_kohm);
  Serial.print(F(" HATA=")); Serial.print(hata_durumu);
  Serial.print(F(" CHK=0x")); Serial.println(checksum, HEX);
#endif

  // YENI HATA BASLANGICI (guvenli->tehlike gecisi): AKS'nin bunu kacirma
  // ihtimalini azaltmak icin (UART'ta onay/tekrar-gonderim mekanizmasi yok)
  // ayni cerceveyi 2 kez daha, kisa aralikla tekrar gonder (toplam 3 kez).
  bool yeniHataBasladi = izolasyonHatasiVar && !oncekiIzolasyonHatasi;
  if (yeniHataBasladi) {
#if UART_DEBUG
    Serial.println(F("[UART-D9] YENI HATA BASLADI - cerceve 2 kez daha tekrarlaniyor."));
#endif
    delay(30);
    gonderUartCercevesi(artiMSB, artiLSB, eksiMSB, eksiLSB, hata_durumu, checksum);
    delay(30);
    gonderUartCercevesi(artiMSB, artiLSB, eksiMSB, eksiLSB, hata_durumu, checksum);
  }
  oncekiIzolasyonHatasi = izolasyonHatasiVar;

  delay(1500);
}

// --- UART CERCEVESI GONDER (D9, SoftwareSerial TX-only) ---
// Format: [0xAA][0x55][artiMSB][artiLSB][eksiMSB][eksiLSB][hata][checksum]
void gonderUartCercevesi(uint8_t artiMSB, uint8_t artiLSB, uint8_t eksiMSB, uint8_t eksiLSB, uint8_t hata, uint8_t checksum) {
  // ISINMA BAYTI (27.08.2026 saha bulgusu): SoftwareSerial, uzun bir bekleme
  // (delay(1500) vb.) sonrasi gonderilen ILK bayti zamanlama acisindan
  // guvenilmez gonderebiliyor (AKS tarafinda 0xAA -> 0x95 gibi tutarli bir
  // bozulma gozlendi, sonraki 7 bayt hep dogruydu). Gercek baslikdan (0xAA)
  // once onemsiz bir bayt gonderip bu ilk-bayt sorununu ona yiktiriyoruz -
  // AKS zaten 0xAA gelene kadar her seyi atiyor, bu fazladan bayt zararsiz.
  aksUart.write((uint8_t)0x00);
  aksUart.write(UART_BASLIK_1);
  aksUart.write(UART_BASLIK_2);
  aksUart.write(artiMSB);
  aksUart.write(artiLSB);
  aksUart.write(eksiMSB);
  aksUart.write(eksiLSB);
  aksUart.write(hata);
  aksUart.write(checksum);
}

// --- ADS1115 YÜKSEK HASSASİYET ---
float voltajOku() {
  long toplamAdc = 0;
  const int ornekSayisi = 20;

  ads.readADC_SingleEnded(0);
  ads.readADC_SingleEnded(0);
  ads.readADC_SingleEnded(0);

  for(int i = 0; i < ornekSayisi; i++) {
    toplamAdc += ads.readADC_SingleEnded(0);
  }

  float ortalamaAdc = (float)toplamAdc / ornekSayisi;
  return ads.computeVolts(ortalamaAdc);
}

// --- TABLO TABANLI KALİBRASYON: ham ölçümü (kOhm) gerçek dirence (kOhm) çevirir ---
// tablolar PROGMEM'de tutulur (RAM harcamaz), pgm_read_float ile okunur.
// ham_kOhm >= 9000 ise (kodun "sonsuz/çözümsüz" sentinel değeri) tabloya hiç
// sokmadan aynen geçirir - "OK (Sonsuz)" olarak basılmaya devam eder.
// Tablo aralığının dışına düşen ham değerler, en yakın iki noktanın eğimiyle
// dogrusal olarak dışa ekstrapole edilir.
float tabloIleKalibreEt(float ham_kOhm, const float* hamTablo, const float* gercekTablo, int n) {
  // NOT: buradaki 9000 esigi HAM olcume ait, asagidaki SONSUZ_KOHM kelepcesi ise
  // KALIBRE EDILMIS sonuca ait - ikisi ayri seyler, karistirilmamali.
  if (ham_kOhm >= 9000.0) return SONSUZ_KOHM;

  float sonuc;
  float ilkHam = pgm_read_float(&hamTablo[0]);
  float sonHam = pgm_read_float(&hamTablo[n - 1]);

  if (ham_kOhm <= ilkHam) {
    float h0 = ilkHam, h1 = pgm_read_float(&hamTablo[1]);
    float g0 = pgm_read_float(&gercekTablo[0]), g1 = pgm_read_float(&gercekTablo[1]);
    sonuc = g0 + (ham_kOhm - h0) * (g1 - g0) / (h1 - h0);
  } else if (ham_kOhm >= sonHam) {
    float h0 = pgm_read_float(&hamTablo[n - 2]), h1 = sonHam;
    float g0 = pgm_read_float(&gercekTablo[n - 2]), g1 = pgm_read_float(&gercekTablo[n - 1]);
    sonuc = g1 + (ham_kOhm - h1) * (g1 - g0) / (h1 - h0);
  } else {
    sonuc = ham_kOhm; // buraya düşülmemeli, güvenlik ağı - döngü bulamazsa ham değeri gecir
    for (int i = 0; i < n - 1; i++) {
      float h0 = pgm_read_float(&hamTablo[i]), h1 = pgm_read_float(&hamTablo[i + 1]);
      if (ham_kOhm >= h0 && ham_kOhm <= h1) {
        float g0 = pgm_read_float(&gercekTablo[i]), g1 = pgm_read_float(&gercekTablo[i + 1]);
        float oran = (ham_kOhm - h0) / (h1 - h0);
        sonuc = g0 + oran * (g1 - g0);
        break;
      }
    }
  }

  // Guvenlik kelepcesi: gercek direnc fiziksel olarak negatif olamaz -
  // tablo disina ekstrapolasyon asiri dusuk ham degerlerde negatife
  // savurabilir (bkz. 25.08.2026 saha testi, 9.8k -> -0.7 kOhm hatasi).
  if (sonuc < 0.0) sonuc = 0.0;

  // UST KELEPCE (01.09.2026): ekstrapolasyon tablo aralignin USTUNDE de sinirsizdi
  // ve iki kanalin egimi cok farkli oldugu icin sonuc tutarsizdi. Olculen ornek:
  //   EKSI kanali ham 7910 -> 74000 kOhm, sonra constrain() ile 65535'e kirpiliyordu
  //   ama ham 9000 -> 9999 (sentinel). Yani girdide 1 birimlik degisim ciktida
  //   6.5 kat sicrama yapiyordu; ekranda ikisi de "Sonsuz" gorundugu icin fark
  //   edilmiyor ama gonderilen/loglanan sayi bozuluyordu.
  // Sonucu sentinel'e kelepceleyince cikti monoton olur ve 65535 durumu tamamen
  // ortadan kalkar (asagidaki constrain(..., 65535.0) artik hic devreye girmez).
  if (sonuc > SONSUZ_KOHM) sonuc = SONSUZ_KOHM;
  return sonuc;
}

// Eski direncFormatla(float) -> String yerine: heap/String kullanmadan
// dogrudan Serial'e yazar (ATmega168'in 1KB RAM'inde String heap'i riskli).
// Basilan format AYNI: "OK (Sonsuz)" / "X.XX MOhm" / "X.X kOhm".
void yazdirDirenc(float kOhm) {
  // Esik SONSUZ_KOHM: AKS ve dashboard ile AYNI deger. Eskiden burada 9000 vardi,
  // yani 9000-9999 arasi bir olcum bu ekranda "Sonsuz", digerlerinde sayi olarak
  // gorunuyordu.
  if (kOhm >= SONSUZ_KOHM) {
    Serial.println(F("OK (Sonsuz)"));
  } else if (kOhm >= 1000.0) {
    Serial.print(kOhm / 1000.0, 2);
    Serial.println(F(" MOhm"));
  } else {
    Serial.print(kOhm, 1);
    Serial.println(F(" kOhm"));
  }
}
