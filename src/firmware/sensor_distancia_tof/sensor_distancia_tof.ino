/*
  ESP32-WROVER-IE / NodeMCU-32 dedicada al sensor TOF VL53L0X
  IP estatica: 192.168.1.15

  Conexion VL53L0X -> ESP32-WROVER:
    VIN  -> 3V3
    GND  -> GND
    SDA  -> GPIO21
    SCL  -> GPIO22

  Endpoints:
    http://192.168.1.15/          -> pagina con distancia
    http://192.168.1.15/distance  -> distancia filtrada y calibrada en mm
    http://192.168.1.15/raw       -> distancia cruda en mm
*/

#include <Wire.h>
#include <VL53L0X.h>
#include <WiFi.h>
#include <WebServer.h>

// ===================== WIFI =====================
#define WIFI_SSID "TP-Link_95E0"
#define WIFI_PASS "20375010"

IPAddress local_IP(192, 168, 1, 17);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);

// ===================== I2C =====================
#define SDA_PIN 21
#define SCL_PIN 22

// ===================== FRECUENCIA =====================
// 50 ms equivale aproximadamente a 20 Hz
#define SAMPLE_PERIOD_MS 50

// ===================== CALIBRACION =====================
// Modelo lineal:
// distancia_calibrada = CAL_A * distancia_sensor + CAL_B
//
// Deja estos valores por defecto al inicio.
// Luego ajustalos con mediciones reales usando una regla.
#define CAL_A 1.25f
#define CAL_B -120.0f

// Limites utiles de medicion
#define DIST_MIN_MM 0
#define DIST_MAX_MM 2000

// ===================== FILTRO =====================
// Mediana: elimina picos bruscos.
// Media movil: suaviza variaciones pequenas.
#define N_MEDIANA 5
#define N_MEDIA 5

VL53L0X tof;
WebServer server(80);

uint16_t rawDist = 0;
uint16_t lastDist = 0;

bool rawOk = false;
bool lastOk = false;
bool sensorOk = false;

uint16_t ventanaMediana[N_MEDIANA];
uint16_t ventanaMedia[N_MEDIA];

uint8_t idxMediana = 0;
uint8_t idxMedia = 0;

bool medianaLlena = false;
bool mediaLlena = false;

// ===================== CALIBRACION =====================
/*
 * calibrarDistancia()
 * Aplica el modelo de calibración lineal d' = CAL_A*d + CAL_B obtenido con
 * mediciones de referencia (regla), y satura al rango [DIST_MIN_MM, DIST_MAX_MM].
 * Entrada : d (uint16_t) — distancia cruda del sensor en mm
 * Salida  : uint16_t — distancia calibrada en mm
 * Ejemplo : uint16_t mm = calibrarDistancia(850);
 */
uint16_t calibrarDistancia(uint16_t d) {
  float corregida = CAL_A * (float)d + CAL_B;

  if (corregida < DIST_MIN_MM) corregida = DIST_MIN_MM;
  if (corregida > DIST_MAX_MM) corregida = DIST_MAX_MM;

  return (uint16_t)(corregida + 0.5f);
}

// ===================== FILTRO MEDIANA =====================
/*
 * filtroMediana()
 * Filtro de mediana con ventana deslizante de N_MEDIANA muestras.
 * Elimina picos espurios (outliers) de la medición TOF.
 * Entrada : nueva (uint16_t) — muestra en mm
 * Salida  : uint16_t — mediana de la ventana actual en mm
 */
uint16_t filtroMediana(uint16_t nueva) {
  ventanaMediana[idxMediana] = nueva;
  idxMediana++;

  if (idxMediana >= N_MEDIANA) {
    idxMediana = 0;
    medianaLlena = true;
  }

  uint8_t n = medianaLlena ? N_MEDIANA : idxMediana;

  if (n == 0) return nueva;

  uint16_t temp[N_MEDIANA];

  for (uint8_t i = 0; i < n; i++) {
    temp[i] = ventanaMediana[i];
  }

  for (uint8_t i = 0; i < n - 1; i++) {
    for (uint8_t j = i + 1; j < n; j++) {
      if (temp[j] < temp[i]) {
        uint16_t aux = temp[i];
        temp[i] = temp[j];
        temp[j] = aux;
      }
    }
  }

  return temp[n / 2];
}

// ===================== FILTRO MEDIA MOVIL =====================
/*
 * filtroMediaMovil()
 * Media móvil de N_MEDIA muestras; suaviza el ruido de alta frecuencia.
 * Entrada : nueva (uint16_t) — muestra en mm (ya pasada por la mediana)
 * Salida  : uint16_t — promedio de la ventana actual en mm
 */
uint16_t filtroMediaMovil(uint16_t nueva) {
  ventanaMedia[idxMedia] = nueva;
  idxMedia++;

  if (idxMedia >= N_MEDIA) {
    idxMedia = 0;
    mediaLlena = true;
  }

  uint8_t n = mediaLlena ? N_MEDIA : idxMedia;

  if (n == 0) return nueva;

  uint32_t suma = 0;

  for (uint8_t i = 0; i < n; i++) {
    suma += ventanaMedia[i];
  }

  return (uint16_t)(suma / n);
}

// ===================== FILTRO COMPLETO =====================
/*
 * procesarDistancia()
 * Cadena completa de procesamiento: calibración -> mediana -> media móvil.
 * Entrada : dRaw (uint16_t) — lectura cruda del VL53L0X en mm
 * Salida  : uint16_t — distancia final filtrada y calibrada en mm
 */
uint16_t procesarDistancia(uint16_t dRaw) {
  uint16_t dCalibrada = calibrarDistancia(dRaw);
  uint16_t dMediana = filtroMediana(dCalibrada);
  uint16_t dFiltrada = filtroMediaMovil(dMediana);

  return dFiltrada;
}

// ===================== HTTP =====================
/*
 * handleRoot()
 * Handler HTTP GET / — página HTML simple con la distancia en vivo.
 * Entradas: ninguna | Salida: ninguna (responde por WebServer, text/html)
 */
void handleRoot() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta charset='utf-8'>";
  html += "<meta http-equiv='refresh' content='0.2'>";
  html += "<title>TOF VL53L0X</title>";
  html += "<style>";
  html += "body{font-family:sans-serif;text-align:center;margin-top:50px;background:#111;color:#eee;}";
  html += "h1{font-size:72px;margin:10px;color:#0f0;}";
  html += "h2{font-size:28px;margin:10px;color:#aaa;}";
  html += "small{color:#888;}";
  html += ".box{display:inline-block;border:1px solid #333;border-radius:10px;padding:20px;min-width:360px;}";
  html += "</style>";
  html += "</head><body>";
  html += "<div class='box'>";
  html += "<small>Distancia VL53L0X filtrada y calibrada</small>";
  html += "<h1>";

  if (!sensorOk) {
    html += "SIN SENSOR";
  } else if (lastOk) {
    html += String(lastDist);
    html += " mm";
  } else {
    html += "---";
  }

  html += "</h1>";
  html += "<h2>Raw: ";

  if (rawOk) {
    html += String(rawDist);
    html += " mm";
  } else {
    html += "---";
  }

  html += "</h2>";
  html += "<small>IP: 192.168.1.15 &middot; /distance &middot; /raw</small>";
  html += "</div>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

/*
 * handleDistance()
 * Handler HTTP GET /distance — usado por server.py para leer el nivel.
 * Salida: texto plano con la distancia filtrada en mm, o "-1" si hay error.
 */
void handleDistance() {
  server.sendHeader("Access-Control-Allow-Origin", "*");

  if (!sensorOk) {
    server.send(200, "text/plain", "NO_SENSOR");
  } else if (lastOk) {
    server.send(200, "text/plain", String(lastDist));
  } else {
    server.send(200, "text/plain", "ERR");
  }
}

/*
 * handleRaw()
 * Handler HTTP GET /raw — distancia cruda sin filtrar (para depuración
 * y para levantar la curva de calibración CAL_A / CAL_B).
 * Salida: texto plano con la distancia cruda en mm, o "-1" si hay error.
 */
void handleRaw() {
  server.sendHeader("Access-Control-Allow-Origin", "*");

  if (!sensorOk) {
    server.send(200, "text/plain", "NO_SENSOR");
  } else if (rawOk) {
    server.send(200, "text/plain", String(rawDist));
  } else {
    server.send(200, "text/plain", "ERR");
  }
}

// ===================== WIFI =====================
/*
 * connectWiFi()
 * Conexión WiFi con IP estática 192.168.1.17. Bloqueante con timeout de
 * 30 s; si expira, reinicia el ESP32.
 * Entradas: ninguna | Salidas: ninguna (void)
 */
void connectWiFi() {
  Serial.println();
  Serial.print("WiFi connecting");

  WiFi.mode(WIFI_STA);

  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("Error configurando IP estatica");
  }

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setSleep(false);

  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");

    if (millis() - start > 30000) {
      Serial.println();
      Serial.println("Timeout WiFi. Reinicio.");
      ESP.restart();
    }
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.print("RSSI: ");
  Serial.println(WiFi.RSSI());
}

// ===================== SETUP =====================
/*
 * setup()
 * Inicializa Serial, bus I2C (SDA=21, SCL=22), sensor VL53L0X en modo
 * continuo, WiFi y servidor HTTP con sus 3 endpoints.
 */
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("--- ESP32-WROVER TOF VL53L0X 20 Hz filtrado/calibrado ---");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  tof.setTimeout(100);

  if (tof.init()) {
    sensorOk = true;

    // Timing budget de 33 ms. Permite lectura cercana a 20 Hz.
    tof.setMeasurementTimingBudget(33000);

    // Periodo continuo de 50 ms.
    tof.startContinuous(SAMPLE_PERIOD_MS);

    Serial.println("VL53L0X OK");
  } else {
    sensorOk = false;
    Serial.println("ERROR: VL53L0X no detectado. Revisa SDA, SCL, 3V3 y GND.");
  }

  connectWiFi();

  server.on("/", handleRoot);
  server.on("/distance", handleDistance);
  server.on("/raw", handleRaw);
  server.begin();

  Serial.println("Servidor HTTP iniciado");
  Serial.print("Distancia filtrada: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/distance");
  Serial.print("Distancia cruda: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/raw");
}

// ===================== LOOP =====================
/*
 * loop()
 * Cada SAMPLE_PERIOD_MS (50 ms ≈ 20 Hz): lee el TOF, procesa la muestra
 * y atiende las peticiones HTTP pendientes.
 */
void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi perdido. Reinicio.");
    delay(500);
    ESP.restart();
  }

  server.handleClient();

  static uint32_t tPrev = 0;

  if (sensorOk && millis() - tPrev >= SAMPLE_PERIOD_MS) {
    tPrev = millis();

    uint16_t d = tof.readRangeContinuousMillimeters();

    if (tof.timeoutOccurred() || d >= 8000) {
      rawOk = false;
      lastOk = false;
    } else {
      rawDist = d;
      rawOk = true;

      lastDist = procesarDistancia(d);
      lastOk = true;
    }
  }

  static uint32_t tLog = 0;

  if (millis() - tLog >= 2000) {
    tLog = millis();

    Serial.print("RAW: ");

    if (rawOk) {
      Serial.print(rawDist);
      Serial.print(" mm");
    } else {
      Serial.print("ERR");
    }

    Serial.print(" | FILTRADA: ");

    if (lastOk) {
      Serial.print(lastDist);
      Serial.print(" mm");
    } else {
      Serial.print("ERR");
    }

    Serial.print(" | RSSI: ");
    Serial.println(WiFi.RSSI());
  }
}