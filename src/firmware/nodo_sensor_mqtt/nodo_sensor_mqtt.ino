/*
 * Nodo Sensor MQTT — Evaporador al vacío
 * Lab Procesos Industriales — PUCP
 *
 * Hardware:
 * - ESP32-S3
 * - 4x sensores DS18B20 en bus OneWire (pin 4)
 * - Sensor de flujo YFB1 (pin 5)
 *
 * ─────────────────────────────────────────────────────────
 * CONFIGURACIÓN — solo editar esta sección antes de subir
 * ─────────────────────────────────────────────────────────
 */

#include <WiFi.h> // Movido aquí arriba para permitir definir las IPAddress estructuradas

// ── Red WiFi (router TP-Link del laboratorio) ──────────────
#define WIFI_SSID   "TP-Link_95E0"     // SSID del TP-Link
#define WIFI_PASS   "20375010"         // Contraseña del TP-Link

// ── IP Estática (Importado de CameraWebServer para asegurar conexión) ──
IPAddress local_IP(192, 168, 1, 15);   // IP fija para este ESP32
IPAddress gateway(192, 168, 1, 1);     // IP del router
IPAddress subnet(255, 255, 255, 0);    // Máscara de subred
IPAddress primaryDNS(8, 8, 8, 8);      // DNS Primario
IPAddress secondaryDNS(8, 8, 4, 4);    // DNS Secundario

// ── Broker MQTT (IP del PC donde corre server.py) ─────────
#define MQTT_SERVER "192.168.1.35"         // IP real del PC
#define MQTT_PORT   1883

// ── Pines ──────────────────────────────────────────────────
#define ONE_WIRE_PIN  4    // Bus DS18B20
#define FLOW_PIN      5    // Sensor de flujo YFB1 (GPIO5 — pin seguro del ESP32-S3)

// Debounce del sensor de flujo: se ignora cualquier pulso que llegue antes
// de este tiempo (en microsegundos) desde el anterior. Un pulso más rápido
// que esto no puede ser flujo real, es ruido. Sube el valor si aún ves
// lecturas infladas; bájalo si a caudal alto pierde pulsos.
#define FLOW_MIN_PULSE_US 500

// ── Intervalo de publicación (ms) ─────────────────────────
#define PUBLISH_INTERVAL_MS 2000 

/*
 * ─────────────────────────────────────────────────────────
 * Fin de configuración — no editar debajo de esta línea
 * ─────────────────────────────────────────────────────────
 */

#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Tópicos MQTT
const char* TOPIC_TEMP = "lab/evaporador/temperatura"; 
const char* TOPIC_FLOW = "lab/evaporador/flujo"; 

// Direcciones físicas de los sensores DS18B20
DeviceAddress sensor_1 = { 0x28, 0xE0, 0x94, 0x87, 0x00, 0x89, 0x7A, 0xDB }; 
DeviceAddress sensor_2 = { 0x28, 0x74, 0x0B, 0x88, 0x00, 0x00, 0x00, 0xEF }; 
DeviceAddress sensor_3 = { 0x28, 0x7D, 0xC6, 0x87, 0x00, 0x94, 0x3F, 0x2A }; 
DeviceAddress sensor_4 = { 0x28, 0x63, 0xA0, 0x87, 0x00, 0x0E, 0x42, 0xCE }; 
OneWire           oneWireBus(ONE_WIRE_PIN); 
DallasTemperature tempSensors(&oneWireBus); 

WiFiClient    espClient; 
PubSubClient  mqttClient(espClient); 
WiFiClient    tcpTestClient; 

// Flujo — variables del ISR
volatile unsigned long flowPulses      = 0; 
volatile unsigned long lastPulseMicros = 0; // marca de tiempo del último pulso válido (debounce)
unsigned long          lastFlowTime    = 0; 
float                  flowLmin        = 0.0; 

/*
 * flowISR()
 * ISR (rutina de interrupción) del sensor de flujo YF-B1.
 * Se ejecuta en cada flanco de bajada del pin FLOW_PIN.
 * Entradas : ninguna (hardware) | Salidas: ninguna (incrementa flowPulses, volatile unsigned long)
 */
void IRAM_ATTR flowISR() {
  // Debounce: descarta rebotes/ruido más rápidos que FLOW_MIN_PULSE_US.
  unsigned long now = micros();
  if (now - lastPulseMicros >= FLOW_MIN_PULSE_US) {
    flowPulses++; 
    lastPulseMicros = now;
  }
}

/*
 * connectWiFi()
 * Conecta el ESP32 a la red WiFi con IP estática. Bloqueante: si no
 * conecta en 30 s, reinicia el microcontrolador.
 * Entradas : ninguna (usa constantes WIFI_SSID, WIFI_PASS, local_IP)
 * Salidas  : ninguna (void). Efecto: WiFi.status() == WL_CONNECTED
 */
void connectWiFi() {
  Serial.printf("\nConectando a WiFi '%s'", WIFI_SSID); 
  WiFi.mode(WIFI_STA); 
  WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK); 
  
  // Aplicar IP estática antes de iniciar el WiFi
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) { 
    Serial.println("Error configurando IP estática"); 
  }

  WiFi.begin(WIFI_SSID, WIFI_PASS); 
  WiFi.setSleep(false); // Desactiva el modo de suspensión para evitar desconexiones en el S3 

  unsigned long start = millis(); 
  while (WiFi.status() != WL_CONNECTED) { 
    if (millis() - start > 30000) { 
      Serial.println("\nTimeout WiFi — reiniciando ESP32..."); 
      delay(1000); 
      ESP.restart(); 
    }
    delay(500); 
    Serial.print("."); 
  }

  Serial.println("\nWiFi conectado."); 
  Serial.printf("  IP ESP32 : %s\n", WiFi.localIP().toString().c_str()); 
  Serial.printf("  Gateway  : %s\n", WiFi.gatewayIP().toString().c_str()); 
  Serial.printf("  Broker   : %s:%d\n", MQTT_SERVER, MQTT_PORT); 
}

/*
 * testTCP()
 * Verifica que el broker MQTT sea alcanzable abriendo un socket TCP.
 * Entradas : ninguna (usa MQTT_SERVER:MQTT_PORT)
 * Salida   : bool — true si el broker responde, false en caso contrario
 * Ejemplo  : bool ok = testTCP();  // ok == true si hay conexión
 */
bool testTCP() {
  Serial.printf("Probando TCP a %s:%d ... ", MQTT_SERVER, MQTT_PORT); 
  if (tcpTestClient.connect(MQTT_SERVER, MQTT_PORT)) { 
    Serial.println("OK — broker alcanzable"); 
    tcpTestClient.stop(); 
    return true; 
  } else {
    Serial.println("FAIL — broker NO alcanzable"); 
    Serial.println("  Verifica:"); 
    Serial.println("  1. Que server.py esté corriendo en el PC"); 
    Serial.println("  2. Que Mosquitto esté activo"); 
    Serial.println("  3. Que la IP del MQTT_SERVER sea correcta"); 
    Serial.println("  4. Que el firewall de Windows no bloquee el puerto 1883"); 
    return false; 
  }
}

/*
 * reconnectMQTT()
 * Intenta conectar al broker MQTT hasta 5 veces (clientId único derivado
 * de la MAC). Si falla, reinicia el ESP32. Publica estado "online" al conectar.
 * Entradas : ninguna | Salidas: ninguna (void)
 */
void reconnectMQTT() {
  int attempts = 0; 
  while (!mqttClient.connected() && attempts < 5) { 
    attempts++; 
    String clientId = "ESP32-EvapNodo1-"; 
    clientId += String((uint32_t)ESP.getEfuseMac(), HEX); 

    Serial.printf("MQTT conectando (intento %d/5)... ", attempts); 

    if (mqttClient.connect(clientId.c_str())) { 
      Serial.println("conectado."); 
      mqttClient.publish("lab/evaporador/status", "{\"status\":\"online\",\"node\":\"nodo_1\"}"); 
    } else {
      int rc = mqttClient.state(); 
      Serial.printf("fallo (rc=%d)\n", rc); 
      delay(3000); 
    }
  }

  if (!mqttClient.connected()) { 
    Serial.println("No se pudo conectar a MQTT tras 5 intentos."); 
    Serial.println("Reiniciando ESP32 en 5 segundos..."); 
    delay(5000); 
    ESP.restart(); 
  }
}

/*
 * updateFlow()
 * Convierte los pulsos acumulados por flowISR() a caudal en L/min una vez
 * por segundo. Factor del YF-B1: 11 pulsos/s = 1 L/min (hoja de datos).
 * Entradas : ninguna (lee flowPulses con interrupciones deshabilitadas)
 * Salidas  : ninguna (actualiza la global flowLmin, float, L/min)
 */
void updateFlow() {
  unsigned long now = millis(); 
  if (now - lastFlowTime >= 1000) { 
    noInterrupts(); 
    unsigned long pulses = flowPulses; 
    flowPulses = 0; 
    interrupts(); 

    flowLmin = (float)pulses / 11.0; 
    lastFlowTime = now; 
  }
}

/*
 * publishSensors()
 * Lee las 4 temperaturas DS18B20, calcula el promedio de sensores válidos
 * (lectura > -100 °C descarta el código de error -127) y publica dos JSON:
 *   lab/evaporador/temperatura -> {"value":prom,"t1":..,"t2":..,"t3":..,"t4":..}  [°C, float]
 *   lab/evaporador/flujo       -> {"value":caudal}                                [L/min, float]
 * Entradas : ninguna | Salidas: ninguna (publica por MQTT e imprime por Serial)
 */
void publishSensors() {
  tempSensors.requestTemperatures(); 
  float t1 = tempSensors.getTempC(sensor_1); 
  float t2 = tempSensors.getTempC(sensor_2); 
  float t3 = tempSensors.getTempC(sensor_3); 
  float t4 = tempSensors.getTempC(sensor_4); 

  float validSum   = 0.0; 
  int   validCount = 0; 
  if (t1 > -100.0) { validSum += t1; validCount++; } 
  if (t2 > -100.0) { validSum += t2; validCount++; } 
  if (t3 > -100.0) { validSum += t3; validCount++; } 
  if (t4 > -100.0) { validSum += t4; validCount++; } 

  float avg = (validCount > 0) ? (validSum / validCount) : 0.0; 

  char jsonTemp[200]; 
  char t1s[10], t2s[10], t3s[10], t4s[10]; 

  if (t1 > -100.0) dtostrf(t1, 5, 2, t1s); else strcpy(t1s, "null"); 
  if (t2 > -100.0) dtostrf(t2, 5, 2, t2s); else strcpy(t2s, "null"); 
  if (t3 > -100.0) dtostrf(t3, 5, 2, t3s); else strcpy(t3s, "null"); 
  if (t4 > -100.0) dtostrf(t4, 5, 2, t4s); else strcpy(t4s, "null"); 

  snprintf(jsonTemp, sizeof(jsonTemp),
    "{\"value\":%.2f,\"t1\":%s,\"t2\":%s,\"t3\":%s,\"t4\":%s}",
    avg, t1s, t2s, t3s, t4s); 

  char jsonFlow[60]; 
  snprintf(jsonFlow, sizeof(jsonFlow), "{\"value\":%.3f}", flowLmin); 

  bool okTemp = mqttClient.publish(TOPIC_TEMP, jsonTemp); 
  bool okFlow = mqttClient.publish(TOPIC_FLOW, jsonFlow); 

  Serial.println("════════════════════════════════"); 
  Serial.println("  NODO SENSOR — Evaporador"); 
  Serial.println("════════════════════════════════"); 
  Serial.printf("  T1: %s °C\n", t1 > -100.0 ? String(t1, 2).c_str() : "ERROR"); 
  Serial.printf("  T2: %s °C\n", t2 > -100.0 ? String(t2, 2).c_str() : "ERROR"); 
  Serial.printf("  T3: %s °C\n", t3 > -100.0 ? String(t3, 2).c_str() : "ERROR"); 
  Serial.printf("  T4: %s °C\n", t4 > -100.0 ? String(t4, 2).c_str() : "ERROR"); 
  Serial.printf("  Prom: %.2f °C\n", avg); 
  Serial.printf("  Flujo: %.3f L/min\n", flowLmin); 
  Serial.printf("  MQTT temp: %s  flujo: %s\n", okTemp ? "OK" : "FAIL", okFlow ? "OK" : "FAIL"); 
  Serial.println("════════════════════════════════"); 
  Serial.println(); 
}

/*
 * setup()
 * Inicialización única: Serial (115200), sensores DS18B20 (resolución 10 bits),
 * interrupción del sensor de flujo, WiFi, prueba TCP y conexión MQTT.
 */
void setup() {
  Serial.begin(115200); 
  delay(1000); 

  Serial.println("\n\n════════════════════════════════"); 
  Serial.println("  ESP32 Nodo Sensor arrancando"); 
  Serial.println("════════════════════════════════"); 

  tempSensors.begin(); 
  int found = tempSensors.getDeviceCount(); 
  Serial.printf("Sensores DS18B20 encontrados: %d\n", found); 
  if (found < 4) { 
    Serial.println("ADVERTENCIA: se esperaban 4 sensores."); 
    Serial.println("Verifica el bus OneWire y la resistencia pull-up."); 
  }
  tempSensors.setResolution(sensor_1, 10); 
  tempSensors.setResolution(sensor_2, 10); 
  tempSensors.setResolution(sensor_3, 10); 
  tempSensors.setResolution(sensor_4, 10); 

  // INPUT: usa el pull-up EXTERNO a 3.3V (opción recomendada).
  // Si prefieres el pull-up interno del ESP32, cambia INPUT por INPUT_PULLUP.
  pinMode(FLOW_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowISR, FALLING);
  Serial.printf("Sensor de flujo inicializado en GPIO%d (debounce %d us).\n",
                FLOW_PIN, FLOW_MIN_PULSE_US);

  connectWiFi(); 
  testTCP(); 

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT); 
  mqttClient.setKeepAlive(30); 
  mqttClient.setSocketTimeout(10); 
  reconnectMQTT(); 

  lastFlowTime = millis(); 
  Serial.println("Sistema listo. Publicando datos...\n"); 
}

/*
 * loop()
 * Bucle principal: vigila WiFi/MQTT (reconecta si caen), atiende el cliente
 * MQTT, integra el caudal y publica lecturas cada PUBLISH_INTERVAL_MS (2 s).
 */
void loop() {
  if (WiFi.status() != WL_CONNECTED) { 
    Serial.println("WiFi perdido — reconectando..."); 
    connectWiFi(); 
    testTCP(); 
  }

  if (!mqttClient.connected()) { 
    Serial.println("MQTT desconectado — reconectando..."); 
    reconnectMQTT(); 
  }

  mqttClient.loop(); 
  updateFlow(); 

  static unsigned long lastPublish = 0; 
  if (millis() - lastPublish >= PUBLISH_INTERVAL_MS) { 
    lastPublish = millis(); 
    publishSensors(); 
  }
}