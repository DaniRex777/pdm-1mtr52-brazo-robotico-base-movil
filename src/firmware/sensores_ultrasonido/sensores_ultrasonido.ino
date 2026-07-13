/*
 * Sensores de ultrasonido HC-SR04 (x4) — Detección de obstáculos de la
 * plataforma móvil. Proyecto 1MTR52 — Lab. Procesos Industriales, PUCP.
 *
 * Placa   : Arduino (conectado por USB-serial a la Raspberry Pi / PC)
 * Función : mide las 4 distancias y envía por Serial (115200 baudios) una
 *           trama por ciclo: <trasera,derecha,delantera,izquierda> en cm,
 *           con -1.0 si el sensor no detecta eco (fuera de rango 0-400 cm).
 *           La Raspberry Pi de la base la lee y la expone en /ultrasonido;
 *           server.py la usa para bloquear comandos hacia un obstáculo
 *           (_poll_ultrasonido / _obstacle_blocks_command).
 *
 * Conexiones (sensor i = 0..3):
 *   TRIG: pines 2, 4, 6, 8   |   ECHO: pines 3, 5, 7, 9
 *   VCC 5V, GND común. ECHO pasa por divisor de voltaje (ver informe 2.4.4).
 */

const int trigPins[4] = {2, 4, 6, 8};
const int echoPins[4] = {3, 5, 7, 9};

float distancia[4];

/*
 * setup()
 * Configura Serial a 115200 y los 4 pares TRIG (salida) / ECHO (entrada).
 */
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
  }
}

/*
 * medirDistancia()
 * Dispara un pulso de 10 us en TRIG y mide el ancho del pulso ECHO.
 * Entradas : trigPin (int), echoPin (int) — pines del sensor
 * Salida   : float — distancia en cm; -1.0 si no hubo eco en 30 ms (fuera de rango)
 * Ejemplo  : float d = medirDistancia(2, 3);
 */
float medirDistancia(int trigPin, int echoPin) {

  long duracion;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duracion = pulseIn(echoPin, HIGH, 30000);

  if (duracion == 0) {
    return -1.0;
  }

  return duracion * 0.0343 / 2.0;
}

/*
 * loop()
 * Mide los 4 sensores secuencialmente y publica una línea CSV por Serial.
 */
void loop() {

  for (int i = 0; i < 4; i++) {

    distancia[i] = medirDistancia(trigPins[i], echoPins[i]);

    if (distancia[i] < 0 || distancia[i] > 400) {
      distancia[i] = -1.0;
    }

    delay(50);
  }

  // Trama:
  // <trasera,derecha,delantera,izquierda>

  Serial.print("<");

  Serial.print(distancia[0], 1);
  Serial.print(",");

  Serial.print(distancia[1], 1);
  Serial.print(",");

  Serial.print(distancia[2], 1);
  Serial.print(",");

  Serial.print(distancia[3], 1);

  Serial.println(">");

  delay(100);
}