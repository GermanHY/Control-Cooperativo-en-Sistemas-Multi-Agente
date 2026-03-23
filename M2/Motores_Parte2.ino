#include <Arduino.h>
#include <ESP32Encoder.h>
#include "driver/ledc.h"

// =========================
// Pines motores
// =========================
#define ain1 18
#define ain2 19
#define pwm_a 23   // motor derecho

#define bin1 17
#define bin2 16
#define pwm_b 4    // motor izquierdo

// =========================
// Pines encoders
// =========================
#define enc1_pinA 34   // encoder derecho A
#define enc1_pinB 35   // encoder derecho B
#define enc2_pinA 36   // encoder izquierdo A
#define enc2_pinB 39   // encoder izquierdo B
int pwm_test = 27;
unsigned long lastSubida = 0;
// =========================
// Configuración PWM ESP32
// =========================
static const int freq = 5000;
static const ledc_mode_t speed_mode = LEDC_HIGH_SPEED_MODE;
static const ledc_timer_t timer_num = LEDC_TIMER_0;
static const ledc_timer_bit_t duty_resolution = LEDC_TIMER_8_BIT;

// Canal 0 -> motor derecho
// Canal 1 -> motor izquierdo
static const ledc_channel_t ch_der = LEDC_CHANNEL_0;
static const ledc_channel_t ch_izq = LEDC_CHANNEL_1;

// =========================
// Objetos globales
// =========================
ESP32Encoder encoder_der;
ESP32Encoder encoder_izq;

// =========================
// Variables globales
// =========================
volatile int pwm_cmd_L = 0;
volatile int pwm_cmd_R = 0;

long prevCountR = 0;
long prevCountL = 0;
unsigned long prevPrint = 0;

// =========================
// Prototipos
// =========================
void motor(int Velocidad_motor_izq, int Velocidad_motor_der);
void setupPWM();
void setupMotores();
void setupEncoders();

// =========================
// Función motor
// motor(izq, der)
// rango esperado: -255 a 255
// =========================
void motor(int Velocidad_motor_izq, int Velocidad_motor_der) {
  Velocidad_motor_der = constrain(Velocidad_motor_der, -255, 255);
  Velocidad_motor_izq = constrain(Velocidad_motor_izq, -255, 255);

  pwm_cmd_L = Velocidad_motor_izq;
  pwm_cmd_R = Velocidad_motor_der;

  int pwm_der = abs(Velocidad_motor_der);
  int pwm_izq = abs(Velocidad_motor_izq);

  // =========================
  // Motor derecho
  // =========================
  if (Velocidad_motor_der > 0) {
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
  } else if (Velocidad_motor_der < 0) {
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, HIGH);
  } else {
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, LOW);
    pwm_der = 0;
  }

  ledc_set_duty(speed_mode, ch_der, pwm_der);
  ledc_update_duty(speed_mode, ch_der);

  // =========================
  // Motor izquierdo
  // =========================
  if (Velocidad_motor_izq > 0) {
    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, LOW);
  } else if (Velocidad_motor_izq < 0) {
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, HIGH);
  } else {
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, LOW);
    pwm_izq = 0;
  }

  ledc_set_duty(speed_mode, ch_izq, pwm_izq);
  ledc_update_duty(speed_mode, ch_izq);
}

// =========================
// Configuración PWM
// =========================
void setupPWM() {
  ledc_timer_config_t timer_conf = {};
  timer_conf.speed_mode = speed_mode;
  timer_conf.duty_resolution = duty_resolution;
  timer_conf.timer_num = timer_num;
  timer_conf.freq_hz = freq;
  timer_conf.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t chA = {};
  chA.gpio_num = pwm_a;
  chA.speed_mode = speed_mode;
  chA.channel = ch_der;
  chA.intr_type = LEDC_INTR_DISABLE;
  chA.timer_sel = timer_num;
  chA.duty = 0;
  chA.hpoint = 0;
  ledc_channel_config(&chA);

  ledc_channel_config_t chB = {};
  chB.gpio_num = pwm_b;
  chB.speed_mode = speed_mode;
  chB.channel = ch_izq;
  chB.intr_type = LEDC_INTR_DISABLE;
  chB.timer_sel = timer_num;
  chB.duty = 0;
  chB.hpoint = 0;
  ledc_channel_config(&chB);
}

// =========================
// Configuración motores
// =========================
void setupMotores() {
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);

  digitalWrite(ain1, LOW);
  digitalWrite(ain2, LOW);
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, LOW);

  setupPWM();
}

// =========================
// Configuración encoders
// =========================
void setupEncoders() {
  // ESP32Encoder::useInternalWeakPullResistors = puType::up;

  encoder_der.attachFullQuad(enc1_pinA, enc1_pinB);
  encoder_izq.attachFullQuad(enc2_pinA, enc2_pinB);

  encoder_der.clearCount();
  encoder_izq.clearCount();
}

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  delay(1000);

  setupMotores();
  setupEncoders();

  Serial.println("Inicio prueba motores + encoders");
}

// =========================
// Loop principal
// =========================
void loop() {

  // Subir PWM cada 500ms
  if (millis() - lastSubida >= 100) {
    lastSubida = millis();
    if (pwm_test < 255) pwm_test += 1;
  }

  motor(pwm_test, pwm_test);

  // =========================
  // Impresión cada 200 ms
  // =========================
  if (millis() - prevPrint >= 200) {
    prevPrint = millis();

    long countR = encoder_der.getCount();
    long countL = encoder_izq.getCount();

    long deltaR = countR - prevCountR;
    long deltaL = countL - prevCountL;

    prevCountR = countR;
    prevCountL = countL;

    Serial.print("PWM: ");
    Serial.print(pwm_test);
    Serial.print(" | PWM_L: ");
    Serial.print(pwm_cmd_L);
    Serial.print(" | PWM_R: ");
    Serial.print(pwm_cmd_R);

    Serial.print(" || ENC_L: ");
    Serial.print(countL);
    Serial.print(" | ENC_R: ");
    Serial.print(countR);

    Serial.print(" || dENC_L: ");
    Serial.print(deltaL);
    Serial.print(" | dENC_R: ");
    Serial.println(deltaR);
  }
}