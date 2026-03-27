#include <esp_wifi.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include <ESP32Encoder.h>
#include "driver/ledc.h"
#include <FastIMU.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>

// =========================
// Configuracion WiFi / UDP
// =========================
#define ssid "Linksys02764"
#define password "ddqvcdkki7"

#define IP_monitoreo "192.168.1.145"
#define puerto_monitoreo 1305
#define IP_sucesor "192.168.1.178"
#define puerto_sucesor 1111
#define puerto_local 1111

#define EtiquetaRobot "L"

// =========================
// Parametros del vehiculo
// =========================
#define radio_rueda 2.15
#define l 5.6
#define N 1
#define CPR 1600

#define VEL_MAX_CM_S 50.0
#define PWM_MAX 255.0
#define PWM_MIN_MOV 30.0

// =========================
// Pines ESP32
// =========================
#define ain1 18
#define ain2 19
#define pwm_a 23

#define bin1 17
#define bin2 16
#define pwm_b 4

#define enc1_pinA 34
#define enc1_pinB 35
#define enc2_pinA 36
#define enc2_pinB 39

#define S0 27
#define S1 26
#define S2 25
#define S3 33
#define SIG 32

// =========================
// VL53L0X / IMU
// =========================
#define IMU_ADDRESS 0x68
#define WHO_AM_I_REG 0x75

#define DIST_BLOQUEO_CM 1.5
#define DIST_SENSOR_MAX 30.0
float dist_cm = 0.0;

// =========================
// Sensores IR
// =========================
#define NUM_SENSORES 16
#define IR_FRONT_BASE 8
#define IR_REAR_BASE 0

const int THR_HIGH_FIJO = 4070;
const int THR_LOW_FIJO  = 4030;

static int maxV[NUM_SENSORES];
static int minV[NUM_SENSORES];
static int thrHigh[NUM_SENSORES];
static int thrLow[NUM_SENSORES];
static uint8_t binState[NUM_SENSORES] = {0};

// =========================
// Maquina de estados
// =========================
#define inicio 0
#define calibracion 1
#define controlLoop 2

volatile byte estado = inicio;
volatile byte estado_siguiente = inicio;
volatile byte calibrar = 0;

// =========================
// Objetos globales
// =========================
WiFiUDP udp;
Adafruit_VL53L0X vl53 = Adafruit_VL53L0X();
ESP32Encoder encoder_der;
ESP32Encoder encoder_izq;
unsigned long lastMicros = 0;

// =========================
// IMU
// =========================
enum TipoIMU {
  IMU_NONE = 0,
  IMU_IS_MPU6050,
  IMU_IS_MPU6500
};

struct IMUData {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
};

TipoIMU tipoIMU = IMU_NONE;
MPU6050 imu6050;
MPU6500 imu6500;
calData calib = { 0 };

AccelData accelData;
GyroData gyroData;

bool imu_ok = false;

// =========================
// Variables PWM
// =========================
int pwm_cmd_R = 0;
int pwm_cmd_L = 0;

// =========================
// Variables de estado del robot
// =========================
float yaw_deg        = 0.0f;
float vel_robot_cms  = 0.0f;

static long  prevCountR = 0;
static long  prevCountL = 0;
static unsigned long lastEstMicros = 0;

// =========================
// Debug controladores
// =========================
struct ControlDebug {
  double input = 0.0;
  double setpoint = 0.0;
  double error = 0.0;
  double output = 0.0;
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
};

ControlDebug dbg_velocidad;
ControlDebug dbg_orientacion;
ControlDebug dbg_distancia;

// =========================
// Control velocidad
// =========================
double vel_ref = 10.0;
float Kp_vel = 2.0f;
float Ki_vel = 0.5f;
float Kd_vel = 0.1f;
double vel_error_ant = 0.0;
double vel_integral = 0.0;

// =========================
// Control distancia
// =========================
double distancia_ref = 10.0;

// Completar

// =========================
// Control orientacion
// =========================
const float PWM_BASE_ORI = 80.0f;  // velocidad base del seguidor de línea
float Kp_ori = 5.0f;
float Ki_ori = 0.0f;
float Kd_ori = 0.1f;
int   error_ori_prev = 0;          // memoria del error anterior (requerida por la firma)
int   irBin[NUM_SENSORES] = {0};   // lecturas binarias de los 16 sensores IR

// Completar

// =========================
// Comunicacion y FSM
// =========================
char paquete_entrante[64];
char msg[128];
String parar = "si";

// =========================
// Prototipos generales
// =========================
void setup_wifi();

void motor(int, int);
double velToPWM(double);

double distancia();

void calibrarSensoresIR();

uint8_t readByte(uint8_t address, uint8_t reg);
TipoIMU detectarIMU();
IMUData leerIMU();
bool iniciarIMU();

double ControladorVelocidad(double, double, double&, double&, double, double, double, double);
double ControladorDistancia(double, double, double&, double&, double, double, double, double);
float ControladorOrientacion(int, int&, float, float, float);

void udp_recep();
void udp_transm();
void udp_recep_from_robot();
void udp_recep_from_gui();
void send_state_to_gui();
void send_comunication_to_gui(const char*, const char*);
void send_control_to_gui();

void TaskEstado(void*);
void TaskControl(void*);

// =========================
// Tasks
// =========================
TaskHandle_t handleTaskEstado;
TaskHandle_t handleTaskControl;

// =========================
// Setup
// =========================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  if (!vl53.begin()) {
    Serial.println("VL53L0X no detectado");
  } else {
    Serial.println("VL53L0X OK");
  }

  tipoIMU = detectarIMU();

  if (tipoIMU == IMU_NONE) {
    Serial.println("No se detecto MPU6050/MPU6500");
    imu_ok = false;
  } else {
    imu_ok = iniciarIMU();
    if (imu_ok) {
      lastMicros = micros();
      Serial.println("IMU lista");
    } else {
      Serial.println("Fallo la inicializacion de FastIMU");
    }
  }

  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);
  setup_wifi();

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(SIG, INPUT);

  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);

  ledc_timer_config_t timer_conf = {
    LEDC_HIGH_SPEED_MODE,
    LEDC_TIMER_8_BIT,
    LEDC_TIMER_0,
    5000,
    LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t chA = {
    pwm_a, LEDC_HIGH_SPEED_MODE,
    LEDC_CHANNEL_0, LEDC_INTR_DISABLE,
    LEDC_TIMER_0, 0, 0
  };
  ledc_channel_config(&chA);

  ledc_channel_config_t chB = {
    pwm_b, LEDC_HIGH_SPEED_MODE,
    LEDC_CHANNEL_1, LEDC_INTR_DISABLE,
    LEDC_TIMER_0, 0, 0
  };
  ledc_channel_config(&chB);

  encoder_der.attachFullQuad(enc1_pinA, enc1_pinB);
  encoder_izq.attachFullQuad(enc2_pinA, enc2_pinB);

  encoder_der.clearCount();
  encoder_izq.clearCount();

  xTaskCreatePinnedToCore(TaskEstado, "TaskEstado", 10240, NULL, 1, &handleTaskEstado, 0);
  xTaskCreatePinnedToCore(TaskControl, "TaskControl", 4096, NULL, 2, &handleTaskControl, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// =========================
// Cálculo del error de orientación
// =========================
int calcularErrorOrientacion() {
  float suma_ponderada = 0.0f;
  float suma_activos   = 0.0f;

  for (int i = 0; i < 8; i++) {
    float posicion = (float)i - 3.5f;
    suma_ponderada += posicion * irBin[IR_FRONT_BASE + i];
    suma_activos   += irBin[IR_FRONT_BASE + i];
  }

  if (suma_activos == 0.0f) return 0;
  return (int)(suma_ponderada / suma_activos);  // cast a int por la firma del controlador
}


// =========================
// Task de estado
// =========================
void TaskEstado(void* parameter) {
  for (;;) {

    udp_recep();
    switch (estado) {
      case inicio: {
        
        motor(0, 0);
        encoder_der.clearCount();
        encoder_izq.clearCount();
        // Completar
        // ...
        send_state_to_gui();
        break;
      }

      case calibracion: {
        calibrarSensoresIR();
        calibrar = 0;
        break;
      }

      case controlLoop: {
        // Completar

        // ---- Leer IMU y actualizar yaw ----

        // Esto evita saltos bruscos en el ángulo si la tarea se interrumpe o hay overflow.
        unsigned long nowUs = micros();
        float dt_est = (nowUs - lastEstMicros) * 1e-6f;
        if (dt_est <= 0.0f || dt_est > 0.5f) dt_est = 0.005f;
        lastEstMicros = nowUs;

        IMUData imuData = leerIMU();
        yaw_deg += imuData.gz * dt_est; // gz ya viene en deg/s desde FastIMU

        // ---- Calcular velocidad desde encoders ----
        long countR = encoder_der.getCount();
        long countL = encoder_izq.getCount();

        long deltaR = countR - prevCountR;
        long deltaL = countL - prevCountL;

        prevCountR = countR;
        prevCountL = countL;

        // dt = 0.02 segundos (ciclo de 20ms)
        float dt = 0.02f;

        // Velocidad angular de cada rueda (rad/s)
        float omega_R = (2.0f * PI * deltaR) / (CPR * N * dt);
        float omega_L = (2.0f * PI * deltaL) / (CPR * N * dt);

        // Velocidad lineal de cada rueda (cm/s)
        float vel_R = omega_R * radio_rueda; 
        float vel_L = omega_L * radio_rueda;

        // Velocidad lineal del robot (promedio)
        vel_robot_cms = (vel_R + vel_L) / 2.0f;

        // ---- Leer distancia ----
        distancia();   // actualiza la variable global dist_cm

        udp_transm();
        send_state_to_gui();
        break;
      }
    }

    if (estado == inicio && calibrar) estado_siguiente = calibracion;
    else if (estado == inicio && parar == "si") estado_siguiente = inicio;
    else if (estado == inicio && parar == "no") estado_siguiente = controlLoop;
    else if (estado == calibracion) estado_siguiente = inicio;
    else if (estado == controlLoop && parar == "no") estado_siguiente = controlLoop;
    else estado_siguiente = inicio;

    estado = estado_siguiente;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// =========================
// Task de control
// =========================
void TaskControl(void* parameter) {
  const TickType_t periodo = pdMS_TO_TICKS(20);
  TickType_t lastWakeTime = xTaskGetTickCount();

  static float lastL = 0.0f, lastR = 0.0f;

  for (;;) {
    if (estado == controlLoop && parar == "no") {
      // 1. Leer sensores IR en modo binario
      leerIR16(irBin, true);

      // 2. Controlador de VELOCIDAD
      double u_vel = ControladorVelocidad(vel_robot_cms, vel_ref, 
                                          vel_error_ant, vel_integral,
                                          Kp_vel, Ki_vel, Kd_vel, PWM_MAX);

      // 3. Calcular error de orientación (centroide)
      int error_ori = calcularErrorOrientacion();

      // 4. Controlador PD de orientación
      float u_ori = ControladorOrientacion(error_ori, error_ori_prev, Kp_ori, Ki_ori, Kd_ori);

      // 5. Combinar: velocidad base + corrección diferencial
      int pwm_base = constrain((int)(PWM_BASE_ORI + u_vel), -255, 255);
      int cmd_izq = constrain(pwm_base + (int)u_ori, -255, 255);
      int cmd_der = constrain(pwm_base - (int)u_ori, -255, 255);
      motor(cmd_izq, cmd_der);

      // 6. Actualizar estructuras de debug para la GUI
      dbg_velocidad.input    = vel_robot_cms;
      dbg_velocidad.setpoint = vel_ref;
      dbg_velocidad.error    = vel_ref - vel_robot_cms;
      dbg_velocidad.output   = u_vel;
      dbg_velocidad.kp       = Kp_vel;
      dbg_velocidad.ki       = Ki_vel;
      dbg_velocidad.kd       = Kd_vel;

      dbg_orientacion.input    = error_ori;
      dbg_orientacion.setpoint = 0.0;
      dbg_orientacion.error    = error_ori;
      dbg_orientacion.output   = u_ori;
      dbg_orientacion.kp       = Kp_ori;
      dbg_orientacion.ki       = Ki_ori;
      dbg_orientacion.kd       = Kd_ori;

      dbg_distancia.input    = dist_cm;
      dbg_distancia.setpoint = distancia_ref;
      dbg_distancia.error    = 0.0;
      dbg_distancia.output   = 0.0;
      dbg_distancia.kp       = 0.0;
      dbg_distancia.ki       = 0.0;
      dbg_distancia.kd       = 0.0;

      send_control_to_gui();
    } else {
      motor(0, 0);
    }

    vTaskDelayUntil(&lastWakeTime, periodo);
  }
}