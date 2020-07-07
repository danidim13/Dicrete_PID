#include <TaskScheduler.h>
#include <TaskSchedulerDeclarations.h>

#include "PID.hpp"



// Parametros del controlador

//Alfaro
//double Kp = 8.731;
//double Ti = 6.910;
//double Td = 0;
//double Ts = 0.4;

// Gamze
//double Kp = 10.273;
//double Ti = 15.584;
//double Td = 0;
//double Ts = 0.4;

// Skogestad 1er (Sobrepaso)
//double Kp = 6.7896;
//double Ti= 3.6878;
//double Td = 0;
//double Ts = 0.4;

//// Skogestad 1er v3 (Mejor)
//double Kp = 2.5617;
//double Ti = 6.2056;
//double Td = 0;
//double Ts = 0.1;

// Skogestad 1er Final
//double Kp = 8.1701;
//double Ti = 3.0384;
//double Td = 0;
//double Ts = 0.1;

// Skogestad 2do (Inestable a bajas frecuencias)
double Kp = 28.9185;
double Ti = 0.8;
double Td = 0.8438;
double Ts = 0.005;

double alfa = 10;
double beta = 0.8;

/**************************/
/* CONFIGURACION DE PINES */
/**************************/
// Pines analogicos
int x_pin = 2;
int sp_pin = 1;
int y_pin = 0;

// Pines PWM
int co_pin = 3;

// Pines digitales
int m_pin = 7;
int button_pin = 7;

/**************************/

// Constantes de scala para entrada/salida
double input_scale = 5.0 / 1023.0;
double output_scale = 255.0 / 5.0;

enum modo_t {
  MANUAL,
  AUTOMATICO
};
modo_t M = MANUAL;

unsigned long start = 0;
unsigned long now = 0;
int cuentas;

PID controlador( Kp, Ti, Td, alfa, beta, Ts);

void PidAutoCallback();
void PidManCallback();
void CheckModeCallback();
void PlotterCallback();
void Blink();
void SerialPlot(double u, double r, double y);
void SerialPlot(double u, double r, double y, double t);

double PlotVals[4];

Scheduler scheduler;

Task ModeTask(100, TASK_FOREVER, &CheckModeCallback, &scheduler);
Task PidTask((int)(Ts*1000), TASK_FOREVER, &PidAutoCallback, &scheduler);
//Task PlotTask(200, TASK_FOREVER, &PlotterCallback, &scheduler);

/**
 * Toma alrededor de 500 us cada ejecucion!
 * es decir, Ts max ~ 5 ms
 */
void PidAutoCallback() {
  int in_y = analogRead(y_pin);
  int in_sp = analogRead(sp_pin);

  double y = in_y * input_scale;
  double r = in_sp * input_scale;

  controlador.UpdateSetpoint(r);
  double u = controlador.StepAntiWind(y);

  int out = (int)(u * output_scale);
  analogWrite(co_pin, out);
  PlotVals[0] = u;
  PlotVals[1] = r;
  PlotVals[2] = y;
  return;
}

void PidManCallback() {
  int in_x = analogRead(x_pin);
  int in_y = analogRead(y_pin);
  
  double x = in_x * input_scale;
  double y = in_y * input_scale;

  controlador.Bypass(y, x);
  
  int out = (int)(x*output_scale);
  analogWrite(co_pin, out);

  PlotVals[0] = x;
  PlotVals[1] = x;
  PlotVals[2] = y;
  
  //SerialPlot(x, x, y);
  return;
}

void CheckModeCallback() {
  int m_read = digitalRead(m_pin);
  
  if (m_read == HIGH) {
    M = MANUAL;
  } else {
    M = AUTOMATICO;
  }
  
  if (M == MANUAL) {
    digitalWrite(LED_BUILTIN, LOW);
    PidTask.setCallback(&PidManCallback);
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
    PidTask.setCallback(&PidAutoCallback);
  }
  /*
  cuentas++;
  if (cuentas == 70) {
    M = AUTOMATICO;
  }
  */ 
}

void PlotterCallback() {
  SerialPlot(PlotVals[0], PlotVals[1], PlotVals[2]);
}

void SerialPlot(double u, double r, double y) {
  Serial.print(u);
  Serial.print(" ");
  Serial.print(r);
  Serial.print(" ");
  Serial.println(y);
  return;
}

void SerialPlot(double u, double r, double y, double t) {
  Serial.print(u);
  Serial.print(" ");
  Serial.print(r);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.println(t);
  return;
}

void Blink() {
  if (digitalRead(button_pin) == LOW) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
}

void setup() {
  // put your setup code here, to run once:
  controlador.SetMaxEffort(0, 5);
  controlador.UpdateSetpoint(2.5);

  pinMode(co_pin, OUTPUT);
  pinMode(m_pin, INPUT_PULLUP);
  //pinMode(button_pin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  

  // El código que se ponga acá se ejecuta una única vez al inicio:
  Serial.begin(9600);
  //start = millis();
  cuentas = 0;


  scheduler.addTask(ModeTask);
  scheduler.addTask(PidTask);
  //scheduler.addTask(PlotTask);
  
  ModeTask.enable();
  PidTask.enable();
  //PlotTask.enable();
}

void loop() {
  scheduler.execute();
}
