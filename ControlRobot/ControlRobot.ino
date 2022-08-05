#include "DualVNH5019MotorShield.h"

DualVNH5019MotorShield motor;

#define PinA 21
#define PinB 20
#define PinC 19
#define PinD 18

char msgEnd = ';';
bool newMsg = false;

float rD = 0; //Varia segun referencia obtenida
float rI = 0;

// Valores der M2
float eD = 0;
float eD_ = 0;
float eD__ = 0;

float outD = 0;
float outD_ = 0;

volatile double posD = 0.0;
volatile double pD = 0.0;
volatile double pD_ = 0.0;
volatile double vD = 0.0;

// Valores izq M1
float eI = 0;
float eI_ = 0;
float eI__ = 0;

float outI = 0;
float outI_ = 0;

volatile double posI = 0.0;
volatile double pI = 0.0;
volatile double pI_ = 0.0;
volatile double vI = 0.0;
volatile float dt = 0.0;

// Tiempo
volatile long t = 0;
volatile long t_ = 0;
volatile long t__ = 0;
volatile long at__ = 0;

float kv = 1920;

float kpD = 0.5;
float kiD = 15;
float kdD = 0.01;
float k0D = 0;
float k1D = 0;
float k2D = 0;

float kpI = 0.5;
float kiI = 15;
float kdI = 0.01;
float k0I = 0;
float k1I = 0;
float k2I = 0;

int max_vel = 250;
int min_vel = -250;


void EncoderA() {
  if (digitalRead(PinA) != digitalRead(PinB)) { //
    posD = posD + 1;
  }
  else { //reversa
    posD = posD - 1;
  }
}
void EncoderB() {
  if (digitalRead(PinA) == digitalRead(PinB)) { //
    posD = posD + 1;
  }
  else { //Reversa
    posD = posD - 1;
  }
}
void EncoderC() {
  if (digitalRead(PinC) != digitalRead(PinD)) { //
    posI = posI + 1;
  }
  else { //Reversa
    posI = posI - 1;
  }
}
void EncoderD() {
  if (digitalRead(PinC) == digitalRead(PinD)) { //
    posI = posI + 1;
  }
  else { //Reversa
    posI = posI - 1;
  }
}

void setup() {
  Serial.begin(115200);
  Serial3.begin(38400);

  pinMode(PinA, INPUT);
  pinMode(PinB, INPUT);
  pinMode(PinC, INPUT);
  pinMode(PinD, INPUT);

  attachInterrupt(digitalPinToInterrupt(PinA), EncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PinB), EncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PinC), EncoderC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PinD), EncoderD, CHANGE);

  motor.init();
}

void velocidad(double dt) {
  if (dt > 0 ) {
    vD = (pD - pD_) * 60 / (kv * dt);
    vI = (pI - pI_) * 60 / (kv * dt);
  }
}

void loop() {
  t_ = t;
  delay(4);
  t = micros();
  if (t - at__ > 10000) {
    pD_ = pD;
    pD = posD;
    pI_ = pI;
    pI = posI;

    dt = float(t - t_) / 1000000.0;
    velocidad(dt);

    outD_ = outD;
    eD__ = eD_;
    eD_ = eD;
    eD = rD - vD;
    outI_ = outI;
    eI__ = eI_;
    eI_ = eI;
    eI = rI - vI;

    // Complete aquí
    float k0D = kpD * (1 + dt * kiD + kdD/dt);
    float k1D = -kpD*(1 + 2*kdD/dt);
    float k2D = kpD*kdD/dt;
    double du = k0D * eD + k1D * eD_ + k2D * eD__;
    outD = outD_ + du; // Aquí falta un elemento

    float k0I = kpI * (1 + dt * kiI + kdI/dt);
    float k1I = -kpI*(1 + 2*kdI/dt);
    float k2I = kpI*kdI/dt;
    double Iu = k0I * eI + k1I * eI_ + k2I * eI__;
    outI = outI_ - Iu; // Aquí falta un elemento
  }
  outD = min(max(outD, min_vel), max_vel);
  outI = min(max(outI, min_vel), max_vel);
  motor.setM2Speed(int(outD));
  motor.setM1Speed(int(outI));

  String mensaje = readBuff(); // O es este delay??
  if (t - t__ > 100000) {
    t__ = t;
    // mensaje = processmsg(mensaje);
    if (newMsg){
      int comapos = mensaje.indexOf(",");
      String nrD = mensaje.substring(0, comapos);
      String nrI = mensaje.substring(comapos+1);
      if (not (isnan(nrD.toInt()) || isnan(nrD.toInt())))
      {
        Serial.println("mensaje");
        Serial.println(mensaje);
        rD = nrD.toInt();
        rI = nrI.toInt();
      }
      else
      {
        Serial.println("No hay mensaje");
        Serial.println(mensaje);
        rD = 0;
        rI = 0;
      }
      newMsg = false;
    }
    /*Serial.println("Velocidades");
    Serial.println(vD);
    Serial.println("OutD");
    Serial.println(outD);
    Serial.println("Error1");
    Serial.println(eD);
    Serial.println("Error2");
    Serial.println(eD_);
    Serial.println("dt");
    Serial.println(dt);
    Serial.println("\n");*/
    
    /*Serial.println("Velocidades");
    Serial.println(vI);
    Serial.println("OutI");
    Serial.println(outI);
    Serial.println("Error1");
    Serial.println(eI);
    Serial.println("Error2");
    Serial.println(eI_);
    Serial.println("rD");
    Serial.println(rD);
    Serial.println("rI");
    Serial.println(rI);
    Serial.println("\n");*/
  }
}

String readBuff() {
  String buffArray;
  //int i = 0;

  while (Serial3.available() > 0) { //Entro a este while mientras exista algo en el puerto serial
    char buff = Serial3.read(); //Leo el byte entrante
    if (buff == msgEnd) {
      newMsg = true;
      break; //Si el byte entrante coincide con mi delimitador, me salgo del while
    } else {
      buffArray += buff; //Si no, agrego el byte a mi string para construir el mensaje
      //i += 1;
    }
    //delay(10); // Ver si este delay afecta las instrucciones del robot
  }

  return buffArray;  //Retorno el mensaje
}
