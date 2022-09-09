//Implementación de un control PID en un motor de CD

//Libreria
#include <TimerOne.h>

//Declaración de los Pines
const int Pin_1A = 13;
const int Pin_2A = 12;
const int Pin_En = 11;
const int interruptPin = 2;
int trig = 10;
int eco = 9;

//Declaración de variables 
long unsigned int counter = 0;
float tiempo = 0.0;
float T = 0.02;
int DutyCycle = 250;
int error;
int dur;
int dis=0;
bool direc[2] = {LOW,HIGH};

int wanted_dis; //mm
char buffer[6];

//Parametros del PID
float referencia = 0.0;
float Kp = 0.05;
float Ti = 0.07;
float Td = 0.0;

//Parámetros del control PID Discreto
float K0 = Kp+(Kp*T/Ti)+(Kp*Td/T);
float K1 = -Kp-(2*Kp*Td/T);
float K2 = Kp*Td/T;

//Inicializar el error y el control
float e=0, e1=0, e2=0;
float u=0, u1=0;


void setup() {
  pinMode(Pin_1A,OUTPUT);
  pinMode(Pin_2A,OUTPUT);
  pinMode(Pin_En,OUTPUT);
  pinMode(interruptPin, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(eco , INPUT);
  Serial.begin(9600);
  Serial.print("Indique la distancia deseada: ");

  attachInterrupt(digitalPinToInterrupt(interruptPin), counting, RISING);
  //Timer 1: Se activa cada 20,000 microsegundos =0.02segundos
  Timer1.initialize(20000);
  Timer1.attachInterrupt(ISR_RevolucionesPorMinuto);
  //Comunicación Serial
  Serial.begin(9600);
}

void loop() {
  int index;  
  while (Serial.available()) {
    index = Serial.readBytesUntil('\n', buffer, 5);  //newline or max of 5 chars
    buffer[index] = '\0';
    //Serial.print(buffer);  //so you can see the captured String 
    wanted_dis = atoi(buffer);  //convert readString into a number
    buffer[0] = '\0';
    
  }
  digitalWrite(trig, HIGH);
  delay(500);
  digitalWrite(trig,LOW);
  dur = pulseIn(eco, HIGH);
  dis = (dur / 5.82 ) - 50; //mm distance
  Serial.print("Distance: ");
  Serial.println(dis);
  Serial.print("Wanted Distance: ");
  Serial.println(wanted_dis);
  delay(500);
  
  error = wanted_dis - dis;

  if (error < -3){
    //mover el motor sentido opuesto
    direc[0] = LOW;
    direc[1] = HIGH;
    mov_mot(direc);
    
  }
  else if(error > 3){
    direc[0] = HIGH;  
    direc[1] = LOW;
    mov_mot(direc);
  }
  else {
  //fast motor stop
    direc[0] = LOW;
    direc[1] = LOW;
    mov_mot(direc);
  }
  delay(200);
}

bool mov_mot( bool direc[2]){
  digitalWrite(Pin_1A, direc[0]);
  digitalWrite(Pin_2A, direc[1]);
  analogWrite(Pin_En, DutyCycle);
  return true;
}

//Funciones
void counting(){
  //Contador
  counter++;
}

//Funcion Revoluciones por Minuto 
void ISR_RevolucionesPorMinuto(){
  dis = (dur / 5.82 ) - 50; //mm distance
  tiempo = tiempo + T;
  e = dis - wanted_dis;
  u = u1+K0*e+K1*e1+K2*e2;
  if (u>250){
    u=255;
  }
  else if (u<0){
    u=0;
  }
  analogWrite(Pin_En,u);
  u1=u;
  e2=e1;
  e1=e;
  counter = 0;
}
