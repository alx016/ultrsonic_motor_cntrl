//rango de distancia entre 50 y 200

//Declaración de Pines
const int Pin_1A = 13;
const int Pin_2A = 12;
const int Pin_EN = 11;
int trig = 10;
int eco = 9;

//Variables
const int interruptPin = 2;
int DutyCycle = 250;

int error;

int dur;
int dis;
bool direc[2] = {LOW,HIGH};
int wanted_dis; //mm
char buffer[6];

void setup() {
  pinMode(Pin_1A,OUTPUT);
  pinMode(Pin_2A,OUTPUT);
  pinMode(Pin_EN,OUTPUT);
  pinMode(interruptPin, INPUT);
  pinMode(trig, OUTPUT);
  pinMode(eco , INPUT);
  Serial.begin(9600);
  Serial.print("Indique la distancia deseada: ");
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
  dis = dur / 5.82 ; //mm distance
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
  analogWrite(Pin_EN, DutyCycle);
  return true;
}





