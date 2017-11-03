#define IR1 A7//leftir
#define IR2 A6//midir
#define IR3 A5//rightir

#define rLED 8
#define gLED 7
#define bLED 4

#define L_MOTOR 5
#define R_MOTOR 3

const int basespeedR = 100;
const int basespeedL = 100; 

template<class T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }
const int DT = 1;
int offs[3];
int avg = 0;

struct pid { //Note that because we have two different types of distance sensors (Andrew's works a little differently than Jeffrey's we should have two different errors. To stay straight though we can just use one side right?)
  double integral = 0;
  int prev = 0;
  double kp = 1; //the ks should be negative to counteract error
  double ki = 0;
  double kd = 0.8;
};

struct ir_in{
  int x,y,z;
};

void resetPid(struct pid *e) {
  e->integral = 0;
  e->prev = 0;
}

int getFix(struct pid *e, int error) {
  double d = (error - e->prev)/DT;
  e->integral += error * DT;
  e->prev = error;
  return (int)(e->kp * error + e->ki * e->integral + e->kd * d);
}

struct ir_in ir_read(){
  ir_in ir_struct;
  int x = analogRead(IR1);
  int y = analogRead(IR2);
  int z = analogRead(IR3);
  if(x > offs[0]){
    x = offs[0];
    }
  if(y > offs[1]){
    y = offs[1];
  }
  if(z > offs[2]){
    z = offs[2];
    }
  ir_struct.x = map(x, 0, offs[0], 255, 0);
  ir_struct.y = map(y, 0, offs[1], 255, 0);
  ir_struct.z = map(z, 0, offs[2], 255, 0);

  return ir_struct;
}

void resetLEDs(){digitalWrite(gLED, LOW); digitalWrite(bLED, LOW); digitalWrite(rLED, LOW);};

void Turn_Perpendicular(){ //Turns left when detects that we're perpendicular to the line. This should hardly go on.
  while(analogRead(IR1) < 40 || analogRead(IR2) < 40 || analogRead(IR3) < 40){
    analogWrite(L_MOTOR, 130);
    digitalWrite(bLED, HIGH);
    }
    digitalWrite(bLED, LOW);
    analogWrite(L_MOTOR, 0);
  }

struct pid lman; struct pid rman;

void process(ir_in& ir_struct){
  resetLEDs();

  int rError = abs(ir_struct.y - ir_struct.x);
  int lError = abs(ir_struct.y - ir_struct.z);
  
  const int level = 40; //Threshold level. Above 40 means black detected, below that means white.
  if(ir_struct.x > 40){
    if(ir_struct.y > 40){
      if(ir_struct.z > 40){
        //We're perpendicular to the line. Turn right until we hit the line again.
        Turn_Perpendicular();
        }
       else{
        digitalWrite(rLED, HIGH);
        //We should be updating PID to go to the right a little bit
        int lmotor = constrain(basespeedL + getFix(&lman, lError), basespeedL, 255);
        int rmotor = basespeedR;
        }
      }
    else{
      digitalWrite(rLED, HIGH);
      int lmotor = constrain(basespeedL + getFix(&lman, lError), basespeedL*1.4, 255);
      int rmotor = basespeedR;
      //We should be updating PID to go to the right a lot.
      }
    }
  else{
    if(ir_struct.y > 40){
      if(ir_struct.z > 40){
        digitalWrite(bLED, HIGH);
          int rmotor = constrain(basespeedR + getFix(&rman, rError), basespeedR, 255);
          int lmotor = basespeedL;
        //We should update PID to go to the left a little bit
        }
      else{
        digitalWrite(gLED, HIGH);
        //We're completely in a straight line. Reset pid integral error in order to make PID recalibrated.
        resetPid(&lman); resetPid(&rman);
        }  
      }
     else if(ir_struct.z > 40){//Update PID to go to the left a lot. 
      digitalWrite(bLED, HIGH);
      int rmotor = constrain(basespeedR + getFix(&rman, rError), (basespeedR)*1.35, 255);
      int lmotor = basespeedL;
     }
     else{//No line to follow, we finished (Jumpstart and see where we can go after this)
      digitalWrite(bLED, HIGH);
      digitalWrite(rLED, HIGH);
     }  
  }
}
void setup() {
  Serial.begin(9600);
  offs[0] = 0;
  offs[1] = 0;
  offs[2] = 0;
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  int nsamples = 5;
  for(int i = 0; i < nsamples; i++){
    avg += analogRead(IR2);
    offs[0] += analogRead(IR1);
    offs[1] += analogRead(IR2);
    offs[2] += analogRead(IR3);
    Serial << "avg:" << avg;
  }
  offs[0] = offs[0]/nsamples;
  offs[1] = offs[1]/nsamples;
  offs[2] = offs[2]/nsamples;
  avg = avg/nsamples;
  Serial << "offs" << offs[0] << " " << offs[1] << "\n";

  //Serial << "x1:  " << ir_struct.x << "  y1:  " << ir_struct.y << "  z1:  " << ir_struct.z << "  \n";  //Previously in loop to test IR
}

void loop() {
  ir_in ir_struct = ir_read();
  process(ir_struct);
 }
