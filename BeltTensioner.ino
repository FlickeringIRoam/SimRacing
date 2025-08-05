#include <Servo.h>
#include <SafeString.h>
#include "SafeStringReader.h"
#include "stdarg.h"

#define SERIAL_SPEED 115200
#define MAX_MSG_LEN 23
#define MAX_FIELD_LEN 7

#define NUM_SERVOS 4

#define LEFT_SHOULDER 0
#define RIGHT_SHOULDER 1
#define LEFT_HIP 2
#define RIGHT_HIP 3

#define ACCEL_HEAVE 0
#define ACCEL_SURGE 1
#define ACCEL_SWAY  2

struct Forces {
  float accelerationHeave, accelerationSurge, accelerationSway;
};

struct Vector {
  float magnitude, direction;
};

struct ServoController {
  Servo servo;
  byte pin;
  byte invert;
  byte maxGy;
  signed char minGy;
  int mindeg;
  int maxdeg;
  int typedeg;
  float position;
  float force;
};

typedef struct Forces forces;
typedef struct Vector vector;
typedef struct ServoController servocontroller;

ServoController servocontrol[NUM_SERVOS];

Forces maxForces, minForces = { 0.00, 0.00, 0.00 };

createSafeStringReader(sfReader, MAX_MSG_LEN, "\r\n");
createSafeString(incomingData, MAX_MSG_LEN);  // -xxx.yy;-xxx.yy;-xxx.yy\n
createSafeString(field, MAX_FIELD_LEN);


/* one fast(ish) lap of suzuka ACC GT3
  Heave 12.62 -5.96
  Surge 20.75 -15
  Sway 22.12 -24.68
*/




void setup() {
  servocontrol[0].pin = 8;
  servocontrol[1].pin = 9;
  servocontrol[2].pin = 10;
  servocontrol[3].pin = 11;
  
  servocontrol[0].invert = 0;
  servocontrol[1].invert = 0;
  servocontrol[2].invert = 0;
  servocontrol[3].invert = 0;

  servocontrol[0].maxGy = 5;
  servocontrol[1].maxGy = 5;
  servocontrol[2].maxGy = 5;
  servocontrol[3].maxGy = 5;

  servocontrol[0].minGy = -25;
  servocontrol[1].minGy = -25;
  servocontrol[2].minGy = -25;
  servocontrol[3].minGy = -25;

  servocontrol[0].mindeg = 25;
  servocontrol[1].mindeg = 10;
  servocontrol[2].mindeg = 90;
  servocontrol[3].mindeg = 30;

  servocontrol[0].maxdeg = 145;
  servocontrol[1].maxdeg = 130;
  servocontrol[2].maxdeg = 180;
  servocontrol[3].maxdeg = 120;

  servocontrol[0].typedeg = 180;
  servocontrol[1].typedeg = 180;
  servocontrol[2].typedeg = 270;
  servocontrol[3].typedeg = 270;

  Serial.begin(SERIAL_SPEED);

  for (byte i = 0; i < NUM_SERVOS; i++) {
    servocontrol[i].servo.attach( servocontrol[i].pin );
  }
  delay(1000);
  for (byte i = 0; i < NUM_SERVOS; i++) {
    writeToServo(i, 0);
  }
  Serial.print(F("ready"));
  sfReader.connect(Serial);
//  sfReader.echoOn(); // default is off
}




void loop() {

  if (sfReader.read()) {
    Forces f = getForces(sfReader);
    Vector v = xlateForce(f);
    applyVectorToServos(v);
    printServoPositions();
  }

}









Forces getForces(SafeStringReader& f) {
  Forces s;
  s.accelerationHeave = 0; 
  s.accelerationSurge = 0;
  s.accelerationSway  = 0;
  int nextIdx = 0;
  char delimiters[] = ";";

  byte fieldCounter = 0;

  while (nextIdx >= 0) { // returns null on end
    nextIdx = f.stoken(field, nextIdx, delimiters, true); // true => return all fields, even empty ones
    float v;
    if (field.toFloat(v)) {
      // valid
      if (fieldCounter == ACCEL_HEAVE) {
        s.accelerationHeave = v;
      } else if (fieldCounter == ACCEL_SURGE) {
        s.accelerationSurge = v;
      } else if (fieldCounter == ACCEL_SWAY) {
        s.accelerationSway = v;
      } 
    } else {
      // F("not a number");
    }
    fieldCounter++;
  }

  Serial.println();
  Serial.print(F("Hv:"));
  Serial.print(s.accelerationHeave);
  Serial.print(F(" Sg:"));
  Serial.print(s.accelerationSurge);
  Serial.print(F(" Sy:"));
  Serial.print(s.accelerationSway);
  Serial.println();

//  storemaxforces(s);
//  printMaxForces();

  return s;
}

void storemaxforces(Forces f) {
  if (f.accelerationHeave > maxForces.accelerationHeave) {
    maxForces.accelerationHeave = f.accelerationHeave;
  }
  if (f.accelerationSurge > maxForces.accelerationSurge) {
    maxForces.accelerationSurge = f.accelerationSurge;
  }
  if (f.accelerationSway > maxForces.accelerationSway) {
    maxForces.accelerationSway = f.accelerationSway;
  }

  if (f.accelerationHeave < minForces.accelerationHeave) {
    minForces.accelerationHeave = f.accelerationHeave;
  }
  if (f.accelerationSurge < minForces.accelerationSurge) {
    minForces.accelerationSurge = f.accelerationSurge;
  }
  if (f.accelerationSway < minForces.accelerationSway) {
    minForces.accelerationSway = f.accelerationSway;
  }
}







Vector xlateForce(Forces f) {

  Vector v;
  float x, y;

  x = f.accelerationSway; // horizontal
  y = f.accelerationSurge; // not great, but simple

  //TODO implement heave into y (one may be +ve, the other -ve, but both may act the same, so it's not just the greater of the 2)

  // get magnitude of heave + surge
  //y = sqrt(f.accelerationHeave*f.accelerationHeave + f.accelerationSurge*f.accelerationSurge);

  v.magnitude = sqrt(x*x + y*y);  // magnitude is the hypoteneuse
  v.direction = atan2(y,x);  // direction is arctan in radians

  // a direction of 90 is vertical, or straight forward, let's fix that, in radians
  v.direction += M_PI/2;
  if (v.direction >= M_PI*2) {
    v.direction -= M_PI*2;
  }

  return v;

}


float degrees_to_radians(float d) {
  return d * (M_PI/180.0);
}


float radians_to_degrees(float r) {
  return r * (180.0/M_PI);
}






double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
  double ret;
//  in_max += 1;
//  out_max += 1;
  ret = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  //serial_printi("mapf(%f, %f, %f, %f, %f) = %f (%d)\n", x, in_min, in_max, out_min, out_max, ret, (int)ret);
  return ret;
}


void writeToServo(byte servoNum, float f) {
  //int degrees = (int)f;
  if (f > servocontrol[servoNum].maxGy) {
    f = servocontrol[servoNum].maxGy;
  }
  if (f < servocontrol[servoNum].minGy) {
    f = servocontrol[servoNum].minGy;
  }

  if (servocontrol[servoNum].force == f) {
    return; // servo already in position
  }

  double degrees;
  if (servocontrol[servoNum].invert == 1) {
    //serial_printi("Converting %f to %f for servo %d\n", degrees, servoTypeDeg[servoNum]-degrees, servoNum);
    degrees = mapf(f, servocontrol[servoNum].minGy, servocontrol[servoNum].maxGy, servocontrol[servoNum].maxdeg, servocontrol[servoNum].mindeg);
    servocontrol[servoNum].servo.write( servocontrol[servoNum].typedeg - degrees );
  } else {
    degrees = mapf(f, servocontrol[servoNum].minGy, servocontrol[servoNum].maxGy, servocontrol[servoNum].mindeg, servocontrol[servoNum].maxdeg);
    servocontrol[servoNum].servo.write( degrees );
  }
  servocontrol[servoNum].position = degrees;
  servocontrol[servoNum].force = f;
}






void applyVectorToServos(Vector v) {  

  serial_printi("D:%f V:%f\n", radians_to_degrees(v.direction), v.magnitude );

  float l = cos(v.direction) * v.magnitude + sin(v.direction) * v.magnitude;
  float r = cos(v.direction) * v.magnitude - sin(v.direction) * v.magnitude;

  //serial_printi("%f\t\t%f\n", l, r);

  writeToServo(LEFT_SHOULDER, l);
  writeToServo(RIGHT_SHOULDER, r);
  writeToServo(LEFT_HIP, l);
  writeToServo(RIGHT_HIP, r);
}









void printServoPositions() {
  serial_printi("%f(%f)\t\t%f(%f)\n", servocontrol[LEFT_SHOULDER].force, servocontrol[LEFT_SHOULDER].position, servocontrol[RIGHT_SHOULDER].force, servocontrol[RIGHT_SHOULDER].position);
  //serial_printi("%d(%d)\t\t%d(%d)\n", servocontrol[LEFT_HIP].force, servocontrol[LEFT_HIP].position, servocontrol[RIGHT_HIP].force, servocontrol[RIGHT_HIP].position);
}



void printMaxForces() {
  serial_printi("Hv Max :%f Min:%f\t\tSg Max:%f Min:%f\t\tSy Max:%f Min:%f\n", maxForces.accelerationHeave, minForces.accelerationHeave, maxForces.accelerationSurge, minForces.accelerationSurge, maxForces.accelerationSway, minForces.accelerationSway);
}





void serial_printi(const char *format, ...){
	
	char ch;
	bool flgInterpolate = false;
	va_list args;
	va_start( args, format );
	for( ; *format ; ++format ){
		ch = *format;
		if(flgInterpolate){
			flgInterpolate = false;
			if((ch=='d') || (ch=='c')){
				Serial.print(va_arg(args, int));
			}else if(ch=='s'){
				Serial.print(va_arg(args, char*));
			}else if(ch=='o'){
				Serial.print(va_arg(args, unsigned int));
			}else if((ch=='f') || (ch=='e') || (ch=='a') || (ch=='g')){
				Serial.print(va_arg(args, double));
			}else{
				Serial.print('%');
				Serial.print(ch);
			}
		}else if(ch=='%'){
			flgInterpolate = true;
		}else{
			Serial.print(ch);
		}
	}
	
	va_end( args );
}
