
/*     Control loop for SCARA robot
 *      
 *     by Aurélien LOHMANN
 *  
 */

//pin used for motors
const int stepPinZ = 3;
const int dirPinZ = 4;
const int stepPin_1Arm = 5;
const int dirPin_1Arm = 6;
const int stepPin_2Arm = 7;
const int dirPin_2Arm = 8;

//pin used to calibrate Z axis
const int stopSwitch = 9;

//safety stop for the system
const int INTERRUPT_PIN = 2;


// variables for control and motor positions
float z_range = 0;
float z_offset = 0;
float z_total = 330;
float z_pos;
float x_pos;
float y_pos = 0;
float theta1, theta2, theta1_obj, theta2_obj;
int nbStepZ,nbStepL1,nbStepL2;

//Links definition
const float L1=100;
const float L2=100;

// Struct to define trajectories 
struct Position {
  float x;
  float y;
  float z;
};

//Store the objective positions
const int NUM_POSITIONS = 10;
Position positions[NUM_POSITIONS];


void setup() {
  // put your setup code here, to run once:
  // This is a necessary line for the serial monitor to display information
  Serial.begin(9600);

  // Defining the attachinterrupt for the emergency stop
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), myInterruptFunction, LOW);

  //definition of motor control pin
  pinMode(stepPinZ,OUTPUT); 
  pinMode(dirPinZ,OUTPUT);
  pinMode(stepPin_1Arm,OUTPUT); 
  pinMode(dirPin_1Arm,OUTPUT);
  pinMode(stepPin_2Arm,OUTPUT); 
  pinMode(dirPin_2Arm,OUTPUT);
  pinMode(stopSwitch, INPUT);

  
  // create random objective in the workspace
  for (int i = 0; i < NUM_POSITIONS; i++) {
  
    positions[i].x = randomInRange(20,175);
    positions[i].y = randomInRange(-75,75);
    positions[i].z = randomInRange(z_offset+20,z_total - z_offset);

    Serial.println(i);
    //prevent any points to be outside reach
    if (sqrt(positions[i].x * positions[i].x + positions[i].y * positions[i].y) > (L1+L2 - 20)) {
      i--;
    }
  }

  //print every position
  for (int i = 0; i < NUM_POSITIONS; i++) {
    Serial.print("Position ");
    Serial.print(i);
    Serial.print(": (");
    Serial.print(positions[i].x);
    Serial.print(", ");
    Serial.print(positions[i].y);
    Serial.print(", ");
    Serial.print(positions[i].z);
    Serial.println(")");
  }

  //put the plateform at its highest position, both arms have to be put in a straight line
  calibration();
  z_pos = 0;
  theta1 = 0;
  theta2 = 0;
}


//Main loop
void loop() {
  for (int j=0; j < NUM_POSITIONS;j++){ 
    // for every positions to go to : calculate the inverse kinematics, determine the number of steps, move the motors.
    calculateJointAngles(positions[j].x, positions[j].y, theta1_obj, theta2_obj);
    nbStepZ = inverseKin_Z(positions[j].z);
    nbStepL1 = -1 * calculateNbStepL1(theta1_obj); //the rotation is reversed due to gear ratio
    nbStepL1 = calculateNbStepL1(theta2_obj);

    //control the motors
    moveJoint(1,nbStepZ);
    moveJoint(2,nbStepL1);
    moveJoint(3,nbStepL2);

    delay(5000);
  }

  
}




//Utilitary functions
void moveJoint(int j, int nbStep){
  // j is used to identify the link moved (j=1 for z-axis, j=2 for first arm, j=3 for second arm)
  // dirStep corresponds to the rotation, clockwise is 1, anticlockwise is 0
  // nbStep corresponds to the number of steps ran
  int dirStep,dir;
  if (nbStep > 0){
    dirStep = 1; //clockwise
    dir=1;
  }
  else{
    dirStep = 0; //anticlockwise
    dir=-1;
  }
  nbStep = abs(nbStep);
  
  if ( j == 1){
    digitalWrite(dirPinZ, dirStep); //Changes the rotations direction
    for(int x = 0; x < nbStep; x++) {
      digitalWrite(stepPinZ,HIGH);
      delayMicroseconds(1500);
      digitalWrite(stepPinZ,LOW);
      delayMicroseconds(1500);
      z_pos = z_pos + 0.04*dir;
    }
  }
  if ( j == 2){ 
    digitalWrite(dirPin_1Arm, dirStep); //Changes the rotations direction
    for(int x = 0; x < nbStep; x++) {
      digitalWrite(stepPin_1Arm,HIGH);
      delayMicroseconds(1500);
      digitalWrite(stepPin_1Arm,LOW);
      delayMicroseconds(1500);
      theta1 = theta1 + 0.9*dir;
    }
  }

  if ( j ==3){
    digitalWrite(dirPin_2Arm, dirStep); //Changes the rotations direction
    for(int x = 0; x < nbStep; x++) {
      digitalWrite(stepPin_2Arm,HIGH);
      delayMicroseconds(1500);
      digitalWrite(stepPin_2Arm,LOW);
      delayMicroseconds(1500);
      theta2 = theta2 + 0.9*dir;
    }
  }    
}


int inverseKin_Z (float z_obj){
  float z_distance, moveZ;
  int nbStep;
   
  z_distance = z_total - z_obj - z_offset;

  if ((z_distance + z_offset) < z_range){
    moveZ = z_distance - z_pos;
    nbStep = moveZ/0.04;
    return nbStep;
  }
  return 0;
}

void calculateJointAngles(float x, float y, float &theta1, float &theta2) {
  // Calculate the length of the line between the origin and the input point
  float d = sqrt(x*x + y*y);

  // Calculate the angles of the two joints
  theta2 = acos((L1*L1 + L2*L2 - d*d) / (2*L1*L2));
  theta1 = atan2(x, y) + atan((L1*L1 + d*d - L2*L2) / (2*L1*d));

  // Convert the angles from radians to degrees
  theta1 = theta1 * 180 / PI;
  theta2 = theta2 * 180 / PI;
}

int calculateNbStepL1(float theta1_obj){
  float tmpL1, tmpL1_obj, distance;
  tmpL1 = theta1 + 80;
  tmpL1_obj = theta1_obj + 80;

  distance = tmpL1_obj - tmpL1;
  distance = distance / 0.9;
  
  return round(distance);
}

int calculateNbStepL2(float theta2_obj){
  float tmpL2, tmpL2_obj, distance;
  tmpL2 = theta2 + 80;
  tmpL2_obj = theta2_obj + 80;

  distance = tmpL2_obj - tmpL2;
  distance = distance / 0.9;
  
  return round(distance);
}

void calibration(){
  digitalWrite(dirPinZ, HIGH);
   while (digitalRead(stopSwitch)!= 1){
      digitalWrite(stepPinZ,HIGH);
      delayMicroseconds(1500);
      digitalWrite(stepPinZ,LOW);
      delayMicroseconds(1500);
   }
}

void myInterruptFunction() {
  // This function will be called when the interrupt is triggered
  // Put your interrupt code here
  Serial.println("EMERGENCY STOP ENABLED");
  digitalWrite(stepPinZ,LOW);
  digitalWrite(stepPin_1Arm,LOW);
  digitalWrite(stepPin_2Arm,LOW);
  while (1!=0){
      Serial.println("EMERGENCY STOP ENABLED");
  }
}


float randomInRange(float minVal, float maxVal) {
  return random(minVal * 100, maxVal * 100) / 100.0; // Generate a random number between minVal and maxVal with 2 decimal places
}
