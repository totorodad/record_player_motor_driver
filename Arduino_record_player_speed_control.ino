// *****************************************************************************************************************
// Project: Record Speed control revision 2.1
// Author: Nolan Engineering
// Date: 25-Nov-2024 (Thanksgiving Edition)
// Software Description: 
// 1) Read OEM push buttons to set the mode (33 RPM, 45 RPM, 78 RPM, Reverse, Start/Stop.
// 2) Set the PWM to the H-bridge driver (L298N) via arduino PWM output based on the hall sensor RPM speed sensor.
// 3) Maintain speed dynamically.
// Target Hardware: Arduino Uno
// I/O    Direction   PU/PD   Description
// *******************************************************************************************************************
// D2     In          PU      Start stop button
// D3     Out(pwm)    NA      L298N_IN1 Input 1
// D4     Out(pwm)    NA      L298N_IN2 Input 2
// D7     In          PU      HALL Effect sensor input (http://adafru.it/158) US5881LUA 
// D8     In          PU      45 RPM push button
// D9     In          PU      Reverse push button
// D10    In          PU      33 RPM push button
// A3     Out         NA      Sync LED (falshed once per hall sensor read i.e. per Revolution)
// A4     Out         NA      45 RPM LED
// A5     Out         NA      33 RPM LED
// *******************************************************************************************************************

#define THRESHOLD_LIMIT (20000) //empirically set for this system.
#define THIRTY_THREE_RPM_MICROSECONDS_PER_ROTATION  (1800000) //((1/(33+(1/3))*60*1000000)) = 1800000 us/rot or 1.8s/rot
#define FOURTY_FIVE_RPM_MICROSECONDS_PER_ROTATION   (1333333) //((1/45)*60*1000000) = 1333333 us/rot or 1.33s/rot
#define SEVENTY_EIGHT_RPM_MICROSECONDS_PER_ROTATION (769231)  //((1/78)*60*1000000) = 769231 us/rot or 1.33s/rot

#define THIRTY_THREE_RPM_PERIOD_UPPER_LIMIT (THIRTY_THREE_RPM_MICROSECONDS_PER_ROTATION + THRESHOLD_LIMIT)
#define THIRTY_THREE_RPM_PERIOD_LOWER_LIMIT (THIRTY_THREE_RPM_MICROSECONDS_PER_ROTATION - THRESHOLD_LIMIT)
#define FOURTY_FIVE_RPM_PERIOD_UPPER_LIMIT  (FOURTY_FIVE_RPM_MICROSECONDS_PER_ROTATION + THRESHOLD_LIMIT)
#define FOURTY_FIVE_RPM_PERIOD_LOWER_LIMIT  (FOURTY_FIVE_RPM_MICROSECONDS_PER_ROTATION - THRESHOLD_LIMIT)
#define SEVENTY_EIGHT_RPM_PERIOD_UPPER_LIMIT  (SEVENTY_EIGHT_RPM_MICROSECONDS_PER_ROTATION + THRESHOLD_LIMIT)
#define SEVENTY_EIGHT_RPM_PERIOD_LOWER_LIMIT  (SEVENTY_EIGHT_RPM_MICROSECONDS_PER_ROTATION - THRESHOLD_LIMIT)

#define INITIAL_PWM_33 (102);
#define INITIAL_PWM_45 (126);
#define INITIAL_PWM_78 (210);

#define START_STOP_PUSH_BUTTON (2)
#define L298N_IN1 (3) //PWM analog channel
#define L298N_IN2 (5) //PWM analog channel
#define HALL_SENSOR_INPUT (7)
#define SYNC_LED (A3)
#define THIRTY_THREE_RPM_LED (A5)
#define FOURTY_FIVE_RPM_LED (A4)
#define THIRTY_THREE_RPM_PUSH_BUTTON (10)
#define REVERSE_PUSH_BUTTON (9)
#define FOURTY_FIVE_RPM_PUSH_BUTTON (8)

//Modes
#define STOPPED                   (0)
#define THIRTY_THREE_RPM_FORWARD  (1)
#define THIRTY_THREE_RPM_REVERSE  (2)
#define FOURTY_FIVE_RPM_FORWARD   (3)
#define FOURTY_FIVE_RPM_REVERSE   (4)
#define SEVENTY_EIGHT_RPM_FORWARD (5)
#define SEVENTY_EIGHT_RPM_REVERSE (6)

#define MOTOR_STOP    (0)
#define MOTOR_FORWARD (1)
#define MOTOR_REVERSE (2)
#define MOTOR_RUNNING (3)
#define MOTOR_STOPPED (4)

#define PULSE_PER_REV (2.0)

int debug = 0;

unsigned long start_micros=micros();
int in;
int pwm = INITIAL_PWM_33; // Start at full speed for 33 1/3 RPM
int last_pwm = 0;
unsigned long current_micros;
unsigned long period;
float RPM;
float RPM_ERROR;
int mode = STOPPED;
int motor_status = MOTOR_STOPPED;

//Debounce management for reading buttons
#define DEBOUNCE_COUNT (500)
int start_stop_button_debounce_count = 0;
int rpm_33_button_debounce_count = 0;
int rpm_45_button_debounce_count = 0;
int rev_button_debounce_count = 0;
int rpm_78_button_debounce_count = 0;

void setup() {
  // Setup I/O pins
  pinMode (THIRTY_THREE_RPM_LED, OUTPUT);
  digitalWrite(THIRTY_THREE_RPM_LED, LOW);

  pinMode (FOURTY_FIVE_RPM_LED, OUTPUT);
  digitalWrite(FOURTY_FIVE_RPM_LED, LOW);
  
  //pinMode (THIRTY_THREE_RPM_PUSH_BUTTON, INPUT);
  pinMode (THIRTY_THREE_RPM_PUSH_BUTTON, INPUT_PULLUP);

  //pinMode (FOURTY_FIVE_RPM_PUSH_BUTTON, INPUT);
  pinMode (FOURTY_FIVE_RPM_PUSH_BUTTON, INPUT_PULLUP);
  
  //pinMode (REVERSE_PUSH_BUTTON, INPUT);
  pinMode (REVERSE_PUSH_BUTTON, INPUT_PULLUP);
  
  //pinMode (HALL_SENSOR_INPUT, INPUT);
  pinMode (HALL_SENSOR_INPUT, INPUT_PULLUP);

  pinMode (SYNC_LED, OUTPUT);

  pinMode (L298N_IN1, OUTPUT); // L298N IN1
  pinMode (L298N_IN2, OUTPUT); // L298N IN2
  digitalWrite (L298N_IN1, LOW); 
  digitalWrite (L298N_IN2, LOW); 
  
  pinMode (START_STOP_PUSH_BUTTON, INPUT); // On/Off button
  pinMode (START_STOP_PUSH_BUTTON, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println("Nolan Engineering:");
  Serial.println("Gemini Record Player Motor Driver Replacement Project Rev 2.1");
  Serial.println("26-Nov-2024, Thanksgiving eddition.");
}

void set_motor_speed(int pulse_width_modulation, int setting) {
  if (pulse_width_modulation != last_pwm) {
    last_pwm = pulse_width_modulation;
    switch (setting) {
      case MOTOR_STOP:
        analogWrite(L298N_IN1,0); // Turn motor off
        analogWrite(L298N_IN2,0);
        if (debug == 1) {
          Serial.println("\nMotor Set to Stop\n");
        }
        break;
      case MOTOR_FORWARD:
        motor_status = MOTOR_RUNNING;
        analogWrite(L298N_IN2, HIGH);
        analogWrite(L298N_IN1, pulse_width_modulation); // analog write to digital port = pwm mode, 0-255 value = 0-100% duty
        if (debug == 1) {
          Serial.println("\nMotor set to new Forward speed");
        }
        break;
      case MOTOR_REVERSE:
        motor_status = MOTOR_RUNNING;
        analogWrite(L298N_IN2, pulse_width_modulation);
        analogWrite(L298N_IN1, HIGH); // analog write to digital port = pwm mode, 0-255 value = 0-100% duty
        if (debug == 1) {
          Serial.println("\nMotor set to new Reverse speed\n");
        }
        break;
    }
  }
  else {
    if (debug == 1) {
      Serial.println("\nNo motor speed change\n");
    }
  }
}

void stop_case(void) {
      if (motor_status == MOTOR_RUNNING) {
        // Halt the motor
        set_motor_speed(0,MOTOR_STOP);
        // Turn LED's off
        digitalWrite(THIRTY_THREE_RPM_LED, LOW);
        digitalWrite(FOURTY_FIVE_RPM_LED, LOW);
        motor_status = MOTOR_STOPPED;
      }
}

void thirty_three_rpm_forward_case(void) {
      set_motor_speed(pwm, MOTOR_FORWARD);

      if (debug == 1) {
        Serial.print("\nmotor pwm: ");
        Serial.print(pwm);
        Serial.println("\n");
      }

      //Wait for the first magnet to pass (don't use)
      while (digitalRead(HALL_SENSOR_INPUT) == 1) {  // Wait for the hall sensor to detect the magnet
        readButtons();
      }
      while (digitalRead(HALL_SENSOR_INPUT) == 0) {  // First magnet detected
        readButtons();
      }

      //Wait for second magent (I dont' think he knows about second magnet Frodo)
      while (digitalRead(HALL_SENSOR_INPUT) == 1) {  // Wait for the hall sensor to detect the magnet
        readButtons();
      }
      while (digitalRead(HALL_SENSOR_INPUT) == 0) {  // First magnet detected
        readButtons();
        digitalWrite(SYNC_LED,HIGH); // Turn on sync led (green)
      }

      current_micros = micros();

      period = current_micros-start_micros;
      start_micros = current_micros;

      digitalWrite(SYNC_LED,LOW); // Turn off sync led

      // Recalculate the period
      Serial.print("Motor Duty: ");
      Serial.print(pwm);
      Serial.print(", P(us): ");
      Serial.print(period);
      
      // Calculate RPM
      RPM = 60000000 / (float)period;
      Serial.print(", RPM: ");
      Serial.print(RPM, 5);  

      // Calcualte rotation error
      RPM_ERROR = ((33.000 + (1/3))/RPM)/100.0;
      Serial.print(", RPM Error: ");
      Serial.print(RPM_ERROR, 5);  
      Serial.println("%");   

      // Auto Adjust algorithm for 33 1/3
      if (period > THIRTY_THREE_RPM_PERIOD_UPPER_LIMIT) {
        pwm++;
      }
      if (period < THIRTY_THREE_RPM_PERIOD_LOWER_LIMIT) {
        pwm--;
      }

      if (pwm < 0) {
        pwm = 0;
      }
      if (pwm > 255) {
        pwm = 255;
      }
}

void thirty_three_rpm_reverse_case(void) {
  set_motor_speed(pwm, MOTOR_REVERSE);

  if (debug == 1) {
    Serial.print("\nmotor pwm: ");
    Serial.print(pwm);
    Serial.println("\n");
  }

  //Wait for the first magnet to pass (don't use)
  while (digitalRead(HALL_SENSOR_INPUT) == 1) {  // Wait for the hall sensor to detect the magnet
    readButtons();
  }
  while (digitalRead(HALL_SENSOR_INPUT) == 0) {  // First magnet detected
    readButtons();
  }

  //Wait for second magent (I dont' think he knows about second magnet Frodo)
  while (digitalRead(HALL_SENSOR_INPUT) == 1) {  // Wait for the hall sensor to detect the magnet
    readButtons();
  }
  while (digitalRead(HALL_SENSOR_INPUT) == 0) {  // First magnet detected
    readButtons();
    digitalWrite(SYNC_LED,HIGH); // Turn on sync led (green)
  }

  current_micros = micros();

  period = current_micros-start_micros;
  start_micros = current_micros;

  digitalWrite(SYNC_LED,LOW); // Turn off sync led

  // Recalculate the period
  Serial.print("Motor Duty: ");
  Serial.print(pwm);
  Serial.print(", P(us): ");
  Serial.print(period);
  
  // Calculate RPM
  RPM = 60000000 / (float)period;
  Serial.print(", RPM: ");
  Serial.print(RPM, 5);  

  // Calcualte rotation error
  RPM_ERROR = ((33.000 + (1/3))/RPM)/100.0;
  Serial.print(", RPM Error: ");
  Serial.print(RPM_ERROR, 5);  
  Serial.println("%");   

  // Auto Adjust algorithm for 33 1/3
  if (period > THIRTY_THREE_RPM_PERIOD_UPPER_LIMIT) {
    pwm++;
  }
  if (period < THIRTY_THREE_RPM_PERIOD_LOWER_LIMIT) {
    pwm--;
  }

  if (pwm < 0) {
    pwm = 0;
  }
  if (pwm > 255) {
    pwm = 255;
  }
      

}

void fourty_five_rpm_forward_case(void) {
  set_motor_speed(pwm, MOTOR_FORWARD);

  if (debug == 1) {
    Serial.print("\nmotor pwm: ");
    Serial.print(pwm);
    Serial.println("\n");
  }

  //Wait for the first magnet to pass (don't use)
  while (digitalRead(HALL_SENSOR_INPUT) == 1) {  // Wait for the hall sensor to detect the magnet
    readButtons();
  }
  while (digitalRead(HALL_SENSOR_INPUT) == 0) {  // First magnet detected
    readButtons();
  }

  //Wait for second magent (I dont' think he knows about second magnet Frodo)
  while (digitalRead(HALL_SENSOR_INPUT) == 1) {  // Wait for the hall sensor to detect the magnet
    readButtons();
  }
  while (digitalRead(HALL_SENSOR_INPUT) == 0) {  // First magnet detected
    readButtons();
    digitalWrite(SYNC_LED,HIGH); // Turn on sync led (green)
  }

  current_micros = micros();

  period = current_micros-start_micros;
  start_micros = current_micros;

  digitalWrite(SYNC_LED,LOW); // Turn off sync led

  // Recalculate the period
  Serial.print("Motor Duty: ");
  Serial.print(pwm);
  Serial.print(", P(us): ");
  Serial.print(period);
  
  // Calculate RPM
  RPM = 60000000 / (float)period;
  Serial.print(", RPM: ");
  Serial.print(RPM, 5);  

  // Calcualte rotation error
  RPM_ERROR = ((45.000)/RPM)/100.0;
  Serial.print(", RPM Error: ");
  Serial.print(RPM_ERROR, 5);  
  Serial.println("%");   

  // Auto Adjust algorithm for 44
  if (period > FOURTY_FIVE_RPM_PERIOD_UPPER_LIMIT) {
    pwm++;
  }
  if (period < FOURTY_FIVE_RPM_PERIOD_LOWER_LIMIT) {
    pwm--;
  }

  if (pwm < 0) {
    pwm = 0;
  }
  if (pwm > 255) {
    pwm = 255;
  }
}

void fourty_five_rpm_reverse_case (void) {
  set_motor_speed(pwm, MOTOR_REVERSE);

  if (debug == 1) {
    Serial.print("\nmotor pwm: ");
    Serial.print(pwm);
    Serial.println("\n");
  }

  //Wait for the first magnet to pass (don't use)
  while (digitalRead(HALL_SENSOR_INPUT) == 1) {  // Wait for the hall sensor to detect the magnet
    readButtons();
  }
  while (digitalRead(HALL_SENSOR_INPUT) == 0) {  // First magnet detected
    readButtons();
  }

  //Wait for second magent (I dont' think he knows about second magnet Frodo)
  while (digitalRead(HALL_SENSOR_INPUT) == 1) {  // Wait for the hall sensor to detect the magnet
    readButtons();
  }
  while (digitalRead(HALL_SENSOR_INPUT) == 0) {  // First magnet detected
    readButtons();
    digitalWrite(SYNC_LED,HIGH); // Turn on sync led (green)
  }

  current_micros = micros();

  period = current_micros-start_micros;
  start_micros = current_micros;

  digitalWrite(SYNC_LED,LOW); // Turn off sync led

  // Recalculate the period
  Serial.print("Motor Duty: ");
  Serial.print(pwm);
  Serial.print(", P(us): ");
  Serial.print(period);
  
  // Calculate RPM
  RPM = 60000000 / (float)period;
  Serial.print(", RPM: ");
  Serial.print(RPM, 5);  

  // Calcualte rotation error
  RPM_ERROR = ((45.000)/RPM)/100.0;
  Serial.print(", RPM Error: ");
  Serial.print(RPM_ERROR, 5);  
  Serial.println("%");   

  // Auto Adjust algorithm for 44
  if (period > FOURTY_FIVE_RPM_PERIOD_UPPER_LIMIT) {
    pwm++;
  }
  if (period < FOURTY_FIVE_RPM_PERIOD_LOWER_LIMIT) {
    pwm--;
  }

  if (pwm < 0) {
    pwm = 0;
  }
  if (pwm > 255) {
    pwm = 255;
  }
}

void seventy_eight_rpm_forward_case (void) {
  set_motor_speed(pwm, MOTOR_FORWARD);

  if (debug == 1) {
    Serial.print("\nmotor pwm: ");
    Serial.print(pwm);
    Serial.println("\n");
  }

  //Wait for the first magnet to pass (don't use)
  while (digitalRead(HALL_SENSOR_INPUT) == 1) {  // Wait for the hall sensor to detect the magnet
    readButtons();
  }
  while (digitalRead(HALL_SENSOR_INPUT) == 0) {  // First magnet detected
    readButtons();
  }

  //Wait for second magent (I dont' think he knows about second magnet Frodo)
  while (digitalRead(HALL_SENSOR_INPUT) == 1) {  // Wait for the hall sensor to detect the magnet
    readButtons();
  }
  while (digitalRead(HALL_SENSOR_INPUT) == 0) {  // First magnet detected
    readButtons();
    digitalWrite(SYNC_LED,HIGH); // Turn on sync led (green)
  }

  current_micros = micros();

  period = current_micros-start_micros;
  start_micros = current_micros;

  digitalWrite(SYNC_LED,LOW); // Turn off sync led

  // Recalculate the period
  Serial.print("Motor Duty: ");
  Serial.print(pwm);
  Serial.print(", P(us): ");
  Serial.print(period);
  
  // Calculate RPM
  RPM = 60000000 / (float)period;
  Serial.print(", RPM: ");
  Serial.print(RPM, 5);  

  // Calcualte rotation error
  RPM_ERROR = ((78.000)/RPM)/100.0;
  Serial.print(", RPM Error: ");
  Serial.print(RPM_ERROR, 5);  
  Serial.println("%");   

  // Auto Adjust algorithm for 44
  if (period > SEVENTY_EIGHT_RPM_PERIOD_UPPER_LIMIT) {
    pwm++;
  }
  if (period < SEVENTY_EIGHT_RPM_PERIOD_LOWER_LIMIT) {
    pwm--;
  }

  if (pwm < 0) {
    pwm = 0;
  }
  if (pwm > 255) {
    pwm = 255;
  }
}

void seventy_eight_rpm_reverse_case (void) {
  set_motor_speed(pwm, MOTOR_REVERSE);

  if (debug == 1) {
    Serial.print("\nmotor pwm: ");
    Serial.print(pwm);
    Serial.println("\n");
  }

  //Wait for the first magnet to pass (don't use)
  while (digitalRead(HALL_SENSOR_INPUT) == 1) {  // Wait for the hall sensor to detect the magnet
    readButtons();
  }
  while (digitalRead(HALL_SENSOR_INPUT) == 0) {  // First magnet detected
    readButtons();
  }

  //Wait for second magent (I dont' think he knows about second magnet Frodo)
  while (digitalRead(HALL_SENSOR_INPUT) == 1) {  // Wait for the hall sensor to detect the magnet
    readButtons();
  }
  while (digitalRead(HALL_SENSOR_INPUT) == 0) {  // First magnet detected
    readButtons();
    digitalWrite(SYNC_LED,HIGH); // Turn on sync led (green)
  }

  current_micros = micros();

  period = current_micros-start_micros;
  start_micros = current_micros;

  digitalWrite(SYNC_LED,LOW); // Turn off sync led

  // Recalculate the period
  Serial.print("Motor Duty: ");
  Serial.print(pwm);
  Serial.print(", P(us): ");
  Serial.print(period);
  
  // Calculate RPM
  RPM = 60000000 / (float)period;
  Serial.print(", RPM: ");
  Serial.print(RPM, 5);  

  // Calcualte rotation error
  RPM_ERROR = ((78.000)/RPM)/100.0;
  Serial.print(", RPM Error: ");
  Serial.print(RPM_ERROR, 5);  
  Serial.println("%");   

  // Auto Adjust algorithm for 44
  if (period > SEVENTY_EIGHT_RPM_PERIOD_UPPER_LIMIT) {
    pwm++;
  }
  if (period < SEVENTY_EIGHT_RPM_PERIOD_LOWER_LIMIT) {
    pwm--;
  }

  if (pwm < 0) {
    pwm = 0;
  }
  if (pwm > 255) {
    pwm = 255;
  }
}

void loop() {
  // Read buttons to set the mode
  readButtons();

  switch (mode) {

    case STOPPED:
      stop_case();
      break;

    case THIRTY_THREE_RPM_FORWARD:
      thirty_three_rpm_forward_case();
      break;

    case THIRTY_THREE_RPM_REVERSE:
      thirty_three_rpm_reverse_case();
      break;

    case FOURTY_FIVE_RPM_FORWARD:
      fourty_five_rpm_forward_case();
      break;

    case FOURTY_FIVE_RPM_REVERSE:
      fourty_five_rpm_reverse_case();
      break;

    case SEVENTY_EIGHT_RPM_FORWARD:
      seventy_eight_rpm_forward_case();
      break;

    case SEVENTY_EIGHT_RPM_REVERSE:
      seventy_eight_rpm_reverse_case();
      break;

    default:
      Serial.print("oops illeale mode set");
      stop_case();
  }
 
}

void printmode(char* source) {
  if(debug == 1) {
    Serial.print("Debug source: ");
    Serial.print(source);
    Serial.println("\n");

    switch (mode) {
      case STOPPED:
        Serial.println("mode = STOPPED");
        break;
      case THIRTY_THREE_RPM_FORWARD:
        Serial.println("mode = THIRTY_THREE_RPM_FORWARD");
        break;
      case THIRTY_THREE_RPM_REVERSE:
        Serial.println("mode = THIRTY_THREE_RPM_REVERSE");
        break;
      case FOURTY_FIVE_RPM_FORWARD:
        Serial.println("mode = FORTY_FIVE_RPM_FORWARD");
        break;
      case FOURTY_FIVE_RPM_REVERSE:
        Serial.println("mode = FORTY_FIVE_RPN_REVERSE");
        break;
    }
  }
}

void readButtons (void) {
  int start_stop_but;
  int rpm_33_but;
  int rpm_45_but;
  int rpm_78_but;
  int rev_but;

  start_stop_but = digitalRead(START_STOP_PUSH_BUTTON);
  if (start_stop_but == LOW) {
    start_stop_button_debounce_count++;
    if (start_stop_button_debounce_count < DEBOUNCE_COUNT){
      start_stop_but = HIGH; // Not a real reading yet
    }
    else {
      start_stop_button_debounce_count = 0; //prepare for next debounced read
    }
  }
  // Not asserted case (reset the debounce count)
  else {
    start_stop_button_debounce_count = 0;
  }

  // look for both buttons down for 78RPM
  rpm_78_but = HIGH; //Virtual button (when both real buttons are pressed)
  rpm_33_but = digitalRead(THIRTY_THREE_RPM_PUSH_BUTTON);
  rpm_45_but = digitalRead(FOURTY_FIVE_RPM_PUSH_BUTTON);
  
  //delay(1); //This delay is needed for digitalRead to work ?mystery time
  
  //78 RPM case
  if ((rpm_33_but == LOW) && (rpm_45_but == LOW)) {
    rpm_78_button_debounce_count++;
    if (rpm_78_button_debounce_count < DEBOUNCE_COUNT){
      rpm_78_but = HIGH; // Not a real reading yet
    }
    else {
      rpm_78_but = LOW;
      rpm_78_button_debounce_count = 0; //prepare for next debounced read
    }
  }
  else {
    rpm_78_button_debounce_count = 0;
  }

  // 33 RPM case
  if (rpm_33_but == LOW && rpm_45_but == HIGH) {
    rpm_33_button_debounce_count++;
    if (rpm_33_button_debounce_count < DEBOUNCE_COUNT){
      rpm_33_but = HIGH; // Not a real reading yet
    }
    else {
      rpm_33_button_debounce_count = 0; //prepare for next debounced read
    }
  }
  else {
    rpm_33_button_debounce_count = 0;
  }
  
  // 45 RPM case
  if (rpm_33_but == HIGH && rpm_45_but == LOW) {
    rpm_45_button_debounce_count++;
    if (rpm_45_button_debounce_count < DEBOUNCE_COUNT){
      rpm_45_but = HIGH; // Not a real reading yet
    }
    else {
      rpm_45_button_debounce_count = 0; //prepare for next debounced read
    }
  }
  else {
    rpm_45_button_debounce_count = 0;
  }
  
  rev_but = digitalRead(REVERSE_PUSH_BUTTON);
  if (rev_but == LOW) {
    rev_button_debounce_count++;
    if (rev_button_debounce_count < DEBOUNCE_COUNT){
      rev_but = HIGH; // Not a real reading yet
    }
    else {
      rev_button_debounce_count = 0; //prepare for next debounced read
    }
  }
  else {
    rev_button_debounce_count = 0;
  }


  // Check the power button
  if (start_stop_but == LOW) {
    while(digitalRead(START_STOP_PUSH_BUTTON) == LOW); //Wait for the user to let go of the button
    if (mode == STOPPED) {
      mode = THIRTY_THREE_RPM_FORWARD;
      pwm = INITIAL_PWM_33; // Inital value for 33 1/3 RPM records
      digitalWrite(THIRTY_THREE_RPM_LED, HIGH);
      digitalWrite(FOURTY_FIVE_RPM_LED, LOW);
      printmode("mode set to THIRTY_THREE_RPM_FORWARD in read start-stop-button");
    }
    else {
      mode = STOPPED;
      digitalWrite(THIRTY_THREE_RPM_LED, LOW);
      digitalWrite(FOURTY_FIVE_RPM_LED, LOW);
      printmode("mode set to STOPPED in read start-stop button");
    }
  }

  // 78 mode (both 33 and 45 RPM buttons pressed = 78 mode)
  else if (rpm_78_but == LOW) {
    while(digitalRead(THIRTY_THREE_RPM_PUSH_BUTTON) == LOW || digitalRead(FOURTY_FIVE_RPM_PUSH_BUTTON) == LOW); //Wait for the user to let go of the buttons

    if (mode == STOPPED) {
      mode = SEVENTY_EIGHT_RPM_FORWARD;
      pwm = INITIAL_PWM_78; // Initial value for 78 RPM records
      digitalWrite(THIRTY_THREE_RPM_LED, HIGH);
      digitalWrite(FOURTY_FIVE_RPM_LED, HIGH);
      printmode("mode set to SEVENTY_EIGHT_RPM_FORWARD in read 78 button");
    }

    else if (mode == SEVENTY_EIGHT_RPM_FORWARD || mode == SEVENTY_EIGHT_RPM_REVERSE) {
      mode = STOPPED;
      digitalWrite(THIRTY_THREE_RPM_LED, LOW);
      digitalWrite(FOURTY_FIVE_RPM_LED, LOW);
      printmode("set to stopped from read 78 button");
    }

    else if (mode == THIRTY_THREE_RPM_FORWARD) {
        mode = SEVENTY_EIGHT_RPM_FORWARD;
        pwm = INITIAL_PWM_78; // Initial value for 45 RPM records
        digitalWrite(THIRTY_THREE_RPM_LED, HIGH);
        digitalWrite(FOURTY_FIVE_RPM_LED, HIGH);
        printmode("mode set to SEVENTY_EIGHT_RPM_FORWARD in read 78 button");
    }

    else if (mode == THIRTY_THREE_RPM_REVERSE) {
        mode = SEVENTY_EIGHT_RPM_REVERSE;
        pwm = INITIAL_PWM_78; // Initial value for 45 RPM records
        digitalWrite(THIRTY_THREE_RPM_LED, HIGH);
        digitalWrite(FOURTY_FIVE_RPM_LED, HIGH);
        printmode("mode set to SEVENTY_EIGHT_RPM_REVERSE in read 78 button");
    }

    else if (mode == FOURTY_FIVE_RPM_FORWARD) {
        mode = SEVENTY_EIGHT_RPM_FORWARD;
        pwm = INITIAL_PWM_78; // Initial value for 45 RPM records
        digitalWrite(THIRTY_THREE_RPM_LED, HIGH);
        digitalWrite(FOURTY_FIVE_RPM_LED, HIGH);
        printmode("mode set to SEVENTY_EIGHT_RPM_FORWARD in read 78 button");
    }

    else if (mode == FOURTY_FIVE_RPM_REVERSE) {
        mode = SEVENTY_EIGHT_RPM_REVERSE;
        pwm = INITIAL_PWM_78; // Initial value for 45 RPM records
        digitalWrite(THIRTY_THREE_RPM_LED, HIGH);
        digitalWrite(FOURTY_FIVE_RPM_LED, HIGH);
        printmode("mode set to SEVENTY_EIGHT_RPM_REVERSE in read 78 button");
    }

  }

  else if (rpm_33_but == LOW && rpm_45_but == HIGH) {
    while(digitalRead(THIRTY_THREE_RPM_PUSH_BUTTON) == LOW); //Wait for the user to let go of the button
    if (mode == STOPPED) {
      mode = THIRTY_THREE_RPM_FORWARD;
      pwm = INITIAL_PWM_33; // Initial value for 33 1/3 RPM records
      digitalWrite(THIRTY_THREE_RPM_LED, HIGH);
      digitalWrite(FOURTY_FIVE_RPM_LED, LOW);
      printmode("mode set to THIRTY_THREE_RPM_FORWARD in read 33 button");
    }

    else if (mode == THIRTY_THREE_RPM_FORWARD || mode == THIRTY_THREE_RPM_REVERSE) {
      mode = STOPPED;
      digitalWrite(THIRTY_THREE_RPM_LED, LOW);
      digitalWrite(FOURTY_FIVE_RPM_LED, LOW);
      printmode("set to stopped from read 33 button");
    }

    else if (mode == FOURTY_FIVE_RPM_FORWARD) {
        mode = THIRTY_THREE_RPM_FORWARD;
        pwm = INITIAL_PWM_33; // Initial value for 45 RPM records
        digitalWrite(THIRTY_THREE_RPM_LED, HIGH);
        digitalWrite(FOURTY_FIVE_RPM_LED, LOW);
        printmode("mode set to THIRTY_THREE_RPM_FORWARD in read 33 button");
    }

    else if (mode == FOURTY_FIVE_RPM_REVERSE) {
        mode = THIRTY_THREE_RPM_REVERSE;
        digitalWrite(THIRTY_THREE_RPM_LED, HIGH);
        digitalWrite(FOURTY_FIVE_RPM_LED, LOW);
        pwm = INITIAL_PWM_33; // Initial value for 45 RPM records
        printmode("mode set to THIRTY_THREE_RPM_REVERSE in read 33 button");
    }

    else if (mode == SEVENTY_EIGHT_RPM_FORWARD) {
        mode = THIRTY_THREE_RPM_FORWARD;
        digitalWrite(THIRTY_THREE_RPM_LED, HIGH);
        digitalWrite(FOURTY_FIVE_RPM_LED, LOW);    
        pwm = INITIAL_PWM_33; // Initial value for 45 RPM records
        printmode("mode set to THIRTY_THREE_RPM_FORWARD in read 33 button");
    }

    else if (mode == SEVENTY_EIGHT_RPM_REVERSE) {
        mode = THIRTY_THREE_RPM_REVERSE;
        digitalWrite(THIRTY_THREE_RPM_LED, HIGH);
        digitalWrite(FOURTY_FIVE_RPM_LED, LOW);
        pwm = INITIAL_PWM_33; // Initial value for 45 RPM records
        printmode("mode set to THIRTY_THREE_RPM_REVERSE in read 33 button");
    }
  }

  else if (rpm_45_but == LOW && rpm_33_but == HIGH) {
    while(digitalRead(FOURTY_FIVE_RPM_PUSH_BUTTON) == LOW); //Wait for the user to let go of the button

    if (mode == STOPPED) {
      mode = FOURTY_FIVE_RPM_FORWARD;
      digitalWrite(THIRTY_THREE_RPM_LED, LOW);
      digitalWrite(FOURTY_FIVE_RPM_LED, HIGH);
      pwm = INITIAL_PWM_45; // Initial value for 33 1/3 RPM records
      printmode("mode set to FOURTY_FIVE_RPM_FORWARD in read 45 button");
    }

    else if (mode == FOURTY_FIVE_RPM_FORWARD || mode == FOURTY_FIVE_RPM_REVERSE) {
      mode = STOPPED;
      digitalWrite(THIRTY_THREE_RPM_LED, LOW);
      digitalWrite(FOURTY_FIVE_RPM_LED, LOW);
      printmode("set to stopped from read 45 button");
    }

    else if (mode == THIRTY_THREE_RPM_FORWARD) {
        mode = FOURTY_FIVE_RPM_FORWARD;
        pwm = INITIAL_PWM_45; // Initial value for 45 RPM records
        digitalWrite(THIRTY_THREE_RPM_LED, LOW);
        digitalWrite(FOURTY_FIVE_RPM_LED, HIGH);
        printmode("mode set to FOURTY_FIVE_RPM_FORWARD in read 45 button");
    }

    else if (mode == THIRTY_THREE_RPM_REVERSE) {
        mode = FOURTY_FIVE_RPM_REVERSE;
        pwm = INITIAL_PWM_45; // Initial value for 45 RPM records
        digitalWrite(THIRTY_THREE_RPM_LED, LOW);
        digitalWrite(FOURTY_FIVE_RPM_LED, HIGH);
        printmode("mode set to FOURTY_FIVE_RPM_REVERSE in read 45 button");
    }

    else if (mode == SEVENTY_EIGHT_RPM_FORWARD) {
        mode = FOURTY_FIVE_RPM_FORWARD;
        pwm = INITIAL_PWM_45; // Initial value for 45 RPM records
        digitalWrite(THIRTY_THREE_RPM_LED, LOW);
        digitalWrite(FOURTY_FIVE_RPM_LED, HIGH);
        printmode("mode set to FOURTY_FIVE_RPM_FORWARD in read 45 button");
    }

    else if (mode == SEVENTY_EIGHT_RPM_REVERSE) {
        mode = FOURTY_FIVE_RPM_REVERSE;
        pwm = INITIAL_PWM_45; // Initial value for 45 RPM records
        digitalWrite(THIRTY_THREE_RPM_LED, LOW);
        digitalWrite(FOURTY_FIVE_RPM_LED, HIGH);
        printmode("mode set to FOURTY_FIVE_RPM_REVERSE in read 45 button");
    }
   
  }

  else if (rev_but == LOW) {
    while(digitalRead(REVERSE_PUSH_BUTTON) == LOW); //Wait for the user to let go of the button
    
    if (mode == THIRTY_THREE_RPM_FORWARD) {
      mode = THIRTY_THREE_RPM_REVERSE;
      printmode("mode set to THIRTY_THREE_RPM_REVERSED in read reverse button");
    }
    else if (mode == THIRTY_THREE_RPM_REVERSE) {
      mode = THIRTY_THREE_RPM_FORWARD;
      printmode("mode set to THIRTY_THREE_RPM_FORWARD in read reverase button");
    }
    else if (mode == FOURTY_FIVE_RPM_FORWARD) {
      mode = FOURTY_FIVE_RPM_REVERSE;
      printmode("mode set to FOURTY_FIVE_RPM_REVERSE in read reverse button");
    }
    else if (mode == FOURTY_FIVE_RPM_REVERSE) {
      mode = FOURTY_FIVE_RPM_FORWARD;
      printmode("mode set to FOURTY_FIVE_FORWARD in read reverse button");
    }
    else if (mode == SEVENTY_EIGHT_RPM_FORWARD) {
      mode = SEVENTY_EIGHT_RPM_REVERSE;
      printmode("mode set to SEVENTY_EIGHT_RPM_REVERSE in read reverse button");
    }
    else if (mode == SEVENTY_EIGHT_RPM_REVERSE) {
      mode = SEVENTY_EIGHT_RPM_FORWARD;
      printmode("mode set to SEVENTY_EIGHT_FORWARD in read reverse button");
    }
  }  
}

