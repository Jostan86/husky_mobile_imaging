#include <AccelStepper.h>
#include <ezButton.h>
#include <math.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <husky_mobile_imaging/ActuatorPosition.h>
#include <husky_mobile_imaging/ActuatorCmd.h>
#include <husky_mobile_imaging/ActuatorStatus.h>
#include <std_msgs/Header.h>

// Make ros libraries with:
// rosrun rosserial_arduino make_libraries.py /path/to/libraries/
// eg:
// rosrun rosserial_arduino make_libraries.py /home/jostan/Arduino/libraries/


// Struct for linear actuator
struct LinearActuator {
  AccelStepper stepper;
  ezButton limit_switch_nc;
  ezButton limit_switch_no;
  int travel_length;
  long total_steps;
  int move_speed;
  int home_speed;
  int steps_per_mm;
  bool homed;
  bool enabled;
  String name;
};

// The Stepper and limit switch pins, stepper 1 is for vertical actuator, stepper 2 is for horizontal
#define STEPPER1_DIR_PIN 3
#define STEPPER1_STEP_PIN 2
#define STEPPER1_ENABLE_PIN 4
#define STEPPER1_LIMIT_PIN_NC 5
#define STEPPER1_LIMIT_PIN_NO 10

#define STEPPER2_DIR_PIN 7
#define STEPPER2_STEP_PIN 6
#define STEPPER2_ENABLE_PIN 8
#define STEPPER2_LIMIT_PIN_NC 9
#define STEPPER2_LIMIT_PIN_NO 11

// Define the messages and set up the publishers
husky_mobile_imaging::ActuatorPosition pos_msg;
ros::Publisher actuator_positions("imaging_platform/actuator_positions", &pos_msg);

husky_mobile_imaging::ActuatorStatus status_msg;
ros::Publisher status("imaging_platform/actuator_status", &status_msg);

// Define the node handle
ros::NodeHandle nh;

// Define the control message
husky_mobile_imaging::ActuatorCmd control_msg;

// Actuator Parameters
// Total travel length in mm
int actuator1_travel_length = 850;                      //mm
int actuator2_travel_length = 350;                       //mm
// Steps stepper takes per revolution
int steps_per_rev = 200;                                 //steps
// Distance traveled per revolution by carriage in mm
float travel_per_rev = 8.0;                              //mm
// Number of steps to move 1 mm
int steps_per_mm = round(steps_per_rev / travel_per_rev);
// Total length of each actuator in steps
long total_steps_1 = actuator1_travel_length * steps_per_mm;  //steps
long total_steps_2 = actuator2_travel_length * steps_per_mm;  //steps

// Speed to home at, in mm/s
int home_speed = -30;
// Initial speed setting for movements
int camera_initial_speed = 20;

// Set names for actuators, for status messages
String name1 = "v_actuator";
String name2 = "h_actuator";

// Define steppers, switches, and actuators
AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
ezButton limit_switch_1_nc(STEPPER1_LIMIT_PIN_NC);
ezButton limit_switch_1_no(STEPPER1_LIMIT_PIN_NO);

LinearActuator linAc1 = { stepper1, limit_switch_1_nc, limit_switch_1_no, actuator1_travel_length, total_steps_1, camera_initial_speed, home_speed, steps_per_mm, false, false, name1 };

AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);
ezButton limit_switch_2_nc(STEPPER2_LIMIT_PIN_NC);
ezButton limit_switch_2_no(STEPPER2_LIMIT_PIN_NO);

LinearActuator linAc2 = { stepper2, limit_switch_2_nc, limit_switch_2_no, actuator2_travel_length, total_steps_2, camera_initial_speed, home_speed, steps_per_mm, false, false, name2 };

// Define some other variables
unsigned long last_message = 0;
int update_wait_ms = 20;
bool state_nc = true;
bool state_no = false;
bool limit_status = true;
bool limit_status_1 = true;
bool limit_status_2 = true;
bool time_set = false;

//============
void messageCb(const husky_mobile_imaging::ActuatorCmd &control_msg) {
  // Disable the actuator if it is enabled and the command is to disable
  if (strcmp(control_msg.cmd_type.data, "disable") == 0) {
    if (strcmp(control_msg.actuator_name.data, "v_actuator") == 0 && linAc1.enabled == 1) {
      disableLinAc(linAc1);
    } else if (strcmp(control_msg.actuator_name.data, "h_actuator") == 0 && linAc2.enabled == 1) {
      disableLinAc(linAc2);
    }

  // Enable the actuator if it is disabled and the command is to enable
  } else if (strcmp(control_msg.cmd_type.data, "enable") == 0) {
    if (strcmp(control_msg.actuator_name.data, "v_actuator") == 0 && linAc1.enabled == 0) {
      enableLinAc(linAc1);
    } else if (strcmp(control_msg.actuator_name.data, "h_actuator") == 0 && linAc2.enabled == 0) {
      enableLinAc(linAc2);
    }

  // Send a status update if the command is to send a status update
  } else if (strcmp(control_msg.cmd_type.data, "status_update") == 0) {
    String message = "Sending status update";
    sendStatus(message);

  // Stop the actuator
  } else if (strcmp(control_msg.cmd_type.data, "stop") == 0) {
    if (strcmp(control_msg.actuator_name.data, "v_actuator") == 0) {
      delay(10);
      linAc1.stepper.move(0);
    } else if (strcmp(control_msg.actuator_name.data, "h_actuator") == 0) {
      delay(10);
      linAc2.stepper.move(0);
    }

  // Move the actuator to a position at a given speed
  } else if (strcmp(control_msg.cmd_type.data, "move") == 0) {
    if (strcmp(control_msg.actuator_name.data, "v_actuator") == 0 && linAc1.homed == 1) {
      // Position to move to
      long moveToStep = steps_per_mm * control_msg.position;
      // Velocity to move at
      int speed = control_msg.velocity;
      // Set the position to move to
      linAc1.stepper.moveTo(moveToStep);
      // Set the speed to move at
      if (moveToStep > linAc1.stepper.currentPosition()) {
        linAc1.move_speed = speed;
      } else {
        linAc1.move_speed = -speed;
      }

    } else if (strcmp(control_msg.actuator_name.data, "h_actuator") == 0 && linAc2.homed == 1) {
      long moveToStep = steps_per_mm * control_msg.position;
      int speed = control_msg.velocity;

      linAc2.stepper.moveTo(moveToStep);
      if (moveToStep > linAc2.stepper.currentPosition()) {
        linAc2.move_speed = speed;
      } else {
        linAc2.move_speed = -speed;
      }
    }
  }
}

// Setup the subscriber for commands
ros::Subscriber<husky_mobile_imaging::ActuatorCmd> control("imaging_platform/actuator_control", &messageCb);

void setup() {

  // Start the node handle, subscribe to the control message, and advertise the status and position messages
  nh.initNode();
  nh.subscribe(control);
  nh.advertise(actuator_positions);
  nh.advertise(status);
  ros::Time current_time = nh.now();

  // Setup the stepper drivers and limit switches
  linAc1.stepper.setEnablePin(STEPPER1_ENABLE_PIN);
  linAc1.stepper.setPinsInverted(false, false, true);
  disableLinAc(linAc1);
  linAc1.stepper.setMaxSpeed(1500.0);
  linAc1.stepper.setAcceleration(1000.0);
  linAc1.limit_switch_no.setDebounceTime(50);
  linAc1.limit_switch_nc.setDebounceTime(50);

  linAc2.stepper.setEnablePin(STEPPER2_ENABLE_PIN);
  linAc2.stepper.setPinsInverted(false, false, true);
  disableLinAc(linAc2);
  linAc2.stepper.setMaxSpeed(1500.0);
  linAc2.stepper.setAcceleration(1000.0);
  linAc2.limit_switch_no.setDebounceTime(50);
  linAc2.limit_switch_nc.setDebounceTime(50);
}


void loop() {

  // Spin the node once
  nh.spinOnce();

  // Check the limit switches, returns true if triggered
  limit_status_1 = checkLimit(linAc1);
  limit_status_2 = checkLimit(linAc2);

  // If the limit switch is triggered and the actuator is enabled, stop the actuator and disable it
  if (limit_status_1 && linAc1.enabled) {
    linAc1.stepper.move(0);
    disableLinAc(linAc1);
    String message = "Limit Switch Triggered for " + linAc1.name + "  -> disabling";
    sendStatus(message);
  }
  if (limit_status_2 && linAc2.enabled) {
    linAc2.stepper.move(0);
    disableLinAc(linAc2);
    String message = "Limit Switch Triggered for " + linAc2.name + "  -> disabling";
    sendStatus(message);
  }

  // If actuator needs to move, and it's not at the end or beginning, move it
  if (linAc1.stepper.distanceToGo() != 0 && linAc1.enabled && linAc1.stepper.currentPosition() < linAc1.total_steps && linAc1.stepper.currentPosition() > 0) {
      setMotorSpeed(linAc1.move_speed, linAc1.stepper);
      linAc1.stepper.runSpeed();
  // If actuator is at zero and enabled, need to handle that
  } else if(linAc1.stepper.distanceToGo() != 0 && linAc1.enabled && linAc1.stepper.currentPosition() == 0 && linAc1.move_speed > 0){
    setMotorSpeed(linAc1.move_speed, linAc1.stepper);
      linAc1.stepper.runSpeed();
  // If actuator is at end, need to handle that
  } else if (linAc1.stepper.distanceToGo() != 0 && linAc1.enabled && linAc1.stepper.currentPosition() == linAc1.total_steps && linAc1.move_speed < 0){
    setMotorSpeed(linAc1.move_speed, linAc1.stepper);
    linAc1.stepper.runSpeed();
  }

  if (linAc2.stepper.distanceToGo() != 0 && linAc2.enabled && linAc2.stepper.currentPosition() < linAc2.total_steps && linAc2.stepper.currentPosition() > 0) {
      setMotorSpeed(linAc2.move_speed, linAc2.stepper);
      linAc2.stepper.runSpeed();
  } else if(linAc2.stepper.distanceToGo() != 0 && linAc2.enabled && linAc2.stepper.currentPosition() == 0 && linAc2.move_speed > 0){
    setMotorSpeed(linAc2.move_speed, linAc2.stepper);
      linAc2.stepper.runSpeed();
  } else if (linAc2.stepper.distanceToGo() != 0 && linAc2.enabled && linAc2.stepper.currentPosition() == linAc2.total_steps && linAc2.move_speed < 0){
    setMotorSpeed(linAc2.move_speed, linAc2.stepper);
    linAc2.stepper.runSpeed();
  }

  // Send a status message at a set interval
  if (millis() - last_message > update_wait_ms) {
    last_message = millis();
    sendPos();
  }
}


void setMotorSpeed(int move_speed, AccelStepper &stepper) {
  // Sets the speed of the stepper motor
  int motor_speed = move_speed * steps_per_mm;
  stepper.setSpeed(motor_speed);
}

void disableLinAc(LinearActuator &linAc) {
  // Disables the actuator
  linAc.stepper.disableOutputs();
  linAc.homed = false;
  linAc.enabled = false;
  String message = "Disabled " + linAc.name;
  sendStatus(message);
}

void enableLinAc(LinearActuator &linAc) {
  // Enables the actuator
  linAc.stepper.enableOutputs();
  linAc.enabled = true;
  String message = "Enabled " + linAc.name + "  -> homing";
  sendStatus(message);
  // Home the actuator
  homeActuator(linAc);
}

void homeActuator(LinearActuator &linAc) {
  // Homes the actuator

  // Delay for ... some reason, makes me feel better
  delay(50);
  limit_status = checkLimit(linAc);
  delay(50);

  // Move the actuator until the limit switch is triggered
  while (limit_status == false) {
    limit_status = checkLimit(linAc);
    linAc.stepper.move(10);
    setMotorSpeed(linAc.home_speed, linAc.stepper);
    linAc.stepper.runSpeed();
  }

  // Move the actuator a little bit (8mm) and set that as the zero position
  long start_loc = 8 * steps_per_mm;
  linAc.stepper.move(start_loc);

  while (linAc.stepper.distanceToGo() != 0) {
    setMotorSpeed(-linAc.home_speed, linAc.stepper);
    linAc.stepper.runSpeed();
  }

  linAc.stepper.setCurrentPosition(0);
  linAc.stepper.move(0);

  // Wait a half second to ensure the limit switch is no longer triggered
  delay(500);

  // Update the limit switch status
  linAc.homed = true;
  String message = "Homed " + linAc.name;
  sendStatus(message);
}

void sendStatus(String &debug_msg) {
  // Send a status message

  status_msg.v_act_enabled = linAc1.enabled;
  status_msg.h_act_enabled = linAc2.enabled;
  status_msg.v_act_homed = linAc1.homed;
  status_msg.h_act_homed = linAc2.homed;

  status_msg.header.stamp = nh.now();

  std_msgs::String ros_msg;
  ros_msg.data = debug_msg.c_str();
  status_msg.debug = ros_msg;

  status.publish(&status_msg);
  nh.spinOnce();
}

void sendPos() {
  // Send a position message in micro meters
  long act1_cur_position = (1000 * linAc1.stepper.currentPosition()) / (steps_per_mm);
  long act2_cur_position = (1000 * linAc2.stepper.currentPosition()) / (steps_per_mm);

  pos_msg.v_act_pos = act1_cur_position;
  pos_msg.h_act_pos = act2_cur_position;
  pos_msg.header.stamp = nh.now();

  actuator_positions.publish(&pos_msg);
}

bool checkLimit(LinearActuator &linAc) {
  // Loop the limit switches
  linAc.limit_switch_nc.loop();
  linAc.limit_switch_no.loop();

  // Update the states
  state_nc = linAc.limit_switch_nc.getStateRaw();
  state_no = linAc.limit_switch_no.getStateRaw();

  // Return true if limit switch is triggered (or disconnected), and false otherwise
  if (state_no == HIGH && state_nc == LOW) {
    return false;
  } else {
    return true;
  }
}
