#include <Wire.h>
#include <Servo.h>
#include <IBusBM.h>
#include <MPU6050_light.h>

struct MPUData {
  float x, y, z;
  float yaw, pitch, roll;
};

// will be created for yaw, pitch, and roll controllers
struct PIDGains {
  float p, i, d;
  float net_error;
  float prev_error;
  float set_point;
  float curr;
};

// global variables
float time_prev, time;

Servo front_right, front_left, back_right, back_left;
Servo motors[4] = {front_left, front_right, back_left, back_right};

MPU6050 mpu(Wire);
MPUData mpu_data;

PIDGains yaw_gains, pitch_gains, roll_gains;

IBusBM ibus;

float thrust_setpoint = 1000;

void config_mpu(MPU6050 *mpu) {
  Wire.begin();
  byte status = mpu->begin();

  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) { } // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu->calcOffsets(true, true); // gyro and accel
  Serial.println("Done!\n");
}

void update_mpu(MPUData *mpu_data, MPU6050 *mpu) {
  mpu_data->pitch = mpu->getAngleX();
  mpu_data->roll = mpu->getAngleY();
  mpu_data->yaw = mpu->getAngleZ();

  /* logic to find x, y, z positions */
}

void update_current_targets(PIDGains *yaw_gains, PIDGains *pitch_gains, PIDGains *roll_gains, MPUData *data) {
  yaw_gains->curr = data->yaw;
  roll_gains->curr = data->roll;
  pitch_gains->curr = data->pitch;
}

float calculate_pid_output(PIDGains *gains, float elapsed_time) {
  float error = gains->set_point - gains->curr;
  float pid_p = gains->p * error;
  float pid_d = gains->d * (error - gains->prev_error) / elapsed_time;
  float pid_i = gains->i * (gains->net_error + error);

  gains->prev_error = error;
  gains->net_error += error;

  return pid_p + pid_d + pid_i;
}

void config_motors(Servo motors[4]) {
  motors[0].attach(1);
  motors[1].attach(2);
  motors[2].attach(3);
  motors[3].attach(4);

  motors[0].writeMicroseconds(1000);
  motors[1].writeMicroseconds(1000);
  motors[2].writeMicroseconds(1000);
  motors[3].writeMicroseconds(1000);
  delay(7000);
}

/*
   Order of motors: front left, front right, back left, back right
*/
void write_to_motors(float thrust, PIDGains *yaw_gains, PIDGains *pitch_gains, PIDGains *roll_gains, Servo motors[4], float elapsed_time) {
  float yaw = calculate_pid_output(yaw_gains, elapsed_time);
  float pitch = calculate_pid_output(pitch_gains, elapsed_time);
  float roll = calculate_pid_output(roll_gains, elapsed_time);

  float front_right = thrust + yaw + pitch + roll;
  float front_left = thrust - yaw + pitch - roll;
  float back_right = thrust - yaw - pitch + roll;
  float back_left = thrust + yaw - pitch - roll;

  motors[0].writeMicroseconds(front_left);
  motors[1].writeMicroseconds(front_right);
  motors[2].writeMicroseconds(back_left);
  motors[3].writeMicroseconds(back_right);
}

int read_channel(int channel_input, int min_limit, int max_limit, int default_value) {
  int ch = ibus.readChannel(channel_input);
  if (ch < 100) return default_value;
  return map(ch, 1000, 2000, min_limit, max_limit);
}

bool read_switch(byte channel_input, bool default_value) {
  int int_default_value = (default_value) ? 100 : 0;
  int ch = read_channel(channel_input, 0, 100, int_default_value);
  return (ch > 50);
}

void update_setpoints(PIDGains *yaw_gains, PIDGains *pitch_gains, PIDGains *roll_gains) {
  float yaw = read_channel(1, 1000, 2000, 1000);
  float pitch = read_channel(2, 1000, 2000, 1000);
  float roll = read_channel(3, 1000, 2000, 1000);

  yaw_gains->set_point = yaw;
  pitch_gains->set_point = pitch;
  roll_gains->set_point = roll;
  thrust_setpoint = read_channel(4, 1000, 2000, 1000);
}

void setup() {
  Serial.begin(115200);

  config_mpu(&mpu);
  config_motors(motors);

  ibus.begin(Serial);
}

/**
   Event loop process:
   1. Update the gyro
   2. Update the setpoints
   3. Update the current targets
   4. Use that and write to the motors
*/
void loop() {
  time_prev = time;
  time = millis();
  float elapsed_time = (time - time_prev) / 1000;

  update_mpu(&mpu_data, &mpu);

  /* RC communications to get setpoints */
  /* Set setpoints in pid gains */
  update_setpoints(&yaw_gains, &pitch_gains, &roll_gains);

  update_current_targets(&yaw_gains, &pitch_gains, &roll_gains, &mpu_data);

  write_to_motors(thrust_setpoint, &yaw_gains, &pitch_gains, &roll_gains, motors, elapsed_time);
}

