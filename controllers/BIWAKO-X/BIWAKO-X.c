/*
 * File:          OKEBOT_control.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <stdio.h>
#include <stdlib.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define MAX_VELOCITY 15.0
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  /* necessary to initialize webots stuff */
  
  // bool log = true;

  wb_robot_init();
  const WbDeviceTag motor1 = wb_robot_get_device("thruster1");
  wb_motor_set_position(motor1, INFINITY);
  const WbDeviceTag motor2 = wb_robot_get_device("thruster2");
  wb_motor_set_position(motor2, INFINITY);
  const WbDeviceTag motor3 = wb_robot_get_device("thruster3");
  wb_motor_set_position(motor3, INFINITY);
  const WbDeviceTag motor4 = wb_robot_get_device("thruster4");
  wb_motor_set_position(motor4, INFINITY);
  
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);
  
  
  while (wb_robot_step(TIME_STEP) != -1) {
    const double *position = wb_gps_get_values(gps);
    const char *latitude = wb_gps_convert_to_degrees_minutes_seconds(position[0]);
    const double altitude = position[2];
    const char *longitude = wb_gps_convert_to_degrees_minutes_seconds(position[1]);
    const double speed = wb_gps_get_speed(gps);
    
    printf("Latitude is: %lfdeg / %s\n", position[0], latitude);
    printf("Longitude is: %lfdeg / %s\n", position[1], longitude);
    printf("Altitude is: %lf [m]\n", altitude);
    printf("Speed is: %lf [m/s]\n", speed);
    printf("\n");

    /* those char * returned by 'wb_gps_convert_to_degrees_minutes_seconds' need to be freed */
    free((void *)latitude);
    free((void *)longitude);
    
    
    wb_motor_set_velocity(motor1, MAX_VELOCITY);
    wb_motor_set_velocity(motor2, MAX_VELOCITY+1);
    wb_motor_set_velocity(motor3, MAX_VELOCITY+1);
    wb_motor_set_velocity(motor4, MAX_VELOCITY);
 
  }
  
  wb_robot_cleanup();

  return 0;
}
