/**

  This is a temporary file that exists as a short term fix for defining what services are called by
  the sensor control plugin. The plugin can not be run from a launch file since it must be compiled
  and run by RVIZ. Due to this I cannot specify service names in a launch file. Eventually I will
  work into the GUI a way to enter the topic names, but for now this is faster.

  **/

#ifndef HUBO_DEFAULTS_
#define HUBO_DEFAULTS_

#define HUBO_JOINT_STATES_SERVICE "joint_states/rate"
#define HUBO_FORCE_SENSOR_SERVICE "force_sensors/rate"
#define HUBO_ACCEL_GRYO_SERVICE "accel_gyro/rate"
#define HUBO_TOUCH_SENSORS_SERVICE "touch_sensors/rate"

#define HUBO_CAMERA_SERVICE "camera/rate"
#define HUBO_PLANAR_SERVICE "planar/rate"

#endif //HUBO_DEFAULTS_
