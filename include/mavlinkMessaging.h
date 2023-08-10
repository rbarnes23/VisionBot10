/*
 * @ Author: Richard Barnes
 * @ Create Time: 2022-05-14 13:41:48
 * @ Modified by: Richard Barnes
 * @ Modified time: 2022-05-16 04:53:59
 * @ Modified time: 2022-06-28 04:50:42
 */

// default sensors are present and healthy: gyro, accelerometer,
// rate_control, attitude_stabilization, yaw_position, altitude control,
// x/y position control, motor_control
#define MAVLINK_SENSOR_PRESENT_DEFAULT (MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_3D_MAG | MAV_SYS_STATUS_AHRS | MAV_SYS_STATUS_GPS)
#define MAV_SYS_STATUS_SENSOR_3D_GYRO 1
#define MAV_SYS_STATUS_SENSOR_3D_ACCEL 2
#define MAV_SYS_STATUS_SENSOR_3D_MAG 4
#define MAV_SYS_STATUS_GPS 32
#define MAV_SYS_STATUS_AHRS 2097152

const uint8_t mvl_chan = MAVLINK_COMM_1; // MAVLink channel 1 appears to be required at least for Blue Robotics QGC
const uint8_t mvl_compid = 1;            // Component ID and System ID identify us to QGroundControl
const uint8_t mvl_sysid = 1;
mavlink_system_t mavlink_system;
// mavlink_system.sysid= 1;//mvl_sysid;
// mavlink_system.compid = 1;//mvl_compid;
uint8_t bufTx[MAVLINK_MAX_PACKET_LEN];
uint8_t bufRx[MAVLINK_MAX_PACKET_LEN];

mavlink_message_t msg;
mavlink_message_t mvl_tx_message; // A special MAVLink message data structure.
mavlink_message_t receivedMsg;
mavlink_status_t mvl_rx_status;
mavlink_message_t heartbeatMsg;
mavlink_message_t gpsMsg;
mavlink_message_t ahrsMsg;
mavlink_message_t paramMsg;
mavlink_message_t rc_overMsg;
mavlink_message_t statMsg;
mavlink_message_t misReq;
mavlink_message_t servoOutputMsg;
mavlink_message_t fsyMsg;
mavlink_message_t globalSetpointMsg;
mavlink_message_t bmpMsg;

typedef struct local_param_set
{
  char param_id[16];  ///< Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
  uint8_t param_type; ///< Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
  float param_value;  ///< Onboard parameter value
} local_params_type;

// Set up rover Parameters
int paramCount = 22;
local_params_type local_param[22] =
    {
        {"CRUISE_SPEED", MAV_PARAM_TYPE_REAL32, 2.0},
        {"CRUISE_THROTTLE", MAV_PARAM_TYPE_REAL32, 95.0},
        {"FS_THR_VALUE", MAV_PARAM_TYPE_INT8, 1},
        {"NAVL1_DAMPING", MAV_PARAM_TYPE_REAL32, 1},
        {"NAVL1_PERIOD", MAV_PARAM_TYPE_INT8, 1},
        {"SONAR_DEBOUNCE", MAV_PARAM_TYPE_INT8, 1},
        {"SONAR_TRIGGER", MAV_PARAM_TYPE_INT8, 1},
        {"SONAR_TURN_TIME", MAV_PARAM_TYPE_REAL32, 1},
        {"SPEED2THR_D", MAV_PARAM_TYPE_REAL32, 1},
        {"SPEED2THR_I", MAV_PARAM_TYPE_REAL32, 1},
        {"SPEED2THR_P", MAV_PARAM_TYPE_REAL32, 1},
        {"SPEED2THR_IMAX", MAV_PARAM_TYPE_UINT16, 1},
        {"SPEED_TURN_DIST", MAV_PARAM_TYPE_REAL32, .8},
        {"SPEED_TURN_GAIN", MAV_PARAM_TYPE_INT8, 1},
        {"STEER2SRV_D", MAV_PARAM_TYPE_REAL32, 1},
        {"STEER2SRV_I", MAV_PARAM_TYPE_REAL32, 1},
        {"SPEED2SRV_P", MAV_PARAM_TYPE_REAL32, 1},
        {"THR_MIN", MAV_PARAM_TYPE_REAL32, 1},
        {"WP_RADIUS", MAV_PARAM_TYPE_REAL32, .5},
        {"ARMING_REQUIRE", MAV_PARAM_TYPE_UINT8, 0},
        {"INITIAL_MODE", MAV_PARAM_TYPE_UINT8, 220},
        {"ARMING_CHECK", MAV_PARAM_TYPE_UINT8, 0}};

uint32_t t_last_sys_stat = 0;
int16_t sys_stat_count = 0;
uint8_t mvl_armed = 1;
// uint8_t mvl_packet_received = 0;
uint8_t wp_count;
void send_udp_packet(uint8_t bufTx[MAVLINK_MAX_PACKET_LEN]);
void send_home_position(float lat, float lng, float alt);
void sendSystemStatus(mavlink_sys_status_t sys_status);

char rxBuffer[255];

/*Begin Transmit Functions*/

/*==============================================================
 * send_udp_packet(uint8_t bufTx[MAVLINK_MAX_PACKET_LEN])
 * Transmits udp packets to Qgroundcontrol
 * This function is used when messages are "packed" vs "encoded"
 * Moving all to encoded messages long term as they allow channels etc easier.
 *==============================================================*/
void send_udp_packet(uint8_t bufTx[MAVLINK_MAX_PACKET_LEN])
{
  //if (connected)
  //{
    //xSemaphoreTake(udpMutex, pdMS_TO_TICKS(5));
    //udpGC.beginPacket(udpAddress, udpPortGC);
    //udpGC.write(bufTx, sizeof(bufTx));
     telem.write(bufTx, sizeof(bufTx));
    //udpGC.endPacket();
    //xSemaphoreGive(udpMutex);
  //}
}

/*==============================================================
 * void MVL_Transmit_Message(mavlink_message_t *mvl_msg_ptr)
 * Transmits udp packets to Qgroundcontrol
 * This function is used when messages are "encoded" vs "packets"
 * Encoded messages are better as the message itself is passed and channels etc are easily added
 *==============================================================*/
void MVL_Transmit_Message(mavlink_message_t *mvl_msg_ptr)
{
  //if (connected)
  //{
    //xSemaphoreTake(udpMutex, pdMS_TO_TICKS(15));
    byte tx_byte_buffer[512] = {0}; // A byte buffer that will be sent from the serial port.
    uint16_t tx_buflen = mavlink_msg_to_send_buffer(tx_byte_buffer, mvl_msg_ptr);
    telem.write(tx_byte_buffer, tx_buflen);
    //udpGC.beginPacket(udpAddress, udpPortGC);
    //udpGC.write(tx_byte_buffer, tx_buflen);
    //udpGC.endPacket();
    //xSemaphoreGive(udpMutex);
  //}
}

void heartbeat(void *arg)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 500;

  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    mavlink_heartbeat_t mvl_hb;               // struct with user fields: uint32_t custom_mode, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint8_t system_status;
    mvl_hb.type = MAV_TYPE_GROUND_ROVER;      // My vehicle is an underwater ROV. Change as appropriate. See: https://github.com/mavlink/c_library_v2/blob/748192f661d0df3763501cfc432861d981952921/common/common.h#L69
    mvl_hb.autopilot = MAV_AUTOPILOT_GENERIC; // See https://github.com/mavlink/c_library_v2/blob/748192f661d0df3763501cfc432861d981952921/common/common.h#L40
    mvl_hb.system_status = MAV_STATE_ACTIVE;
    mvl_hb.mavlink_version = 2;
    if (mvl_armed)
    {
      mvl_hb.base_mode = MAV_MODE_MANUAL_ARMED;
    }
    else
    {
      mvl_hb.base_mode = MAV_MODE_MANUAL_DISARMED;
    }
    mvl_hb.base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; // I always use CUSTOM_MODE_ENABLED
    mvl_hb.custom_mode = 2323;                             // custom mode, can be anything, I guess
    mavlink_msg_heartbeat_encode_chan(mvl_sysid, mvl_compid, mvl_chan,
                                      &mvl_tx_message, &mvl_hb);
    MVL_Transmit_Message(&mvl_tx_message);
    /*
    mavlink_sys_status_t sys_status;
    sys_status.battery_remaining=90;
    sys_status.onboard_control_sensors_enabled=1;
    sys_status.onboard_control_sensors_health= 1;
    sys_status.onboard_control_sensors_present=MAVLINK_SENSOR_PRESENT_DEFAULT;
    sys_status.voltage_battery=7400;

    sendSystemStatus(sys_status);*/

  }
}

/*==============================================================
 * send_parameters(void *arg)
 * Qgroundcontrol uses a variety of parameters to perform various functions
 *==============================================================*/
void send_parameters(void *arg)
{
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 5000;

  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    // Wait for the next cycle.
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    mavlink_param_value_t mvl_param[paramCount];

    for (int i = 0; i <= paramCount; i++)
    {
      mvl_param[i].param_count = paramCount;
      strncpy(mvl_param[i].param_id, local_param[i].param_id, sizeof(mvl_param[i].param_id));
      mvl_param[i].param_index = i;
      mvl_param[i].param_type = local_param[i].param_type;
      mvl_param[i].param_value = local_param[i].param_value;
      mavlink_msg_param_value_encode_chan(mvl_sysid, mvl_compid, mvl_chan,
                                          &mvl_tx_message, &mvl_param[i]);
      MVL_Transmit_Message(&mvl_tx_message);
      //vTaskDelay(pdMS_TO_TICKS(20));
    }

    /*float lat = 28.2594760;
    float lng = -82.2845242;
    float alt = 12.2;
    send_home_position(lat, lng, alt);*/

    //vTaskDelay(pdMS_TO_TICKS(100));
  }
  vTaskDelete(NULL);
}

void send_servo_output_raw()
{
  memset(bufTx, mavlink_system.sysid, sizeof(bufTx));
  //  mavlink_msg_servo_output_raw_pack(mavlink_system.sysid, mavlink_system.compid, &servoOutputMsg,
  //                   bootTime,0, uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw, uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw, uint16_t servo7_raw, uint16_t servo8_raw);
  // mavlink_msg_servo_output_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t *msg, uint32_t time_usec, uint8_t port, uint16_t servo1_raw, uint16_t servo2_raw, uint16_t servo3_raw, uint16_t servo4_raw, uint16_t servo5_raw, uint16_t servo6_raw, uint16_t servo7_raw, uint16_t servo8_raw, uint16_t servo9_raw, uint16_t servo10_raw, uint16_t servo11_raw, uint16_t servo12_raw, uint16_t servo13_raw, uint16_t servo14_raw, uint16_t servo15_raw, uint16_t servo16_raw)

  mavlink_msg_servo_output_raw_pack(mvl_sysid, mvl_compid, &servoOutputMsg,
                                    bootTime, 0, (int)abs(lr.L * 1e3), (int)abs(lr.R * 1e3), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  /// Copy the message to send buffer
  uint16_t len = mavlink_msg_to_send_buffer(bufTx, &servoOutputMsg);
  // Write Message
  // telem.write(bufTx, len);
  send_udp_packet(bufTx);
}

// int no = 0;

/*==============================================================
 * void sendMissionAck(mavlink_mission_item_t mission_item)
 * Transmits Mission Acknowlegement to Qgroundcontrol
 * Message 47
 *==============================================================*/
void sendMissionAck(mavlink_mission_ack_t mission_ack)
{
  mavlink_msg_mission_ack_encode_chan(mvl_sysid, mvl_compid, mvl_chan, &mvl_tx_message, &mission_ack);
  MVL_Transmit_Message(&mvl_tx_message);
}

/*==============================================================
 * void sendMissionItem(mavlink_mission_item_t mission_item)
 * Transmits Mission Item to Qgroundcontrol
 * Message 39
 *==============================================================*/
void sendMissionItem(mavlink_mission_item_t mission_item)
{
  mavlink_msg_mission_item_encode_chan(mvl_sysid, mvl_compid, mvl_chan, &mvl_tx_message, &mission_item);
  MVL_Transmit_Message(&mvl_tx_message);
}

/*==============================================================
 * void sendMissionItemInt(mavlink_mission_item_int_t mission_item)
 * Transmits Mission Item Int to Qgroundcontrol
 * Message 73
 *==============================================================*/
void sendMissionItemInt(mavlink_mission_item_int_t mission_item)
{
  mavlink_msg_mission_item_int_encode_chan(mvl_sysid, mvl_compid, mvl_chan, &mvl_tx_message, &mission_item);
  MVL_Transmit_Message(&mvl_tx_message);
}

/*==============================================================
 * void sendMissionRequestList(mavlink_mission_item_int_t mission_item)
 * Transmits Mission Request to Qgroundcontrol
 * Message 43
 *==============================================================*/
void sendMissionRequestList(mavlink_mission_request_list_t mission_request_list)
{
  mavlink_msg_mission_request_list_encode_chan(mvl_sysid, mvl_compid, mvl_chan, &mvl_tx_message, &mission_request_list);
  MVL_Transmit_Message(&mvl_tx_message);
}

/*==============================================================
 * send_home_position(mavlink_home_position_t home_position)
 * Transmits Home Position to Qgroundcontrol
 *==============================================================*/
void send_home_position(mavlink_home_position_t home_position)
{
  mavlink_msg_home_position_encode_chan(mvl_sysid, mvl_compid, mvl_chan, &mvl_tx_message, &home_position);
  MVL_Transmit_Message(&mvl_tx_message);
}

/*==============================================================
 * send_home_position(mavlink_home_position_t home_position)
 * Transmits Home Position to Qgroundcontrol
 *==============================================================*/
void send_waypoints(mavlink_home_position_t home_position)
{
  mavlink_msg_home_position_encode_chan(mvl_sysid, mvl_compid, mvl_chan, &mvl_tx_message, &home_position);
  MVL_Transmit_Message(&mvl_tx_message);
}

/*==============================================================
 * send_servo_output_raw(mavlink_servo_output_raw_t sor)
 * Transmits Servo ouput from Flysky to Qgroundcontrol
 *==============================================================*/
void send_servo_output_raw(mavlink_servo_output_raw_t sor)
{
  mavlink_msg_servo_output_raw_encode_chan(mvl_sysid, mvl_compid, mvl_chan, &mvl_tx_message, &sor);
  MVL_Transmit_Message(&mvl_tx_message);
}

/*==============================================================
 * send_gps_msg(mavlink_gps_raw_int_t gpsMav)
 * Transmits location information to Qgroundcontrol
 *==============================================================*/
void send_gps_msg(mavlink_gps_raw_int_t gpsMav)
{
  mavlink_msg_gps_raw_int_encode_chan(mvl_sysid, mvl_compid, mvl_chan, &mvl_tx_message, &gpsMav);
  // if(xQueueSend(queueTransmit, &mvl_tx_message, 0) != pdPASS){
  // Serial.println("no queue space.");
  //}
  MVL_Transmit_Message(&mvl_tx_message);
}
/*==============================================================
 * void send_attitude_msg(mavlink_attitude_t attMav)
 * Transmits Attitude to Qgroundcontrol
 *==============================================================*/
void send_attitude_msg(mavlink_attitude_t attMav)
{
  mavlink_msg_attitude_encode_chan(mvl_sysid, mvl_compid, mvl_chan, &mvl_tx_message, &attMav);
  MVL_Transmit_Message(&mvl_tx_message);
}

/*==============================================================
 * void sendAttitudeQuaternion()
 * Transmits Attitude Quaternion to Qgroundcontrol
 * Message 61
 *==============================================================*/
void sendAttitudeQuaternion(mavlink_attitude_quaternion_t mvl_att_quat)
{
  mavlink_msg_attitude_quaternion_encode_chan(mvl_sysid, mvl_compid, mvl_chan, &mvl_tx_message, &mvl_att_quat);
  MVL_Transmit_Message(&mvl_tx_message);
}

/*==============================================================
 * void send_raw_pressure_msg(mavlink_raw_pressure_t raw_pressure)
 * Transmits raw barometric pressure to Qgroundcontrol
 * Message 28
 *==============================================================*/
void send_raw_pressure_msg(mavlink_raw_pressure_t raw_pressure)
{
  mavlink_msg_raw_pressure_encode_chan(mvl_sysid, mvl_compid, mvl_chan, &mvl_tx_message, &raw_pressure);
  MVL_Transmit_Message(&mvl_tx_message);
}

/*==============================================================
 * void sendSystemStatus(mavlink_sys_status_t sys_status)
 * Transmits System Status to Qgroundcontrol
 * Message 1
 *==============================================================*/
void sendSystemStatus(mavlink_sys_status_t sys_status)
{
  mavlink_msg_sys_status_encode_chan(mvl_sysid, mvl_compid, mvl_chan, &mvl_tx_message, &sys_status);
  MVL_Transmit_Message(&mvl_tx_message);
}

/*
void send_raw_pressure_msg()
{
  memset(bufTx, mavlink_system.sysid, sizeof(bufTx));
  // mavlink_msg_raw_pressure_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
  //                              uint64_t time_usec, int16_t press_abs, int16_t press_diff1, int16_t press_diff2, int16_t temperature);
  // mavlink_msg_raw_pressure_pack(mavlink_system.sysid, mavlink_system.compid, &bmpMsg,
  //                           bootTime, rPressure.pressure, 0 , 0,rPressure.temperature);
  mavlink_msg_raw_pressure_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                                bootTime, (int16_t)(rPressure.pressure / 100), 0, 0, (int16_t)rPressure.temperature);

  // cout.println("X0: " + String(flysky_msg.x0) + " Y1: " + String(flysky_msg.y1));
  // cout.println("LeftDir: " + String(md.ldir) + " LeftSp: " + String(md.lspeed)+ " RightDir: " + String(md.rdir) + " RightSp: " + String(md.rspeed));

  /// Copy the message to send buffer
  uint16_t len = mavlink_msg_to_send_buffer(bufTx, &msg);

  // Write Message
  // telem.write(bufTx, len);
}
*/
/*void send_fsy_msg()
{
  memset(bufTx, mavlink_system.sysid, sizeof(bufTx));
  mavlink_msg_rc_channels_raw_pack(mavlink_system.sysid, mavlink_system.compid, &fsyMsg,
                                   bootTime, 0, flysky_msg.x0, flysky_msg.y1, flysky_msg.y3, flysky_msg.x2,
                                   flysky_msg.swa4, flysky_msg.swb5, flysky_msg.swc6, flysky_msg.swd7, 100);

  cout.println("X0: " + String(flysky_msg.x0) + " Y1: " + String(flysky_msg.y1));
  // cout.println("LeftDir: " + String(md.ldir) + " LeftSp: " + String(md.lspeed)+ " RightDir: " + String(md.rdir) + " RightSp: " + String(md.rspeed));

  /// Copy the message to send buffer
  uint16_t len = mavlink_msg_to_send_buffer(bufTx, &fsyMsg);

  // Write Message
  //telem.write(bufTx, len);
}*/
void send_rc_channel_msg(mavlink_rc_channels_t mvl_rc)
{
  mavlink_msg_rc_channels_encode_chan(mvl_sysid, mvl_compid, mvl_chan, &mvl_tx_message, &mvl_rc);
  MVL_Transmit_Message(&mvl_tx_message);
}

void send_status()
{
  memset(bufTx, mavlink_system.sysid, sizeof(bufTx));
  mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &statMsg,
                              MAVLINK_SENSOR_PRESENT_DEFAULT, MAVLINK_SENSOR_PRESENT_DEFAULT,
                              MAVLINK_SENSOR_PRESENT_DEFAULT, 500, 7400, 330, 50, 0, 0, 0, 0, 0, 0);

  /// Copy the message to send buffer
  uint16_t len = mavlink_msg_to_send_buffer(bufTx, &statMsg);

  // Write Message
  // telem.write(bufTx, len);
}

void send_mission_ack()
{
  memset(bufTx, mavlink_system.sysid, sizeof(bufTx));
  // mavlink_msg_mission_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t *msg, uint8_t target_system, uint8_t target_component, uint8_t type, uint8_t mission_type)
  mavlink_msg_mission_ack_pack(mavlink_system.sysid, mavlink_system.compid, &msg, mavlink_system.sysid, mavlink_system.compid, MAV_TYPE_GROUND_ROVER, MAV_MISSION_TYPE_MISSION);
  /// Copy the message to send buffer
  uint16_t len = mavlink_msg_to_send_buffer(bufTx, &msg);
  // Write Message
  // telem.write(bufTx, len);
  send_udp_packet(bufTx);
}

void send_mission_req()
{
  for (uint16_t i = 0; i < wp_count; i++)
  {
    memset(bufTx, mavlink_system.sysid, sizeof(bufTx));
    // mavlink_msg_mission_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t *msg, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t mission_type)
    mavlink_msg_mission_request_pack(mavlink_system.sysid, mavlink_system.compid, &misReq, mavlink_system.sysid, mavlink_system.compid, i, MAV_MISSION_TYPE_MISSION);
    /// Copy the message to send buffer
    uint16_t len = mavlink_msg_to_send_buffer(bufTx, &misReq);
    // Write Message
    // telem.write(bufTx, len);
    send_udp_packet(bufTx);
  }
}

void mav_arm_pack(boolean state)
{
  // mavlink_message_t msg;
  // uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  memset(bufTx, mavlink_system.sysid, sizeof(bufTx));

  // Arm the drone
  // 400 stands for MAV_CMD_COMPONENT_ARM_DISARM
  //  1 an 8'th argument is for ARM (0 for DISARM)
  if (state)
  {
    // ARM
    // mavlink_msg_command_long_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 1, 1, 400, 1, 1.0, 0, 0, 0, 0, 0, 0);
    mavlink_msg_command_long_pack(mavlink_system.sysid, mavlink_system.compid, &msg, mavlink_system.sysid, mavlink_system.compid, 400, 1, 1, 0, 0, 0, 0, 0, 0);
  }
  else
  {
    // DISARM
    // mavlink_msg_command_long_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 1, 1, 400, 1, 0.0, 0, 0, 0, 0, 0, 0);
    mavlink_msg_command_long_pack(mavlink_system.sysid, mavlink_system.compid, &msg, mavlink_system.sysid, mavlink_system.compid, 400, 1, 0, 0, 0, 0, 0, 0, 0);
  }
  uint16_t len = mavlink_msg_to_send_buffer(bufTx, &msg);
  send_udp_packet(bufTx);
  // telem.write(bufTx, len);
}

void create_home()
{
  // Step 3 of uploading a new waypoint (send HOME coordinates)
  // uint8_t _system_id = 255;      // system id of sending station. 255 is Ground control software
  // uint8_t _component_id = 2;     // component id of sending station 2 works fine
  uint8_t _target_system = 1;    // Pixhawk id
  uint8_t _target_component = 0; // Pixhawk component id, 0 = all (seems to work fine)

  uint16_t seq = 0;                        // Sequence number
  uint8_t frame = 0;                       // Set target frame to global default
  uint16_t command = MAV_CMD_NAV_WAYPOINT; // Can be substituted with other commands https://mavlink.io/en/services/mission.html#mission_types
  uint8_t current = 0;                     // false:0, true:1 - When downloading, whether the item is the current mission item.
  uint8_t autocontinue = 0;                // Always 0
  float param1 = 0;                        // Loiter time
  float param2 = 0;                        // Acceptable range from target - radius in meters
  float param3 = 0;                        // Pass through waypoint
  float param4 = 0;                        // Desired yaw angle
  float x = 45.464217;                     // Latitude - degrees
  float y = -1.280222;                     // Longitude - degrees
  float z = 200;                           // Altitude - meters

  // Initialize the required buffers
  mavlink_message_t msg;
  memset(bufTx, mavlink_system.sysid, sizeof(bufTx));

  // Pack the message
  mavlink_msg_mission_item_pack(mavlink_system.sysid, mavlink_system.compid, &msg, _target_system, _target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z, MAV_MISSION_TYPE_MISSION);

  // uint16_t mavlink_msg_mission_item_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(bufTx, &msg);

  // Send the message (.write sends as bytes)
  // Serial1.write(buf, len);
  send_udp_packet(bufTx);
}

void create_waypoint()
{
  // Step 3 continuation of uploading a new waypoint (send 1st coordinates)
  // uint8_t _system_id = 255;      // system id of sending station. 255 is Ground control software
  // uint8_t _component_id = 2;     // component id of sending station 2 works fine
  uint8_t _target_system = 1;    // Pixhawk id
  uint8_t _target_component = 0; // Pixhawk component id, 0 = all (seems to work fine)

  uint16_t seq = 1;                        // Sequence number
  uint8_t frame = 0;                       // Set target frame to global default
  uint16_t command = MAV_CMD_NAV_WAYPOINT; // Can be substituted with other commands https://mavlink.io/en/services/mission.html#mission_types
  uint8_t current = 1;                     // false:0, true:1 - When downloading, whether the item is the current mission item.
  uint8_t autocontinue = 0;                // Always 0
  float param1 = 0;                        // Loiter time
  float param2 = 0;                        // Acceptable range from target - radius in meters
  float param3 = 0;                        // Pass through waypoint
  float param4 = 0;                        // Desired yaw angle
  float x = 15.464217;                     // Latitude - degrees
  float y = -11.280222;                    // Longitude - degrees
  float z = 200;                           // Altitude - meters

  // Initialize the required buffers
  mavlink_message_t msg;
  memset(bufTx, mvl_sysid, sizeof(bufTx));

  // Pack the message
  mavlink_msg_mission_item_pack(mavlink_system.sysid, mavlink_system.compid, &msg, _target_system, _target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z, MAV_MISSION_TYPE_MISSION);

  // uint16_t mavlink_msg_mission_item_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(bufTx, &msg);

  // Send the message (.write sends as bytes)
  // Serial1.write(buf, len);
  send_udp_packet(bufTx);
}

void send_home_position(float lat, float lng, float alt)
{
  // Step 3 of uploading a new waypoint (send HOME coordinates)
  // uint8_t _system_id = 255; // system id of sending station. 255 is Ground control software
  // uint8_t _component_id = 20; // component id of sending station 2 works fine
  uint8_t _target_system = 1;    // Pixhawk id
  uint8_t _target_component = 0; // Pixhawk component id, 0 = all (seems to work fine)

  uint16_t seq = 0;                        // Sequence number
  uint8_t frame = 0;                       // Set target frame to global default
  uint16_t command = MAV_CMD_NAV_WAYPOINT; // Can be substituted with other commands https://mavlink.io/en/services/mission.html#mission_types
  uint8_t current = 0;                     // Guided mode waypoint
  uint8_t autocontinue = 0;                // Always 0
  float param1 = 0;                        // Loiter time
  float param2 = 1;                        // Acceptable range from target - radius in meters
  float param3 = 0;                        // Pass through waypoint
  float param4 = 0;                        // Desired yaw angle
  int32_t x = (int32_t)(lat * 1.e7);       // Latitude - degrees
  int32_t y = (int32_t)(lng * 1.e7);       // Longitude - degrees
  float z = alt;                           // Altitude - meters

  mavlink_mission_item_int_t mission_item_int;
  mission_item_int.autocontinue = 0;
  mission_item_int.target_system = mvl_sysid;
  mission_item_int.target_component = mvl_compid;
  mission_item_int.command = command;
  mission_item_int.current = current;
  mission_item_int.mission_type = MAV_MISSION_TYPE_MISSION;
  mission_item_int.param1 = 0;
  mission_item_int.param2 = 0;
  mission_item_int.param3 = 0;
  mission_item_int.param4 = 0;
  mission_item_int.seq = 0;
  mission_item_int.x = x;
  mission_item_int.y = y;
  mission_item_int.z = z;

  mavlink_msg_mission_item_int_encode_chan(mvl_sysid, mvl_compid, mvl_chan, &mvl_tx_message, &mission_item_int);

  MVL_Transmit_Message(&mvl_tx_message);
}
/*
void create_waypoint()
{
  // Step 3 continuation of uploading a new waypoint (send 1st coordinates)
  // uint8_t _system_id = 255; // system id of sending station. 255 is Ground control software
  // uint8_t _component_id = 20; // component id of sending station 2 works fine
  uint8_t _target_system = 1;    // Pixhawk id
  uint8_t _target_component = 0; // Pixhawk component id, 0 = all (seems to work fine)

  uint16_t seq = 1;                        // Sequence number
  uint8_t frame = 0;                       // Set target frame to global default
  uint16_t command = MAV_CMD_NAV_WAYPOINT; // Can be substituted with other commands https://mavlink.io/en/services/mission.html#mission_types
  uint8_t current = 1;                     // Guided mode waypoint
  uint8_t autocontinue = 0;                // Always 0
  float param1 = 0;                        // Loiter time
  float param2 = 0;                        // Acceptable range from target - radius in meters
  float param3 = 0;                        // Pass through waypoint
  float param4 = 0;                        // Desired yaw angle
  float x = 282594760;                     // 15.464217; // Latitude - degrees
  float y = -822845242;                    //-11.280222; // Longitude - degrees
  float z = 200;                           // Altitude - meters

  // Initialize the required buffers
  mavlink_message_t msg;
  // uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  memset(bufTx, mavlink_system.sysid, sizeof(bufTx));
  // Pack the message
  mavlink_msg_mission_item_pack(mavlink_system.sysid, mavlink_system.compid, &msg, _target_system, _target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z, MAV_MISSION_TYPE_MISSION);

  // uint16_t mavlink_msg_mission_item_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(bufTx, &msg);

  // Send the message (.write sends as bytes)
  //telem.write(bufTx, len);
  MVL_Transmit_Message(&mvl_tx_message);
}
*/
void mav_set_mode(String value)
{
  /*  mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    value.trim();

    // SET_MODE
    // Works with 1 at 4'th parameter
    if (value == STABILIZE)
    {
      mavlink_msg_set_mode_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 1, 209, 0);
    }

    if (value == ALTHOLD)
    {
      mavlink_msg_set_mode_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 1, 209, 2);
    }

    if (value == LOITER)
    {
      mavlink_msg_set_mode_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 1, 209, 5);
    }

    if (value == AUTO)
    {
      mavlink_msg_set_mode_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 1, 209, 3);
    }

    if (value == CIRCLE)
    {
      mavlink_msg_set_mode_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 1, 209, 7);
    }

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    telem.write(buf, len);*/
}

void mav_set_mode()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Set flight mode 'Stabilize'
  mavlink_msg_set_mode_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 1, 209, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // telem.write(buf,len);
}

void mavlink_send_waypoints(void)
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t MAX_WAYPOINTS = 10;
  mavlink_message_t msg;

  // mavlink_msg_mission_clear_all_pack(mavlink_system.sysid, mavlink_system.compid, &msg, mavlink_system.sysid, mavlink_system.compid, MAV_MISSION_TYPE_MISSION);
  // uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // telem.write(buf, len);
  //  mavlink_send_message(&msg);
  delay(100);
  struct WP
  {
    uint16_t p_lat;
    uint16_t p_lng;
    uint16_t p_alt;
  };
  struct WP wp[100] = {};
  wp[0].p_lat = 28.259476;
  wp[0].p_lng = -82.2845242;
  wp[0].p_alt = 35.9;

  wp[1].p_lat = 28.2596;
  wp[1].p_lng = -82.2846;
  wp[1].p_alt = 35.9;

  wp[2].p_lat = 28.258;
  wp[2].p_lng = -82.284;
  wp[2].p_alt = 35.9;

  uint16_t n = 0;
  for (n = 0; n < MAX_WAYPOINTS; n++)
  {
    if (wp[n].p_lat == 0.0)
    {
      break;
    }
  }
  // if (ModelData.teletype == TELETYPE_MEGAPIRATE_NG || ModelData.teletype == TELETYPE_ARDUPILOT) {
  // SDL_Log("mavlink: WORKAROUND: MEGAPIRATE_NG: fake one WP\n");
  // n++;
  //}
  // SDL_Log("mavlink: sending Waypoints (%i)\n", n - 1);
  mavlink_msg_mission_count_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 1, 0, (n - 1), MAV_MISSION_TYPE_MISSION);
  // mavlink_send_message(&msg);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // telem.write(buf, len);
}

void mission_count()
{
  // Step #1 of uploading a new waypoint
  // uint8_t _system_id = 255;      // system id of sending station. 255 is Ground control software
  // uint8_t _component_id = 2;     // component id of sending station 2 works fine
  uint8_t _target_system = 1;    // Pixhawk id
  uint8_t _target_component = 0; // Pixhawk component id, 0 = all (seems to work fine)

  uint16_t count = 2; // How many items to upload (HOME coordinates are always the first way-point)

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_mission_count_pack(mavlink_system.sysid, mavlink_system.compid, &msg, _target_system, _target_component, count, MAV_MISSION_TYPE_ALL);
  // uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, uint16_t count

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // cout.println("MR"+ String(jk)+ " "+String(len)+" "+String(MAVLINK_MAX_PACKET_LEN));
  //  Send the message (.write sends as bytes)
  //.write(buf, len);
}

void MVL_Handle_Manual_Control(mavlink_message_t *mvl_msg_ptr)
{
  mavlink_manual_control_t mvl_joy; // manual control data structure into which we decode the message
  mavlink_msg_manual_control_decode(mvl_msg_ptr, &mvl_joy);
  // For now, let's just retransmit the manual control message to see it in MAVLink Inspector
  mavlink_msg_manual_control_encode_chan(mvl_sysid, mvl_compid, mvl_chan,
                                         &mvl_tx_message, &mvl_joy);
  MVL_Transmit_Message(&mvl_tx_message);
}

void MVL_Handle_Param_Request_List(mavlink_message_t *mvl_msg_ptr)
{
  mavlink_param_value_t mvl_param;

  mvl_param.param_id[0] = 'a'; // a parameter ID string, less than 16 characters.
  mvl_param.param_id[1] = '_';
  mvl_param.param_id[2] = 'p';
  mvl_param.param_id[3] = 'a';
  mvl_param.param_id[4] = 'r';
  mvl_param.param_id[5] = 'm';
  mvl_param.param_id[6] = 0;                    // null terminated
  mvl_param.param_value = 123.456;              // the parameter value as a float
  mvl_param.param_type = MAV_PARAM_TYPE_REAL32; // https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE
  mvl_param.param_count = 1;                    // We have just one parameter to send.
  mvl_param.param_index = 0;
  mavlink_msg_param_value_encode_chan(mvl_sysid, mvl_compid, mvl_chan,
                                      &mvl_tx_message, &mvl_param);
  MVL_Transmit_Message(&mvl_tx_message);
}

void MVL_Handle_Command_Long(mavlink_message_t *mvl_msg_ptr)
{
  mavlink_command_long_t mvl_cmd;
  mavlink_msg_command_long_decode(mvl_msg_ptr, &mvl_cmd);
  switch (mvl_cmd.command)
  {
  case (MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES): // 520
  {
    if (1 == mvl_cmd.param1)
    {
      mavlink_autopilot_version_t mvl_apv;
      // https: // mavlink.io/en/messages/common.html#AUTOPILOT_VERSION
      mvl_apv.flight_sw_version = 2;
      mvl_apv.middleware_sw_version = 1;
      mvl_apv.board_version = 1;
      mvl_apv.vendor_id = 10101;
      mvl_apv.product_id = 20202;
      mvl_apv.uid = 0;
      mvl_apv.capabilities = 0;                                            // See: https://mavlink.io/en/messages/common.html#MAV_PROTOCOL_CAPABILITY
      mvl_apv.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET; // Just as an example, code does not support! https://mavlink.io/en/messages/common.html#MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET
      mvl_apv.capabilities |= MAV_PROTOCOL_CAPABILITY_MAVLINK2;
      mavlink_msg_autopilot_version_encode_chan(mvl_sysid, mvl_compid, mvl_chan,
                                                &mvl_tx_message, &mvl_apv);
      MVL_Transmit_Message(&mvl_tx_message);
    }
    break;
  }                                    // end handling of autopilot capabilities request
  case (MAV_CMD_COMPONENT_ARM_DISARM): // 400
  {
    if (1 == mvl_cmd.param1)
    {
      mvl_armed = 1;
    }
    else
    {
      mvl_armed = 0;
    }
    // Acknowledge the arm/disarm command.
    mavlink_command_ack_t mvl_ack;                  // https://mavlink.io/en/messages/common.html#COMMAND_ACK
    mvl_ack.command = MAV_CMD_COMPONENT_ARM_DISARM; // https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
    mvl_ack.result = MAV_RESULT_ACCEPTED;           // https://mavlink.io/en/messages/common.html#MAV_RESULT
    mavlink_msg_command_ack_encode_chan(mvl_sysid, mvl_compid, mvl_chan,
                                        &mvl_tx_message, &mvl_ack);
    // skipped setting several fields here, with unknown consequences.
    MVL_Transmit_Message(&mvl_tx_message);
    break;
  } // end handling of arm/disarm command
  } // end switch/case
} // end MVL_Handle_Command_Long()

void MVL_Handle_Mission_Request_List(mavlink_message_t *mvl_msg_ptr)
{
  mavlink_mission_count_t mvl_mc;
  mvl_mc.target_system = mvl_sysid;
  mvl_mc.target_component = mvl_compid;
  mvl_mc.count = 0;
  mavlink_msg_mission_count_encode_chan(mvl_sysid, mvl_compid, mvl_chan,
                                        &mvl_tx_message, &mvl_mc);

  MVL_Transmit_Message(&mvl_tx_message);
}

/* ======================== Serial MAVLink Message Reception ====================
 * If bytes come in on the serial port, we feed them to mavlink_parse_char().
 * This helper function keeps track of incoming bytes and alerts us when it's
 * received a complete, valid MAVLink message.
 * See https://github.com/mavlink/c_library_v2/blob/master/mavlink_helpers.h#L966
 *==============================================================================*/
//******************************************************
//  Read Mavlink Message from GCS
//******************************************************



