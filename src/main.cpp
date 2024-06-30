#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <cybergear_driver.h>

// FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;
uint8_t CYBERGEAR_CAN_ID = 0x7E;
uint8_t CYBERGEAR2_CAN_ID = 0x7F;
uint8_t MASTER_CAN_ID = 0x00;
CyberGearDriver cybergear1 = CyberGearDriver(CYBERGEAR_CAN_ID, MASTER_CAN_ID);
CyberGearDriver cybergear2 = CyberGearDriver(CYBERGEAR2_CAN_ID, MASTER_CAN_ID);
CyberGearStatus motor_status1;
CyberGearStatus motor_status2;
float current[2]{};
const float MAX_CURRENT = 3;
const float LEADER_FOLLOWER_GAIN = 7;
// put function declarations here:

CAN_message_t msg;
void setup() {
  Serial.begin(115200); delay(400);
  // put your setup code here, to run once:
  cybergear1.init_can();
  delay(1);
  cybergear1.init_motor(MODE_POSITION);
  delay(1);
  // cybergear1.init_motor(MODE_CURRENT);
  cybergear1.set_limit_speed(10.0f); /* set the maximum speed of the motor */
  delay(1);
  cybergear1.set_limit_current(1.0); /* current limit allows faster operation */
  delay(1);
  // can3.begin();
  // can3.setBaudRate(1000000);
  // cybergear2.init_can();
  cybergear2.init_motor(MODE_POSITION);
  delay(1);
  cybergear2.set_limit_speed(10.0f); /* set the maximum speed of the motor */
  delay(1);
  cybergear2.set_limit_current(1.0); /* current limit allows faster operation */
  delay(1);
  cybergear1.enable_motor();
  delay(1);
  cybergear2.enable_motor();
  delay(1);

  cybergear1.set_position(0);

  delay(1);
  cybergear2.set_position(0);

  delay(1000);

  cybergear1.stop_motor();
  delay(1);
  cybergear2.stop_motor();
  delay(1);

  cybergear1.init_motor(MODE_CURRENT);
  delay(1);
  cybergear2.init_motor(MODE_CURRENT);
  



}

void loop() {
  // static int count = 0;
  static bool flag = false;
  // count++;
  //  String str = String(Serial.read());
  char str;
  if (Serial.available())
  {
    // Serial.println(Serial.read());
  // }
  str = Serial.read();
  Serial.print(str);
  }
  // String str = Serial.readString();
  // str=String(str);

  
  //  static float pos = 0;
  // if(count%10 == 0){
  cybergear1.request_status();
  cybergear2.request_status();
  // }

   if(str == 'e') {
    Serial.print(" enable ");
    // cybergear1.init_motor(MODE_POSITION);
    // cybergear2.init_motor(MODE_POSITION);
    cybergear1.enable_motor();
    cybergear2.enable_motor();
    flag = true;

  }else if(str == 'd') {
    // pos= 0;
    cybergear1.stop_motor();
    cybergear2.stop_motor();
    flag = false;
  } 
  // else if(str == 'k') {
  //   pos--;
  //   cybergear1.set_position(pos);
  //   cybergear2.set_position(pos);
  // } else if(str == 'l') {
  //   pos++;
  //   cybergear1.set_position(pos);
  //   cybergear2.set_position(pos);
  // } 

  motor_status1 = cybergear1.get_status();
  motor_status2 = cybergear2.get_status();
  
  // Serial.print(" Count: "); Serial.print(count);
  Serial.print(" Angle: "); Serial.print(motor_status1.position);
  // Serial.print(" Angular_vel: "); Serial.print(motor_status1.speed);
  // Serial.print(" Torque: "); Serial.print(motor_status1.torque);
  // // Serial.print(" Tempareture: "); Serial.print(motor_status1.temperature);
  Serial.print("  2: "); 
  Serial.print(" Angle: "); Serial.println(motor_status2.position);
  // Serial.print(" Angular_vel: "); Serial.print(motor_status2.speed);
  // Serial.print(" Torque: "); Serial.println(motor_status2.torque);
  // Serial.print(" Tempareture: "); Serial.println(motor_status1.temperature);
  if(flag){

      // cybergear2.set_position(motor_status1.position);
      current[0] = (motor_status1.position - motor_status2.position) * LEADER_FOLLOWER_GAIN;
      current[1] = (motor_status2.position - motor_status1.position) * LEADER_FOLLOWER_GAIN;

  // clamp data range
      current[0] = min(max(current[0], -MAX_CURRENT), MAX_CURRENT);
      current[1] = min(max(current[1], -MAX_CURRENT), MAX_CURRENT);

      cybergear1.set_current(current[1]);
      cybergear2.set_current(current[0]);
      Serial.print(current[0]); Serial.print("  2: ");Serial.print(current[1]);

  }








  // if (((message.id & 0xFF00) >> 8) == CYBERGEAR_CAN_ID){
  //   cybergear.process_message(message);
  // }


  //  if ( can3.read(msg) ) {
  //   Serial.print("CAN3 "); 
  //   Serial.print("MB: "); Serial.print(msg.mb);
  //   Serial.print("  ID: 0x"); Serial.print(msg.id, DEC);
  //   Serial.print("  EXT: "); Serial.print(msg.flags.extended );
  //   Serial.print("  LEN: "); Serial.print(msg.len);
  //   Serial.print(" DATA: ");
  //   for ( uint8_t i = 0; i < 8; i++ ) {
  //     Serial.print(msg.buf[i]); Serial.print(" ");
  //   }
  //   Serial.print("  TS: "); Serial.println(msg.timestamp);
  //   // CybergearDriver.send_motion_control(msg)
  // }



  // put your main code here, to run repeatedly:
  // delayMicroseconds(250);
  delay(1);
}


