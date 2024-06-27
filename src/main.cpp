#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <cybergear_driver.h>

// FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;
uint8_t CYBERGEAR_CAN_ID = 0x7E;
uint8_t MASTER_CAN_ID = 0x00;
CyberGearDriver cybergear = CyberGearDriver(CYBERGEAR_CAN_ID, MASTER_CAN_ID);

CyberGearStatus motor_status;
#include <cybergear_driver.h>

// FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can3;
uint8_t CYBERGEAR_CAN_ID = 0x7E;
uint8_t MASTER_CAN_ID = 0x00;
CyberGearDriver cybergear = CyberGearDriver(CYBERGEAR_CAN_ID, MASTER_CAN_ID);

CyberGearStatus motor_status;

// put function declarations here:

CAN_message_t msg;
CAN_message_t msg;
void setup() {
  Serial.begin(115200); delay(400);
  Serial.begin(115200); delay(400);
  // put your setup code here, to run once:
  // int result = myFunction(2, 3);
  // can3.begin();
  // can3.setBaudRate(1000000);
  cybergear.init_can();
  cybergear.init_motor(MODE_POSITION);
  cybergear.set_limit_speed(10.0f); /* set the maximum speed of the motor */
  cybergear.set_limit_current(5.0); /* current limit allows faster operation */
  // can3.begin();
  // can3.setBaudRate(1000000);
  cybergear.init_can();
  cybergear.init_motor(MODE_POSITION);
  cybergear.set_limit_speed(10.0f); /* set the maximum speed of the motor */
  cybergear.set_limit_current(5.0); /* current limit allows faster operation */
}

void loop() {
  static int count = 0;
  count++;
   String str = Serial.readString();
   static float pos = 0;
  // if(count%10 == 0){
  cybergear.request_status();
  // }

  //  if(str == "c") {
  //   // cybergear.init_motor(MODE_POSITION);
  //   // cybergear.enable_motor();

  // }else if(str == "x") {
  //   pos= 0;
  //   // cybergear.stop_motor();
  // } 
  // else if(str == "j") {
  //   pos--;
  //   // cybergear.set_position(pos);
  // } else if(str == "l") {
  //   pos++;
  //   // cybergear.set_position(pos);
  // } 

  motor_status = cybergear.get_status();
  
  Serial.print(" Count: "); Serial.print(count);
  Serial.print(" Angle: "); Serial.print(motor_status.position);
  Serial.print(" Angular_vel: "); Serial.print(motor_status.speed);
  Serial.print(" Torque: "); Serial.print(motor_status.torque);
  Serial.print(" Tempareture: "); Serial.println(motor_status.temperature);





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
  delayMicroseconds(20);
}


