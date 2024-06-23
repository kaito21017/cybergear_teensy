#include <Arduino.h>
#include <FlexCAN.h>


#define P_MIN -12.5f  
#define P_MAX 12.5f  
#define V_MIN -30.0f  
#define V_MAX 30.0f  
#define KP_MIN 0.0f  
#define KP_MAX 500.0f  
#define KD_MIN 0.0f  
#define KD_MAX 5.0f  
#define T_MIN -12.0f  
#define T_MAX 12.0f  

struct exCanIdInfo{  
uint32_t id:8;  
uint32_t data:16;  
W/R 
uint32_t mode:5;  
uint32_t res:3;  
};  
CAN_message_t msg;

can_receive_message_struct rxMsg;  
can_trasnmit_message_struct txMsg={  
  .tx_sfid = 0,  
  .tx_efid = 0xff,  
  .tx_ft = CAN_FT_DATA,  
  .tx_ff = CAN_FF_EXTENDED,  
  .tx_dlen = 8,  
};  
#define txCanIdEx (((struct exCanIdInfo)&(txMsg.tx_efid)))  
#define rxCanIdEx (((struct exCanIdInfo)&(rxMsg.rx_efid))) //将扩展帧id 解析为自定义数据结构  
int float_to_uint(float x, float x_min, float x_max, int bits){  
float span = x_max - x_min;  
float offset = x_min;  
if(x > x_max) x=x_max;  
else if(x < x_min) x= x_min;  
return (int) ((x-offset)*((float)((1<<bits)-1))/span);  
}  
// #define can_txd() can_message_transmit(CAN0, &txMsg)  
// #define can_rxd() can_message_receive(CAN0, CAN_FIFO1, &rxMsg) 


void enable_moter(uint64_t can_id){
    
}

void set_run_mode(uint64_t run_mode){

}

// put function declarations here:


void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}