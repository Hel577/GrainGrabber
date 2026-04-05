#include "main.h"
#include "can.h"
#include "motor.h"
#include "math.h"

const double pi;

uint32_t construct_txid(TXID* txid) {
    return (uint32_t)txid->category << 24 | (uint32_t)txid->message << 8 | (uint32_t)txid->target_id;
}

void write_command_to_motor(uint8_t action_id, uint8_t target_id, uint8_t* data){
    /*
    向对应的电机写入命令
    action_id: 命令ID
    target_id: 目标电机ID
    data: 数据，长度为8字节
    */
    //向电机写入命令的函数
    TXID txid;
    txid.category = action_id;
    txid.message = (uint32_t)MASTER_CAN_ID;
    txid.target_id = target_id;
    uint32_t extid = construct_txid(&txid);
    

    //TODO: 这里需要考虑多线程的情况，要在这里设置一个互斥锁
    TXHeader.ExtId = extid;
    //发送CAN消息
    uint8_t can_data[8];
    for(int i=0; i<8; i++){
        can_data[i] = data[i];
    }
    HAL_CAN_AddTxMessage(&hcan1, &TXHeader, can_data, (uint32_t*)CAN_TX_MAILBOX0);
}

void read_message_from_motor(RXID* rxid, uint8_t* data){
    /*
    这个函数后续可能要根据具体的情况修改
    从电机读取消息
    rxid: 读取到的消息ID
    data: 读取到的数据，长度为8字节
    */
    //从电机读取消息的函数
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RXHeader, data);
    if(RXHeader.IDE == CAN_ID_EXT){
        uint32_t extid = RXHeader.ExtId;
        if(extid&0xFF!= MASTER_CAN_ID){
            //如果不是主控发来的消息，直接丢弃
            return;
        }
        rxid->category = (extid >> 24) & 0xFF;
        rxid->message = (extid >> 16) & 0xFF;
        rxid->source_id = (extid >> 8) & 0xFF;
        rxid->target_id = (extid >> 0) & 0xFF;
    }
}

void read_device(RXID* rxid, uint8_t* data, DEVICE* device){
    /*
    这个函数后续可能要根据具体的情况修改
    从电机读取消息
    rxid: 读取到的消息ID
    data: 读取到的数据，长度为8字节
    */
    //从电机读取消息的函数
    if(rxid->category != GET_DEVICE_ID){
        //TODO: 报错调试
        return;
    }

    device->id = rxid->source_id;
    for(int i=0; i<8; i++){
        device->MCU[i] = data[i];
    }
}

void read_state(RXID* rxid, uint8_t* data, STATE* state){
    /*
    这个函数后续可能要根据具体的情况修改
    从电机读取状态
    rxid: 读取到的消息ID
    data: 读取到的数据，长度为8字节
    */
    //从电机读取状态的函数
    if(rxid->category != GET_STATE_ID){
        //TODO: 报错调试
        return;
    }

    state->mode = MOTOR;
    state->error = (uint8_t)(rxid->message & 0x3F != 0);
    state->angle = ((float)((data[0] << 8) | data[1])/65535.0f - 0.5)* 8 * pi;
    state->ange_vel = ((float)((data[2] << 8) | data[3])/65535.0f - 0.5)* 60;
    state->torque = ((float)((data[4] << 8) | data[5])/65535.0f - 0.5)* 24;
    state->temp = (float)((data[6] << 8) | data[7])/10;
}

void read_parameter(RXID* rxid, uint8_t* data, para* parameter){
    /*
    这个函数后续可能要根据具体的情况修改
    从电机读取参数
    rxid: 读取到的消息ID
    data: 读取到的数据，长度为8字节
    */
    //从电机读取参数的函数
    if(rxid->category != GET_PARA_ID){
        //TODO: 报错调试
        return;
    }
    parameter->index = (data[0] << 8) | data[1];
    parameter->data = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
}




