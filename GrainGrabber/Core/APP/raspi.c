#include "raspi.h"
#include "string.h"
#include "usart.h"

// 全局变量定义
Raspi_Date raspi;

// 添加全局变量来表示是否接收到树莓派的确认
volatile uint8_t raspi_detect_ok = 0;
int bean_id_index = 0;//记录第bean_id_index个拾取的豆子

/**
 * @brief 初始化树莓派通信
 * @param huart 串口句柄
 */
void Init_Raspi(void)
{
    raspi.vision_x = 0;
    raspi.vision_y = 0;
    raspi.real_x[0] = 300; 
    raspi.real_y[0] = 190;

    raspi.real_x[1] = 656; 
    raspi.real_y[1] = 336;

    raspi.real_x[2] = 648;  
    raspi.real_y[2] = 370;

    raspi.real_x[3] = 660;
    raspi.real_y[3] = 349;

    raspi.real_x[4] = 300;
    raspi.real_y[4] = 190;//移动到对面时的视觉坐标，后续需要根据实际情况调整

    raspi.real_x[5] = 360;
    raspi.real_y[5] = 448;

    raspi.real_x[6] = 192;
    raspi.real_y[6] = 445;
    
    raspi.real_x[7] = 183;
    raspi.real_y[7] = 455;
    
    raspi.real_x[8] = 1098;
    raspi.real_y[8] = 467;

    raspi.real_x[9] = 940;
    raspi.real_y[9] = 450;

    raspi.cmd = 0;
    
    // 初始化纸垛区域ID数组
    memset(raspi.bean_order, 0, sizeof(raspi.bean_order));
    
    // 初始化储存槽ID数组
    memset(raspi.box_id, 0, sizeof(raspi.box_id));

    memset(raspi.buffer, 0, RASPI_BUFFER_SIZE);
    //树莓派串口初始化,启动dma接收中断
    HAL_UART_Receive_DMA(&huart3, rxcmd3_dma, RXCMD3_DMA_SIZE);
    memset(rxcmd3_dma, 0, RXCMD3_DMA_SIZE);  
}

/**
 * @brief 发送任务命令到树莓派
 * @param task_type 任务类型
 * @param data 附加数据
 */
void Raspi_Send_Task(uint8_t task_type)
{
    raspi.vision_x = 0;
    raspi.vision_y = 0;
    uint8_t tx_data[4];
    tx_data[0] = 0xEE;  // STX
    tx_data[1] = task_type;
    tx_data[2] = 0x00;
    tx_data[3] = 0xFF;  // ETX

    // printf("Raspi_Send_Task: task_type=%d, data=%d\r\n", task_type, data);
    HAL_UART_Transmit(&huart3, tx_data, 4, 10);
    osDelay(5); 
    
}

void Raspi_Finish_Task(uint8_t task_type)
{
    if(task_type == TASK_MOVE_BY_BEAN)
    //当结束根据豆子微调任务时，认为这个豆子的种类已经确认了，将指针指向下一个豆子
    //（残留信息会被新的微调任务覆盖，影响不大）
    {
        bean_id_index++;
    }
    uint8_t tx_data[4];
    tx_data[0] = 0xEE;  // STX
    tx_data[1] = task_type;
    tx_data[2] = 0xAC;
    tx_data[3] = 0xFF;  // ETX

    // printf("Raspi_Finish_Task: task_type=%d\r\n", task_type);
    HAL_UART_Transmit(&huart3, tx_data, 4, 10);
    osDelay(5); 
}


/**
 * @brief 处理从树莓派接收到的数据
 * @param rx_data 接收到的数据
 * @param size 数据大小
 */
// uint32_t T=0;
uint8_t recognize = 0;
void Raspi_Process_Data(uint8_t *rx_data, uint16_t size)
{

    
    // 检查数据帧格式: STX + CMD + DATA... + ETX
    if (size == 8 && rx_data[0] == 0xEE && rx_data[size-1] == 0xFF)
    {


        // printf("Time: %d\r\n",HAL_GetTick()-T);
        // T = HAL_GetTick();
        // 将数据帧复制到raspi.buffer
        memcpy(raspi.buffer, rx_data, size);
        raspi.cmd = rx_data[1];

        // for(int i=0;i<8;i++){
        //     printf("%x ",rx_data[i]);
        // }
        // printf("\r\n");

        // 根据命令类型处理
        switch (raspi.cmd)
        {
            case PLAN_MOVE_BY_BEAN:  // 根据豆子位置微调命令
            case PLAN_MOVE_BY_BOX:  // 根据箱子位置微调命令
            {
                float x = (float)((uint16_t)((rx_data[2] << 8) | rx_data[3])) ;
                float y = (float)((uint16_t)((rx_data[4] << 8) | rx_data[5])) ;
                uint8_t type = rx_data[6];

                if(type<1 || type>5)
                {
                    break;
                }

                //处理异常值
                if(x >= 1280 || y >= 720 || x<0 || y<0)
                {
                    break;
                }
                raspi.vision_x = x;
                raspi.vision_y = y;

                if(bean_id_index < 3) // 最多记录3个豆子
                {
                    raspi.bean_order[bean_id_index] = type;
                }

                recognize = type;
            
                // 调试输出
                // printf("Plan_Move: ");
                // for (int i = 0; i < 6; i++) {
                //     printf("%d ", raspi.paper_id[i]);
                // }
                // printf("\r\n");
                break;
            }
                
            case PLAN_BOX_ID:  // 货箱ID计划
            {

                memcpy(raspi.box_id, &rx_data[2], 5);  // 复制5个字节到box_id
                raspi_detect_ok = 1;
                // 调试输出
                // printf("Plan_Box_ID: ");
                // for (int i = 0; i < 5; i++) {
                //     printf("%d ", raspi.box_id[i]);
                // }
                // printf("\r\n");
                break;


            }
            // case RASPI_DETECT_OK:  // 树莓派检测完成标志
            // {
            //     // 检查第三个到第八个字节是否全为0x00
            //     if (rx_data[2] == 0x00 && rx_data[3] == 0x00 && rx_data[4] == 0x00 && 
            //         rx_data[5] == 0x00 && rx_data[6] == 0x00 && rx_data[7] == 0x00) {
            //         raspi_detect_ok = 1;
            //         // printf("raspi_detect_ok\r\n");
            //     }
            //     break;
            // }
                
            default:
                // 未知命令
            {
                break;
            }
        }
    }
}
