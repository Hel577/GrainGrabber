#include "raspi.h"
#include "string.h"
#include "usart.h"

// 全局变量定义
Raspi_Date raspi;

// 添加全局变量来表示是否接收到树莓派的确认
volatile uint8_t raspi_detect_ok = 0;

/**
 * @brief 初始化树莓派通信
 * @param huart 串口句柄
 */
void Init_Raspi(void)
{
    raspi.vision_x = 0;
    raspi.vision_y = 0;
    raspi.real_x[0] = 226; //货架箱子中心偏置
    raspi.real_y[0] = 401;

    raspi.real_x[1] = 474; //侧面纸垛中心偏置
    raspi.real_y[1] = 292;

    raspi.real_x[2] = 358;  //中间纸垛中心偏置
    raspi.real_y[2] = 284;

    raspi.cmd = 0;
    
    // 初始化纸垛区域ID数组
    memset(raspi.paper_id, 0, sizeof(raspi.paper_id));
    
    // 初始化储存槽ID数组
    memset(raspi.box_id, 0, sizeof(raspi.box_id));
    
    // 初始化箱子方向数组
    memset(raspi.box_dir, 0, sizeof(raspi.box_dir));
    
    // 初始化码垛标志数组
    memset(raspi.maduo, 0, sizeof(raspi.maduo));

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
void Raspi_Send_Task(uint8_t task_type, uint8_t data)
{
    uint8_t tx_data[4];
    tx_data[0] = 0x04;  // STX
    tx_data[1] = task_type;
    tx_data[2] = data;
    tx_data[3] = 0x05;  // ETX

    // printf("Raspi_Send_Task: task_type=%d, data=%d\r\n", task_type, data);
    HAL_UART_Transmit(&huart3, tx_data, 4, 10);
    osDelay(5); 
    
    
}



//唤醒树莓派前后识别箱子和纸垛
void Start_Raspi_Detect(void)
{
    Raspi_Send_Task(TASK_DETECT_BOX, 0);
}



/**
 * @brief 处理从树莓派接收到的数据
 * @param rx_data 接收到的数据
 * @param size 数据大小
 */
void Raspi_Process_Data(uint8_t *rx_data, uint16_t size)
{

    
    // 检查数据帧格式: STX + CMD + DATA... + ETX
    if (size == 9 && rx_data[0] == 0xEE && rx_data[size-1] == 0xFF)
    {
        // 将数据帧复制到raspi.buffer
        memcpy(raspi.buffer, rx_data, size);
        raspi.cmd = rx_data[1];

        // 根据命令类型处理
        switch (raspi.cmd)
        {
            case PLAN_MOVE:  // MOVE命令
            {
                // 解析移动计划：每个字节高4位是起点ID，低4位是终点ID
                for (int i = 0; i < 6; i++) {
                    uint8_t byte = rx_data[i+2];
                    uint8_t from_id = (byte >> 4) & 0x0F;  // 高4位是起点ID
                    uint8_t to_id = byte & 0x0F;          // 低4位是终点ID
                    raspi.paper_id[i] = to_id;            // 只存储终点ID
                }
            
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

                memcpy(raspi.box_id, &rx_data[2], 6);  // 复制6个字节到box_id

                // 调试输出
                // printf("Plan_Box_ID: ");
                // for (int i = 0; i < 6; i++) {
                //     printf("%d ", raspi.box_id[i]);
                // }
                // printf("\r\n");
                break;


            }
                
            case PLAN_SIDE:  // 放置方向计划
            {

                memcpy(raspi.box_dir, &rx_data[2], 6);  // 复制6个字节到box_dir

                // 调试输出
                // printf("Plan_Side: ");
                // for (int i = 0; i < 6; i++) {
                //     printf("%d ", raspi.box_dir[i]);
                // }   
                // printf("\r\n");
                break;

            }
                
            case PLAN_HEIGHT:  // 放置高度计划
            {
                memcpy(raspi.maduo, &rx_data[2], 6);  // 复制6个字节到maduo

                // 调试输出
                // printf("Plan_Height: ");
                // for (int i = 0; i < 6; i++) {
                //     printf("%d ", raspi.maduo[i]);
                // }
                // printf("\r\n");
                break;
            }
            case 0x14:
            {
                // 检查第三个到第八个字节是否全为0x00
                if (rx_data[2] == 0x00 && rx_data[3] == 0x00 && rx_data[4] == 0x00 && 
                    rx_data[5] == 0x00 && rx_data[6] == 0x00 && rx_data[7] == 0x00) {
                    raspi_detect_ok = 1;
                    // printf("raspi_detect_ok\r\n");
                }
                break;
            }
            case CMD_LOCATE:  // 定位坐标
            {
                    float x = (float)((uint16_t)((rx_data[2] << 8) | rx_data[3])) ;
                    float y = (float)((uint16_t)((rx_data[4] << 8) | rx_data[5])) ;

                    //处理异常值
                    if(x >= 640 || y >= 480 || x<0 || y<0)
                    {
                        break;
                    }
                    raspi.vision_x = x;
                    raspi.vision_y = y;
                    // printf("vision_x: %d, vision_y: %d\r\n", (int)raspi.vision_x, (int)raspi.vision_y);
                    break;

            }
                
            default:
                // 未知命令
            {
                break;
            }
        }
    }
}
