#include "main.h"

FPM383D myFPM;
DY_SV17F myAudio;
Servo myServo;
// SHTC3 mySHTC3 = SHTC3(GPIO_NUM_3, GPIO_NUM_0);
void task_create();
TaskHandle_t finger_identify_task_handle;
TaskHandle_t voice_play_task_handle;
TaskHandle_t servo_task_handle;
uint8_t Audio_status = 255;
void finger_identify_task(void *parameter);
extern "C" void app_main(void)
{
    myAudio.begin(5, 4, UART_NUM_0);
    myFPM.begin(6, 7, UART_NUM_1, GPIO_NUM_1);
    myServo.begin(270, GPIO_NUM_10);
    vTaskDelay(200); // 指纹模块启动需要200ms

    task_create();
    vTaskSuspend(voice_play_task_handle);
    vTaskSuspend(servo_task_handle);
}

// void temp_humi_sensor_task(void *parameter)
// {
//     while (1)
//     {
//         mySHTC3.update();
//         printf("%f,%f\n", mySHTC3.toDegC(), mySHTC3.toPercent());
//         vTaskDelay(1000);
//     }
// }
void finger_identify_task(void *parameter)
{
    static uint8_t wrong_times = 0;
    while (1)
    {
        if (myFPM.ScanState == 0)
        { // 第一次进入，之后进入都是==1
            myFPM.SetSystemPolicy(true, true, true);
            // myFPM.SetPassword(0x10403020);
            myFPM.SetLED(PWM, RG, 100, 0, 50);
            // myFPM.SetSleepMode();
        }
        else
        {
            if (myFPM.Identify() == 0)
            { // 成功
                myFPM.SetLED(Blink, B, 200, 200, 2);
                Audio_status = correct;
                vTaskResume(voice_play_task_handle);
                myServo.write(0); // 开门
                vTaskDelay(5000);
                myServo.write(0); // 关门
                vTaskDelay(1000);
            }
            else
            { // 失败
                Audio_status = wrong;
                vTaskResume(voice_play_task_handle);
                wrong_times++;
                if (wrong_times > 3)
                {
                    Audio_status = forbidden;
                    vTaskResume(voice_play_task_handle);
                    wrong_times = 0;
                    myFPM.SetLED(Blink, R, 200, 200, 10);
                    vTaskDelay(4000);
                }
                myFPM.SetLED(Blink, R, 200, 200, 2);
            }
            myFPM.ISR_Resume();
        }
        // 恢复等待状态
        myFPM.ScanState = 0;
        vTaskSuspend(voice_play_task_handle); // 挂起任务
        vTaskSuspend(NULL);                   // 挂起任务
    }
}

void voice_play_task(void *parameter)
{
    const uint8_t correct_start = 4, correct_end = 11;
    while (1)
    {
        switch (Audio_status)
        {
        case correct:
            myAudio.SetSoundPlay(1);
            vTaskDelay(1000);
            myAudio.SetSoundPlay(random(correct_start, correct_end));
            vTaskDelay(4000);
            break;
        case wrong:
            myAudio.SetSoundPlay(2);
            vTaskDelay(1000);
            break;
        case forbidden:
            myAudio.SetSoundPlay(3);
            vTaskDelay(4000);
            break;
        default:
            break;
        }
        Audio_status = 255;
        myAudio.SetVolume(31);
        vTaskSuspend(NULL); // 挂起任务
    }
}
void servo_task(void *parameter)
{
    vTaskDelay(1000);
    float angle;
    while (1)
    {
        for (angle = servo_close; angle >= servo_open; angle--)
        {
            myServo.write(angle);
            vTaskDelay(6);
        }
        vTaskDelay(2500);
        for (angle = servo_open; angle <= servo_close; angle++)
        {
            myServo.write(angle);
            vTaskDelay(6);
        }

        vTaskSuspend(NULL);
    }
}

void task_create()
{
    // xTaskCreate(
    // 	temp_humi_sensor_task,    // Function that should be called
    // 	"temp&humi sensor",  // Name of the task (for debugging)
    // 	1024,            // Stack size (bytes)
    // 	NULL,            // Parameter to pass
    // 	3,               // Task priority
    // 	NULL             // Task handle
    // );
    // xTaskCreate(
    //	finger_identify_task,    // Function that should be called
    //	"finger identify",  // Name of the task (for debugging)
    //	1024,            // Stack size (bytes)
    //	NULL,            // Parameter to pass
    //	5,               // Task priority
    //	&finger_identify_task_handle             // Task handle
    //);
    xTaskCreate(
        voice_play_task,        // Function that should be called
        "voice play",           // Name of the task (for debugging)
        1024,                   // Stack size (bytes)
        NULL,                   // Parameter to pass
        4,                      // Task priority
        &voice_play_task_handle // Task handle
    );
    xTaskCreate(
        servo_task,        // Function that should be called
        "Servo",           // Name of the task (for debugging)
        1024,              // Stack size (bytes)
        NULL,              // Parameter to pass
        1,                 // Task priority
        &servo_task_handle // Task handle
    );
}
inline uint32_t random(uint32_t min, uint32_t max)
{
    return esp_random() % (max - min + 1) + min;
}
