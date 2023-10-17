/********************
 *
 * - FreeRTOS starting example with Tiva C Series TM4C123GH6PM
 * - GitHub: AndresCasasola
 *
 ********************/

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/cpu_usage.h"
#include "drivers/rgb.h"

// Global variables
uint32_t g_ui32CPUUsage;
uint32_t g_ulSystemClock; // System clock speed

// Routine functions (RTI)
void ButtonHandler(void);

// The error routine that is called if the driver library or freeRTOS encounters an error.
#ifdef DEBUG
void __error__(char *nombrefich, uint32_t linea){
    while(1);
}
#endif

/********** Events links for FreeRTOS **********/

// Stack overflow detected
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName){
    while(1);
}

// CPU monitoring every system tick
void vApplicationTickHook( void ){
    static uint8_t count = 0;
    if (++count == 10){
        g_ui32CPUUsage = CPUUsageTick();
        count = 0;
    }
}

// Executes when Idle task runs
void vApplicationIdleHook (void){
    SysCtlSleep();
}
// Executes when Idle task runs
void vApplicationMallocFailedHook (void){
    while(1);
}

void system_config(){
    /******************** System Configuration ********************/
    // Set clock to 40 MHz (200 MHz from PLL with divider of 5)
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);	//

    // Get the system clock speed
    g_ulSystemClock = SysCtlClockGet();

    // Enables clock gating of peripherals in low power
    // Enable peripherals in sleep mode with SysCtlPeripheralSleepEnable()
    ROM_SysCtlPeripheralClockGating(true);

    // CPU usage measure subsystem (It measures the time that CPU is awake. It uses timer 3)
    CPUUsageInit(g_ulSystemClock, configTICK_RATE_HZ/10, 3);

    // UART initialization and configuration
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, SysCtlClockGet());

    // Enables UART and GPIOA even if CPU is in sleep mode
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);

    // Buttons and interruptions configuration
    ButtonsInit();
    IntRegister(INT_GPIOF, ButtonHandler);
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_BOTH_EDGES);
    IntPrioritySet(INT_GPIOF, configMAX_SYSCALL_INTERRUPT_PRIORITY);

    // Initialize LEDs through RGB library. It uses Timers 0 y 1
    RGBInit(1);
    SysCtlPeripheralSleepEnable(GREEN_TIMER_PERIPH);
    SysCtlPeripheralSleepEnable(BLUE_TIMER_PERIPH);
    SysCtlPeripheralSleepEnable(RED_TIMER_PERIPH);	// BLUE_TIMER_PERIPH y GREEN_TIMER_PERIPH are the same

    // Configure LEDs in GPIO mode
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
    /******************** System Configuration ********************/
}

typedef struct {
    uint32_t ID;
    uint32_t data;
}sensor_data;

sensor_data sensor_1 = {8,11111111};
sensor_data sensor_2 = {9,22222222};

/* Declare a variable of type xQueueHandle. This is used to store the handle
to the queue that is accessed by all three tasks. */
xQueueHandle xQueue;

/*Tasks Definition.*/

/*********1) Sender Task prototype.*************/
static void vSenderTask( void *pvParameters )
{
    sensor_data * lValueToSend;
    portBASE_TYPE xStatus;
    /* Two instances of this task are created so the value that is sent to the
 queue is passed in via the task parameter - this way each instance can use
 a different value. The queue was created to hold values of type long,
 so cast the parameter to the required type. */
    lValueToSend = ( sensor_data *) pvParameters;
    /* As per most tasks, this task is implemented within an infinite loop. */
    for( ;; )
    {
        /* Send the value to the queue.
 The first parameter is the queue to which data is being sent. The
 queue was created before the scheduler was started, so before this task
 started to execute.
 The second parameter is the address of the data to be sent, in this case
 the address of lValueToSend.
 The third parameter is the Block time – the time the task should be kept
 in the Blocked state to wait for space to become available on the queue
 should the queue already be full. In this case a block time is not
 specified because the queue should never contain more than one item and
 therefore never be full. */
        xStatus = xQueueSendToBack( xQueue, lValueToSend, 0 );
        if( xStatus != pdPASS )
        {
            /* The send operation could not complete because the queue was full -
 this must be an error as the queue should never contain more than
 one item! */
            //            vPrintString( "Could not send to the queue.\n" );
        }
        /* Allow the other sender task to execute. taskYIELD() informs the
 scheduler that a switch to another task should occur now rather than
 keeping this task in the Running state until the end of the current time
 slice. */
        taskYIELD();
    }
}

/*********2) Receiver Task prototype.*************/
static void vReceiverTask( void *pvParameters )
{
    /* Declare the variable that will hold the values received from the queue. */
    sensor_data lReceivedValue;
    portBASE_TYPE xStatus;
    const portTickType xTicksToWait = 100 / portTICK_RATE_MS;
    /* This task is also defined within an infinite loop. */
    for( ;; )
    {
        /* This call should always find the queue empty because this task will
 immediately remove any data that is written to the queue. */
        if( uxQueueMessagesWaiting( xQueue ) != 0 )
        {
            //            vPrintString( "Queue should have been empty!\n" );
        }
        /* Receive data from the queue.
 The first parameter is the queue from which data is to be received. The
 queue is created before the scheduler is started, and therefore before this
 task runs for the first time.
 The second parameter is the buffer into which the received data will be
 placed. In this case the buffer is simply the address of a variable that
 has the required size to hold the received data.
 The last parameter is the block time – the maximum amount of time that the
 task should remain in the Blocked state to wait for data to be available
 should the queue already be empty. In this case the constant
 portTICK_RATE_MS is used to convert 100 milliseconds to a time specified in
 ticks. */
        xStatus = xQueueReceive( xQueue, &lReceivedValue, xTicksToWait );
        if( xStatus == pdPASS )
        {
            /* Data was successfully received from the queue, print out the received value. */
            //            vPrintStringAndNumber( "Received = ", lReceivedValue );
        }
        else
        {
            /* Data was not received from the queue even after waiting for 100ms.
 This must be an error as the sending tasks are free running and will be
 continuously writing to the queue. */
            //            vPrintString( "Could not receive from the queue.\n" );
        }
    }
}


// Main function
int main(void){

    system_config();

    // Tasks Creation
    /* The queue is created to hold a maximum of 5 values, each of which is
     large enough to hold a variable of type long. */
    xQueue = xQueueCreate( 5, sizeof(sensor_data) );
    if( xQueue != NULL )
    {
        /* Create two instances of the task that will send to the queue. The task
     parameter is used to pass the value that the task will write to the queue,
     so one task will continuously write 100 to the queue while the other task
     will continuously write 200 to the queue. Both tasks are created at
     priority 1. */
        xTaskCreate( vSenderTask, "Sender1", 100, &sensor_1, 1, NULL );
        xTaskCreate( vSenderTask, "Sender2", 100, &sensor_2, 1, NULL );
        /* Create the task that will read from the queue. The task is created with
     priority 2, so above the priority of the sender tasks. */
        xTaskCreate( vReceiverTask, "Receiver", 100, NULL, 2, NULL );
        /* Start the scheduler so the created tasks start executing. */
        vTaskStartScheduler();
    }
    else
    {
        /* The queue could not be created. */
    }

    /* If all is well then main() will never reach here as the scheduler will
     now be running the tasks. If main() does reach here then it is likely that
     there was insufficient heap memory available for the idle task to be created.
     Chapter 5 provides more information on memory management. */
    for( ;; );

}

/********** RTIs **********/

void ButtonHandler(void){
    // Empty
}


