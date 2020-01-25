/* ========================================================================== */
/*                                                                            */
// Sam Siewert, December 2017
//
// Modified to run on freeRTOS by Mohit Rane
//
// Sequencer Generic - seqgen.c
//
// Sequencer - 30 Hz
//                   [gives semaphores to all other services]
// Service_1 - 3 Hz  , every 10th Sequencer loop
//                   [buffers 3 images per second]
// Service_2 - 1 Hz  , every 30th Sequencer loop
//                   [time-stamp middle sample image with cvPutText or header]
// Service_3 - 0.5 Hz, every 60th Sequencer loop
//                   [difference current and previous time stamped images]
// Service_4 - 1 Hz, every 30th Sequencer loop
//                   [save time stamped image with cvSaveImage or write()]
// Service_5 - 0.5 Hz, every 60th Sequencer loop
//                   [save difference image with cvSaveImage or write()]
// Service_6 - 1 Hz, every 30th Sequencer loop
//                   [write current time-stamped image to TCP socket server]
// Service_7 - 0.1 Hz, every 300th Sequencer loop
//                   [syslog the time for debug]
//
// With the above, priorities by RM policy would be:
//
// Sequencer = tskIDLE_PRIORITY + 5 @ 30 Hz (33.33 ms)
// Servcie_1 = tskIDLE_PRIORITY + 4 @ 3 Hz
// Service_2 = tskIDLE_PRIORITY + 3 @ 1 Hz
// Service_3 = tskIDLE_PRIORITY + 2 @ 0.5 Hz
// Service_4 = tskIDLE_PRIORITY + 3 @ 1 Hz
// Service_5 = tskIDLE_PRIORITY + 2 @ 0.5 Hz
// Service_6 = tskIDLE_PRIORITY + 3 @ 1 Hz
// Service_7 = tskIDLE_PRIORITY + 1 @ 0.1 Hz

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "priorities.h"
#include "third_party/FreeRTOS/Source/include/timers.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

// including following 3 header files gives error
//#include <sched.h>
//#include <syslog.h>
//#include "sys/time.h"

// these header files can be included but are not required
//#include <time.h>
//#include <errno.h>

// number of services
#define NUMBER_OF_SERVICES      (7)

// frequency of services
#define SEQUENCER_FREQUENCY     (30)
#define SERVICE_1_FREQUENCY     (3)
#define SERVICE_2_FREQUENCY     (1)
#define SERVICE_3_FREQUENCY     (0.5)
#define SERVICE_4_FREQUENCY     (1)
#define SERVICE_5_FREQUENCY     (0.5)
#define SERVICE_6_FREQUENCY     (1)
#define SERVICE_7_FREQUENCY     (0.1)

// priority of services
#define PRIORITY_SEQUENCER      (tskIDLE_PRIORITY + 5)
#define PRIORITY_SERVICE_1      (tskIDLE_PRIORITY + 4)
#define PRIORITY_SERVICE_2      (tskIDLE_PRIORITY + 3)
#define PRIORITY_SERVICE_3      (tskIDLE_PRIORITY + 2)
#define PRIORITY_SERVICE_4      (tskIDLE_PRIORITY + 3)
#define PRIORITY_SERVICE_5      (tskIDLE_PRIORITY + 2)
#define PRIORITY_SERVICE_6      (tskIDLE_PRIORITY + 3)
#define PRIORITY_SERVICE_7      (tskIDLE_PRIORITY + 1)

// service count
#define SEQ_CNT_FOR_S1          ((int) (SEQUENCER_FREQUENCY / SERVICE_1_FREQUENCY))
#define SEQ_CNT_FOR_S2          ((int) (SEQUENCER_FREQUENCY / SERVICE_2_FREQUENCY))
#define SEQ_CNT_FOR_S3          ((int) (SEQUENCER_FREQUENCY / SERVICE_3_FREQUENCY))
#define SEQ_CNT_FOR_S4          ((int) (SEQUENCER_FREQUENCY / SERVICE_4_FREQUENCY))
#define SEQ_CNT_FOR_S5          ((int) (SEQUENCER_FREQUENCY / SERVICE_5_FREQUENCY))
#define SEQ_CNT_FOR_S6          ((int) (SEQUENCER_FREQUENCY / SERVICE_6_FREQUENCY))
#define SEQ_CNT_FOR_S7          ((int) (SEQUENCER_FREQUENCY / SERVICE_7_FREQUENCY))

// sequence iterations required
#define SEQUENCER_ITERATIONS     (900)

// based on 50 MHz clock frequency as set in main function
#define USEC_TICK_COUNT         (50)
#define MSEC_TICK_COUNT         (50000)
#define SEC_TICK_COUNT          (50000000)

// ticks for sequencer
#define SEQ_TIMER_TICKS         ((int) (SEC_TICK_COUNT / SEQUENCER_FREQUENCY))

// ticks for normal timer; added 3 seconds for safety
#define NORMAL_TIMER_TICKS      ((int) (SEQUENCER_ITERATIONS / SEQUENCER_FREQUENCY + 3) * SEC_TICK_COUNT)

#define TRUE                    (1)
#define FALSE                   (0)

// abort flags for services
int abortS1=FALSE, abortS2=FALSE, abortS3=FALSE, abortS4=FALSE, abortS5=FALSE, abortS6=FALSE, abortS7=FALSE;

// Worst Case Execution Time variables for services
struct wcet{
    int time[NUMBER_OF_SERVICES];
    int release[NUMBER_OF_SERVICES];
} WCET;

// declaring services semaphore
xSemaphoreHandle semSeq, semS1, semS2, semS3, semS4, semS5, semS6, semS7;

extern void Timer0AHandler(void);

//*****************************************************************************
//
// The mutex that protects concurrent access of UART from multiple tasks.
//
//*****************************************************************************
xSemaphoreHandle g_pUARTSemaphore;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void
vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

/* Configure Timer */
void ConfigureTimer()
{
    /* TIMER0A for Sequencer */
    /* TIMER1A for Timestamp */

    // Enable peripheral timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    // Enable interrupts
    IntMasterEnable();

    // Configure Timer0 and Timer1, both counts up
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC_UP);

    // 50 MHz, so 50M ticks means 1 secs
    TimerLoadSet(TIMER0_BASE, TIMER_A, SEQ_TIMER_TICKS);
    TimerLoadSet(TIMER1_BASE, TIMER_A, NORMAL_TIMER_TICKS);

    // Timer timeout interrupt setup
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Start both Timers
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerEnable(TIMER1_BASE, TIMER_A);
}

/* Timer Interrupt Handler */
void Timer0AHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    xSemaphoreGive(semSeq);
}

/* Sequencer */
// problem of writing sequencer in ISR:
// can't include UART mutex as it might create deadlock, and if we don't include that, print gets out of order, since interrupt has highest priority
void Sequencer(void *noarg)
{
    int seqCnt = 0;

    // Sequencer stops all services after 900 cycles
    while(seqCnt < SEQUENCER_ITERATIONS)
    {
        xSemaphoreTake(semSeq, portMAX_DELAY);

        seqCnt++;

        // If sequencer code is written in ISR, then including UART mutex might create deadlock in case some low priority service is in middle of printing something
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Sequencer cycle %d @ sec=%d, msec=%d\n", seqCnt, TimerValueGet(TIMER1_BASE, TIMER_A)/SEC_TICK_COUNT, (TimerValueGet(TIMER1_BASE, TIMER_A)/MSEC_TICK_COUNT)%1000);
        xSemaphoreGive(g_pUARTSemaphore);

        // Release each service at a sub-rate of the generic sequencer rate

        if((seqCnt % SEQ_CNT_FOR_S1) == 0) xSemaphoreGive(semS1);

        if((seqCnt % SEQ_CNT_FOR_S2) == 0) xSemaphoreGive(semS2);

        if((seqCnt % SEQ_CNT_FOR_S3) == 0) xSemaphoreGive(semS3);

        if((seqCnt % SEQ_CNT_FOR_S4) == 0) xSemaphoreGive(semS4);

        if((seqCnt % SEQ_CNT_FOR_S5) == 0) xSemaphoreGive(semS5);

        if((seqCnt % SEQ_CNT_FOR_S6) == 0) xSemaphoreGive(semS6);

        if((seqCnt % SEQ_CNT_FOR_S7) == 0) xSemaphoreGive(semS7);
    }

    // Aborting tasks after required sequencer iterations
    abortS1=TRUE; abortS2=TRUE; abortS3=TRUE;
    abortS4=TRUE; abortS5=TRUE; abortS6=TRUE;
    abortS7=TRUE;

    vTaskDelete(NULL);
}


/* Servcie_1 */
void Service_1(void *noarg)
{
    int S1Cnt=0;
    int startTimeInUsec, endTimeInUsec;

    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("Frame Sampler thread @ sec=%d, msec=%d\n", TimerValueGet(TIMER1_BASE, TIMER_A)/SEC_TICK_COUNT, (TimerValueGet(TIMER1_BASE, TIMER_A)/MSEC_TICK_COUNT)%1000);
    xSemaphoreGive(g_pUARTSemaphore);

    // abortS1 gets true in Sequencer after Sequencer executes for required iterations
    while(!abortS1)
    {
        // wait for semaphore from sequencer
        xSemaphoreTake(semS1, portMAX_DELAY);

        startTimeInUsec = TimerValueGet(TIMER1_BASE, TIMER_A)/USEC_TICK_COUNT;
        S1Cnt++;
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Frame Sampler release %d @ sec=%d, msec=%d\n", S1Cnt, TimerValueGet(TIMER1_BASE, TIMER_A)/SEC_TICK_COUNT, (TimerValueGet(TIMER1_BASE, TIMER_A)/MSEC_TICK_COUNT)%1000);
        xSemaphoreGive(g_pUARTSemaphore);
        endTimeInUsec = TimerValueGet(TIMER1_BASE, TIMER_A)/USEC_TICK_COUNT;

        // takes out maximum wcet in all iterations
        if((endTimeInUsec - startTimeInUsec) > WCET.time[0])
        {
            WCET.time[0] = endTimeInUsec - startTimeInUsec;
            WCET.release[0] = S1Cnt;
        }
    }

    vTaskDelete(NULL);
}

/* Servcie_2 */
void Service_2(void *noarg)
{
    int S2Cnt=0;
    int startTimeInUsec, endTimeInUsec;

    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("Time-stamp with Image Analysis thread @ sec=%d, msec=%d\n", TimerValueGet(TIMER1_BASE, TIMER_A)/SEC_TICK_COUNT, (TimerValueGet(TIMER1_BASE, TIMER_A)/MSEC_TICK_COUNT)%1000);
    xSemaphoreGive(g_pUARTSemaphore);

    // abortS2 gets true in Sequencer after Sequencer executes for required iterations
    while(!abortS2)
    {
        // wait for semaphore from sequencer
        xSemaphoreTake(semS2, portMAX_DELAY);

        startTimeInUsec = TimerValueGet(TIMER1_BASE, TIMER_A)/USEC_TICK_COUNT;
        S2Cnt++;
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Time-stamp with Image Analysis release %d @ sec=%d, msec=%d\n", S2Cnt, TimerValueGet(TIMER1_BASE, TIMER_A)/SEC_TICK_COUNT, (TimerValueGet(TIMER1_BASE, TIMER_A)/MSEC_TICK_COUNT)%1000);
        xSemaphoreGive(g_pUARTSemaphore);
        endTimeInUsec = TimerValueGet(TIMER1_BASE, TIMER_A)/USEC_TICK_COUNT;

        // takes out maximum wcet in all iterations
        if((endTimeInUsec - startTimeInUsec) > WCET.time[1])
        {
            WCET.time[1] = endTimeInUsec - startTimeInUsec;
            WCET.release[1] = S2Cnt;
        }
    }

    vTaskDelete(NULL);
}

/* Servcie_3 */
void Service_3(void *noarg)
{
    int S3Cnt=0;
    int startTimeInUsec, endTimeInUsec;

    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("Difference Image Proc thread @ sec=%d, msec=%d\n", TimerValueGet(TIMER1_BASE, TIMER_A)/SEC_TICK_COUNT, (TimerValueGet(TIMER1_BASE, TIMER_A)/MSEC_TICK_COUNT)%1000);
    xSemaphoreGive(g_pUARTSemaphore);

    // abortS3 gets true in Sequencer after Sequencer executes for required iterations
    while(!abortS3)
    {
        // wait for semaphore from sequencer
        xSemaphoreTake(semS3, portMAX_DELAY);

        startTimeInUsec = TimerValueGet(TIMER1_BASE, TIMER_A)/USEC_TICK_COUNT;
        S3Cnt++;
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Difference Image Proc release %d @ sec=%d, msec=%d\n", S3Cnt, TimerValueGet(TIMER1_BASE, TIMER_A)/SEC_TICK_COUNT, (TimerValueGet(TIMER1_BASE, TIMER_A)/MSEC_TICK_COUNT)%1000);
        xSemaphoreGive(g_pUARTSemaphore);
        endTimeInUsec = TimerValueGet(TIMER1_BASE, TIMER_A)/USEC_TICK_COUNT;

        // takes out maximum wcet in all iterations
        if((endTimeInUsec - startTimeInUsec) > WCET.time[2])
        {
            WCET.time[2] = endTimeInUsec - startTimeInUsec;
            WCET.release[2] = S3Cnt;
        }
    }

    vTaskDelete(NULL);
}

/* Servcie_4 */
void Service_4(void *noarg)
{
    int S4Cnt=0;
    int startTimeInUsec, endTimeInUsec;

    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("Time-stamp Image Save to File thread @ sec=%d, msec=%d\n", TimerValueGet(TIMER1_BASE, TIMER_A)/SEC_TICK_COUNT, (TimerValueGet(TIMER1_BASE, TIMER_A)/MSEC_TICK_COUNT)%1000);
    xSemaphoreGive(g_pUARTSemaphore);

    // abortS4 gets true in Sequencer after Sequencer executes for required iterations
    while(!abortS4)
    {
        // wait for semaphore from sequencer
        xSemaphoreTake(semS4, portMAX_DELAY);

        startTimeInUsec = TimerValueGet(TIMER1_BASE, TIMER_A)/USEC_TICK_COUNT;
        S4Cnt++;
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Time-stamp Image Save to File release %d @ sec=%d, msec=%d\n", S4Cnt, TimerValueGet(TIMER1_BASE, TIMER_A)/SEC_TICK_COUNT, (TimerValueGet(TIMER1_BASE, TIMER_A)/MSEC_TICK_COUNT)%1000);
        xSemaphoreGive(g_pUARTSemaphore);
        endTimeInUsec = TimerValueGet(TIMER1_BASE, TIMER_A)/USEC_TICK_COUNT;

        // takes out maximum wcet in all iterations
        if((endTimeInUsec - startTimeInUsec) > WCET.time[3])
        {
            WCET.time[3] = endTimeInUsec - startTimeInUsec;
            WCET.release[3] = S4Cnt;
        }
    }

    vTaskDelete(NULL);
}

/* Servcie_5 */
void Service_5(void *noarg)
{
    int S5Cnt=0;
    int startTimeInUsec, endTimeInUsec;

    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("Processed Image Save to File thread @ sec=%d, msec=%d\n", TimerValueGet(TIMER1_BASE, TIMER_A)/SEC_TICK_COUNT, (TimerValueGet(TIMER1_BASE, TIMER_A)/MSEC_TICK_COUNT)%1000);
    xSemaphoreGive(g_pUARTSemaphore);

    // abortS5 gets true in Sequencer after Sequencer executes for required iterations
    while(!abortS5)
    {
        // wait for semaphore from sequencer
        xSemaphoreTake(semS5, portMAX_DELAY);

        startTimeInUsec = TimerValueGet(TIMER1_BASE, TIMER_A)/USEC_TICK_COUNT;
        S5Cnt++;
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Processed Image Save to File release %d @ sec=%d, msec=%d\n", S5Cnt, TimerValueGet(TIMER1_BASE, TIMER_A)/SEC_TICK_COUNT, (TimerValueGet(TIMER1_BASE, TIMER_A)/MSEC_TICK_COUNT)%1000);
        xSemaphoreGive(g_pUARTSemaphore);
        endTimeInUsec = TimerValueGet(TIMER1_BASE, TIMER_A)/USEC_TICK_COUNT;

        // takes out maximum wcet in all iterations
        if((endTimeInUsec - startTimeInUsec) > WCET.time[4])
        {
            WCET.time[4] = endTimeInUsec - startTimeInUsec;
            WCET.release[4] = S5Cnt;
        }
    }

    vTaskDelete(NULL);
}

/* Servcie_6 */
void Service_6(void *noarg)
{
    int S6Cnt=0;
    int startTimeInUsec, endTimeInUsec;

    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("Send Time-stamped Image to Remote thread @ sec=%d, msec=%d\n", TimerValueGet(TIMER1_BASE, TIMER_A)/SEC_TICK_COUNT, (TimerValueGet(TIMER1_BASE, TIMER_A)/MSEC_TICK_COUNT)%1000);
    xSemaphoreGive(g_pUARTSemaphore);

    // abortS6 gets true in Sequencer after Sequencer executes for required iterations
    while(!abortS6)
    {
        // wait for semaphore from sequencer
        xSemaphoreTake(semS6, portMAX_DELAY);

        startTimeInUsec = TimerValueGet(TIMER1_BASE, TIMER_A)/USEC_TICK_COUNT;
        S6Cnt++;
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("Send Time-stamped Image to Remote release %d @ sec=%d, msec=%d\n", S6Cnt, TimerValueGet(TIMER1_BASE, TIMER_A)/SEC_TICK_COUNT, (TimerValueGet(TIMER1_BASE, TIMER_A)/MSEC_TICK_COUNT)%1000);
        xSemaphoreGive(g_pUARTSemaphore);
        endTimeInUsec = TimerValueGet(TIMER1_BASE, TIMER_A)/USEC_TICK_COUNT;

        // takes out maximum wcet in all iterations
        if((endTimeInUsec - startTimeInUsec) > WCET.time[5])
        {
            WCET.time[5] = endTimeInUsec - startTimeInUsec;
            WCET.release[5] = S6Cnt;
        }
    }

    vTaskDelete(NULL);
}

/* Servcie_7 */
void Service_7(void *noarg)
{
    int S7Cnt=0;
    int startTimeInUsec, endTimeInUsec;

    xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
    UARTprintf("10 sec Tick Debug thread @ sec=%d, msec=%d\n", TimerValueGet(TIMER1_BASE, TIMER_A)/SEC_TICK_COUNT, (TimerValueGet(TIMER1_BASE, TIMER_A)/MSEC_TICK_COUNT)%1000);
    xSemaphoreGive(g_pUARTSemaphore);

    // abortS7 gets true in Sequencer after Sequencer executes for required iterations
    while(!abortS7)
    {
        // wait for semaphore from sequencer
        xSemaphoreTake(semS7, portMAX_DELAY);

        startTimeInUsec = TimerValueGet(TIMER1_BASE, TIMER_A)/USEC_TICK_COUNT;
        S7Cnt++;
        xSemaphoreTake(g_pUARTSemaphore, portMAX_DELAY);
        UARTprintf("10 Sec Tick Debug release %d @ sec=%d, msec=%d\n", S7Cnt, TimerValueGet(TIMER1_BASE, TIMER_A)/SEC_TICK_COUNT, (TimerValueGet(TIMER1_BASE, TIMER_A)/MSEC_TICK_COUNT)%1000);
        xSemaphoreGive(g_pUARTSemaphore);
        endTimeInUsec = TimerValueGet(TIMER1_BASE, TIMER_A)/USEC_TICK_COUNT;

        // takes out maximum wcet in all iterations
        if((endTimeInUsec - startTimeInUsec) > WCET.time[6])
        {
            WCET.time[6] = endTimeInUsec - startTimeInUsec;
            WCET.release[6] = S7Cnt;
        }
    }

    // since Service 7 has least priority, the following statements will be executed at the last
    // the following statements (including WCET printing) are included here instead of ISR to keep it short or Sequencer to keep correct order of print

    // disabling Timer0A interrupt
    TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // disabling timers
    TimerDisable(TIMER0_BASE, TIMER_A);
    TimerDisable(TIMER1_BASE, TIMER_A);

    // printing out WCET of all services
    UARTprintf("\n");
    UARTprintf("TEST COMPLETE\n");

    UARTprintf("WCET of Service 1 = %d usec     at release %d   at Sequencer count %d\n", WCET.time[0], WCET.release[0], WCET.release[0] * SEQ_CNT_FOR_S1);
    UARTprintf("WCET of Service 2 = %d usec     at release %d   at Sequencer count %d\n", WCET.time[1], WCET.release[1], WCET.release[1] * SEQ_CNT_FOR_S2);
    UARTprintf("WCET of Service 3 = %d usec     at release %d   at Sequencer count %d\n", WCET.time[2], WCET.release[2], WCET.release[2] * SEQ_CNT_FOR_S3);
    UARTprintf("WCET of Service 4 = %d usec     at release %d   at Sequencer count %d\n", WCET.time[3], WCET.release[3], WCET.release[3] * SEQ_CNT_FOR_S4);
    UARTprintf("WCET of Service 5 = %d usec     at release %d   at Sequencer count %d\n", WCET.time[4], WCET.release[4], WCET.release[4] * SEQ_CNT_FOR_S5);
    UARTprintf("WCET of Service 6 = %d usec     at release %d   at Sequencer count %d\n", WCET.time[5], WCET.release[5], WCET.release[5] * SEQ_CNT_FOR_S6);
    UARTprintf("WCET of Service 7 = %d usec     at release %d   at Sequencer count %d\n", WCET.time[6], WCET.release[6], WCET.release[6] * SEQ_CNT_FOR_S7);
    UARTprintf("\n");

    vTaskDelete(NULL);
}


//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.
//
//*****************************************************************************
int main(void)
{
    // Set the clocking to run at 50 MHz from the PLL.
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    // Initialize the UART and configure it for 115,200, 8-N-1 operation.
    ConfigureUART();

    UARTprintf("Starting Sequencer Demo\n");

    // Create a mutex to guard the UART
    g_pUARTSemaphore = xSemaphoreCreateMutex();

    // Initialize the sequencer semaphores
    semSeq = xSemaphoreCreateBinary();
    semS1 = xSemaphoreCreateBinary();
    semS2 = xSemaphoreCreateBinary();
    semS3 = xSemaphoreCreateBinary();
    semS4 = xSemaphoreCreateBinary();
    semS5 = xSemaphoreCreateBinary();
    semS6 = xSemaphoreCreateBinary();
    semS7 = xSemaphoreCreateBinary();

    // Creating service tasks
    xTaskCreate(Service_1, (const portCHAR *)"SERVICE 1", 128, NULL, PRIORITY_SERVICE_1, NULL);
    xTaskCreate(Service_2, (const portCHAR *)"SERVICE 2", 128, NULL, PRIORITY_SERVICE_2, NULL);
    xTaskCreate(Service_3, (const portCHAR *)"SERVICE 3", 128, NULL, PRIORITY_SERVICE_3, NULL);
    xTaskCreate(Service_4, (const portCHAR *)"SERVICE 4", 128, NULL, PRIORITY_SERVICE_4, NULL);
    xTaskCreate(Service_5, (const portCHAR *)"SERVICE 5", 128, NULL, PRIORITY_SERVICE_5, NULL);
    xTaskCreate(Service_6, (const portCHAR *)"SERVICE 6", 128, NULL, PRIORITY_SERVICE_6, NULL);
    xTaskCreate(Service_7, (const portCHAR *)"SERVICE 7", 128, NULL, PRIORITY_SERVICE_7, NULL);
    xTaskCreate(Sequencer, (const portCHAR *)"SEQUENCER", 128, NULL, PRIORITY_SEQUENCER, NULL);

    // Initializing timer
    ConfigureTimer();

    // Start the scheduler.  This should not return.
    vTaskStartScheduler();

    while(1)
    {
    }
}
