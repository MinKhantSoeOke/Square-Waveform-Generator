#include <string.h>   // Include standard string manipulation library
#include <stdio.h>    // Include standard I/O library

#include "wdt/wdt.h"   // Include Watchdog Timer library
#include "pmc/pmc.h"   // Include Power Management Controller library
#include "pio/pio.h"   // Include Parallel Input/Output controller library
#include "pwm/pwm.h"   // Include PWM controller library

#include "board.h"     // Include board-specific definitions
#include "lcd-ge8.h"   // Include LCD screen handling library
#include "adc/adc.h"   // Include Analog-to-Digital Converter library

#include "aic/aic.h"   // Include Advanced Interrupt Controller library
#include "pit/pit.h"   // Include Periodic Interval Timer library

//=============================================================================
// Define macros for various parameters and configurations

#define PWM0_FRQ_MIN    40    // Minimum frequency for PWM
#define PWM0_FRQ_MAX    2500  // Maximum frequency for PWM

#define PWM0_DTY_MIN    10    // Minimum duty cycle percentage
#define PWM0_DTY_MAX    90    // Maximum duty cycle percentage

#define PWM0_DTY_DELTA  10    // Duty cycle adjustment step
#define PWM0_DTY_DFT    50    // Default duty cycle percentage

#define ADC_DELTA_MIN   3     // Minimum change in ADC reading to trigger update

#define ADC_CLKPRES     7     // ADC clock prescaler
#define ADC_STARTTIME   7     // ADC start-up time
#define ADC_SHTINE      2     // ADC sample-and-hold time

#define PWM0_CHNUM      0     // PWM channel number
#define PWM0_CHMASK     AT91C_PWMC_CHID0  // PWM channel mask

#define PWM0_OUT1_BASE  AT91C_BASE_PIOB   // Base address for PWM output
#define PWM0_OUT1_PIN_bm (1 << 19)        // Bit mask for PWM output pin

#define PWM0_CLK        2400000U          // PWM clock frequency

#define MAIN_LOOP_PERIOD        100       // Main loop period in milliseconds
#define PITC_PIV_INITVAL        (MAIN_LOOP_PERIOD * (BOARD_MCK / 16 / 1000) - 1)  // PIT timer interval

volatile unsigned int PITC_Tick;  // Global tick variable for PIT interrupt
//=============================================================================
// Global variables

int pwmOn = 0;    // Flag indicating whether PWM is on or off
int frq0 = PWM0_FRQ_MIN;  // Initial frequency
int dty0 = PWM0_DTY_DFT;  // Initial duty cycle
int adc0 = 0;     // Initial ADC reading

//=============================================================================
// Function to convert ADC reading to frequency

int ADC2Frq(int adc) {
    return PWM0_FRQ_MIN + ((PWM0_FRQ_MAX - PWM0_FRQ_MIN) * adc) / 1023;
}

//=============================================================================
// Interrupt handler for PIT (Periodic Interval Timer)

void PITC_Int_Handler(void) {
    PITC_ReadPIVR(pPITC);  // Read and clear the PIT interrupt status
    PITC_Tick = 1;         // Set tick flag
}

//=============================================================================
// Interrupt handler for system controller

void SYSC_Int_Handler(void) {
    if ((PITC_GetStatus(pPITC) & AT91C_PITC_PITS) != 0)
        PITC_Int_Handler();  // Call PIT interrupt handler if PIT interrupt is triggered
}

//=============================================================================
// Function to display PWM parameters and state on the LCD

void LCD_Display(int frequency, int duty, int state) {
    char buffer[20];  // Buffer for text display
    
    // Display frequency
    LCDGotoXY(5, 50);
    sprintf(buffer, " f0 = %d Hz ", frequency);
    LCDCharColor(YELLOW, BLUE);
    LCDPutStr(buffer);
    
    // Display duty cycle
    LCDGotoXY(5, 70);
    sprintf(buffer, " d0 =   %d %% ", duty);
    LCDCharColor(YELLOW, BLUE);
    LCDPutStr(buffer);
    
    // Display PWM state (ON/OFF)
    LCDGotoXY(5, 20);
    if (state) {
        LCDCharColor(BLACK, GREEN);
        LCDPutStr("  ON   ");
    } else {
        LCDCharColor(YELLOW, RED);
        LCDPutStr("  OFF  ");
    }
}

//=============================================================================
// Function to get ADC conversion result for a specific channel

int ADC_GetChxConvResult(AT91PS_ADC pAdc, int channel) {
    return (pADC->ADC_CDR6);  // Return conversion result for channel 6
}

//=============================================================================

int main() {
    // Declare variables for joystick and button states
    unsigned int joyst0, joyst1, joystREdge;
    unsigned int but0, but1, butREdge;

//-----------------------------------
// Local variables for ADC result and PWM calculations

    int adcResult;  // ADC conversion result
    int CPRDR, CDTYR;  // PWM period and duty cycle register values

//-----------------------------------

// Disable the watchdog timer to prevent system resets during operation
    WDTC_Disable(pWDTC);

//-----------------------------------

// Enable clock for PIOA, PIOB, ADC, and PWM peripherals
    PMC_EnablePeriphClock(pPMC, AT91C_ID_PIOA);
    PMC_EnablePeriphClock(pPMC, AT91C_ID_PIOB);
    PMC_EnablePeriphClock(pPMC, AT91C_ID_ADC);
    PMC_EnablePeriphClock(pPMC, AT91C_ID_PWMC);

//-----------------------------------

// Configure joystick I/O pins as inputs with deglitching
    PIO_CfgPin(JOYSTICK_PIO_BASE, PIO_INPUT, PIO_DEGLITCH, JOYSTICK_ALL_bm);

// Configure buttons I/O pins as inputs with deglitching
    PIO_CfgPin(BUTTONS_PIO_BASE, PIO_INPUT, PIO_DEGLITCH, BUTTON_ALL_bm);

//-----------------------------------

// Configure SPI pins, SPI and LCD controllers
    CfgLCDCtrlPins();
    LCDInitSpi(LCD_SPI_BASE, LCD_SPI_ID);
    LCDInitCtrl(LCDRstPin);

// Clear the LCD screen and initialize character output
    LCDClrScr(BLACK);
    LCDInitCharIO();

// Configure LCD backlight pin and turn on the backlight
    CfgLCDBacklightPin();
    LCDBacklight(LCD_BL_ON);

//-----------------------------------
// Configure the PIT (Periodic Interval Timer) and enable its interrupt

    AIC_CfgIt(pAIC, AT91C_ID_SYS, AT91C_AIC_PRIOR_LOWEST,
              AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL, SYSC_Int_Handler);

    PITC_EnableIt(pPITC);  // Enable PIT interrupt
    AIC_EnableIt(pAIC, AT91C_ID_SYS);  // Enable system interrupt

    PITC_Start(pPITC, PITC_PIV_INITVAL);  // Start PIT with the calculated interval

//-----------------------------------
// Configure ADC and PWM peripherals

    // Set ADC mode and timing
    ADC_SetMode(pADC, AT91C_ADC_TRGEN_DIS, 0, AT91C_ADC_LOWRES_10_BIT, AT91C_ADC_SLEEP_NORMAL_MODE);
    ADC_SetTiming(pADC, ADC_CLKPRES, ADC_STARTTIME, ADC_SHTINE);
    
    // Enable ADC channel 6
    ADC_EnableCh(pADC, AT91C_ADC_CH6);
    
    // Configure PWM output pin
    PIO_CfgPin(PWM0_OUT1_BASE, PIO_PERIPH_A, PIO_DEFAULT, PWM0_OUT1_PIN_bm);
    
    // Set PWM clock and mode
    PWMC_SetClkA(pPWMC, AT91C_PWMC_PRE_MCK_2, 10);
    PWMC_SetChMode(pPWMC, PWM0_CHMASK, AT91C_PWMC_CPRE_MCKA, 0, 0);
    PWMC_SetPrdAndDty(pPWMC, PWM0_CHMASK, AT91C_PWMC_PRE_MCK_2, 10);

//-----------------------------------
// Display initial PWM parameters on the LCD

    LCD_Display(frq0, dty0, pwmOn);

//-----------------------------------
// Initialize joystick and button states

    joyst0 = ~PIO_GetInput(JOYSTICK_PIO_BASE);
    joyst1 = joyst0;

    but0 = ~PIO_GetInput(BUTTONS_PIO_BASE);
    but1 = but0;

//-----------------------------------

    while (1)  // Main loop
    {
        // Read current joystick and button states
        joyst0 = ~PIO_GetInput(JOYSTICK_PIO_BASE);
        but0 = ~PIO_GetInput(BUTTONS_PIO_BASE);

        // Detect rising edge on SW1 button
        butREdge = but0 & ~but1;

        // Detect rising edge on joystick inputs
        joystREdge = ~joyst1 & joyst0;

        // Calculate PWM period and duty cycle
        CPRDR = PWM0_CLK / frq0;
        CDTYR = (CPRDR * dty0) / 100;
        
        // Toggle PWM generator state if SW1 button is pressed
        if (butREdge & BUTTON_SW1_bm) {
            pwmOn = !pwmOn;  // Toggle PWM on/off state
            if (pwmOn == 1) {
                PWMC_SetPrdAndDty(pPWMC, PWM0_CHMASK, CPRDR, CDTYR);  // Set PWM period and duty cycle
                PWMC_EnableCh(pPWMC, PWM0_CHMASK);  // Enable PWM channel
            } else {
                PWMC_DisableCh(pPWMC, PWM0_CHMASK);  // Disable PWM channel
            }
            LCD_Display(frq0, dty0, pwmOn);  // Update LCD display
        }

        // Adjust duty cycle based on joystick input
        if (joystREdge & JOYSTICK_UP_bm) {
            dty0 += PWM0_DTY_DELTA;  // Increase duty cycle
            if (dty0 > PWM0_DTY_MAX) {
                dty0 = PWM0_DTY_MAX;  // Cap duty cycle at maximum
            }
            if (pwmOn == 1) {
                CDTYR = (CPRDR * dty0) / 100;  // Recalculate duty cycle register value
                PWMC_SetPrdAndDty(pPWMC, PWM0_CHMASK, CPRDR, CDTYR);  // Update PWM settings
            }
            LCD_Display(frq0, dty0, pwmOn);  // Update LCD display
        } else if (joystREdge & JOYSTICK_DOWN_bm) {
            dty0 -= PWM0_DTY_DELTA;  // Decrease duty cycle
            if (dty0 < PWM0_DTY_MIN) {
                dty0 = PWM0_DTY_MIN;  // Floor duty cycle at minimum
            }
            if (pwmOn == 1) {
                CDTYR = (CPRDR * dty0) / 100;  // Recalculate duty cycle register value
                PWMC_SetPrdAndDty(pPWMC, PWM0_CHMASK, CPRDR, CDTYR);  // Update PWM settings
            }
            LCD_Display(frq0, dty0, pwmOn);  // Update LCD display
        }

        // Measure voltage from potentiometer using ADC
        ADC_StartConversion(pADC);
        while (!(ADC_GetStatus(pADC) & AT91C_ADC_EOC6));  // Wait for end of conversion
        adcResult = ADC_GetChxConvResult(pADC, 6);  // Get ADC conversion result

        // Update frequency based on ADC reading if the change is significant
        if (abs(adcResult - adc0) > ADC_DELTA_MIN) {
            adc0 = adcResult;
            frq0 = ADC2Frq(adc0);  // Convert ADC value to frequency
            if (pwmOn == 1) {
                CPRDR = PWM0_CLK / frq0;
                CDTYR = (CPRDR * dty0) / 100;
                PWMC_SetPrdAndDty(pPWMC, PWM0_CHMASK, CPRDR, CDTYR);  // Update PWM settings
            }
            LCD_Display(frq0, dty0, pwmOn);  // Update LCD display
        }

        // Store current joystick and button states for next loop iteration
        joyst1 = joyst0;
        but1 = but0;
                
        // Wait until PIT tick occurs before continuing the loop
        while (PITC_Tick == 0);
        PITC_Tick = 0;  // Reset PIT tick flag
    }

    return 0;
}
