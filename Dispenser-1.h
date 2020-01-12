/*
/
  Copyright 2019
  Authors:
   Todd Gilbert
   DJ Wadhwa
  Contact:
    awsunit@uw.edu
    djwadhwa@uw.edu

  This file contains declarations and aliases for a Tiva board program which:
    1. Is freeRTOS controlled
    2. controls a servo to open/close a 3D printed device
    3. device dispenses cereal
    4. LCD screen displays ??????????????????

/
*/
/**/
#ifndef DISPENSER_H_
#define DISPENSER_H_
/**/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
/**/
/*PIN nums*/
#define P0 0x1
#define P1 0x2
#define P2 0x4
#define P3 0x8
#define P4 0x10
#define P5 0x20
#define P6 0x40
#define P7 0x80

#define MILLI_IN_DAY 86400000
#define MAX_SCHEDULES 20
/* up/down -> set time, override -> opens device */
#define NUM_BUTTONS 3
/* red */
#define NUM_LEDS 1
/**/
// UART enable UART modules
// offset: 0x618 | pg. 344
#define TSYSCTL_RCGCUART_R  (*((volatile uint32_t *) 0x400FE618))
#define TUART1_CTL_R (*((volatile uint32_t *)0x4000D030))
#define TUART1_LCRH_R (*((volatile uint32_t *)0x4000D02C))
#define TUART1_CC_R (*((volatile uint32_t *)0x4000DFC8))
#define TUART1_IBRD_R (*((volatile uint32_t *)0x4000D024))
#define TUART1_FBRD_R (*((volatile uint32_t *)0x4000D028))
#define TUART1_DR_R (*((volatile uint32_t *)0x4000D000))
#define TUART1_FR_R (*((volatile uint32_t *)0x4000D018))
//
#define TUART2_CTL_R (*((volatile uint32_t *)0x4000E030))
#define TUART2_LCRH_R (*((volatile uint32_t *)0x4000E02C))
#define TUART2_CC_R (*((volatile uint32_t *)0x4000EFC8))
#define TUART2_IBRD_R (*((volatile uint32_t *)0x4000E024))
#define TUART2_FBRD_R (*((volatile uint32_t *)0x4000E028))
#define TUART2_DR_R (*((volatile uint32_t *)0x4000E000))
#define TUART2_FR_R (*((volatile uint32_t *)0x4000E018))
//
#define TSYSCTL_RCGC2_R (*((volatile uint32_t *)0x400FE108))
#define TGPIO_PORTB_AFSEL_R (*((volatile uint32_t *)0x40005420))
#define TGPIO_PORTB_DEN_R (*((volatile uint32_t *)0x4000551C))
#define TGPIO_PORTB_PCTL_R (*((volatile uint32_t *)0x4000552C))
#define TGPIO_PORTB_DIR_R (*((volatile uint32_t *) 0x40005400))
// clock gating control | pg. 340
#define TSYSCTL_RCGCGPIO_R (*((volatile uint32_t *) 0x400FE608))
/**/
/* Port A */
/**/
#define TSYSCTL_RCGC2_GPIOA 0x01
// mode control select register
// bit clear -> pin used as GPIO and controlled via
// GPIO registers!
// chooses peripheral function?
// offset: 0x420 | pg. 671
#define TGPIO_PORTA_AFSEL_R (*((volatile uint32_t *) 0x40004420))
// GPIO Port Control, offset 0x52C | pg. 688
// sets signal definition
#define TGPIO_PORTA_PCTL_R  (*((volatile uint32_t *)0x4000452C))
// DEN allows digit output
#define TGPIO_PORTA_DEN_R (*((volatile uint32_t *) 0x4000451C))
// controls isolation circuits
// analog circuitry requires isolations from pins
// offset: 0x528 | pg. 687
#define TGPIO_PORTA_AMSEL_R (*((volatile uint32_t *) 0x40004528))
// GPIODIR offset: 0x400
#define TGPIO_PORTA_DIR_R (*((volatile uint32_t *) 0x40004400))
//
#define TGPIO_PORTA_DATA_R (*((volatile uint32_t *) 0x400043FC))
/**/
/* Port D */
/**/
#define TSYSCTL_RCGC2_GPIOD 0x00000008  // Port D Clock Gating Control
// mode control select register
// bit clear -> pin used as GPIO and controlled via
// GPIO registers!
// chooses peripheral function?
// offset: 0x420 | pg. 671
#define TGPIO_PORTD_AFSEL_R (*((volatile uint32_t *) 0x40007420))
// GPIO Port Control, offset 0x52C | pg. 688
// sets signal definition
#define TGPIO_PORTD_PCTL_R  (*((volatile uint32_t *)0x4000752C))
// DEN allows digit output
#define TGPIO_PORTD_DEN_R (*((volatile uint32_t *) 0x4000751C))
// controls isolation circuits
// analog circuitry requires isolations from pins
// offset: 0x528 | pg. 687
#define TGPIO_PORTD_AMSEL_R (*((volatile uint32_t *) 0x40007528))
// GPIODIR offset: 0x400
#define TGPIO_PORTD_DIR_R (*((volatile uint32_t *) 0x40007400))
//
#define TGPIO_PORTD_DATA_R (*((volatile uint32_t *) 0x400073FC))
// pg. 684
#define TGPIO_PORTD_LOCK_R (*((volatile uint32_t *)0x40007520))
// pg. 685
#define TGPIO_PORTD_CR_R (*((volatile uint32_t *)0x40007524))
/**/
/* PORT F */
/**/
#define TGPIO_PORTF_AFSEL_R (*((volatile uint32_t *) 0x40025420))
#define TGPIO_PORTF_PCTL_R  (*((volatile uint32_t *)0x4002552C))
#define TGPIO_PORTF_DEN_R (*((volatile uint32_t *) 0x4002551C))
#define TGPIO_PORTF_AMSEL_R (*((volatile uint32_t *) 0x40025528))
#define TGPIO_PORTF_DIR_R (*((volatile uint32_t *) 0x40025400))
#define TGPIO_PORTF_DATA_R (*((volatile uint32_t *) 0x400253FC))
#define TGPIO_PORTF_LOCK_R (*((volatile uint32_t *)0x40025520))
#define TGPIO_PORTF_CR_R (*((volatile uint32_t *)0x40025524))
#define TGPIO_PORTF_PUR_R (*((volatile uint32_t *)0x40025510))
#define TGPIO_PORTF_PDR_R (*((volatile uint32_t *) 0x40025514))
#define TSYSCTL_RCGC2_GPIOF      0x00000020  // Port F Clock Gating Control

/*PWM configuration*/
//enable system clock to sync with pwm frequency
#define RCC (*(volatile unsigned int *) 0x400FE060)
//clock gating control for pwm
#define RCGCPWM (*(volatile unsigned int *) 0x400FE640)
//ready status for pwm
#define PRPWM (*(volatile unsigned int *) 0x400FEA40)
//controller for pwm generator
#define PWM0_CTL (*(volatile unsigned int *) 0x40029040)
//store load value for pwm (in relation to servo frequency and sys clock)
#define PWM0_LOAD (*(volatile unsigned int *) 0x40029050)
//pwm duty value (change the angle of the servo)
#define PWM0_CMPA (*(volatile unsigned int *) 0x40029058)
//align pwm
#define PWM0_GENA (*(volatile unsigned int *) 0x40029060)
//pwm module enabler
#define PWM_ENABLE (*(volatile unsigned int *) 0x40029008)
//clock gating for GPIO
#define RCGCGPIO (*(volatile unsigned int *) 0x400FE608)
//direction for port d
#define GPIODIR_PORTD (*(volatile unsigned int *) 0x40007400)
//digital enable for port d
#define GPIODEN_PORTD (*(volatile unsigned int *) 0x4000751C)
//port control for port d
#define GPIOPCTL_PORTD (*(volatile unsigned int *) 0x4000752C)
//alt function for port d
#define GPIOAFSEL_PORTD (*(volatile unsigned int *) 0x40007420)

/*
  Used for booking dispensings
*/
typedef struct t {
  long time;
  int triggered_today;
} Booking;
/**/
/*
  Initializes registers/pins for use with:
    Servo
    LCD
    External Button
    UART - read from putty
*/
void setup(void);
/**/
/*
  Instructs servo to open 3D device

  - Opens device iff:
     Open Button being pressed
     scheduled_open_status == 1
*/
void Servo_Open(void *p);
/**/
/*
  Determines if a client is attempting to:
    add a new time when the 3D device should operate
    sets global flag if both buttons pressed
      - flag triggers I/O process
      - I/O requests information from client
*/
void Client_Schedule(void *p);
/**/
/*
  Determines if any Record in records are due for servicing
  modifies: scheduled_open_status
  effects: scheduled_open_status = 1 iff any record is due for servicing
*/
void Schedule_Check(void *p);
/*
  Updates global clock variable
*/
void Clock_Update(void *p);
/**/
/*
  Changes the LCD to display information

  - Tracks seconds since last display:
    updates every five seconds

  int c could be (command c), maybe unnecessary
*/
void LCD_Update(void *p);
/**/
/*

*/
void FSM(void);
/**/
/*
  Initializes UART0 for client I/O
*/
void UART_Init(void);
/**/
/*
  Configures Tiva to control Servo
*/
void Servo_Init(void);
/**/
/*
  Configures Tiva for use with external toggle buttons
*/
void Button_Init(void);
/*
  Helper to make sure button is recieving a true press
  eventual replacement with iterrupt I'm sure
*/
void pop_button(void);
/*
*/
int switch_input(int PIN);
/**/
/*
  Checks if down button is being pressed
  returns: 1 iff button pressed for 2 seconds
           0 otherwise
*/
int down_button_pressed(void);
/**/
/*
  Checks if up button is being pressed
  returns: 1 iff button pressed for 2 seconds
           0 otherwise
*/
int up_button_pressed(void);
/*
  Helper to accept client input
  returns: the hour to open the 3D device
*/
int GetHours(void);
/*
  Helper to accept client input
  returns: the minutes to open the 3D device
*/
int GetMinutes(void);
/**/
/*
  Checks if override button is being pressed
  returns: 1 iff button pressed for 2 seconds
           0 otherwise
*/
int override_button_pressed(void);
/*
  Helper for printing to LCD
*/
void TransmitData(char* data);
/*
  Helper method for LCD to print a whole c-style string
*/
void StringToChar(char * sentence);
/**/
/*
  Helper function to create mask for PCTL_R
  args: PIN is a the number to left shift OxF by
  returns: 0xF << (4 * PIN)
*/
int mask_regular_GPIO(int PIN);
/**/
/*
  called by FreeRTOS when stack overflow occurs
*/
void vApplicationStackOverflowHook(xTaskHandle *pxTask, char *pcTaskName);
/**/
#endif  // DISPENSER_H_
/**/
/*EOF*/
