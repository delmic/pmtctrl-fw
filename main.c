//*****************************************************************************
//
// main.c - This module implements the PMT control logic and the communication
// protocol with the connected devices. Parts of it are based on Tivaware's
// usb_dev_serial example code.
//
// Copyright (c) 2015 Kimon Tsitsikas, Delmic
// 
// This file is part of the PMT Control Unit Firmware.
//
// PMT Control Unit Firmware is free software: you can redistribute it and/or 
// modify it under the terms of the GNU General Public License version 2 as 
// published by the Free Software Foundation.
// 
// PMT Control Unit Firmware is distributed in the hope that it will be useful, 
// but WITHOUT ANY WARRANTY; without even the implied warranty of 
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General 
// Public License for more details.
// 
// You should have received a copy of the GNU General Public License along with 
// PMT Control Unit Firmware. If not, see http://www.gnu.org/licenses/.
//
// Linking PMT Control Unit Firmware statically or dynamically with other 
// modules is making a combined work based on PMT Control Unit Firmware. Thus, 
// the terms and conditions of the GNU General Public License cover the whole 
// combination.
//
// In addition, as a special exception, the copyright holders of PMT Control 
// Unit Firmware give you permission to combine PMT Control Unit Firmware with 
// free software programs or libraries that are released under the GNU LGPL and 
// with code included in the standard release of Tiva Firmware Development 
// Package under its own license (or modified versions of such code, with 
// unchanged license). You may copy and distribute such a system following the 
// terms of the GNU GPL for PMT Control Unit Firmware and the licenses of the 
// other code concerned.
//
// Note that people who make modified versions of PMT Control Unit Firmware are 
// not obligated to grant this special exception for their modified versions; 
// it is their choice whether to do so. The GNU General Public License gives 
// permission to release a modified version without this exception; this 
// exception also makes it possible to release a modified version which carries 
// forward this exception.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/ssi.h"
#include "driverlib/timer.h"
#include "driverlib/usb.h"
#include "driverlib/rom.h"
#include "driverlib/hibernate.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "utils/ustdlib.h"
#include "usb_serial_structs.h"

//Min and Max voltage values in V
#define MAX_VOLT 1.11
#define MIN_VOLT 0
//Min and Max current values in microamperes
#define MAX_PCURR 100
#define MIN_PCURR 0
//Min and Max time values in s
#define MAX_PTIME 100
#define MIN_PTIME 0.000001
//Device identification
#define IDN "Delmic Analog PMT version 1.0"
//12bit DAC range, 2^12 
#define DAC_RANGE 4095
//Mask for DAC A command
#define DAC_A 3145728
//Mask for DAC B command
#define DAC_B 3211264

// The system tick rate expressed both as ticks per second and a millisecond
// period.
#define SYSTICKS_PER_SECOND 100
#define SYSTICK_PERIOD_MS (1000 / SYSTICKS_PER_SECOND)

// Global system tick counter
volatile uint32_t g_ui32SysTickCount = 0;

// Flags used to pass commands from interrupt context to the main loop.
#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002

volatile uint32_t g_ui32Flags = 0;
char *g_pcStatus;
// Global flag indicating that a USB configuration has been set.
static volatile bool g_bUSBConfigured = false;

void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1)
    {
    }
}

void
SysTickIntHandler(void)
{
    // Update our system time.
    g_ui32SysTickCount++;
}

uint32_t
ControlHandler(void *pvCBData, uint32_t ui32Event,
               uint32_t ui32MsgValue, void *pvMsgData)
{
    uint32_t ui32IntsOff;

    switch(ui32Event)
    {

        // We are connected to a host and communication is now possible.
        case USB_EVENT_CONNECTED:
            g_bUSBConfigured = true;

            // Flush our buffers.
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);

            // Tell the main loop to update the display.
            ui32IntsOff = ROM_IntMasterDisable();
            g_pcStatus = "Connected";
            g_ui32Flags |= COMMAND_STATUS_UPDATE;
            if(!ui32IntsOff)
            {
                ROM_IntMasterEnable();
            }
            break;

        // The host has disconnected.
        case USB_EVENT_DISCONNECTED:
            g_bUSBConfigured = false;
            ui32IntsOff = ROM_IntMasterDisable();
            g_pcStatus = "Disconnected";
            g_ui32Flags |= COMMAND_STATUS_UPDATE;
            if(!ui32IntsOff)
            {
                ROM_IntMasterEnable();
            }
            break;

        // Ignore SUSPEND and RESUME for now.
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        default:
            break;
    }

    return(0);
}

//Callbacks are kept for later use
uint32_t
TxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    switch(ui32Event)
    {
        case USB_EVENT_TX_COMPLETE:
            // Since we are using the USBBuffer, we don't need to do anything
            // here.
            break;
        default:
            break;
    }
    return(0);
}

uint32_t
RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
          void *pvMsgData)
{
    return(0);
}

void SPIsendInt(uint32_t num){
    SSIDataPut(SSI0_BASE, num);
    // Wait until done transferring
    while(SSIBusy(SSI0_BASE))
    {
    }
}

float GetElapsedTime(){
    time_t rawtime; 
    float secdec, cur_time;

    rawtime = HibernateRTCGet();         // read current RTC counter in seconds
    secdec = HibernateRTCSSGet() / 32768.0;  // sub-second accuracy
    cur_time = rawtime + secdec;
    return cur_time;
}

int
main(void)
{
    char stringRecv[32]; //32 bytes will be reserved for this array
    char buffer[32]; //32 bytes will be reserved for this array
    uint32_t ui32Read;
    uint8_t ui8Char;
    char* token;
    float value;
    char cThisChar = ' ';
    int i=0,j=0,c=0;
    int wspaces=0; 
    int qmarks=0; 
    long lvalue;
    int s_length;
    double time_spent;
    double begin = 0, end;
    //Dummy values for initialization
    char* idn = IDN; 
    int pwr = 0;
    int prot = 0;
    int relay = 0;
    float volt = 0, tvolt;
    float pcurr = 50, tpcurr;
    float ptime = 0.001, tptime;
    //SPI buffers
    uint32_t pui32DataTx, pui32DataRx, dac_data;
    //GPIO input
    uint8_t gpin, gpout, trip_bit;

    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    ROM_FPULazyStackingEnable();

    // Set the clocking to run from the PLL at 50MHz
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);

    // Configure the required pins for USB operation.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_5 | GPIO_PIN_4);

    // Enable the GPIO port that is used for the on-board LED.
    //ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Enable the GPIO pins for the LED (PF2 & PF3).
    //ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1);

    // Not configured initially.
    g_bUSBConfigured = false;

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable the system tick.
    ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / SYSTICKS_PER_SECOND);
    ROM_SysTickIntEnable();
    ROM_SysTickEnable();

    // Initialize the transmit and receive buffers.
    USBBufferInit(&g_sTxBuffer);
    USBBufferInit(&g_sRxBuffer);

    // Set the USB stack mode to Device mode with VBUS monitoring.
    USBStackModeSet(0, eUSBModeForceDevice, 0);

    // Pass our device information to the USB library and place the device
    // on the bus.
    USBDCDCInit(0, &g_sCDCDevice);

    // Configure GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    // pin 0 - PROT, pin 1 - PWR, pin 3 - RELAY
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
    // pin 2 - Comparator result
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_7);

    // Configure SPI
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK); //SCK
    GPIOPinConfigure(GPIO_PA3_SSI0FSS); //CS
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);  //SDI
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                   GPIO_PIN_2);
    // 12-bit DAC
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 8);
    SSIEnable(SSI0_BASE);
    // Get rid of residual data from the SSI port
    while(SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx))
    {
    }

    SysCtlPeripheralEnable(SYSCTL_PERIPH_HIBERNATE);
    HibernateEnableExpClk(SysCtlClockGet());
    SysCtlDelay(10);
    HibernateClockConfig(HIBERNATE_OSC_LOWDRIVE);
    HibernateRTCSet(0);
    HibernateRTCEnable(); 

    //
    // Main application loop.
    //
    while(1)
    {

        if(g_ui32Flags & COMMAND_STATUS_UPDATE)
        {
            // Clear the command flag
            ROM_IntMasterDisable();
            g_ui32Flags &= ~COMMAND_STATUS_UPDATE;
            ROM_IntMasterEnable();
        }

        do
        {
	    // In the meantime, check for protection trip
	    gpin = GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7);
	    // Mask to read the comparator returned bit
            trip_bit = gpin & 0x04;
	    if (trip_bit == 4){
	    	if (begin == 0){
	     	    begin = GetElapsedTime();
	    	}
		else{
	    	    end = GetElapsedTime();   
	    	    time_spent = (double)(end - begin);
		    if (time_spent > ptime){
			//Force protection to active
	                prot = 1;
		        // Set pwr and prot pin values
		        gpout = (pwr<<1) | prot;	
		        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1, gpout);
		    }
		}
	    }
	    else{
		begin = 0;
	    }

	    ui32Read = USBBufferRead((tUSBBuffer *)&g_sRxBuffer, &ui8Char, 1);
	    if(ui32Read)
	    {
	        cThisChar = (char)ui8Char;
		stringRecv[i] = cThisChar;
		i++;  
	    }  
        }
        while((cThisChar != '\n') || (i == 0));

	//Check if it's a setter
	s_length = strlen(stringRecv);
	while( j < s_length)  {
	   //Search for special symbols
	   j++;
	   if(stringRecv[j] == ' ')
	      wspaces++;
	   else if(stringRecv[j] == '?')
	      qmarks++;
	}

	//
        //Protocol implementation for PMT ctrl unit command processing
        //
	if (((wspaces > 0) && (qmarks > 0)) || (wspaces > 1) || (qmarks > 1)){
    	    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "ERROR: Cannot parse this command\n", 33);
	}
	else if (wspaces){
	    // split data to get the value in case of setter
	    token = strtok(stringRecv, " ");  
	    token = strtok(NULL," \n\r");
	    value = ustrtof(token, NULL);
	    if (strcmp(stringRecv,"PWR") == 0){
	        if ((value != 0) && (value != 1))
		    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "ERROR: Out of range set value\n", 30);
	        else{
	            pwr = value;
		    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "\n", 1);
		    // Set pwr and prot pin values
		    gpout = (pwr<<1) | prot;	
		    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1, gpout);
		}
	    }
	    else if (strcmp(stringRecv,"PROT") == 0){
	        if ((value != 0) && (value != 1))
		    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "ERROR: Out of range set value\n", 30);
	        else{
	            prot = value;
		    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "\n", 1);
		    // Set pwr and prot pin values
		    gpout = (pwr<<1) | prot;	
		    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1, gpout);
		}
	    }
	    else if (strcmp(stringRecv,"RELAY") == 0){
	        if ((value != 0) && (value != 1))
		    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "ERROR: Out of range set value\n", 30);
	        else{
	            relay = value;
		    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "\n", 1);
		    gpout = (relay<<3) | (pwr<<1) | prot;	
		    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3, gpout);
		}
	    }
	    else if (strcmp(stringRecv,"VOLT") == 0){
	        if ((value < MIN_VOLT) || (value > MAX_VOLT))
		    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "ERROR: Out of range set value\n", 30);
	        else{
	            volt = value;
		    dac_data = (volt / (MAX_VOLT - MIN_VOLT)) * DAC_RANGE;
		    //Prepare command for DAC
		    pui32DataTx = (dac_data<<4) | DAC_A;
		    SPIsendInt(pui32DataTx);
		    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "\n", 1);
		}
            }
	    else if (strcmp(stringRecv,"PCURR") == 0){
	        if ((value < MIN_PCURR) || (value > MAX_PCURR))
		    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "ERROR: Out of range set value\n", 30);
	        else{
	            pcurr = value;
		    dac_data = (pcurr / (MAX_PCURR - MIN_PCURR)) * DAC_RANGE;
		    //Prepare command for DAC
                    pui32DataTx = (dac_data<<4) | DAC_B;
		    SPIsendInt(pui32DataTx);
		    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "\n", 1);
		}
            }
	    else if (strcmp(stringRecv,"PTIME") == 0){
	        if ((value < MIN_PTIME) || (value > MAX_PTIME))
		    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "ERROR: Out of range set value\n", 30);
	        else{
	            ptime = value;
		    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "\n", 1);
		}
            }
	    else{
		USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "ERROR: Out of range set value\n", 30);
	    }	
	}
	else if (qmarks){
	    token = strtok(stringRecv, " \n\r");
	    if (strcmp(stringRecv,"*IDN?") == 0){
		for(c=0; idn[c]!='\0'; ++c);
		USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, idn, c);
		USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "\n", 1);
	    }
	    else if (strcmp(stringRecv,"PWR?") == 0){
		//int to string
            	usnprintf(buffer, 7, "%d", pwr);
		for(c=0; buffer[c]!='\0'; ++c);
		USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, buffer, c);
		USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "\n", 1);
	    }
	    else if (strcmp(stringRecv,"VOLT?") == 0){
		//float to string
		tvolt = volt;
		lvalue = (long)tvolt;
	    	tvolt -= lvalue;
            	usnprintf(buffer, 15, "%d.%06d", lvalue, (long)(tvolt * 1000000));
		for(c=0; buffer[c]!='\0'; ++c);
		USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, buffer, c);
		USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "\n", 1);
	    }
	    else if (strcmp(stringRecv,"PCURR?") == 0){
		//float to string
		tpcurr = pcurr;
		lvalue = (long)tpcurr;
	    	tpcurr -= lvalue;
            	usnprintf(buffer, 15, "%d.%06d", lvalue, (long)(tpcurr * 1000000));
		for(c=0; buffer[c]!='\0'; ++c);
		USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, buffer, c);
		USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "\n", 1);
	    }
	    else if (strcmp(stringRecv,"PTIME?") == 0){
		//float to string
		tptime = ptime;
		lvalue = (long)tptime;
	    	tptime -= lvalue;
            	usnprintf(buffer, 15, "%d.%06d", lvalue, (long)(tptime * 1000000));
		for(c=0; buffer[c]!='\0'; ++c);
		USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, buffer, c);
		USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "\n", 1);
	    }
	    else if (strcmp(stringRecv,"PROT?") == 0){
		//int to string
            	usnprintf(buffer, 7, "%d", prot);
		for(c=0; buffer[c]!='\0'; ++c);
		USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, buffer, c);
		USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "\n", 1);
	    }
	    else if (strcmp(stringRecv,"RELAY?") == 0){
		//int to string
            	usnprintf(buffer, 7, "%d", relay);
		for(c=0; buffer[c]!='\0'; ++c);
		USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, buffer, c);
		USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "\n", 1);
	    }
	    else{
    	    	USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "ERROR: Cannot parse this command\n", 33);
	    }	
	}
	else {
    	    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, "ERROR: Cannot parse this command\n", 33);
	}

	i=0;
	j=0;
	wspaces = 0;
	qmarks = 0;
	//Empty string
	memset(stringRecv,0,sizeof(stringRecv));
	memset(buffer,0,sizeof(buffer));
    }
}
