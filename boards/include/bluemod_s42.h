
/**************************************************************************
 *
 * $Header: /var/lib/cvs/oemsw/nordic/nRF5_SDK_11.0.0_89a8197/external/stoDevKit/ble_peripheral/include/bluemod_s42.h,v 1.1 2016/07/25 10:14:32 mj Exp $
 *
 * File:        $RCSfile: bluemod_s42.h,v $
 * Version:     $Name:
 *
 * Archive:     $Source: /var/lib/cvs/oemsw/nordic/nRF5_SDK_11.0.0_89a8197/external/stoDevKit/ble_peripheral/include/bluemod_s42.h,v $
 * Revision:    $Revision: 1.1 $
 * Date:        $Date: 2016/07/25 10:14:32 $
 * Author:      $Author: mj $
 *
 * ------------------------------------------------------------------------
 * !MODULE      [  ]
 * ------------------------------------------------------------------------
 * !FILE        [  ]
 * !PROGRAM     [  ]
 * !VERSION     [$Name:]
 * !GROUP       [  ]
 * !AUTHOR      [$Author: mj $]
 *
 *          Copyright (c)           2016 Telit Wireless Solutions
 *                                  Mendelssohnstrasse 15
 *                                  22761 Hamburg
 *                                  Phone: 040/89088-0
 *          All Rights Reserved
 *
 * ------------------------------------------------------------------------
 * !DESCRIPTION
 *
 *  Module I/O definitions for BlueMod+S42
 *
 * ------------------------------------------------------------------------
 *
 * $Log: bluemod_s42.h,v $
 * Revision 1.1  2016/07/25 10:14:32  mj
 * issue #0014746
 * Init. revision.
 *
 *
 **************************************************************************/

#ifndef BLUEMOD_S42_H
#define BLUEMOD_S42_H

#include "nrf_sdm.h"

#define BOOT0					 25
#define TESTMODE			 8

#define BOOT0_NRF_GPIO_PIN_PULL_T     NRF_GPIO_PIN_PULLDOWN
#define TESTMODE_NRF_GPIO_PIN_PULL_T  NRF_GPIO_PIN_PULLUP

#define START_FIRMWARE          (1 << TESTMODE)
#define START_DIRECT_TESTMODE   (1 << BOOT0)
#define START_STO_TESTMODE   		(0)
#define START_BOOT_LOADER       ((1 << TESTMODE) | (1 << BOOT0)) 

#define IURIN					29
#define IUROUT				4

#define HWDETECT0			18 ???
#define HWDETECT1			20 ???

#define RX_PIN_NUMBER  13    // UART RX pin number.
#define TX_PIN_NUMBER  17    // UART TX pin number.
#define CTS_PIN_NUMBER 12    // UART Clear To Send pin number. Not used if HWFC is set to false
#define RTS_PIN_NUMBER 7     // Not used if HWFC is set to false
//#define HWFC           false  // UART hardware flow control

#define GPIO0					27
#define GPIO1					31
#define GPIO2					14
#define GPIO3					 2
#define GPIO4					30
#define GPIO5					 5
#define GPIO6					 3
#define GPIO7					28
#define GPIO8					 6
#define GPIO9					18
#define GPIO10				22
#define GPIO11				20
#define GPIO12				16
#define GPIO13				15
#define GPIO14				26

#define IOA						GPIO8
#define IOB						GPIO3
#define HANGUP				                GPIO4
#define HANGUP_NRF_GPIO_PIN_PULL_T    NRF_GPIO_PIN_PULLDOWN
#define HANGUP_ACTIVE_STATE           1

#define BSP_UART_SUPPORT	1


#endif // BLUEMOD_S42_H
