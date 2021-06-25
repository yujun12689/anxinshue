/*
* BleCtrl.h
*
*  Created on: Apr 6, 2021
*      Author: yaoyu
*/

#include <Arduino.h>
#ifndef INC_BLE_CTRL_H_
#define INC_BLE_CTRL_H_

enum BLE_CTRL_PCG
{
  BLE_IDLE = 0,
  BLE_CONNECTED,
  BLE_DISCONNECTED,
  BLE_ADV,
  BLE_NOTIFY,
};

// extern uint8_t waitTimeForBleCtrl = 10;

#endif /* INC_BLE_CTRL_H_ */