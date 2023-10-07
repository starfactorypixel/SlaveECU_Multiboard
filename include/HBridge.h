#pragma once

#include  <DRV8874.h>

namespace HBridge
{
	static constexpr uint32_t CFG_RefVoltage = 3324000;		// Опорное напряжение, микровольты.
	static constexpr uint16_t CFG_LoadResistance = 2490;	// Сопротивление шунта, омы.

	DRV8874 driver1( {GPIOB, GPIO_PIN_1},  {GPIOB, GPIO_PIN_0},  {GPIOB, GPIO_PIN_8}, {GPIOC, GPIO_PIN_15}, {GPIOA, ADC_CHANNEL_7} );
	DRV8874 driver2( {GPIOB, GPIO_PIN_10}, {GPIOB, GPIO_PIN_11}, {GPIOB, GPIO_PIN_9}, {GPIOC, GPIO_PIN_14}, {GPIOA, ADC_CHANNEL_0} );
	
	inline void Setup()
	{
		driver1.Init();
		driver2.Init();

		driver1.SetTimeout(30000);
		driver2.SetTimeout(30000);

		driver1.SetCurrentParam(CFG_RefVoltage, CFG_LoadResistance);
		driver2.SetCurrentParam(CFG_RefVoltage, CFG_LoadResistance);
	
	/*	
		driver2.Action(DRV8874::DIR_LEFT);
		HAL_Delay(5000);
		driver2.Action(DRV8874::DIR_RIGHT);
		HAL_Delay(5000);
		driver2.Action(DRV8874::DIR_STOP);
		HAL_Delay(5000);
		driver2.Action(DRV8874::DIR_OFF);
		HAL_Delay(5000);
	*/
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		driver1.Processing(current_time);
		driver2.Processing(current_time);
		
		current_time = HAL_GetTick();

		return;
	}
}
