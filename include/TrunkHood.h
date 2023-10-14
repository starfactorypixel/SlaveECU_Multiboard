#pragma once

#include  <DRV8874.h>

namespace TrunkHood
{
	static constexpr uint32_t CFG_RefVoltage = 3324000;		// Опорное напряжение, микровольты.
	static constexpr uint16_t CFG_LoadResistance = 2490;	// Сопротивление шунта, омы.
	static constexpr uint16_t CFG_IdleCurrent = 100;		// Ток, меньше которого считаем что нагрузки нет, мА.
	static constexpr uint16_t CFG_FindDelay = 50;			// Время задержки при поиске положения, мс.


	enum state_t : uint8_t { STATE_UNKNOWN, STATE_STOPPED, STATE_CLOSING, STATE_CLOSED, STATE_OPENING, STATE_OPENED };
	
	state_t state1;
	state_t prev_state1;
	state_t state2;
	state_t prev_state2;


	// Драйвер актуатора капота
	DRV8874 driver1( {GPIOB, GPIO_PIN_1},  {GPIOB, GPIO_PIN_0},  {GPIOB, GPIO_PIN_8}, {GPIOC, GPIO_PIN_15}, {GPIOA, ADC_CHANNEL_7} );
	
	// Драйвер актуатора багажника
	DRV8874 driver2( {GPIOB, GPIO_PIN_10}, {GPIOB, GPIO_PIN_11}, {GPIOB, GPIO_PIN_9}, {GPIOC, GPIO_PIN_14}, {GPIOA, ADC_CHANNEL_0} );


	state_t FindPosition(DRV8874 &driver)
	{
		state_t state;
		uint16_t current;

		driver.ActionLeft();
		HAL_Delay(CFG_FindDelay);
		current = driver.GetCurrent(true);
		if( current > CFG_IdleCurrent )
		{
			driver.ActionRight();
			HAL_Delay(CFG_FindDelay);
			current = driver.GetCurrent(true);
			if( current > CFG_IdleCurrent )
			{
				state = STATE_STOPPED;
			}
			else
			{
				state = STATE_OPENED;
			}
		}
		else
		{
			state = STATE_CLOSED;
		}
		driver.ActionStop();
		
		return state;
	}

	void LogicOn(DRV8874 &driver, state_t &prev_state, state_t &state)
	{
		switch(state)
		{
			case STATE_CLOSED:
			{
				driver.ActionRight();
				prev_state = state;
				state = STATE_OPENING;
				
				break;
			}
			case STATE_OPENED:
			{
				driver.ActionLeft();
				prev_state = state;
				state = STATE_CLOSING;
				
				break;
			}
			case STATE_CLOSING:
			case STATE_OPENING:
			{
				driver.ActionStop();
				prev_state = state;
				state = STATE_STOPPED;
				
				break;
			}
			case STATE_STOPPED:
			{
				state_t tmp = prev_state;
				prev_state = state;
				if(tmp == STATE_CLOSING)
				{
					driver.ActionRight();
					state = STATE_OPENING;
				}
				if(tmp == STATE_OPENING)
				{
					driver.ActionLeft();
					state = STATE_CLOSING;
				}
				
				break;
			}
			default:
			{
				driver.ActionStop();
				prev_state = STATE_OPENING;
				state = STATE_STOPPED;

				break;
			}
		}

		return;
	}

	void LogicOff(DRV8874 &driver, state_t &prev_state, state_t &state)
	{
		if(driver.GetCurrent(true) < CFG_IdleCurrent)
		{
			if(state == STATE_CLOSING)
			{
				driver.ActionOff();
				prev_state = state;
				state = STATE_CLOSED;
			}
			if(state == STATE_OPENING)
			{
				driver.ActionOff();
				prev_state = state;
				state = STATE_OPENED;
			}
		}
		
		return;
	}
	
	
	inline void Setup()
	{
		driver1.Init();
		driver2.Init();

		driver1.SetTimeout(30000);
		driver2.SetTimeout(30000);

		driver1.SetCurrentParam(CFG_RefVoltage, CFG_LoadResistance);
		driver2.SetCurrentParam(CFG_RefVoltage, CFG_LoadResistance);

		prev_state1 = STATE_CLOSING;
		prev_state2 = STATE_CLOSING;
		state1 = FindPosition(driver1);
		state2 = FindPosition(driver2);
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		driver1.Processing(current_time);
		driver2.Processing(current_time);

		static uint32_t last_time = 0;
		if(current_time - last_time > 50)
		{
			last_time = current_time;

			LogicOff(driver1, prev_state1, state1);
			LogicOff(driver2, prev_state2, state2);
		}
		
		current_time = HAL_GetTick();

		return;
	}
	
}
