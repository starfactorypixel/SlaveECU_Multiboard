#pragma once

#include  <DRV8874.h>

namespace TrunkHood
{
	static constexpr uint32_t CFG_RefVoltage = 3324000;		// Опорное напряжение, микровольты.
	static constexpr uint16_t CFG_LoadResistance = 2490;	// Сопротивление шунта, омы.
	static constexpr uint16_t CFG_IdleCurrent = 100;		// Ток, меньше которого считаем что нагрузки нет, мА.
	static constexpr uint16_t CFG_FindDelay = 50;			// Время задержки при поиске положения, мс.
	static constexpr uint16_t CFG_StickIdleTime = 400;		// Время, через которое выключится актуатор, после пропадания флуда set команды.


	enum state_t : uint8_t { STATE_UNKNOWN, STATE_STOPPED, STATE_CLOSING, STATE_CLOSED, STATE_OPENING, STATE_OPENED };

	struct actuator_data_t
	{
		state_t state;				// Текущее (вновь установленное) состояние акутатора.
		state_t prev_state;			// Пред. состояние акутатора.
		int8_t last_rx_position;	// Последнее полученное значение с джостика.
		uint32_t last_rx_time;		// Время получения последнего значение с джостика.
	} actuator_data[2];

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

	void LogicToggle(DRV8874 &driver, actuator_data_t &data)
	{
		switch(data.state)
		{
			case STATE_CLOSED:
			{
				driver.ActionRight();
				data.prev_state = data.state;
				data.state = STATE_OPENING;
				
				break;
			}
			case STATE_OPENED:
			{
				driver.ActionLeft();
				data.prev_state = data.state;
				data.state = STATE_CLOSING;
				
				break;
			}
			case STATE_CLOSING:
			case STATE_OPENING:
			{
				driver.ActionStop();
				data.prev_state = data.state;
				data.state = STATE_STOPPED;
				
				break;
			}
			case STATE_STOPPED:
			{
				state_t tmp = data.prev_state;
				data.prev_state = data.state;
				if(tmp == STATE_CLOSING)
				{
					driver.ActionRight();
					data.state = STATE_OPENING;
				}
				if(tmp == STATE_OPENING)
				{
					driver.ActionLeft();
					data.state = STATE_CLOSING;
				}
				
				break;
			}
			default:
			{
				driver.ActionStop();
				data.prev_state = STATE_OPENING;
				data.state = STATE_STOPPED;

				break;
			}
		}

		return;
	}

	void TimeLogicToggleOff(DRV8874 &driver, actuator_data_t &data)
	{
		if(driver.GetCurrent(true) < CFG_IdleCurrent)
		{
			if(data.state == STATE_CLOSING)
			{
				driver.ActionOff();
				data.prev_state = data.state;
				data.state = STATE_CLOSED;
				data.last_rx_position = 0;
			}
			if(data.state == STATE_OPENING)
			{
				driver.ActionOff();
				data.prev_state = data.state;
				data.state = STATE_OPENED;
				data.last_rx_position = 0;
			}
		}
		
		return;
	}

	void LogicSet(DRV8874 &driver, actuator_data_t &data, int8_t stick_position)
	{
		
		data.prev_state = data.state;
		if(stick_position > 0 && data.last_rx_position <= 0)
		{
			driver.ActionRight();
			data.state = STATE_OPENING;
		}
		else if(stick_position < 0 && data.last_rx_position >= 0)
		{
			driver.ActionLeft();
			data.state = STATE_CLOSING;
		}
		else if(stick_position == 0 && data.last_rx_position != 0)
		{
			driver.ActionStop();
			data.state = STATE_STOPPED;
		}
		data.last_rx_position = stick_position;
		data.last_rx_time = HAL_GetTick();
		
		return;
	}

	void TimeLogicSetOff(DRV8874 &driver, actuator_data_t &data, uint32_t current_time)
	{
		if(current_time - data.last_rx_time > CFG_StickIdleTime && (data.state == STATE_OPENING || data.state == STATE_CLOSING) && (data.last_rx_position != -100 && data.last_rx_position != 100 && data.last_rx_position != 0))
		{
			driver.ActionStop();
			data.prev_state = data.state;
			data.state = STATE_STOPPED;
			data.last_rx_position = 0;
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

		actuator_data[0].prev_state = STATE_CLOSING;
		actuator_data[1].prev_state = STATE_CLOSING;
		actuator_data[0].state = FindPosition(driver1);
		actuator_data[1].state = FindPosition(driver2);
		
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

			TimeLogicToggleOff(driver1, actuator_data[0]);
			TimeLogicToggleOff(driver2, actuator_data[1]);
		}

		TimeLogicSetOff(driver1, actuator_data[0], current_time);
		TimeLogicSetOff(driver2, actuator_data[1], current_time);
		
		
		
		current_time = HAL_GetTick();

		return;
	}
	
}
