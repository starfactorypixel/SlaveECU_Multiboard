#pragma once

#include <inttypes.h>
#include "MovingAverage.h"

extern ADC_HandleTypeDef hadc1;

class DRV8874
{
	// Current mirror scaling factor from datasheet.
	static constexpr float _current_scaling = 0.45f;
	static constexpr uint32_t _adc_size = 4095;
	static constexpr uint32_t _processing_tick = 15;
	
	using error_event_t = void (*)(/*uint8_t id, */uint8_t code);
	
	public:
		
		typedef struct
		{
			GPIO_TypeDef *Port;
			uint16_t Pin;
		} pin_t;
		
		enum direction_t : uint8_t { DIR_NONE, DIR_OFF, DIR_LEFT, DIR_RIGHT, DIR_STOP };
		
		DRV8874(pin_t in1, pin_t in2, pin_t en, pin_t fault, pin_t current)
		{
			_channel.pin_in1 = in1;
			_channel.pin_in2 = in2;
			_channel.pin_en = en;
			_channel.pin_fault = fault;
			_channel.pin_current = current;
			
			return;
		}
		
		void Init()
		{
			_HW_PinInit(_channel.pin_in1, GPIO_MODE_OUTPUT_PP);
			_HW_PinInit(_channel.pin_in2, GPIO_MODE_OUTPUT_PP);
			_HW_PinInit(_channel.pin_en, GPIO_MODE_OUTPUT_PP);
			_HW_PinInit(_channel.pin_fault, GPIO_MODE_INPUT);
			_HW_PinInit(_channel.pin_current, GPIO_MODE_ANALOG);

			HAL_ADCEx_Calibration_Start(&hadc1);
			
			return;
		}
		
		void SetCurrentParam(uint32_t vref, uint16_t rload)
		{
			_vref = vref;
			_rload = rload;
			
			return;
		}
		
		void SetEventCallback(error_event_t event)
		{
			_error_event = event;
			
			return;
		}
		
		void SetTimeout(uint32_t timeout)
		{
			_channel.timeout = timeout;
		}
		
		void Action(direction_t dir)
		{
			switch(dir)
			{
				case DIR_OFF:
				{
					_HW_LOW(_channel.pin_in1);
					_HW_LOW(_channel.pin_in2);
					_HW_LOW(_channel.pin_en);
					_channel.state = DIR_OFF;
					
					break;
				}
				case DIR_LEFT:
				{
					_HW_LOW(_channel.pin_in1);
					_HW_HIGH(_channel.pin_in2);
					_HW_HIGH(_channel.pin_en);
					_channel.state = DIR_LEFT;
					
					break;
				}
				case DIR_RIGHT:
				{
					_HW_HIGH(_channel.pin_in1);
					_HW_LOW(_channel.pin_in2);
					_HW_HIGH(_channel.pin_en);
					_channel.state = DIR_RIGHT;
					
					break;
				}
				case DIR_STOP:
				{
					_HW_HIGH(_channel.pin_in1);
					_HW_HIGH(_channel.pin_in2);
					_HW_HIGH(_channel.pin_en);
					_channel.state = DIR_STOP;
					
					break;
				}
				default:
				{
					//_HW_LOW(_channel.pin_in1);
					//_HW_LOW(_channel.pin_in2);
					//_HW_LOW(_channel.pin_en);
					_channel.state = DIR_NONE;
					
					break;
				}
			}
			_channel.timerun = HAL_GetTick();
			
			return;
		}
		
		void ActionOff()
		{
			return Action(DIR_OFF);
		}
		
		void ActionLeft()
		{
			return Action(DIR_LEFT);
		}

		void ActionRight()
		{
			return Action(DIR_RIGHT);
		}
		
		void ActionStop()
		{
			return Action(DIR_STOP);
		}
		
		void ActionInvert()
		{
			switch(_channel.state)
			{
				case DIR_LEFT: { ActionRight(); break; }
				case DIR_RIGHT: { ActionLeft(); break; }
				default: { ActionRight(); break; }
			}
			
			return;
		}
		
		uint16_t GetCurrent(bool force = false)
		{
			if(force == true)
			{
				_channel.current.Set( _HW_GetCurrent(_channel.pin_current) );
			}
			
			return _channel.current.Get();
		}

		direction_t GetState()
		{
			return _channel.state;
		}
		
		void Processing(uint32_t current_time)
		{
			if(current_time - last_tick < _processing_tick) return;
			last_tick = current_time;
			
			_channel.current.Push( _HW_GetCurrent(_channel.pin_current) );
			
			uint8_t code = 0x00;
			if( (_channel.state == DIR_LEFT || _channel.state == DIR_RIGHT) && current_time - _channel.timerun > _channel.timeout )
			{
				ActionStop();
				code = 0x02;
			}
			
			if( _HW_READ(_channel.pin_fault) == false )
			{
				ActionOff();
				code = 0x01;
			}

			if(code != 0x00 && _error_event != nullptr)
			{
				_error_event(code);
			}
			
			return;
		}

	private:

		typedef struct
		{
			pin_t pin_in1;
			pin_t pin_in2;
			pin_t pin_en;
			pin_t pin_fault;
			pin_t pin_current;
			
			MovingAverage<uint16_t, uint32_t, 8> current;
			direction_t state;
			uint32_t timerun;
			uint32_t timeout;
		} channel_t;
		
		bool _HW_READ(pin_t pin)
		{
			return HAL_GPIO_ReadPin(pin.Port, pin.Pin);
		}
		
		void _HW_HIGH(pin_t pin)
		{
			HAL_GPIO_WritePin(pin.Port, pin.Pin, GPIO_PIN_SET);
			
			return;
		}
		
		void _HW_LOW(pin_t pin)
		{
			HAL_GPIO_WritePin(pin.Port, pin.Pin, GPIO_PIN_RESET);
			
			return;
		}
		
		uint16_t _HW_GetCurrent(pin_t pin)
		{
			_adc_config.Channel = pin.Pin;
			
			HAL_ADC_ConfigChannel(&hadc1, &_adc_config);
			//HAL_ADCEx_Calibration_Start(&hadc1);
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 5);
			uint32_t adc = HAL_ADC_GetValue(&hadc1);
			//HAL_ADC_Stop(&hadc1);
			
			return (((_vref / _adc_size) * adc) / _rload) / _current_scaling;
		}
		
		void _HW_PinInit(pin_t pin, uint32_t mode)
		{
			HAL_GPIO_WritePin(pin.Port, pin.Pin, GPIO_PIN_RESET);
			
			_pin_config.Pin = pin.Pin;
			_pin_config.Mode = mode;
			HAL_GPIO_Init(pin.Port, &_pin_config);
			
			return;
		}
		
		channel_t _channel;
		
		GPIO_InitTypeDef _pin_config = { GPIO_PIN_0, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW };
		ADC_ChannelConfTypeDef _adc_config = { ADC_CHANNEL_0, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_1CYCLE_5 };
		error_event_t _error_event = nullptr;
		
		uint32_t last_tick = 0;
		
		uint32_t _vref;
		uint16_t _rload;
		
};
