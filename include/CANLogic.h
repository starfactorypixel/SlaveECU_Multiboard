#pragma once

#include <CANLibrary.h>

void HAL_CAN_Send(can_object_id_t id, uint8_t *data, uint8_t length);

extern CAN_HandleTypeDef hcan;

namespace CANLib
{
	//*********************************************************************
	// CAN Library settings
	//*********************************************************************

	/// @brief Number of CANObjects in CANManager
	static constexpr uint8_t CFG_CANObjectsCount = 12;

	/// @brief The size of CANManager's internal CAN frame buffer
	static constexpr uint8_t CFG_CANFrameBufferSize = 16;

	enum HardwareErrorCodes : uint8_t
	{
		ERROR_CODE_HW_NONE = 0x00,
		ERROR_CODE_HW_SECONDARY_ELECTRONICS_ERROR = 0x01,
		ERROR_CODE_HW_CABIN_LIGHT_ERROR = 0x02,
		ERROR_CODE_HW_REAR_CAMERA_ERROR = 0x03,
		ERROR_CODE_HW_HORN_ERROR = 0x04,
	};

	//*********************************************************************
	// CAN Manager & CAN Object configuration
	//*********************************************************************

	// structure for all data fields of CANObject
	// rear_light_can_data_t light_ecu_can_data;

	CANManager<CFG_CANObjectsCount, CFG_CANFrameBufferSize> can_manager(&HAL_CAN_Send);

	// ******************** common blocks ********************
	// 0x0180	BlockInfo
	// request | timer:15000
	// byte	1 + 7	{ type[0] data[1..7] }
	// Основная информация о блоке. См. "Системные параметры".
	CANObject<uint8_t, 7> obj_block_info(0x0180);

	// 0x0181	BlockHealth
	// request | event
	// byte	1 + 7	{ type[0] data[1..7] }
	// Информация о здоровье блока. См. "Системные параметры".
	CANObject<uint8_t, 7> obj_block_health(0x0181);

	// 0x0182	BlockFeatures
	// request | timer:15000
	// byte	1 + 7	{ type[0] data[1..7] }
	// Чтение возможностей блока. См. "Системные параметры".
	CANObject<uint8_t, 7> obj_block_features(0x0182);


	// 0x0183	BlockError
	// request | event
	// byte	1 + 7	{ type[0] data[1..7] }
	// Ошибки блока. См. "Системные параметры".
	CANObject<uint8_t, 7> obj_block_error(0x0183);

	// ******************** specific blocks ********************

	// 0x0184	TrunkControl
	// set | toggle | request | event
	// uint8_t	0 .. 255	1 + 1	{ type[0] } or { type[0] data[1] }
	// Управление багажником.
	CANObject<int8_t, 1> obj_trunk_control(0x0184, CAN_TIMER_DISABLED, 300);


	// 0x0185	HoodControl
	// set | toggle | request | event
	// uint8_t	0 .. 255	1 + 1	{ type[0] } or { type[0] data[1] }
	// Управление капотом.
	CANObject<int8_t, 1> obj_hood_control(0x0185, CAN_TIMER_DISABLED, 300);


	// 0x0186	SecElecControl
	// set | toggle | request | event
	// uint8_t	00 || FF	1 + 1	{ type[0] } or { type[0] data[1] }
	// Управление вторичной электроникой.
	CANObject<uint8_t, 1> obj_secelec_control(0x0186, CAN_TIMER_DISABLED, 300);


	// 0x0187	LeftDoorControl
	// action | event
	// -	-	1	{ type[0] }
	// Управление левой дверью.
	CANObject<uint8_t, 1> obj_leftdoor_control(0x0187, CAN_TIMER_DISABLED, 300);


	// 0x0188	RightDoorControl
	// action | event
	// -	-	1	{ type[0] }
	// Управление правой дверью.
	CANObject<uint8_t, 1> obj_rightdoor_control(0x0188, CAN_TIMER_DISABLED, 300);


	// 0x0189	CabinLightControl
	// set | toggle | request | event
	// uint8_t	00 || FF	1 + 1	{ type[0] } or { type[0] data[1] }
	// Управление светом в салоне.
	CANObject<uint8_t, 1> obj_cabinlight_control(0x0189, CAN_TIMER_DISABLED, 300);


	// 0x018A	RearCameraControl
	// set | toggle | request | event
	// uint8_t	00 || FF	1 + 1	{ type[0] } or { type[0] data[1] }
	// Управление камерой заднего вида.
	CANObject<uint8_t, 1> obj_rearcamera_control(0x018A, CAN_TIMER_DISABLED, 300);

	
	// 0x018B	HornControl
	// set | toggle | request | event
	// uint8_t	00 || FF	1 + 1	{ type[0] } or { type[0] data[1] }
	// Управление клаксоном.
	CANObject<uint8_t, 1> obj_horn_control(0x018B, CAN_TIMER_DISABLED, 300);
	
	inline uint8_t on_off_validator(uint8_t value)
	{
		return (value > 0) ? 0xFF : 0;
	}
	
	inline void Setup()
	{
		obj_trunk_control
			.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
			{
				TrunkHood::LogicSet(TrunkHood::driver2, TrunkHood::actuator_data[1], can_frame.data[0]);
				obj_trunk_control.SetValue(0, can_frame.data[0], CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

				return CAN_RESULT_IGNORE;
			})
			.RegisterFunctionToggle([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
			{
				TrunkHood::LogicToggle(TrunkHood::driver2, TrunkHood::actuator_data[1]);
				obj_trunk_control.SetValue(0, TrunkHood::actuator_data[1].state, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

				return CAN_RESULT_IGNORE;
			});
		
		
		obj_hood_control
			.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
			{
				TrunkHood::LogicSet(TrunkHood::driver1, TrunkHood::actuator_data[0], can_frame.data[0]);
				obj_hood_control.SetValue(0, can_frame.data[0], CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

				return CAN_RESULT_IGNORE;
			})
			.RegisterFunctionToggle([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
			{
				TrunkHood::LogicToggle(TrunkHood::driver1, TrunkHood::actuator_data[0]);
				obj_hood_control.SetValue(0, TrunkHood::actuator_data[0].state, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

				return CAN_RESULT_IGNORE;
			});
		
		
		obj_secelec_control
			.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
			{
				bool result = true;
				if(can_frame.data[0] == 0)
				{
					Outputs::outObj.SetOff(1);
				}
				else
				{
					result = Outputs::outObj.SetOn(1);
				}

				if (result)
				{
					obj_secelec_control.SetValue(0, on_off_validator(can_frame.data[0]), CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
					return CAN_RESULT_IGNORE;
				}
				
				error.error_section = ERROR_SECTION_HARDWARE;
				error.error_code = ERROR_CODE_HW_SECONDARY_ELECTRONICS_ERROR;
				return CAN_RESULT_ERROR;
			})
			.RegisterFunctionToggle([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
			{
				Outputs::outObj.SetToggle(1);
				obj_secelec_control.SetValue(0, Outputs::outObj.GetState(1), CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
				
				return CAN_RESULT_IGNORE;
			});
		
		
		obj_leftdoor_control.RegisterFunctionAction([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			Outputs::outObj.SetOn(2, 1000);
			
			can_frame.initialized = true;
			can_frame.function_id = CAN_FUNC_EVENT_OK;
			
			return CAN_RESULT_CAN_FRAME;
		});
		
		
		obj_rightdoor_control.RegisterFunctionAction([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
		{
			Outputs::outObj.SetOn(3, 1000);
			
			can_frame.initialized = true;
			can_frame.function_id = CAN_FUNC_EVENT_OK;
			
			return CAN_RESULT_CAN_FRAME;
		});
		
		
		obj_cabinlight_control
			.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
			{
				bool result = true;
				if(can_frame.data[0] == 0)
				{
					Outputs::outObj.SetOff(4);
				}
				else
				{
					result = Outputs::outObj.SetOn(4);
				}

				if (result)
				{
					obj_cabinlight_control.SetValue(0, on_off_validator(can_frame.data[0]), CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
					return CAN_RESULT_IGNORE;
				}
				
				error.error_section = ERROR_SECTION_HARDWARE;
				error.error_code = ERROR_CODE_HW_CABIN_LIGHT_ERROR;
				return CAN_RESULT_ERROR;
			})
			.RegisterFunctionToggle([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
			{
				Outputs::outObj.SetToggle(4);
				obj_cabinlight_control.SetValue(0, Outputs::outObj.GetState(4), CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
				
				return CAN_RESULT_IGNORE;
			});
		
		obj_rearcamera_control
			.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
			{
				bool result = true;
				if(can_frame.data[0] == 0)
				{
					Outputs::outObj.SetOff(5);
				}
				else
				{
					result = Outputs::outObj.SetOn(5);
				}

				if (result)
				{
					obj_rearcamera_control.SetValue(0, on_off_validator(can_frame.data[0]), CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
					return CAN_RESULT_IGNORE;
				}
				
				error.error_section = ERROR_SECTION_HARDWARE;
				error.error_code = ERROR_CODE_HW_REAR_CAMERA_ERROR;
				return CAN_RESULT_ERROR;
			})
			.RegisterFunctionToggle([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
			{
				Outputs::outObj.SetToggle(5);
				obj_rearcamera_control.SetValue(0, Outputs::outObj.GetState(5), CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
				
				return CAN_RESULT_IGNORE;
			});
		

		obj_horn_control
			.RegisterFunctionSet([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
			{
				bool result = true;
				if(can_frame.data[0] == 0)
				{
					Outputs::outObj.SetOff(6);
				}
				else
				{
					result = Outputs::outObj.SetOn(6);
				}

				if (result)
				{
					obj_horn_control.SetValue(0, on_off_validator(can_frame.data[0]), CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
					return CAN_RESULT_IGNORE;
				}
				
				error.error_section = ERROR_SECTION_HARDWARE;
				error.error_code = ERROR_CODE_HW_HORN_ERROR;
				return CAN_RESULT_ERROR;
			})
			.RegisterFunctionToggle([](can_frame_t &can_frame, can_error_t &error) -> can_result_t
			{
				Outputs::outObj.SetToggle(6);
				obj_horn_control.SetValue(0, Outputs::outObj.GetState(6), CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
				
				return CAN_RESULT_IGNORE;
			});
		
		
		// system blocks
		set_block_info_params(obj_block_info);
		set_block_health_params(obj_block_health);
		set_block_features_params(obj_block_features);
		set_block_error_params(obj_block_error);

		// common blocks
		can_manager.RegisterObject(obj_block_info);
		can_manager.RegisterObject(obj_block_health);
		can_manager.RegisterObject(obj_block_features);
		can_manager.RegisterObject(obj_block_error);

		// specific blocks
		can_manager.RegisterObject(obj_trunk_control);
		can_manager.RegisterObject(obj_hood_control);
		can_manager.RegisterObject(obj_secelec_control);
		can_manager.RegisterObject(obj_leftdoor_control);
		can_manager.RegisterObject(obj_rightdoor_control);
		can_manager.RegisterObject(obj_cabinlight_control);
		can_manager.RegisterObject(obj_rearcamera_control);
		can_manager.RegisterObject(obj_horn_control);

		// Set versions data to block_info.
		obj_block_info.SetValue(0, (About::board_type << 3 | About::board_ver), CAN_TIMER_TYPE_NORMAL);
		obj_block_info.SetValue(1, (About::soft_ver << 2 | About::can_ver), CAN_TIMER_TYPE_NORMAL);
		
		return;
	}

	inline void Loop(uint32_t &current_time)
	{
		can_manager.Process(current_time);
		
		// Set uptime to block_info.
		static uint32_t iter = 0;
		if(current_time - iter > 1000)
		{
			iter = current_time;

			uint8_t *data = (uint8_t *)&current_time;
			obj_block_info.SetValue(2, data[0], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(3, data[1], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(4, data[2], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(5, data[3], CAN_TIMER_TYPE_NORMAL);
		}
		
		current_time = HAL_GetTick();

		return;
	}
}
