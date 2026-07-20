/****************************************************************************
 *
 * Copyright (c) 2021 Px4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in
 * the documentation and/or other materials provided with the
 * distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 * used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIE, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ActuatorEffectivenessControlSurfaces.hpp"

// 构造函数：动态绑定参数句柄
ActuatorEffectivenessControlSurfaces::ActuatorEffectivenessControlSurfaces(ModuleParams *parent) :
	ModuleParams(parent)
{
	for (int i = 0; i < MAX_COUNT; i++) {
		char name[30];

		snprintf(name, sizeof(name), "CA_SV_CS%u_TYPE", i);
		_param_handles[i].type = param_find(name);

		snprintf(name, sizeof(name), "CA_SV_CS%u_TRQ_R", i);
		_param_handles[i].torque[0] = param_find(name);

		snprintf(name, sizeof(name), "CA_SV_CS%u_TRQ_P", i);
		_param_handles[i].torque[1] = param_find(name);

		snprintf(name, sizeof(name), "CA_SV_CS%u_TRQ_Y", i);
		_param_handles[i].torque[2] = param_find(name);

		snprintf(name, sizeof(name), "CA_SV_CS%u_TRIM", i);
		_param_handles[i].trim = param_find(name);

		snprintf(name, sizeof(name), "CA_SV_CS%u_FLAP", i);
		_param_handles[i].scale_flap = param_find(name);

		snprintf(name, sizeof(name), "CA_SV_CS%u_SPOIL", i);
		_param_handles[i].scale_spoiler = param_find(name);

		// ---- 新增：在构造函数中动态查找并绑定在 module.yaml 里注册的鸭翼参数句柄 ----
		snprintf(name, sizeof(name), "CA_SV_CS%u_CANARD", i);
		_param_handles[i].scale_canard = param_find(name);
	}

	_count_handle = param_find("CA_SV_CS_COUNT");

	_flaps_setpoint_with_slewrate.setSlewRate(kFlapSlewRate);
	_spoilers_setpoint_with_slewrate.setSlewRate(kSpoilersSlewRate);
	_canard_setpoint_with_slewrate.setSlewRate(kCanardSlewRate);
}

// 参数更新函数：定时将地面站修改的值同步进内存变量
void ActuatorEffectivenessControlSurfaces::updateParams()
{
	ModuleParams::updateParams();

	int32_t count = 0;

	if (param_get(_count_handle, &count) == 0) {
		_count = math::constrain(count, 0, MAX_COUNT);
	}

	for (int i = 0; i < _count; i++) {
		int32_t type = 0;

		if (param_get(_param_handles[i].type, &type) == 0) {
			_params[i].type = (Type)type;
		}

		param_get(_param_handles[i].torque[0], &_params[i].torque(0));
		param_get(_param_handles[i].torque[1], &_params[i].torque(1));
		param_get(_param_handles[i].torque[2], &_params[i].torque(2));
		param_get(_param_handles[i].trim, &_params[i].trim);
		param_get(_param_handles[i].scale_flap, &_params[i].scale_flap);
		param_get(_param_handles[i].scale_spoiler, &_params[i].scale_spoiler);

		// ---- 新增：将地面站设置的鸭翼缩放系数读入到 _params 物理结构体中 ----
		param_get(_param_handles[i].scale_canard, &_params[i].scale_canard);

		// 根据不同的舵面类型配置基础气动贡献
		switch (_params[i].type) {
		case Type::LeftAileron:
			_params[i].torque = matrix::Vector3f(-1.f, 0.f, 0.f);
			break;

		case Type::RightAileron:
			_params[i].torque = matrix::Vector3f(1.f, 0.f, 0.f);
			break;

		case Type::Elevator:
			_params[i].torque = matrix::Vector3f(0.f, 1.f, 0.f);
			break;

		case Type::Rudder:
			_params[i].torque = matrix::Vector3f(0.f, 0.f, 1.f);
			break;

		case Type::LeftElevon:
			_params[i].torque = matrix::Vector3f(-0.5f, 0.5f, 0.f);
			break;

		case Type::RightElevon:
			_params[i].torque = matrix::Vector3f(0.5f, 0.5f, 0.f);
			break;

		case Type::LeftVTail:
			_params[i].torque = matrix::Vector3f(0.f, 0.5f, -0.5f);
			break;

		case Type::RightVTail:
			_params[i].torque = matrix::Vector3f(0.f, 0.5f, 0.5f);
			break;

		case Type::LeftFlap:
		case Type::RightFlap:
		case Type::Airbrake:
			_params[i].torque.zero();
			break;

		case Type::Custom:
			// Preserve the explicit CA_SV_CSx_TRQ_[RPY] vector. This is
			// required when paired surfaces each represent half of an axis.
			break;

		case Type::LeftATail:
			_params[i].torque = matrix::Vector3f(0.f, 0.5f, 0.5f);
			break;

		case Type::RightATail:
			_params[i].torque = matrix::Vector3f(0.f, 0.5f, -0.5f);
			break;

		case Type::SingleChannelAileron:
			_params[i].torque = matrix::Vector3f(1.f, 0.f, 0.f);
			break;

		case Type::SteeringWheel:
			_params[i].torque = matrix::Vector3f(0.f, 0.f, 1.f);
			break;

		case Type::LeftSpoiler:
		case Type::RightSpoiler:
			_params[i].torque.zero();
			break;

		// ---- 新增：在这里将鸭翼三轴力矩贡献强制归零 ----
		// 这样可以确保控制分配矩阵求解时，鸭翼绝不插手常规主升降舵的闭环解算
		case Type::LeftCanard:
		case Type::RightCanard:
			_params[i].torque.zero();
			break;
		}
	}
}

// 建立常规气动矩阵：将有效的几何控制量添加进矩阵
bool ActuatorEffectivenessControlSurfaces::addActuators(Configuration &configuration)
{
	for (int i = 0; i < _count; i++) {
		int actuator_idx = configuration.addActuator(ActuatorType::SERVOS, _params[i].torque, matrix::Vector3f{});

		if (actuator_idx >= 0) {
			configuration.trim[configuration.selected_matrix](actuator_idx) = _params[i].trim;
		}
	}

	return true;
}

// 襟翼叠加通道（保持原版不变）
void ActuatorEffectivenessControlSurfaces::applyFlaps(float flaps_control, int first_actuator_idx, float dt,
		ActuatorVector &actuator_sp)
{
	_flaps_setpoint_with_slewrate.update(flaps_control, dt);

	for (int i = 0; i < _count; ++i) {
		actuator_sp(i + first_actuator_idx) += (_flaps_setpoint_with_slewrate.getState() * 2.f - 1.f) * _params[i].scale_flap;
	}
}

// 扰流板叠加通道（保持原版不变）
void ActuatorEffectivenessControlSurfaces::applySpoilers(float spoilers_control, int first_actuator_idx, float dt,
		ActuatorVector &actuator_sp)
{
	_spoilers_setpoint_with_slewrate.update(spoilers_control, dt);

	for (int i = 0; i < _count; ++i) {
		actuator_sp(i + first_actuator_idx) += _spoilers_setpoint_with_slewrate.getState() * _params[i].scale_spoiler;
	}
}

// 鸭翼双向控制：canard_control [0,1] → act_sp [-1,1]
//   0   = 后缘极限上偏（空气刹车）
//   0.5 = 中立
//   1   = 后缘极限下偏（起飞抬头）
void ActuatorEffectivenessControlSurfaces::applyCanard(float canard_control, int first_actuator_idx, float dt,
		ActuatorVector &actuator_sp)
{
	_canard_setpoint_with_slewrate.update(canard_control, dt);

	for (int i = 0; i < _count; ++i) {
		if (_params[i].type == Type::LeftCanard || _params[i].type == Type::RightCanard) {
			actuator_sp(i + first_actuator_idx) +=
				(_canard_setpoint_with_slewrate.getState() * 2.f - 1.f) * _params[i].scale_canard;
		}
	}
}
