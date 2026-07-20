/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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

#include "GZMixingInterfaceServo.hpp"

#include <algorithm>
#include <chrono>


float
GZMixingInterfaceServo::get_servo_angle_max(const size_t index)
{
	float angle;

	switch (index) {
	case 0: {angle = _servo_max_angle_1.get(); break;}

	case 1: {angle = _servo_max_angle_2.get(); break;}

	case 2: {angle = _servo_max_angle_3.get(); break;}

	case 3: {angle = _servo_max_angle_4.get(); break;}

	case 4: {angle = _servo_max_angle_5.get(); break;}

	case 5: {angle = _servo_max_angle_6.get(); break;}

	case 6: {angle = _servo_max_angle_7.get(); break;}

	case 7: {angle = _servo_max_angle_8.get(); break;}
	case 8: {angle = _servo_max_angle_9.get(); break;}

	default: {angle = NAN; break;}
	}

	if (!PX4_ISFINITE(angle)) {
		PX4_ERR("max_angle: index out of range, i= %ld, expected i < 9", index);
		return NAN;
	}

	return math::radians(angle);
}

float
GZMixingInterfaceServo::get_servo_angle_min(const size_t index)
{
	float angle;

	switch (index) {
	case 0: {angle = _servo_min_angle_1.get(); break;}

	case 1: {angle = _servo_min_angle_2.get(); break;}

	case 2: {angle = _servo_min_angle_3.get(); break;}

	case 3: {angle = _servo_min_angle_4.get(); break;}

	case 4: {angle = _servo_min_angle_5.get(); break;}

	case 5: {angle = _servo_min_angle_6.get(); break;}

	case 6: {angle = _servo_min_angle_7.get(); break;}

	case 7: {angle = _servo_min_angle_8.get(); break;}
	case 8: {angle = _servo_min_angle_9.get(); break;}

	default: {angle = NAN; break;}

	}

	if (!PX4_ISFINITE(angle)) {
		PX4_ERR("min_angle: index out of range, i= %ld, expected i < 9", index);
		return NAN;
	}

	return math::radians(angle);
}

float
GZMixingInterfaceServo::get_servo_angle_zero(const size_t index)
{
	float angle;

	switch (index) {
	case 0: {angle = _servo_zero_angle_1.get(); break;}
	case 1: {angle = _servo_zero_angle_2.get(); break;}
	case 2: {angle = _servo_zero_angle_3.get(); break;}
	case 3: {angle = _servo_zero_angle_4.get(); break;}
	case 4: {angle = _servo_zero_angle_5.get(); break;}
	case 5: {angle = _servo_zero_angle_6.get(); break;}
	case 6: {angle = _servo_zero_angle_7.get(); break;}
	case 7: {angle = _servo_zero_angle_8.get(); break;}
	case 8: {angle = _servo_zero_angle_9.get(); break;}
	default: {angle = NAN; break;}
	}

	if (!PX4_ISFINITE(angle)) {
		PX4_ERR("zero_angle: index out of range, i= %ld, expected i < 9", index);
		return NAN;
	}

	return math::radians(angle);
}

bool GZMixingInterfaceServo::init(const std::string &model_name)
{
	_model_name = model_name;
	_next_connection_check = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);

	// /model/rascal_110_0/servo_2
	for (int i = 0; i < 9; i++) {
		std::string joint_name = "servo_" + std::to_string(i);
		std::string servo_topic = "/model/" + model_name + "/" + joint_name;

		_servos_pub.push_back(_node.Advertise<gz::msgs::Double>(servo_topic));

		if (!_servos_pub.back().Valid()) {
			PX4_ERR("failed to advertise %s", servo_topic.c_str());
			return false;
		}

		double min_val = get_servo_angle_min(i);
		double zero_val = get_servo_angle_zero(i);
		double max_val = get_servo_angle_max(i);
		_angle_min_rad.push_back(min_val);
		_angle_zero_rad.push_back(zero_val);
		_angle_max_rad.push_back(max_val);
		_angular_range_rad.push_back(max_val - min_val);
	}

	pthread_mutex_init(&_node_mutex, nullptr);

	ScheduleNow();

	return true;
}

bool GZMixingInterfaceServo::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
		unsigned num_control_groups_updated)
{
	bool updated = false;
	// cmd.command_value = (float)outputs[i] / 500.f - 1.f; // [-1, 1]

	int i = 0;

	for (auto &servo_pub : _servos_pub) {
		if (_mixing_output.isFunctionSet(i)) {
			gz::msgs::Double servo_output;

			double output_range = _mixing_output.maxValue(i) - _mixing_output.minValue(i);

			if (output_range <= 0.0) {
				PX4_ERR("servo output range invalid, i=%d", i);
				continue;
			}

			double output;

			if (_servo_zero_mapping.get()) {
				const double normalized = std::clamp(
					2.0 * (outputs[i] - _mixing_output.minValue(i)) / output_range - 1.0, -1.0, 1.0);

				if (normalized <= 0.0) {
					output = _angle_zero_rad[i] + normalized * (_angle_zero_rad[i] - _angle_min_rad[i]);

				} else {
					output = _angle_zero_rad[i] + normalized * (_angle_max_rad[i] - _angle_zero_rad[i]);
				}

			} else {
				output = _angle_min_rad[i] + _angular_range_rad[i]
					 * (outputs[i] - _mixing_output.minValue(i)) / output_range;
			}
			// std::cout << "outputs[" << i << "]: " << outputs[i] << std::endl;
			// std::cout << "  output: " << output << std::endl;
			servo_output.set_data(output);

			if (servo_pub.Valid()) {
				servo_pub.Publish(servo_output);
				updated = true;
			}
		}

		i++;
	}

	return updated;
}

bool GZMixingInterfaceServo::refreshServoPublishers()
{
	_servos_pub.clear();
	_servos_pub.reserve(9);

	for (int i = 0; i < 9; ++i) {
		const std::string topic = "/model/" + _model_name + "/servo_" + std::to_string(i);
		_servos_pub.push_back(_node.Advertise<gz::msgs::Double>(topic));

		if (!_servos_pub.back().Valid()) {
			PX4_ERR("failed to re-advertise %s", topic.c_str());
			return false;
		}
	}

	return true;
}

void GZMixingInterfaceServo::Run()
{
	if (_servo_zero_mapping.get() && !_publishers_connected
	    && std::chrono::steady_clock::now() >= _next_connection_check) {
		_publishers_connected = std::all_of(_servos_pub.begin(), _servos_pub.end(),
		[](const gz::transport::Node::Publisher &publisher) { return publisher.HasConnections(); });

		if (!_publishers_connected) {
			// The model is now advancing and all JointPositionController systems
			// have had a chance to register. Re-advertising repairs the startup
			// discovery race without relying on an external `gz topic -e` peer.
			refreshServoPublishers();
			++_publisher_refresh_count;
			_next_connection_check = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);

			if (_publisher_refresh_count == 10) {
				PX4_WARN("Gazebo servo discovery still incomplete after 10 refreshes");
			}
		}
	}

	pthread_mutex_lock(&_node_mutex);
	_mixing_output.update();
	_mixing_output.updateSubscriptions(false);
	pthread_mutex_unlock(&_node_mutex);

	if (_servo_zero_mapping.get() && !_publishers_connected) {
		ScheduleDelayed(100_ms);
	}
}
