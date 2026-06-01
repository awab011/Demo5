/*
	Copyright 2016-2017 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef BLDC_INTERFACE_H_
#define BLDC_INTERFACE_H_

#include "../../Actuators/VESC_FDCAN/datatypes.h"

// interface functions
void uart_bldc_interface_init(void(*func)(unsigned char *data, unsigned int len));
void uart_bldc_interface_set_forward_func(void(*func)(unsigned char *data, unsigned int len));
void uart_bldc_interface_send_packet(unsigned char *data, unsigned int len);
void uart_bldc_interface_process_packet(unsigned char *data, unsigned int len);

// Function pointer setters
void uart_bldc_interface_set_rx_value_func(void(*func)(mc_values *values));
void uart_bldc_interface_set_rx_printf_func(void(*func)(char *str));
void uart_bldc_interface_set_rx_fw_func(void(*func)(int major, int minor));
void uart_bldc_interface_set_rx_rotor_pos_func(void(*func)(float pos));
void uart_bldc_interface_set_rx_mcconf_func(void(*func)(mc_configuration *conf));
void uart_bldc_interface_set_rx_appconf_func(void(*func)(app_configuration *conf));
void uart_bldc_interface_set_rx_detect_func(void(*func)(float cycle_int_limit, float coupling_k,
		const signed char *hall_table, signed char hall_res));
void uart_bldc_interface_set_rx_dec_ppm_func(void(*func)(float val, float ms));
void uart_bldc_interface_set_rx_dec_adc_func(void(*func)(float val, float voltage));
void uart_bldc_interface_set_rx_dec_chuk_func(void(*func)(float val));
void uart_bldc_interface_set_rx_mcconf_received_func(void(*func)(void));
void uart_bldc_interface_set_rx_appconf_received_func(void(*func)(void));

void uart_bldc_interface_set_sim_control_function(void(*func)(motor_control_mode mode, float value));
void uart_bldc_interface_set_sim_values_func(void(*func)(void));

// Setters
void uart_bldc_interface_terminal_cmd(char* cmd);
void uart_bldc_interface_set_duty_cycle(float dutyCycle);
void uart_bldc_interface_set_current(float current);
void uart_bldc_interface_set_current_brake(float current);
void uart_bldc_interface_set_rpm(int rpm);
void uart_bldc_interface_set_pos(float pos);
void uart_bldc_interface_set_handbrake(float current);
void uart_bldc_interface_set_servo_pos(float pos);
void uart_bldc_interface_set_mcconf(const mc_configuration *mcconf);
void uart_bldc_interface_set_appconf(const app_configuration *appconf);

// Getters
void uart_bldc_interface_get_fw_version(void);
void uart_bldc_interface_get_values(void);
void uart_bldc_interface_get_mcconf(void);
void uart_bldc_interface_get_appconf(void);
void uart_bldc_interface_get_decoded_ppm(void);
void uart_bldc_interface_get_decoded_adc(void);
void uart_bldc_interface_get_decoded_chuk(void);

// Other functions
void uart_bldc_interface_detect_motor_param(float current, float min_rpm, float low_duty);
void uart_bldc_interface_reboot(void);
void uart_bldc_interface_send_alive(void);
void uart_send_values_to_receiver(mc_values *values);

// Helpers
const char* uart_bldc_interface_fault_to_string(mc_fault_code fault);

#endif /* BLDC_INTERFACE_H_ */
