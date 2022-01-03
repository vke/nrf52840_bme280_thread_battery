/**
 * Copyright (c) 2017 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdint.h>

#include "app_scheduler.h"
#include "app_timer.h"
#include "bsp_thread.h"
#include "nrf_log_ctrl.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_saadc.h"
#include "nrf_temp.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_power.h"

#include "settings.h"

#include "bosch_bme280_driver/bme280.h"

#include "thread_coap_utils.h"
#include "thread_utils.h"

#include <openthread/thread.h>

#define ADC_CHANNELS                         1

#define SCHED_QUEUE_SIZE                     32
#define SCHED_EVENT_DATA_SIZE                APP_TIMER_SCHED_EVENT_DATA_SIZE

static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(0);

uint32_t bme280_measurement_delay = 0;
struct bme280_dev bme280_sensor;
uint8_t bme280_sensor_addr = BME280_I2C_ADDR_PRIM;

APP_TIMER_DEF(m_voltage_timer_id);
APP_TIMER_DEF(m_internal_temperature_timer_id);
APP_TIMER_DEF(m_bme280_timer_id);
APP_TIMER_DEF(m_bme280_measurement_delay_timer_id);

sensor_subscription sensor_subscriptions[] = {
	{ .sensor_name = 'P', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = true, .initialized = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = NULL, },
	{ .sensor_name = 'T', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = true, .initialized = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = NULL, },
	{ .sensor_name = 'H', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = true, .initialized = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = NULL, },
	{ .sensor_name = 'v', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = true, .initialized = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = NULL, },
	{ .sensor_name = 't', .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = true, .initialized = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = NULL, },
	{ .sensor_name = SENSOR_SUBSCRIPTION_NAME_LAST, .sent_value = 0, .current_value = 0, .reportable_change = 0, .disable_reporting = true, .read_only = true, .initialized = false, .report_interval = 10000, .last_sent_at = 0, .set_value_handler = NULL, },
};

static nrf_saadc_value_t adc_buf[ADC_CHANNELS * ADC_SAMPLES_PER_CHANNEL];

int32_t internal_temp_prev = 0x7FFFFFFF;

int32_t dc_voltage_3v3_prev = 0x7FFFFFFF;
int32_t dc_voltage_3v3 = 0;

void update_voltage_attributes_callback(void *p_event_data, uint16_t event_size)
{
	set_sensor_value('v', dc_voltage_3v3, false);
}

void saadc_event_handler(nrf_drv_saadc_evt_t const *p_event)
{
	if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
	{
		uint32_t err_code;

		int32_t sums[ADC_CHANNELS];
		for (int i = 0; i < ADC_CHANNELS; i++) {
			sums[i] = 0;
			for (int j = 0; j < ADC_SAMPLES_PER_CHANNEL; j++) {
				sums[i] += p_event->data.done.p_buffer[i * ADC_SAMPLES_PER_CHANNEL + j];
			}
			sums[i] = sums[i] / ADC_SAMPLES_PER_CHANNEL;
		}

		if (sums[0] < 0)
			sums[0] = 0;

		if (dc_voltage_3v3_prev == 0x7FFFFFFF) {
			dc_voltage_3v3_prev = sums[0];
			dc_voltage_3v3 = sums[0];
		} else {
			dc_voltage_3v3 = (dc_voltage_3v3_prev + dc_voltage_3v3_prev + dc_voltage_3v3_prev + sums[0]) >> 2;
			dc_voltage_3v3_prev = dc_voltage_3v3;
		}

		err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, ADC_CHANNELS * ADC_SAMPLES_PER_CHANNEL);
		APP_ERROR_CHECK(err_code);

		app_sched_event_put(NULL, 0, update_voltage_attributes_callback);
	}
	else
	{
		NRF_LOG_INFO("saadc unhandled event: %d", p_event->type);
	}
}

static void adc_configure(void)
{
	ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
	APP_ERROR_CHECK(err_code);

	err_code = nrfx_saadc_calibrate_offset();
	APP_ERROR_CHECK(err_code);

	while (nrfx_saadc_is_busy());

	nrf_saadc_channel_config_t config0 = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
	config0.acq_time = NRF_SAADC_ACQTIME_40US;
	err_code = nrf_drv_saadc_channel_init(0, &config0);
	APP_ERROR_CHECK(err_code);

	err_code = nrf_drv_saadc_buffer_convert(adc_buf, ADC_CHANNELS * ADC_SAMPLES_PER_CHANNEL);
	APP_ERROR_CHECK(err_code);
}

static void voltage_timeout_handler(void *p_context)
{
	UNUSED_PARAMETER(p_context);

	ret_code_t err_code = nrf_drv_saadc_sample();
	APP_ERROR_CHECK(err_code);
}

static void internal_temperature_timeout_handler(void *p_context)
{
	UNUSED_PARAMETER(p_context);

	NRF_TEMP->TASKS_START = 1;
	/* Busy wait while temperature measurement is not finished. */
	while (NRF_TEMP->EVENTS_DATARDY == 0) {
	}
	NRF_TEMP->EVENTS_DATARDY = 0;

	int32_t temp = nrf_temp_read();

	NRF_TEMP->TASKS_STOP = 1;

	if (internal_temp_prev == 0x7FFFFFFF) {
		internal_temp_prev = temp;
	} else {
		temp = (internal_temp_prev + internal_temp_prev + internal_temp_prev + temp) >> 2;
		internal_temp_prev = temp;
		set_sensor_value('t', temp, false);
	}
}

static void bme280_measurement_delay_timeout_handler(void *p_context)
{
	UNUSED_PARAMETER(p_context);

	struct bme280_data comp_data;

	int8_t rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &bme280_sensor);
	if (rslt == BME280_OK) {
		set_sensor_value('P', comp_data.pressure, false);
		set_sensor_value('T', comp_data.temperature, false);
		set_sensor_value('H', comp_data.humidity, false);
	}
}

static void bme280_timeout_handler(void *p_context)
{
	UNUSED_PARAMETER(p_context);

	int8_t rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &bme280_sensor);
	if (rslt != BME280_OK)
		return;

	ret_code_t err_code = app_timer_start(m_bme280_measurement_delay_timer_id, APP_TIMER_TICKS(bme280_measurement_delay + 10), NULL);
	APP_ERROR_CHECK(err_code);
}

static void bsp_event_handler(bsp_event_t event)
{
	switch(event) {
		case BSP_EVENT_KEY_0:
			break;

		default:
			return;
	}
}

static void thread_state_changed_callback(uint32_t flags, void * p_context)
{
	if (flags & OT_CHANGED_THREAD_ROLE) {
		otDeviceRole device_role = otThreadGetDeviceRole(p_context);
		const char *szRole = "UNKNOWN ROLE";
		switch (device_role) {
			case OT_DEVICE_ROLE_CHILD:
				szRole = "OT_DEVICE_ROLE_CHILD";
#ifndef DISABLE_OT_ROLE_LIGHTS
				bsp_board_led_on(LED_CHILD_ROLE);
				bsp_board_led_off(LED_ROUTER_ROLE);
#endif // DISABLE_OT_ROLE_LIGHTS
				break;
			case OT_DEVICE_ROLE_ROUTER:
				szRole = "OT_DEVICE_ROLE_ROUTER";
#ifndef DISABLE_OT_ROLE_LIGHTS
				bsp_board_led_on(LED_ROUTER_ROLE);
				bsp_board_led_off(LED_CHILD_ROLE);
#endif // DISABLE_OT_ROLE_LIGHTS
				break;
			case OT_DEVICE_ROLE_LEADER:
				szRole = "OT_DEVICE_ROLE_LEADER";
#ifndef DISABLE_OT_ROLE_LIGHTS
				bsp_board_led_on(LED_CHILD_ROLE);
				bsp_board_led_on(LED_ROUTER_ROLE);
#endif // DISABLE_OT_ROLE_LIGHTS
				break;

			case OT_DEVICE_ROLE_DISABLED:
				szRole = "OT_DEVICE_ROLE_DISABLED";
#ifndef DISABLE_OT_ROLE_LIGHTS
				bsp_board_led_off(LED_ROUTER_ROLE);
				bsp_board_led_off(LED_CHILD_ROLE);
#endif // DISABLE_OT_ROLE_LIGHTS
				break;

			case OT_DEVICE_ROLE_DETACHED:
				szRole = "OT_DEVICE_ROLE_DETACHED";
#ifndef DISABLE_OT_ROLE_LIGHTS
				bsp_board_led_off(LED_ROUTER_ROLE);
				bsp_board_led_off(LED_CHILD_ROLE);
#endif // DISABLE_OT_ROLE_LIGHTS
				break;
			default:
				break;
		}
		NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %s\r\n", flags, szRole);
	}
}

static void log_init(void)
{
	ret_code_t err_code = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void timer_init(void)
{
	uint32_t error_code = app_timer_init();
	APP_ERROR_CHECK(error_code);

	// Voltage timer
	error_code = app_timer_create(&m_voltage_timer_id, APP_TIMER_MODE_REPEATED, voltage_timeout_handler);
	APP_ERROR_CHECK(error_code);

	// Internal temperature timer
	error_code = app_timer_create(&m_internal_temperature_timer_id, APP_TIMER_MODE_REPEATED, internal_temperature_timeout_handler);
	APP_ERROR_CHECK(error_code);

	// bme280 timer
	error_code = app_timer_create(&m_bme280_timer_id, APP_TIMER_MODE_REPEATED, bme280_timeout_handler);
	APP_ERROR_CHECK(error_code);

	// bme280 measurement delay timer
	error_code = app_timer_create(&m_bme280_measurement_delay_timer_id, APP_TIMER_MODE_SINGLE_SHOT, bme280_measurement_delay_timeout_handler);
	APP_ERROR_CHECK(error_code);
}

static void thread_instance_init(void)
{
	thread_configuration_t thread_configuration =
	{
		.radio_mode            = THREAD_RADIO_MODE_RX_OFF_WHEN_IDLE,
		.autocommissioning     = true,
		.wipe_settings         = false,
		.poll_period           = DEFAULT_POLL_PERIOD,
		.default_child_timeout = DEFAULT_CHILD_TIMEOUT,
	};

	thread_init(&thread_configuration);
	thread_state_changed_callback_set(thread_state_changed_callback);
}

void user_delay_us(uint32_t period, void *intf_ptr)
{
	nrf_delay_us(period);
}

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	ret_code_t err_code = nrf_drv_twi_tx(&m_twi_master, *(uint8_t *)intf_ptr, &reg_addr, 1, false);
	
	APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_twi_rx(&m_twi_master, *(uint8_t *)intf_ptr, reg_data, len);
	
	APP_ERROR_CHECK(err_code);
	
	return err_code;
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	uint8_t write_data[256];

	if (len > sizeof(write_data) - 1)
		return 1;

	write_data[0] = reg_addr;

	memcpy(&write_data[1], reg_data, len);

	ret_code_t err_code = nrf_drv_twi_tx(&m_twi_master, *(uint8_t *)intf_ptr, write_data, len + 1, false);

	APP_ERROR_CHECK(err_code);

	return 0;
}

static bool init_bme280()
{
	bme280_sensor.intf_ptr = &bme280_sensor_addr;
	bme280_sensor.intf = BME280_I2C_INTF;
	bme280_sensor.read = user_i2c_read;
	bme280_sensor.write = user_i2c_write;
	bme280_sensor.delay_us = user_delay_us;

	int8_t rslt = bme280_init(&bme280_sensor);
	if (rslt != BME280_OK)
		return false;
	
	bme280_sensor.settings.osr_h = BME280_OVERSAMPLING_16X;
	bme280_sensor.settings.osr_p = BME280_OVERSAMPLING_16X;
	bme280_sensor.settings.osr_t = BME280_OVERSAMPLING_16X;
	bme280_sensor.settings.filter = BME280_FILTER_COEFF_4;

	uint8_t settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

	rslt = bme280_set_sensor_settings(settings_sel, &bme280_sensor);
	if (rslt != BME280_OK)
		return false;

	bme280_measurement_delay = bme280_cal_meas_delay(&bme280_sensor.settings);

	return rslt == BME280_OK;
}

static bool twi_init()
{
	ret_code_t ret;

	const nrf_drv_twi_config_t config =
	{
		.scl                = TWI_SCL_M,
		.sda                = TWI_SDA_M,
		.frequency          = TWI_DEFAULT_CONFIG_FREQUENCY,
		.interrupt_priority = TWI_DEFAULT_CONFIG_IRQ_PRIORITY,
		.clear_bus_init     = TWI_DEFAULT_CONFIG_CLR_BUS_INIT
	};

	ret = nrf_drv_twi_init(&m_twi_master, &config, NULL, NULL);

	if (NRF_SUCCESS == ret) {
		nrf_drv_twi_enable(&m_twi_master);
	}

	return ret;
}

int main(int argc, char * argv[])
{
	log_init();
	APP_SCHED_INIT(SCHED_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
	timer_init();
	adc_configure();
	nrf_temp_init();
	twi_init();
	init_bme280();

	uint32_t error_code = NRF_SUCCESS;

	error_code = nrf_drv_power_init(NULL);
	APP_ERROR_CHECK(error_code);

	error_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
	APP_ERROR_CHECK(error_code);

	thread_instance_init();
	otPlatRadioSetTransmitPower(thread_ot_instance_get(), RADIO_TRANSMIT_POWER);
	thread_coap_utils_init();

	ret_code_t err_code = app_timer_start(m_voltage_timer_id, APP_TIMER_TICKS(VOLTAGE_TIMER_INTERVAL / ADC_SAMPLES_PER_CHANNEL), NULL);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_start(m_internal_temperature_timer_id, APP_TIMER_TICKS(INTERNAL_TEMPERATURE_TIMER_INTERVAL), NULL);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_start(m_bme280_timer_id, APP_TIMER_TICKS(BME280_TIMER_INTERVAL), NULL);
	APP_ERROR_CHECK(err_code);

	while (true) {
		thread_process();
		app_sched_execute();

		if (NRF_LOG_PROCESS() == false) {
			thread_sleep();
		}
	}
}
