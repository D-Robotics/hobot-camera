/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2024 D-Robotics, Inc.
 * @All rights reserved.
 ***************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <math.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/ioctl.h>
#include <arpa/inet.h>
#include <linux/spi/spidev.h>

#include "inc/hb_cam_utility.h"
#include "inc/hb_i2c.h"
#include "inc/sc230ai_setting.h"
#include "inc/sensor_effect_common.h"

#define TUNING_LUT

static int camera_power_fd = 0;
static int sc230ai_poweron(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	trace("GPIO:%d  %s\n", sensor_info->gpio_pin[0], sensor_info->sensor_name);

	// ret = camera_power_ctrl(sensor_info->gpio_pin[0], 0);
	// if (ret < 0) {
	// 	pr_err("%d : %s set PWDN = 1 fail\n", __LINE__, sensor_info->sensor_name);
	// 	return ret;
	// }

	// usleep(1 * 1000);

	// Enable MCLK
	ret = hb_cam_set_mclk(sensor_info->entry_num, MCLK);
	// ret = hb_cam_set_mclk(0, MCLK);
	if (ret < 0)
	{
		pr_err("%d : %s set mclk fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	ret = hb_cam_enable_mclk(sensor_info->entry_num);
	// ret = hb_cam_enable_mclk(0);
	if (ret < 0)
	{
		pr_err("%d : %s enable mclk fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	usleep(10 * 1000);

	// XSHUTDOWN=1
	// ret = camera_power_ctrl(sensor_info->gpio_pin[0], 1);
	// if (ret < 0) {
	// 	pr_err("%d : %s set RSTB = 0 fail\n", __LINE__, sensor_info->sensor_name);
	// 	return ret;
	// }
	// usleep(10 * 1000);

	return ret;
}

static int sc230ai_poweroff(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	trace("\n");
	// DVB PWDN--GPIO6  RSTB--GPIO5
	ret = camera_power_ctrl(sensor_info->gpio_pin[0], 0);
	if (ret < 0)
	{
		pr_debug("%d : %s set PWDN = 0 fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}

	usleep(100);

	// Disable MCLK
	ret = hb_cam_disable_mclk(sensor_info->entry_num);
	if (ret < 0)
	{
		pr_debug("%d : %s disable MCLK fail\n", __LINE__, sensor_info->sensor_name);
		return ret;
	}
	usleep(1000);
#if SC230AI_PMIC
	// disable PMIC
	close(camera_power_fd);
#endif
	return ret;
}

int sc230ai_reset(sensor_info_t *sensor_info)
{
	int gpio, ret = RET_OK;
	if (sensor_info->power_mode)
	{
		for (gpio = 0; gpio < sensor_info->gpio_num; gpio++)
		{
			if (sensor_info->gpio_pin[gpio] >= 0)
			{
				pr_debug("gpio_num %d  %d %d %d \n", sensor_info->gpio_num,
						 sensor_info->gpio_pin[gpio],
						 sensor_info->gpio_level[gpio],
						 sensor_info->gpio_level[gpio]);
				ret = camera_power_ctrl(sensor_info->gpio_pin[gpio],
										sensor_info->gpio_level[gpio]);
				usleep(sensor_info->power_delay * 1000);
				ret |= camera_power_ctrl(sensor_info->gpio_pin[gpio],
										 1 - sensor_info->gpio_level[gpio]);
				if (ret < 0)
				{
					pr_err("camera_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
			}
		}
	}
	return ret;
}

static int sc230ai_stream_control_set(sensor_turning_data_t *turning_data)
{
	int ret = RET_OK;
	int size;
	uint32_t *stream_on = turning_data->stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data->stream_ctrl.stream_off;

	turning_data->stream_ctrl.data_length = 1;
	size = sizeof(turning_data->stream_ctrl.stream_on);
	if (size >= sizeof(sc230ai_stream_on_setting))
	{
		memcpy(stream_on, sc230ai_stream_on_setting, sizeof(sc230ai_stream_on_setting));
	}
	else
	{
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	size = sizeof(turning_data->stream_ctrl.stream_off);
	if (size >= sizeof(sc230ai_stream_off_setting))
	{
		memcpy(stream_off, sc230ai_stream_off_setting, sizeof(sc230ai_stream_off_setting));
	}
	else
	{
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	return ret;
}

static int sc230ai_mode_config_i2c_write(sensor_info_t *sensor_info, uint32_t *pbuf, size_t size)
{
	int ret = RET_OK;
	int setting_size = 0, i;
	uint16_t reg, data;
	setting_size = size / sizeof(uint32_t) / 2;
	pr_info("sc230ai setting_size %d\n", setting_size);
	for (i = 0; i < setting_size; i++)
	{
		reg = pbuf[i * 2];
		data = pbuf[i * 2 + 1];
		ret = hb_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, reg, data);
		if (ret < 0)
		{
			printf("%d : init %s -- %d:0x%x %d: 0x%x = 0x%x fail\n", __LINE__,
				   sensor_info->sensor_name, sensor_info->bus_num,
				   sensor_info->sensor_addr, i,
				   pbuf[i * 2], pbuf[i * 2 + 1]);
			return ret;
		}
	}
	for (i = 0; i < setting_size; i++)
	{
		reg = pbuf[i * 2];
		data = hb_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, reg);
		printf("--camera RGB read reg: 0x%x val:0x%x\n", pbuf[i * 2], data);	
	}	
	data = hb_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x300a);
	printf("##camera RGB read reg: 0x300a val:0x%x\n", data);	
	return ret;
}
void sc230ai_common_data_init(sensor_info_t *sensor_info,
							  sensor_turning_data_t *turning_data)
{
	turning_data->bus_num = sensor_info->bus_num;
	turning_data->bus_type = sensor_info->bus_type;
	turning_data->port = sensor_info->port;
	turning_data->reg_width = sensor_info->reg_width;
	turning_data->mode = sensor_info->sensor_mode;
	turning_data->sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data->sensor_name, sensor_info->sensor_name,
			sizeof(turning_data->sensor_name));
	return;
}

int sc230ai_linear_data_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	uint16_t VTS_HIGH;
	uint16_t VTS_LOW;
	uint32_t VTS_VALUE;
	uint32_t open_cnt = 0;
	sensor_turning_data_t turning_data;
	uint32_t *stream_on = turning_data.stream_ctrl.stream_on;
	uint32_t *stream_off = turning_data.stream_ctrl.stream_off;
	trace("\n");
	memset(&turning_data, 0, sizeof(sensor_turning_data_t));

	// common data
	turning_data.bus_num = sensor_info->bus_num;
	turning_data.bus_type = sensor_info->bus_type;
	turning_data.port = sensor_info->port;
	turning_data.reg_width = sensor_info->reg_width;
	turning_data.mode = sensor_info->sensor_mode;
	turning_data.sensor_addr = sensor_info->sensor_addr;
	strncpy(turning_data.sensor_name, sensor_info->sensor_name,
			sizeof(turning_data.sensor_name));

	// turning sensor_data
	turning_data.sensor_data.turning_type = 6;
	turning_data.sensor_data.lines_per_second = 33750;  //vts * fps = 1125 * 30
	turning_data.sensor_data.exposure_time_max = 1125;  //lines_per_second/fps

	turning_data.sensor_data.active_width = 1920;
	turning_data.sensor_data.active_height = 1080;
	turning_data.sensor_data.gain_max = 251 * 8192;

	VTS_HIGH = hb_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x320e);
	VTS_HIGH = VTS_HIGH & 0x7f; //0x320e[0:6]
	VTS_LOW = hb_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, 0x320f);
	VTS_VALUE = (VTS_HIGH << 8 | VTS_LOW);
	printf("%s read VTS_HIGH = 0x%x, VTS_LOW = 0x%x, VTS = 0x%x \n",
			__FUNCTION__, VTS_HIGH, VTS_LOW, VTS_VALUE);

	turning_data.sensor_data.analog_gain_max = 251 * 8192;
	turning_data.sensor_data.digital_gain_max = 0;
	turning_data.sensor_data.exposure_time_min = 1;
	turning_data.sensor_data.exposure_time_long_max = 2242;
	// turning_data.sensor_data.conversion = 1;
	turning_data.sensor_data.fps = sensor_info->fps;  // fps
	turning_data.normal.again_control_num = 0;
        turning_data.normal.dgain_control_num = 0;
        turning_data.normal.s_line_length = 0;

	// turning normal
	// turning_data.normal.line_p.ratio = 1 << 8;
	// turning_data.normal.line_p.offset = 0;
	// turning_data.normal.line_p.max = 1500;

#if 0
	turning_data.normal.s_line = SC230AI_EXP_LINE;
	turning_data.normal.s_line_length = 2;
	turning_data.normal.again_control_num = 1;
	turning_data.normal.again_control[0] = SC230AI_PROGRAM_GAIN;
	turning_data.normal.again_control_length[0] = 2;
	turning_data.normal.dgain_control_num = 1;
	turning_data.normal.dgain_control[0] = SC230AI_DIGITAL_GAIN;
	turning_data.normal.dgain_control_length[0] = 2;
#endif
	turning_data.stream_ctrl.data_length = 1;
	if (sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sc230ai_stream_on_setting))
	{
		memcpy(stream_on, sc230ai_stream_on_setting, sizeof(sc230ai_stream_on_setting));
	}
	else
	{
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}
	if (sizeof(turning_data.stream_ctrl.stream_on) >= sizeof(sc230ai_stream_off_setting))
	{
		memcpy(stream_off, sc230ai_stream_off_setting, sizeof(sc230ai_stream_off_setting));
	}
	else
	{
		pr_err("Number of registers on stream over 10\n");
		return -RET_ERROR;
	}

	// turning_data.normal.again_lut = malloc(256 * sizeof(uint32_t));
	// if (turning_data.normal.again_lut != NULL)
	// {
	// 	memset(turning_data.normal.again_lut, 0xff, 256 * sizeof(uint32_t));
	// 	if (256 * sizeof(uint32_t) >= sizeof(sc230ai_again_lut))
	// 	{
	// 		memcpy(turning_data.normal.again_lut, sc230ai_again_lut, sizeof(sc230ai_again_lut));
	// 	}
	// 	else
	// 	{
	// 		pr_err("Number of registers on stream over %d\n", 256 * sizeof(uint32_t));
	// 		return -RET_ERROR;
	// 	}
	// }

	// turning_data.normal.dgain_lut = malloc(256 * sizeof(uint32_t));
	// if (turning_data.normal.dgain_lut != NULL)
	// {
	// 	memset(turning_data.normal.dgain_lut, 0xff, 256 * sizeof(uint32_t));
	// 	if (256 * sizeof(uint32_t) >= sizeof(sc230ai_dgain_lut))
	// 	{
	// 		memcpy(turning_data.normal.dgain_lut, sc230ai_dgain_lut, sizeof(sc230ai_dgain_lut));
	// 	}
	// 	else
	// 	{
	// 		pr_err("Number of registers on stream over %d\n", 256 * sizeof(uint32_t));
	// 		return -RET_ERROR;
	// 	}
	// }

	ret = ioctl(sensor_info->sen_devfd, SENSOR_TURNING_PARAM, &turning_data);
	if (turning_data.normal.again_lut)
	{
		free(turning_data.normal.again_lut);
		turning_data.normal.again_lut = NULL;
	}
	if (turning_data.normal.dgain_lut)
	{
		free(turning_data.normal.dgain_lut);
		turning_data.normal.dgain_lut = NULL;
	}
	if (ret < 0)
	{
		pr_err("sensor_%d ioctl fail %d\n", ret);
		return -RET_ERROR;
	}

	return ret;
}

int sc230ai_mode_config_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int setting_size = 0, i;
	uint16_t tmp;

	switch (sensor_info->sensor_mode)
	{
	case NORMAL_M:
		printf("sc230ai 2 lane mode\n");
		ret = sc230ai_mode_config_i2c_write(sensor_info, sc230ai_2lane_10bit_1920x1080_30fps_setting, sizeof(sc230ai_2lane_10bit_1920x1080_30fps_setting));
		if (ret < 0)
			return ret;
		ret = sc230ai_linear_data_init(sensor_info);
		if (ret < 0)
		{
			pr_err("sensor_normal_data_init %s fail\n", sensor_info->sensor_name);
			return ret;
		}
		pr_info("sc230ai_2lane_10bit_1920x1080_30fps_setting OK!\n");
		break;

	default:
		pr_err("config mode is err, sensor_mode:%d\n", sensor_info->sensor_mode);
		return -RET_ERROR;
	}
	/*
		if(sensor_info->extra_mode == 0) {
	*/
	return ret;
}

static int sc230ai_init(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	char str[12] = {0};

	snprintf(str, sizeof(str), "/dev/port_%d", sensor_info->dev_port);
	if (sensor_info->sen_devfd <= 0)
	{
		if ((sensor_info->sen_devfd = open(str, O_RDWR)) < 0)
		{
			pr_err("port_%d open fail\n", sensor_info->port);
			return -RET_ERROR;
		}
	}
	pr_info("/dev/port_%d success sensor_info->sen_devfd %d===\n",
			sensor_info->dev_port, sensor_info->sen_devfd);

	// ret = sc230ai_poweron(sensor_info);
	// if (ret < 0)
	// {
	// 	pr_err("%d : sensor_poweron %s fail\n", __LINE__, sensor_info->sensor_name);
	// 	return -RET_ERROR;
	// }

	ret = sc230ai_reset(sensor_info);
	if (ret < 0)
	{
		pr_err("%d : reset %s fail\n", __LINE__, sensor_info->sensor_name);
		return -RET_ERROR;
	}

	ret = sc230ai_mode_config_init(sensor_info);
	if (ret < 0)
		pr_err("%d : init %s fail\n", __LINE__, sensor_info->sensor_name);
	return ret;
}

static int sc230ai_start(sensor_info_t *sensor_info)
{
	int setting_size = 0, i;
	int ret = RET_OK;
	uint16_t reg, data;
	uint8_t tmp;

	setting_size = sizeof(sc230ai_stream_on_setting) / sizeof(uint32_t) / 2;
	pr_info("%s sensor_start setting_size %d\n",
			sensor_info->sensor_name, setting_size);
	for (i = 0; i < setting_size; i++)
	{
		reg = sc230ai_stream_on_setting[i * 2];
		data = sc230ai_stream_on_setting[i * 2 + 1];
		ret = hb_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, reg, data);
		if (ret < 0)
		{
			pr_err("%d : start %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
		tmp = hb_i2c_read_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, sc230ai_stream_on_setting[i*2]);
		printf("start : reg_add = 0x%x, data = 0x%x\n",sc230ai_stream_on_setting[i*2], tmp);
	}

	return ret;
}

static int sc230ai_stop(sensor_info_t *sensor_info)
{
	int setting_size = 0, i;
	int ret = RET_OK;
	uint16_t reg, data;

	setting_size = sizeof(sc230ai_stream_off_setting) / sizeof(uint32_t) / 2;
	pr_info("%s sensor_stop setting_size %d\n", sensor_info->sensor_name, setting_size);
	for (i = 0; i < setting_size; i++)
	{
		reg = sc230ai_stream_off_setting[i * 2];
		data = sc230ai_stream_off_setting[i * 2 + 1];
		ret = hb_i2c_write_reg16_data8(sensor_info->bus_num, sensor_info->sensor_addr, reg, data);
		if (ret < 0)
		{
			pr_err("%d : stop %s fail\n", __LINE__, sensor_info->sensor_name);
			return ret;
		}
		// printf("stop:reg_add = 0x%x, data = 0x%x\n",sc230ai_stream_off_setting[i*2], sc230ai_stream_off_setting[i*2 + 1]);
	}

	return ret;
}

static int sc230ai_deinit(sensor_info_t *sensor_info)
{
	int ret = RET_OK;
	int gpio;

	if (sensor_info->power_mode)
	{
		for (gpio = 0; gpio < sensor_info->gpio_num; gpio++)
		{
			if (sensor_info->gpio_pin[gpio] != -1)
			{
				ret = camera_power_ctrl(sensor_info->gpio_pin[gpio],
										sensor_info->gpio_level[gpio]);
				if (ret < 0)
				{
					pr_err("camera_power_ctrl fail\n");
					return -HB_CAM_SENSOR_POWERON_FAIL;
				}
			}
		}
	}
	if (sensor_info->sen_devfd != 0)
	{
		close(sensor_info->sen_devfd);
		sensor_info->sen_devfd = -1;
	}
	return ret;
}

static int sc230ai_aexp_gain_control(hal_control_info_t *info, uint32_t mode, uint32_t *again, uint32_t *dgain, uint32_t gain_num)
{

	//    printf(" again[0]:%x, dgain[0]:%x\n",again[0], dgain[0]);
	const uint16_t AGAIN = 0x3e09;
	const uint16_t DGAIN = 0x3e06;
	const uint16_t DFINE_GAIN = 0x3e07;
	char again_reg_value = 0;
	char dgain_reg_value = 0, d_fine_gain_reg_value = 0;
	int gain_index = 0;

	if (again[0] >= sizeof(sc230ai_again_lut)/sizeof(uint32_t))
		gain_index = sizeof(sc230ai_again_lut)/sizeof(uint32_t) - 1;
	else
		gain_index = again[0];

	again_reg_value = (sc230ai_again_lut[gain_index] >> 16) & 0x000000FF;
	dgain_reg_value = (sc230ai_again_lut[gain_index] >> 8) & 0x000000FF;
	d_fine_gain_reg_value = sc230ai_again_lut[gain_index] & 0x000000FF;

	camera_i2c_write8(info->bus_num, 16, info->sensor_addr, AGAIN, again_reg_value);
	camera_i2c_write8(info->bus_num, 16, info->sensor_addr, DGAIN, dgain_reg_value);
	camera_i2c_write8(info->bus_num, 16, info->sensor_addr, DFINE_GAIN, d_fine_gain_reg_value);

	return 0;
}

static int sc230ai_ae_set(uint32_t bus, uint32_t addr, uint32_t line)
{
	const uint16_t EXP_LINE0 = 0x3e00;
	const uint16_t EXP_LINE1 = 0x3e01;
	const uint16_t EXP_LINE2 = 0x3e02;
	const uint16_t S_EXP_LINE0 = 0x3e04;
	const uint16_t S_EXP_LINE1 = 0x3e05;
	char temp0 = 0, temp1 = 0, temp2 = 0;

	uint32_t sline = line;

	temp0 = (sline & 0xF000) >> 12;
	temp1 = (sline & 0xFF0) >> 4;
	temp2 = (sline & 0x0F) << 4;
	camera_i2c_write8(bus, 16, 0x30, EXP_LINE0, temp0);
	camera_i2c_write8(bus, 16, 0x30, EXP_LINE1, temp1);
	camera_i2c_write8(bus, 16, 0x30, EXP_LINE2, temp2);

	camera_i2c_write8(bus, 16, 0x32, EXP_LINE0, temp0);
	camera_i2c_write8(bus, 16, 0x32, EXP_LINE1, temp1);
	camera_i2c_write8(bus, 16, 0x32, EXP_LINE2, temp2);

	return 0;
}

#define SAMPLECNT 8
static uint32_t sc230ai_line_agv(uint32_t line)
{
	uint32_t average, i;
	uint64_t sum = 0;
	static uint32_t sample_ae[SAMPLECNT];
	static uint32_t index = 0;
	sample_ae[index++] = line;
	if (index == SAMPLECNT)
		index = 0;
	for (i = 0; i < SAMPLECNT; i++)
		sum += sample_ae[i];

	average = sum / SAMPLECNT;
	return average;
}

static int sc230ai_aexp_line_control(hal_control_info_t *info, uint32_t mode, uint32_t *line, uint32_t line_num)
{
	// uint32_t val;
	// if (mode == NORMAL_M)
	// {
	// 	val = sc230ai_line_agv(line[0]);
	// 	sc230ai_ae_set(info->bus_num, info->sensor_addr, val);
	// }
	// else
	// {
	// 	pr_err(" unsupport mode %d\n", mode);
	// }

	const uint16_t EXP_LINE0 = 0x3e00;
	const uint16_t EXP_LINE1 = 0x3e01;
	const uint16_t EXP_LINE2 = 0x3e02;
	char temp0 = 0, temp1 = 0, temp2 = 0;

	uint32_t sline = 2 * line[0];
	/*
	* NOTICE: sensor exposure half line, so sline = 2 * line(from isp)
	* exposure line max, from customer:
	* exposure_time_max = 10ms, line = 337, result = 337 * 2 = 674
	* form spec:
	* exposure_time_max = 2 * VTS - 8, 10fps, result = 11250 * 2 - 8
	* so, we should limit sline = 674
	*/
	if ( sline > 2250) {
		sline = 2250;
	}

	temp0 = (sline >> 12) & 0x0F;
	camera_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE0, temp0);
	temp1 = (sline >> 4) & 0xFF;
	camera_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE1, temp1);
	temp2 = (sline & 0x0F) << 4;
	camera_i2c_write8(info->bus_num, 16, info->sensor_addr, EXP_LINE2, temp2);


	return 0;
}

static int sc230ai_userspace_control(uint32_t port, uint32_t *enable)
{
	trace("enable userspace control\n");
	*enable = HAL_GAIN_CONTROL | HAL_LINE_CONTROL;
	//*enable = HAL_LINE_CONTROL;
	return 0;
}

sensor_module_t sc230ai = {
	.module = "sc230ai",
	.init = sc230ai_init,
	.start = sc230ai_start,
	.stop = sc230ai_stop,
	.deinit = sc230ai_deinit,
	.power_on = sc230ai_poweron,
	.power_off = sc230ai_poweroff,
	.aexp_gain_control = sc230ai_aexp_gain_control,
	.aexp_line_control = sc230ai_aexp_line_control,
	.userspace_control = sc230ai_userspace_control,
};
