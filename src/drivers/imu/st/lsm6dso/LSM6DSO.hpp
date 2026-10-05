/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

/**
 * @file LSM6DSO.hpp
 *
 * Driver for the ST LSM6DSO / LSM6DSOX connected via SPI.
 *
 */

#pragma once

#include "ST_LSM6DSO_Registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace ST_LSM6DSO;

class LSM6DSO : public device::SPI, public I2CSPIDriver<LSM6DSO>
{
public:
	LSM6DSO(const I2CSPIDriverConfig &config);
	~LSM6DSO() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	// Sensor Configuration
	static constexpr float FIFO_SAMPLE_DT{1e6f / ODR};
	static constexpr int32_t FIFO_MAX_SAMPLES{static_cast<int32_t>(FIFO::MAX_DRAIN_SAMPLES)};
	static_assert(FIFO_MAX_SAMPLES <= (int32_t)(sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0])),
		      "FIFO drain exceeds sensor_gyro_fifo capacity");
	static_assert(FIFO_MAX_SAMPLES <= (int32_t)(sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0])),
		      "FIFO drain exceeds sensor_accel_fifo capacity");

	// A FIFO word is a tag byte plus 6 data bytes. With IF_INC set the address rounds from
	// FIFO_DATA_OUT_Z_H back to FIFO_DATA_OUT_TAG at every word boundary, so the whole FIFO drains
	// as a single N*7 byte burst (AN5192 section 9).
	struct FIFOWord {
		uint8_t TAG;
		uint8_t DATA_X_L;
		uint8_t DATA_X_H;
		uint8_t DATA_Y_L;
		uint8_t DATA_Y_H;
		uint8_t DATA_Z_L;
		uint8_t DATA_Z_H;
	};
	static_assert(sizeof(FIFOWord) == FIFO::WORD_SIZE, "FIFO word must be 7 bytes");

	// RunImpl() drains whole sample periods only, at most FIFO_MAX_SAMPLES of them
	static constexpr uint16_t FIFO_MAX_WORDS{static_cast<uint16_t>(FIFO_MAX_SAMPLES * FIFO::WORDS_PER_PERIOD)};

	struct FIFOTransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::FIFO_DATA_OUT_TAG) | DIR_READ};
		FIFOWord words[FIFO_MAX_WORDS] {};
	};
	static_assert(sizeof(FIFOTransferBuffer) == (1 + FIFO_MAX_WORDS * FIFO::WORD_SIZE), "Invalid transfer buffer size");

	// held here rather than on the work queue stack: a wq:SPIx frame is not the place for a buffer
	// this size, and reusing it avoids re-zeroing memory that transfer() overwrites anyway
	FIFOTransferBuffer _fifo_buffer{};

	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Reset();

	bool Configure();
	void ConfigureSampleRate(int sample_rate);

	bool RegisterCheck(const register_config_t &reg_cfg);

	uint8_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint8_t value);
	void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);

	bool FIFORead(const hrt_abstime &timestamp_sample, uint16_t words);
	void FIFOReset();

	void UpdateTemperature();

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();
	void ConfigureFIFOWatermark(uint8_t samples);

	const spi_drdy_gpio_t _drdy_gpio;
	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
	perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};
	perf_counter_t _drdy_missed_perf{nullptr};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	hrt_abstime _temperature_update_timestamp{0};
	int _failure_count{0};
	uint8_t _checked_register{0};

	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};
	bool _data_ready_interrupt_enabled{false};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		FIFO_RESET,
		FIFO_READ,
	} _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{2500}; // default 2500 us / 400 Hz transfer interval
	int32_t _fifo_gyro_samples{static_cast<int32_t>(_fifo_empty_interval_us / FIFO_SAMPLE_DT)};

	static constexpr uint8_t size_register_cfg{13};
	// Configure() writes these in array order: every full-scale, filter, interrupt and FIFO register
	// comes first, and CTRL1_XL/CTRL2_G - which set the ODRs and thereby power the sensors up - last.
	register_config_t _register_cfg[size_register_cfg] {
		// Register                  | Set bits                                                    | Clear bits
		{ Register::CTRL3_C,          CTRL3_C_BIT::BDU | CTRL3_C_BIT::IF_INC,                       CTRL3_C_BIT::H_LACTIVE | CTRL3_C_BIT::PP_OD | CTRL3_C_BIT::SW_RESET },
		// SPI only: disable the I2C interface; gyro LPF1 is the anti-alias stage ahead of decimation
		{ Register::CTRL4_C,          CTRL4_C_BIT::I2C_DISABLE | CTRL4_C_BIT::LPF1_SEL_G,           0 },
		{ Register::CTRL6_C,          0,                                                            CTRL6_C_BIT::XL_HM_MODE | CTRL6_C_BIT::FTYPE_MASK },
		// LPF2 at ODR/10 (667 Hz) is the accel anti-alias stage: PX4 decimates the FIFO with a plain mean
		{ Register::CTRL8_XL,         CTRL8_XL_BIT::HPCF_XL_ODR_DIV_10, (CTRL8_XL_BIT::HPCF_XL_MASK & ~CTRL8_XL_BIT::HPCF_XL_ODR_DIV_10) | CTRL8_XL_BIT::HP_SLOPE_XL_EN | CTRL8_XL_BIT::XL_FS_MODE },
		{ Register::CTRL9_XL,         CTRL9_XL_BIT::I3C_DISABLE,                                    0 },
		{ Register::COUNTER_BDR_REG1, COUNTER_BDR_REG1_BIT::DATAREADY_PULSED,                       0 },
		{ Register::INT1_CTRL,        INT1_CTRL_BIT::INT1_FIFO_TH,                                  0 },
		{ Register::FIFO_CTRL1,       0, 0 }, // WTM[7:0] set at runtime by ConfigureFIFOWatermark()
		{ Register::FIFO_CTRL2,       0,                                                            FIFO_CTRL2_BIT::WTM8 },
		{ Register::FIFO_CTRL3,       FIFO_CTRL3_BIT::BDR_GY_6667HZ | FIFO_CTRL3_BIT::BDR_XL_6667HZ, static_cast<uint8_t>(~(FIFO_CTRL3_BIT::BDR_GY_6667HZ | FIFO_CTRL3_BIT::BDR_XL_6667HZ)) },
		{
			Register::FIFO_CTRL4,     FIFO_CTRL4_BIT::FIFO_MODE_CONTINUOUS,
			(FIFO_CTRL4_BIT::FIFO_MODE_MASK & ~FIFO_CTRL4_BIT::FIFO_MODE_CONTINUOUS) |
			FIFO_CTRL4_BIT::ODR_T_BATCH_MASK | FIFO_CTRL4_BIT::DEC_TS_BATCH_MASK
		},
		{ Register::CTRL1_XL,         CTRL1_XL_BIT::ODR_XL_6667HZ | CTRL1_XL_BIT::FS_XL_16G | CTRL1_XL_BIT::LPF2_XL_EN, static_cast<uint8_t>((CTRL1_XL_BIT::ODR_XL_MASK | CTRL1_XL_BIT::FS_XL_MASK) & ~(CTRL1_XL_BIT::ODR_XL_6667HZ | CTRL1_XL_BIT::FS_XL_16G)) },
		{ Register::CTRL2_G,          CTRL2_G_BIT::ODR_G_6667HZ | CTRL2_G_BIT::FS_G_2000DPS,        static_cast<uint8_t>((CTRL2_G_BIT::ODR_G_MASK | CTRL2_G_BIT::FS_G_MASK) & ~(CTRL2_G_BIT::ODR_G_6667HZ | CTRL2_G_BIT::FS_G_2000DPS)) },
	};
};
