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
 * @file ST_LSM6DSO_Registers.hpp
 *
 * ST LSM6DSO / LSM6DSOX registers.
 *
 */

#pragma once

#include <cstddef>
#include <cstdint>

static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace ST_LSM6DSO
{

static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10 MHz SPI data clock (device max)

static constexpr uint8_t DIR_READ = 0x80;

// LSM6DSO and LSM6DSOX. The LSM6DSO32(X) reports the same ID but its FS_XL table differs
// (01 = ±32 g), so it is not supported by this driver.
static constexpr uint8_t WHO_AM_I_ID = 0x6C;

// High-performance mode ODR. The LSM6DSO steps are 1666/3333/6667 Hz; 6667 Hz is the fastest
// rate both the accelerometer and gyroscope support in high-performance mode.
static constexpr uint32_t ODR = 6667;

enum class Register : uint8_t {
	FIFO_CTRL1       = 0x07, // WTM[7:0]
	FIFO_CTRL2       = 0x08, // WTM[8], compression
	FIFO_CTRL3       = 0x09, // BDR_GY[7:4], BDR_XL[3:0]
	FIFO_CTRL4       = 0x0A, // DEC_TS_BATCH, ODR_T_BATCH, FIFO_MODE

	COUNTER_BDR_REG1 = 0x0B, // dataready_pulsed

	INT1_CTRL        = 0x0D, // INT1 pin control

	WHO_AM_I         = 0x0F,

	CTRL1_XL         = 0x10, // Accel ODR + FS + LPF2 enable
	CTRL2_G          = 0x11, // Gyro  ODR + FS
	CTRL3_C          = 0x12, // BDU, IF_INC, SW_RESET
	CTRL4_C          = 0x13, // I2C disable, gyro LPF1 enable
	CTRL6_C          = 0x15, // Gyro LPF1 bandwidth
	CTRL8_XL         = 0x17, // Accel LPF2 bandwidth
	CTRL9_XL         = 0x18, // I3C disable

	STATUS_REG       = 0x1E,

	OUT_TEMP_L       = 0x20,
	OUT_TEMP_H       = 0x21,

	FIFO_STATUS1     = 0x3A, // DIFF_FIFO[7:0]
	FIFO_STATUS2     = 0x3B, // flags + DIFF_FIFO[9:8]

	FIFO_DATA_OUT_TAG = 0x78,
};

// CTRL1_XL — Accelerometer ODR [7:4], full-scale [3:2], LPF2 enable [1]
enum CTRL1_XL_BIT : uint8_t {
	ODR_XL_MASK   = 0xF0,
	ODR_XL_6667HZ = 0xA0,

	FS_XL_MASK    = Bit3 | Bit2,
	FS_XL_16G     = Bit2, // FS_XL = 01: ±16 g

	LPF2_XL_EN    = Bit1,
};

// CTRL2_G — Gyroscope ODR [7:4], full-scale [3:2], FS_125 [1]
enum CTRL2_G_BIT : uint8_t {
	ODR_G_MASK    = 0xF0,
	ODR_G_6667HZ  = 0xA0,

	FS_G_MASK     = Bit3 | Bit2 | Bit1, // FS_G + FS_125
	FS_G_2000DPS  = Bit3 | Bit2,        // FS_G = 11: ±2000 dps
};

// CTRL3_C
enum CTRL3_C_BIT : uint8_t {
	BDU       = Bit6, // Block Data Update
	H_LACTIVE = Bit5, // Interrupt active-low
	PP_OD     = Bit4, // Push-pull (0) / Open-drain (1)
	IF_INC    = Bit2, // Register address auto-increment
	SW_RESET  = Bit0, // Software reset
};

// CTRL4_C
enum CTRL4_C_BIT : uint8_t {
	I2C_DISABLE = Bit2, // SPI only
	LPF1_SEL_G  = Bit1, // Enable gyroscope digital LPF1
};

// CTRL6_C — FTYPE [2:0] gyro LPF1 bandwidth: 000 = 335 Hz at 6667 Hz ODR (datasheet table 60)
enum CTRL6_C_BIT : uint8_t {
	XL_HM_MODE = Bit4, // 1 disables accelerometer high-performance mode
	FTYPE_MASK = Bit2 | Bit1 | Bit0,
};

// CTRL8_XL — HPCF_XL [7:5] accel LPF2 cutoff when CTRL1_XL.LPF2_XL_EN = 1
enum CTRL8_XL_BIT : uint8_t {
	HPCF_XL_MASK       = Bit7 | Bit6 | Bit5,
	HPCF_XL_ODR_DIV_10 = Bit5, // 001: ODR/10
	HP_SLOPE_XL_EN     = Bit2, // High-pass instead of low-pass
	XL_FS_MODE         = Bit1, // 1 selects the alternate full-scale table (FS_XL 01 = ±2 g)
};

// CTRL9_XL — DEN bits [7:5] default 111, I3C disable [1]
enum CTRL9_XL_BIT : uint8_t {
	I3C_DISABLE = Bit1,
};

// COUNTER_BDR_REG1
enum COUNTER_BDR_REG1_BIT : uint8_t {
	DATAREADY_PULSED = Bit7,
};

// INT1_CTRL
enum INT1_CTRL_BIT : uint8_t {
	INT1_FIFO_TH = Bit3, // FIFO threshold interrupt on INT1
};

// STATUS_REG
enum STATUS_REG_BIT : uint8_t {
	XLDA = Bit0, // Accelerometer new data available
	GDA  = Bit1, // Gyroscope new data available
	TDA  = Bit2, // Temperature new data available
};

// FIFO_CTRL2
enum FIFO_CTRL2_BIT : uint8_t {
	WTM8 = Bit0, // FIFO watermark bit 8
};

// FIFO_CTRL3 — batch data rate codes, gyro in [7:4], accel in [3:0]
enum FIFO_CTRL3_BIT : uint8_t {
	BDR_XL_6667HZ = 0x0A,
	BDR_GY_6667HZ = 0xA0,
};

// FIFO_CTRL4
enum FIFO_CTRL4_BIT : uint8_t {
	FIFO_MODE_MASK       = Bit2 | Bit1 | Bit0,
	ODR_T_BATCH_MASK     = Bit5 | Bit4,
	DEC_TS_BATCH_MASK    = Bit7 | Bit6,
	FIFO_MODE_BYPASS     = 0x00,
	FIFO_MODE_CONTINUOUS = Bit2 | Bit1, // 110
};

// FIFO_STATUS2
enum FIFO_STATUS2_BIT : uint8_t {
	FIFO_WTM_IA      = Bit7,
	FIFO_OVR_IA      = Bit6,
	FIFO_FULL_IA     = Bit5,
	COUNTER_BDR_IA   = Bit4,
	FIFO_OVR_LATCHED = Bit3,
	DIFF_FIFO_MASK   = Bit1 | Bit0, // DIFF_FIFO[9:8]
};

// FIFO tag IDs (upper 5 bits of FIFO_DATA_OUT_TAG >> 3)
enum class FifoTag : uint8_t {
	GYRO_NC  = 0x01,
	ACCEL_NC = 0x02,
};

namespace FIFO
{
// FIFO word: 1-byte tag + 6-byte data = 7 bytes
static constexpr size_t WORD_SIZE = 7;
// Words batched per sample period: gyro + accel
static constexpr size_t WORDS_PER_PERIOD = 2;
// Max sample periods to drain per poll (avoid blocking scheduler)
static constexpr size_t MAX_DRAIN_SAMPLES = 32;
}

} // namespace ST_LSM6DSO
