/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * Lightware SF1xx/SF20/LW20 laser rangefinder (i2c)
 *
 * @reboot_required true
 * @min 0
 * @max 6
 * @group Sensors
 * @value 0 Disabled
 * @value 1 SF10/a
 * @value 2 SF10/b
 * @value 3 SF10/c
 * @value 4 SF11/c
 * @value 5 SF/LW20/b
 * @value 6 SF/LW20/c
 * @value 7 SF/LW30/d
 */
PARAM_DEFINE_INT32(SENS_EN_SF1XX, 0);

/**
 * Lightware SF1xx/SF20/LW20 Operation Mode
 *
 * @value 0 Disabled
 * @value 1 Enabled
 * @value 2 Enabled in VTOL MC mode, listen to request from system in FW mode
 *
 * @min 0
 * @max 2
 */
PARAM_DEFINE_INT32(SF1XX_MODE, 1);


/**
 * Lightware Laser Orientation (serial)
 *
 * Valori come da DistanceSensor.msg (body frame):
 *
 * @reboot_required true
 * @group Sensors
 * @min 0
 * @max 25
 * @value 0  ROTATION_YAW_0 / FORWARD_FACING
 * @value 1  ROTATION_YAW_45
 * @value 2  ROTATION_YAW_90 / RIGHT_FACING
 * @value 3  ROTATION_YAW_135
 * @value 4  ROTATION_YAW_180 / BACKWARD_FACING
 * @value 5  ROTATION_YAW_225
 * @value 6  ROTATION_YAW_270 / LEFT_FACING
 * @value 7  ROTATION_YAW_315
 * @value 24 ROTATION_UPWARD_FACING
 * @value 25 ROTATION_DOWNWARD_FACING (default)
 * @value 100 ROTATION_CUSTOM (usa anche il quaternion 'q')
 */
PARAM_DEFINE_INT32(LW_RNG_ROT, 25);
