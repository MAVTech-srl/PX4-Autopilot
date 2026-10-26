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
 * @file precland_params.c
 *
 * Parameters for precision landing.
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

/**
 * Landing Target Timeout
 *
 * Time after which the landing target is considered lost without any new measurements.
 *
 * @unit s
 * @min 0.0
 * @max 50
 * @decimal 1
 * @increment 0.5
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_BTOUT, 1.0f);

/**
 * Horizontal acceptance radius
 *
 * Start descending if closer above landing target than this.
 *
 * @unit m
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_HACC_RAD, 0.2f);

/**
 * Final approach altitude
 *
 * Allow final approach (without horizontal positioning) if losing landing target closer than this to the ground.
 *
 * @unit m
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_FAPPR_ALT, 0.15f);

/**
 * Height held above the detected target before and during horizontal positioning
 *
 * @unit m
 * @min 0.1
 * @max 20
 * @decimal 1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_HOV_ALT, 2.0f);

/**
 * Horizontal radius used to confirm that the hover reference is settled
 *
 * @unit m
 * @min 0.0
 * @max 10
 * @decimal 2
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_HSET_RAD, 0.6f);

/**
 * Vertical tolerance used to confirm that the hover reference is settled
 *
 * @unit m
 * @min 0.0
 * @max 10
 * @decimal 2
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_VACC_RAD, 0.5f);

/**
 * Time the hover reference must remain settled before approach begins
 *
 * @unit s
 * @min 0.0
 * @max 20
 * @decimal 1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_HSET_T, 1.0f);

/**
 * Horizontal error above which the vertical reference retreats to hover height
 *
 * @unit m
 * @min 0.0
 * @max 10
 * @decimal 2
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_REAPP_RAD, 0.6f);

/**
 * Maximum rate of change of the target-relative vertical reference
 *
 * @unit m/s
 * @min 0.01
 * @max 5
 * @decimal 2
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_OFFS_RATE, 0.3f);

/**
 * Hard target-loss timeout before falling back to a normal landing
 *
 * Must be greater than or equal to PLD_BTOUT to leave time for target recovery.
 *
 * @unit s
 * @min 0.1
 * @max 50
 * @decimal 1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_LOST_TOUT, 2.0f);

/**
 * Target velocity exponential-filter update weight
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_VEL_ALPHA, 0.3f);

/**
 * Maximum dead-reckoning horizon for a normally tracked target
 *
 * @unit s
 * @min 0.0
 * @max 5
 * @decimal 2
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_PRED_T, 0.5f);

/**
 * Plausible upper bound on target speed used by measurement outlier rejection
 *
 * @unit m/s
 * @min 0.0
 * @max 20
 * @decimal 1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_TGT_VMAX, 3.0f);

/**
 * Position innovation margin used by target measurement outlier rejection
 *
 * @unit m
 * @min 0.0
 * @max 10
 * @decimal 2
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_OUT_MARG, 0.3f);

/**
 * Consecutive outlier rejections before accepting a sustained target displacement
 *
 * @min 0
 * @max 100
 * @group Precision Land
 */
PARAM_DEFINE_INT32(PLD_OUT_MAX, 3);

/**
 * Search altitude
 *
 * Altitude above home to which to climb when searching for the landing target.
 *
 * @unit m
 * @min 0.0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_SRCH_ALT, 10.0f);

/**
 * Simple search and center-return timeout
 *
 * Time spent holding after reaching the simple-search point. For spiral
 * search this is only the maximum time allowed to return to the center.
 *
 * @unit s
 * @min 0.0
 * @max 100
 * @decimal 1
 * @increment 0.1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_SRCH_TOUT, 5.0f);

/**
 * Maximum number of search attempts
 *
 * Maximum number of times to search for the landing target if it is lost during the precision landing.
 *
 * @min 0
 * @max 100
 * @group Precision Land
 */
PARAM_DEFINE_INT32(PLD_MAX_SRCH, 3);

/**
 * Landing target search pattern
 *
 * Selects whether the vehicle holds over the search center or follows an
 * expanding spiral after reaching the search altitude.
 *
 * @value 0 Simple climb and hold
 * @value 1 Expanding spiral
 * @group Precision Land
 */
PARAM_DEFINE_INT32(PLD_SRCH_TYPE, 0);

/**
 * Number of landing target search spiral turns
 *
 * The final radius is PLD_SRCH_TURNS multiplied by PLD_SRCH_RSTEP.
 *
 * @min 1
 * @max 20
 * @group Precision Land
 */
PARAM_DEFINE_INT32(PLD_SRCH_TURNS, 3);

/**
 * Landing target search spiral radial increment per turn
 *
 * @unit m
 * @min 0.1
 * @max 20
 * @decimal 1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_SRCH_RSTEP, 3.0f);

/**
 * Landing target search spiral angular speed
 *
 * @unit deg/s
 * @min 1
 * @max 180
 * @decimal 1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_SRCH_OMEGA, 30.0f);

/**
 * Landing target search watchdog
 *
 * Maximum time allowed for climb and search before forcing a return to the
 * search center. Normal spiral completion is determined by PLD_SRCH_TURNS.
 *
 * @unit s
 * @min 0
 * @max 600
 * @decimal 1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_SRCH_WDOG, 60.0f);

/**
 * Enable landing target yaw alignment
 *
 * Uses the temporary Q4X convention where vx_rel/vy_rel contain the target
 * heading unit vector while rel_vel_valid remains false.
 *
 * @boolean
 * @group Precision Land
 */
PARAM_DEFINE_INT32(PLD_YAW_EN, 0);

/**
 * Maximum landing target yaw reference rate
 *
 * @unit rad/s
 * @min 0.0
 * @max 3.14
 * @decimal 2
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_YAW_RATE, 0.3f);

/**
 * Landing target yaw filter update weight
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_YAW_ALPHA, 0.3f);

/**
 * Landing target yaw alignment offset
 *
 * Added to the detected board heading before commanding vehicle yaw.
 *
 * @unit deg
 * @min -180
 * @max 180
 * @decimal 1
 * @group Precision Land
 */
PARAM_DEFINE_FLOAT(PLD_YAW_OFF, 0.0f);
