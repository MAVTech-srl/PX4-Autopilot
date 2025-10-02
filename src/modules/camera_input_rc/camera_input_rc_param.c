/**
 * RC AUX channel to use for manual camera trigger
 *
 * Select which AUX channel (1–6) is used as the trigger input.
 *
 * @reboot_required true
 * @group Camera Input RC
 * @value 0 Disabled
 * @value 1 AUX1
 * @value 2 AUX2
 * @value 3 AUX3
 * @value 4 AUX4
 * @value 5 AUX5
 * @value 6 AUX6
 */
PARAM_DEFINE_INT32(CAM_MAN_AUX, 0);


/**
 * Threshold for trigger activation
 *
 * Value range is normalized RC channel value (-1.0 to +1.0).
 * Typical: 0.5 means the switch must be past mid/high to trigger.
 *
 * @group Camera Input RC
 */
PARAM_DEFINE_FLOAT(CAM_MAN_THR, 0.5f);
