/**
 * Abilita l'avvio automatico del driver ESP32  via I2C
 *
 * 0: disabled 
 * 1: enabled
 *
 * @group ESP32 Closed arms
 * @reboot_required true
 * @boolean
 */
PARAM_DEFINE_INT32(CLOSED_ARMS_EN, 0);
