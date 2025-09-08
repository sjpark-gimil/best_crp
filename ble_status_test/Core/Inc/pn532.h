#ifndef PN532_H
#define PN532_H

#include "stm32l1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 7-bit I2C address (default) */
#ifndef PN532_I2C_ADDR
#define PN532_I2C_ADDR  (0x24)
#endif

/* hi2c1 must be created by CubeMX */
extern I2C_HandleTypeDef hi2c1;

/* Basic init: wakes chip and puts it in “Normal mode” for host control */
bool PN532_Begin(void);

/* Optional: read firmware version (IC, Ver, Rev, Support) into 32-bit */
bool PN532_GetFirmwareVersion(uint32_t *out);

/* Configure the SAM (mandatory before InListPassiveTarget):
   mode=0x01 (Normal), timeout=0x14 (50ms), use_irq=0x01/0x00 (ignored if no IRQ) */
bool PN532_SAMConfiguration(void);

/* Scan for one ISO14443A (106 kbps) card; returns UID bytes and len.
   timeout_ms is overall wait time for a response frame. */
bool PN532_ReadPassiveTargetA(uint8_t *uid, uint8_t *uid_len, uint16_t timeout_ms);

#ifdef __cplusplus
}
#endif
#endif /* PN532_H */
