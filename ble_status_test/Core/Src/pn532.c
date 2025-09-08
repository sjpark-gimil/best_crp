#include "pn532.h"
#include <string.h>

/* ---- PN532 frame constants ---- */
#define PN532_PREAMBLE      0x00
#define PN532_STARTCODE1    0x00
#define PN532_STARTCODE2    0xFF
#define PN532_POSTAMBLE     0x00

#define PN532_HOSTTOPN532   0xD4  /* TFI when host sends */
#define PN532_PN532TOHOST   0xD5  /* TFI when PN532 replies */

/* Commands we use */
#define PN532_CMD_GetFirmwareVersion  0x02
#define PN532_CMD_SAMConfiguration    0x14
#define PN532_CMD_InListPassiveTarget 0x4A

/* I2C helpers */
static HAL_StatusTypeDef i2c_write(const uint8_t *buf, uint16_t len) {
    return HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(PN532_I2C_ADDR << 1), (uint8_t*)buf, len, 50);
}
static HAL_StatusTypeDef i2c_read(uint8_t *buf, uint16_t len) {
    return HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(PN532_I2C_ADDR << 1), buf, len, 50);
}

/* Poll the “status” byte until ready (== 0x01), with timeout ms. */
static bool wait_ready(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    uint8_t  b  = 0;
    do {
        if (i2c_read(&b, 1) == HAL_OK && b == 0x01) return true;
    } while ((HAL_GetTick() - t0) < timeout_ms);
    return false;
}

/* Send a PN532 command frame (TFI=0xD4). data may be NULL if len=0. */
static bool write_command(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    /* Frame:
       [0x00][0x00][0xFF][LEN][LCS][TFI=0xD4][CMD][DATA...][DCS][0x00]
       For I2C write, prepend an extra “0x00” (per NXP/Adafruit notes). */
    uint8_t frame[8 + 255];  /* enough */
    uint8_t idx = 0;

    frame[idx++] = 0x00;                 /* I2C “write” header */
    frame[idx++] = PN532_PREAMBLE;
    frame[idx++] = PN532_STARTCODE1;
    frame[idx++] = PN532_STARTCODE2;

    uint8_t lenTFI = (uint8_t)(1 + 1 + len);       /* TFI + CMD + data */
    uint8_t LCS    = (uint8_t)(0x100 - lenTFI);

    frame[idx++] = lenTFI;
    frame[idx++] = LCS;

    frame[idx++] = PN532_HOSTTOPN532;
    frame[idx++] = cmd;

    uint8_t sum = PN532_HOSTTOPN532 + cmd;
    for (uint8_t i=0; i<len; ++i) {
        frame[idx++] = data[i];
        sum += data[i];
    }
    uint8_t DCS = (uint8_t)(0x100 - sum);
    frame[idx++] = DCS;
    frame[idx++] = PN532_POSTAMBLE;

    return (i2c_write(frame, idx) == HAL_OK);
}

/* Read and verify the 6-byte ACK frame: 00 00 FF 00 FF 00 (with leading 0x01). */
static bool read_ack(uint32_t timeout_ms)
{
    if (!wait_ready(timeout_ms)) return false;

    uint8_t buf[7] = {0};
    if (i2c_read(buf, sizeof(buf)) != HAL_OK) return false;

    /* buf[0] is status (should be 0x01), then 00 00 FF 00 FF 00 */
    return (buf[0] == 0x01 &&
            buf[1] == 0x00 && buf[2] == 0x00 &&
            buf[3] == 0xFF && buf[4] == 0x00 &&
            buf[5] == 0xFF && buf[6] == 0x00);
}

/* Read a whole response frame into 'out'. Returns payload length (TFI+PD length),
   or 0 on error. Expects a ready status first. */
static uint16_t read_response(uint8_t *out, uint16_t out_max, uint32_t timeout_ms)
{
    if (!wait_ready(timeout_ms)) return 0;

    /* Read header first: status + 5 bytes */
    uint8_t hdr[7] = {0};
    if (i2c_read(hdr, sizeof(hdr)) != HAL_OK) return 0;
    if (hdr[0] != 0x01) return 0; /* not ready */
    if (!(hdr[1]==0x00 && hdr[2]==0x00 && hdr[3]==0xFF)) return 0;

    uint8_t LEN = hdr[4];
    uint8_t LCS = hdr[5];
    if ((uint8_t)(LEN + LCS) != 0x00) return 0;

    /* Read payload (LEN bytes) + postamble (1) */
    uint16_t to_read = (uint16_t)LEN + 1; /* + postamble */
    if (to_read > out_max) return 0;
    if (i2c_read(out, to_read) != HAL_OK) return 0;

    /* out[LEN] should be 0x00 (postamble). Checksum is inside the LEN area:
       [TFI][PD0..PDn-1][DCS] */
    if (out[LEN] != 0x00) return 0;

    /* Verify TFI/DCS */
    if (LEN < 3) return 0; /* need at least TFI + RSP + DCS */
    uint8_t sum = 0;
    for (uint8_t i=0; i<LEN-1; ++i) sum += out[i];        /* exclude DCS */
    uint8_t DCS = out[LEN-1];
    if ((uint8_t)(sum + DCS) != 0x00) return 0;

    return LEN; /* includes TFI + PD bytes + DCS (we keep all for parsing) */
}

/* ---------------- Public API ---------------- */

bool PN532_Begin(void)
{
    /* A tiny wake delay helps after power-up */
    HAL_Delay(10);
    /* A no-op command (GetFirmware) also wakes the chip */
    uint8_t dummy;
    (void)PN532_GetFirmwareVersion((uint32_t*)&dummy);
    return PN532_SAMConfiguration();
}

bool PN532_GetFirmwareVersion(uint32_t *out)
{
    if (!write_command(PN532_CMD_GetFirmwareVersion, NULL, 0)) return false;
    if (!read_ack(50)) return false;

    uint8_t resp[40] = {0};
    uint16_t len = read_response(resp, sizeof(resp), 100);
    if (len < 6) return false;

    /* Expect TFI=0xD5, RSP=0x03 (0x02+1), then 4 bytes: IC, Ver, Rev, Support */
    if (!(resp[0] == PN532_PN532TOHOST && resp[1] == (PN532_CMD_GetFirmwareVersion + 1)))
        return false;

    if (out) {
        *out = ( (uint32_t)resp[2]        << 24 ) |
               ( (uint32_t)resp[3]        << 16 ) |
               ( (uint32_t)resp[4]        <<  8 ) |
               ( (uint32_t)resp[5] );
    }
    return true;
}

bool PN532_SAMConfiguration(void)
{
    uint8_t body[3] = { 0x01, 0x14, 0x01 }; /* Normal mode, timeout=50ms, use_irq=1 (ignored) */
    if (!write_command(PN532_CMD_SAMConfiguration, body, sizeof(body))) return false;
    if (!read_ack(50)) return false;

    uint8_t resp[16] = {0};
    uint16_t len = read_response(resp, sizeof(resp), 100);
    if (len < 2) return false;
    return (resp[0] == PN532_PN532TOHOST && resp[1] == (PN532_CMD_SAMConfiguration + 1));
}

bool PN532_ReadPassiveTargetA(uint8_t *uid, uint8_t *uid_len, uint16_t timeout_ms)
{
    if (!uid || !uid_len) return false;
    *uid_len = 0;

    uint8_t body[2] = { 0x01, 0x00 }; /* max 1 target, 106 kbps Type A */
    if (!write_command(PN532_CMD_InListPassiveTarget, body, sizeof(body))) return false;
    if (!read_ack(50)) return false;

    uint8_t resp[64] = {0};
    uint16_t len = read_response(resp, sizeof(resp), timeout_ms);
    if (len < 8) return false;

    /* Expect TFI=0xD5, RSP=0x4B (0x4A+1) */
    if (!(resp[0] == PN532_PN532TOHOST && resp[1] == (PN532_CMD_InListPassiveTarget + 1)))
        return false;

    /* Format:
       [0]TFI(0xD5) [1]0x4B [2]NbTg [3]Tg [4..]ATQA(2) [..]SAK(1) [..]UIDLen [UID...] */
    if (resp[2] == 0x00) return false; /* no card */
    uint8_t off = 3;                 /* start at Tg */
    off += 1;                        /* skip Tg */
    off += 2;                        /* skip ATQA */
    off += 1;                        /* skip SAK */
    if (off >= (len-1)) return false;

    uint8_t ulen = resp[off++];
    if (ulen == 0 || (off + ulen) > (len-1)) return false;

    memcpy(uid, &resp[off], ulen);
    *uid_len = ulen;
    return true;
}
