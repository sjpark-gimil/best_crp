/* PN532 (I2C) -> BLE UART (USART2) */
#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "i2c.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

void SystemClock_Config(void);
void Error_Handler(void);

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

static void ble_print(const char *s)
{
  if (!s) return;
  (void)HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 200);
}

static void ble_print_hex(const uint8_t *buf, uint32_t n)
{
  char out[3*16 + 4];              
  const char *hex = "0123456789ABCDEF";
  uint32_t i=0, w=0;
  for (; i<n && i<16; ++i) {
    out[w++] = hex[(buf[i]>>4) & 0xF];
    out[w++] = hex[buf[i] & 0xF];
  }
  out[w++] = '\r'; out[w++] = '\n'; out[w] = 0;
  (void)HAL_UART_Transmit(&huart2, (uint8_t*)out, (uint16_t)w, 200);
}

#define PN532_I2C_ADDR      (0x24u << 1)   
#define PN532_READY_TIMEOUT 50            
#define PN532_XFER_TIMEOUT  100       

static HAL_StatusTypeDef i2c_read_status(uint8_t *status)
{
  return HAL_I2C_Master_Receive(&hi2c1, PN532_I2C_ADDR, status, 1, PN532_XFER_TIMEOUT);
}

static bool pn532_wait_ready(uint32_t ms)
{
  uint32_t t0 = HAL_GetTick();
  for (;;)
  {
    uint8_t st = 0;
    if (i2c_read_status(&st) == HAL_OK && st == 0x01)
      return true;
    if ((HAL_GetTick() - t0) >= ms)
      return false;
    HAL_Delay(2);
  }
}

static bool pn532_write_cmd(const uint8_t *payload, uint8_t plen)
{
  uint8_t frame[8 + 64]; 
  if (plen > 60) return false;

  uint8_t len  = plen;
  uint8_t lcs  = (uint8_t)(~len + 1);
  uint8_t dcs  = 0;
  for (uint8_t i=0; i<plen; ++i) dcs += payload[i];
  dcs = (uint8_t)(~dcs + 1);

  uint8_t w = 0;
  frame[w++] = 0x00;         
  frame[w++] = 0x00; frame[w++] = 0x00; frame[w++] = 0xFF;
  frame[w++] = len;  frame[w++] = lcs;
  memcpy(&frame[w], payload, plen); w += plen;
  frame[w++] = dcs;
  frame[w++] = 0x00;

  if (HAL_I2C_Master_Transmit(&hi2c1, PN532_I2C_ADDR, frame, w, PN532_XFER_TIMEOUT) != HAL_OK)
    return false;

  if (!pn532_wait_ready(PN532_READY_TIMEOUT)) return false;
  uint8_t ack[6];
  if (HAL_I2C_Master_Receive(&hi2c1, PN532_I2C_ADDR, ack, sizeof(ack), PN532_XFER_TIMEOUT) != HAL_OK)
    return false;
  static const uint8_t expect[6] = {0x00,0x00,0xFF,0x00,0xFF,0x00};
  return (memcmp(ack, expect, 6) == 0);
}

static uint8_t pn532_read_resp(uint8_t *buf, uint8_t max)
{
  if (!pn532_wait_ready(100)) return 0;

  uint8_t head[8];
  if (HAL_I2C_Master_Receive(&hi2c1, PN532_I2C_ADDR, head, 6, PN532_XFER_TIMEOUT) != HAL_OK)
    return 0;

  uint8_t frame[72];
  if (HAL_I2C_Master_Receive(&hi2c1, PN532_I2C_ADDR, frame, sizeof(frame), PN532_XFER_TIMEOUT) != HAL_OK)
    return 0;

  uint32_t i=0;
  while (i + 3 < sizeof(frame) && !(frame[i]==0x00 && frame[i+1]==0x00 && frame[i+2]==0xFF)) i++;
  if (i + 6 >= sizeof(frame)) return 0;

  uint8_t len = frame[i+3];
  uint8_t lcs = frame[i+4];
  if ((uint8_t)(len + lcs) != 0x00) return 0;

  if (i + 5 + len + 2 > sizeof(frame)) return 0;  

  const uint8_t *p = &frame[i+5];
  uint8_t dcs = 0;
  for (uint8_t k=0;k<len;k++) dcs += p[k];
  dcs = (uint8_t)(~dcs + 1);

  if (frame[i+5+len] != dcs) return 0;

  if (len > max) len = max;
  memcpy(buf, p, len);
  return len;
}

static bool pn532_wakeup(void)
{
  const uint8_t wake[] = { 0xD4, 0x02 }; 
  return pn532_write_cmd(wake, sizeof(wake)); 
}

static bool pn532_get_firmware(uint8_t *out, uint8_t *outlen)
{
  const uint8_t cmd[] = { 0xD4, 0x02 };
  if (!pn532_write_cmd(cmd, sizeof(cmd))) return false;
  uint8_t resp[32]; uint8_t n = pn532_read_resp(resp, sizeof(resp));
  if (n < 2 || resp[0] != 0xD5 || resp[1] != (0x02 + 1)) return false;
  if (out && outlen) {
    uint8_t cpy = (n-2 > *outlen) ? *outlen : (n-2);
    memcpy(out, &resp[2], cpy); *outlen = cpy;
  }
  return true;
}

static bool pn532_sam_config(void)
{
  const uint8_t cmd[] = { 0xD4, 0x14, 0x01, 0x14, 0x01 };
  if (!pn532_write_cmd(cmd, sizeof(cmd))) return false;
  uint8_t resp[8]; uint8_t n = pn532_read_resp(resp, sizeof(resp));
  return (n>=2 && resp[0]==0xD5 && resp[1]==(0x14+1));
}

static uint8_t pn532_read_uid_iso14443a(uint8_t *uid, uint8_t max_uid)
{
  const uint8_t cmd[] = { 0xD4, 0x4A, 0x01, 0x00 };
  if (!pn532_write_cmd(cmd, sizeof(cmd))) return 0;
  uint8_t resp[40]; uint8_t n = pn532_read_resp(resp, sizeof(resp));
  if (n < 3 || resp[0] != 0xD5 || resp[1] != 0x4B || resp[2] == 0x00) return 0;

  uint8_t uidlen = 0;
  for (uint8_t i = 7; i + 1 < n; ++i) {
    uint8_t L = resp[i];
    if (L <= 10 && i + 1 + L <= n) { 
      uidlen = L;
      if (uid && max_uid) {
        uint8_t cpy = (L > max_uid) ? max_uid : L;
        memcpy(uid, &resp[i+1], cpy);
      }
      break;
    }
  }
  return uidlen;
}

/* ---------------- Main ---------------- */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  ble_print("BOOT\r\n");

  if (!pn532_wakeup()) {
    ble_print("PN532 WAKE FAIL\r\n");
  } else {
    uint8_t fw[8]; uint8_t fwlen = sizeof(fw);
    if (pn532_get_firmware(fw, &fwlen)) {
      ble_print("PN532 FW: ");
      ble_print_hex(fw, fwlen);   
    } else {
      ble_print("PN532 FW ERR\r\n");
    }

    if (pn532_sam_config())
      ble_print("SAM OK\r\n");
    else
      ble_print("SAM ERR\r\n");
  }

  uint8_t last_uid[10] = {0};
  uint8_t last_len = 0;
  uint32_t quiet_ms_after_hit = 800;

  for (;;)
  {
    uint8_t uid[10] = {0};
    uint8_t ulen = pn532_read_uid_iso14443a(uid, sizeof(uid));

    if (ulen > 0) {
      if (ulen != last_len || memcmp(uid, last_uid, ulen) != 0) {
        ble_print("UID:");
        ble_print_hex(uid, ulen);
        memcpy(last_uid, uid, ulen);
        last_len = ulen;
      }
      HAL_Delay(quiet_ms_after_hit); 
    } else {
      last_len = 0;
      HAL_Delay(300);
    }
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|
                                RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}