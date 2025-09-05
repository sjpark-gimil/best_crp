#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "i2c.h"

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>

void SystemClock_Config(void);
void Error_Handler(void);

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef  hi2c1;

static inline uint32_t ms_now(void) { return HAL_GetTick(); }
static inline bool ms_elapsed(uint32_t t0, uint32_t ms) { return (HAL_GetTick() - t0) >= ms; }

static void uart2_drain_until_quiet(uint32_t ms_quiet)
{
    uint8_t d;
    uint32_t t0 = ms_now();
    for (;;)
    {
        if (HAL_UART_Receive(&huart2, &d, 1, 5) == HAL_OK) {
            t0 = ms_now();
            continue;
        }
        if (ms_elapsed(t0, ms_quiet)) break;
    }
}

static uint32_t ble_send_and_read(const char *tx,
                                  uint8_t *rx, uint32_t rx_sz,
                                  uint32_t quiet_ms, uint32_t tx_timeout_ms)
{
    if (rx && rx_sz) memset(rx, 0, rx_sz);
    uart2_drain_until_quiet(40);

    if (tx && *tx) {
        (void)HAL_UART_Transmit(&huart2, (uint8_t*)tx, (uint16_t)strlen(tx), tx_timeout_ms);
    }

    uint32_t n = 0;
    uint32_t t0 = ms_now();
    for (;;)
    {
        uint8_t b;
        if (HAL_UART_Receive(&huart2, &b, 1, 5) == HAL_OK) {
            if (rx && n < rx_sz) rx[n++] = b;
            t0 = ms_now();
            continue;
        }
        if (ms_elapsed(t0, quiet_ms)) break;
    }
    return n;
}

static void str_trim(char *s)
{
    if (!s) return;
    size_t len = strlen(s);
    size_t i = 0;
    while (i < len && (s[i]=='\r'||s[i]=='\n'||s[i]==' '||s[i]=='\t')) i++;
    if (i) memmove(s, s+i, len - i + 1);
    len = strlen(s);
    while (len && (s[len-1]=='\r'||s[len-1]=='\n'||s[len-1]==' '||s[len-1]=='\t')) s[--len] = 0;
}

static bool parse_name_from_line(const char *line, char *out, size_t out_sz)
{
    if (!line || !out || out_sz==0) return false;
    out[0] = 0;

    const char *p = strstr(line, "NAME");
    if (p) {
        const char *colon = strchr(p, ':');
        p = colon ? (colon + 1) : line;
    } else {
        p = line;
    }
    strncpy(out, p, out_sz-1); out[out_sz-1]=0;
    str_trim(out);

    if (out[0]==0) return false;
    if (!strcasecmp(out,"OK")) return false;
    if (!strncasecmp(out,"OK+",3)) return false;
    if (!strncasecmp(out,"POWER",5)) return false;
    return true;
}
                                char *line_out, size_t out_sz)
{
    if (!line_out || out_sz==0) return;
    line_out[0]=0;
    if (!buf || !len) return;

    char tmp[256];
    uint32_t cpy = (len >= sizeof(tmp)-1) ? (sizeof(tmp)-1) : len;
    memcpy(tmp, buf, cpy);
    tmp[cpy] = 0;

    char *line = tmp, *last = NULL;
    while (line && *line) {
        char *next = strpbrk(line, "\r\n");
        if (next) { *next = 0; next++; if (*next=='\n') next++; }
        char probe[128]; strncpy(probe, line, sizeof(probe)-1); probe[sizeof(probe)-1]=0;
        str_trim(probe);
        if (probe[0]) last = line;
        line = next;
    }
    if (last) { strncpy(line_out, last, out_sz-1); line_out[out_sz-1]=0; }
}

static uint8_t i2c1_scan_first_device(void)
{
    for (uint8_t addr = 1; addr < 0x7F; ++addr) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr<<1), 1, 5) == HAL_OK) {
            return addr;
        }
    }
    return 0xFF;
}

static bool rfid_try_read_uid(uint8_t i2c_addr, uint8_t *uid, uint8_t *uid_len)
{
    if (!uid || !uid_len) return false;
    *uid_len = 0;

    uint8_t probe[8] = {0};
    if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(i2c_addr<<1), probe, 4, 20) != HAL_OK)
        return false;

    uint8_t nz = 0;
    for (int i=0;i<4;i++) nz |= probe[i];
    if (nz == 0) return false;

    memcpy(uid, probe, 4);
    *uid_len = 4;
    return true;
}

static volatile uint32_t g_beat      = 0;
static uint8_t           g_last_rx[256];
static uint32_t          g_last_len   = 0;
static char              g_name[32];
static uint8_t           g_name_len   = 0;
static uint8_t           g_connected  = 0;
static uint8_t           g_rfid_addr  = RFID_ADDR_DEFAULT;
static uint8_t           g_uid[10];
static uint8_t           g_uid_len    = 0;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();

    HAL_Delay(80);
    uart2_drain_until_quiet(60);

    g_last_len = ble_send_and_read("AT\r\n", g_last_rx, sizeof(g_last_rx), 300, 200);

    const char *qnames[] = { "AT+NAME?\r\n", "AT+BTNAME?\r\n", "AT+GNAME\r\n" };
    g_name[0] = 0; g_name_len = 0;

    for (unsigned i = 0; i < sizeof(qnames)/sizeof(qnames[0]); ++i) {
        g_last_len = ble_send_and_read(qnames[i], g_last_rx, sizeof(g_last_rx), 400, 200);
        char line[128]; last_non_empty_line(g_last_rx, g_last_len, line, sizeof(line));
        char parsed[32];
        if (parse_name_from_line(line, parsed, sizeof(parsed))) {
            strncpy(g_name, parsed, sizeof(g_name)-1);
            g_name_len = (uint8_t)strlen(g_name);
            break;
        }
    }

    uint8_t found = i2c1_scan_first_device();
    if (found != 0xFF) {
        g_rfid_addr = found; 
    }

    while (1)
    {
        g_beat++; 

        g_last_len = ble_send_and_read("AT+STAT?\r\n", g_last_rx, sizeof(g_last_rx), 250, 200);
        if (g_last_len == 0) {
            g_last_len = ble_send_and_read("AT\r\n", g_last_rx, sizeof(g_last_rx), 250, 200);
        }

        char v[256]={0};
        memcpy(v, g_last_rx, (g_last_len < sizeof(v)-1) ? g_last_len : sizeof(v)-1);
        if (strstr(v, "CONNECTED") || strstr(v, "A\r") || strstr(v, "A\n"))  g_connected = 1;
        else if (strstr(v,"DISCONNECTED") || strstr(v, "a\r") || strstr(v,"a\n")) g_connected = 0;

        if (found != 0xFF) {
            g_uid_len = 0;
            (void)rfid_try_read_uid(g_rfid_addr, g_uid, &g_uid_len);

            if (g_connected && g_uid_len) {
                char line[64];
                int  n = snprintf(line, sizeof(line), "UID:%02X%02X%02X%02X\r\n",
                                  g_uid[0], g_uid[1], g_uid[2], g_uid[3]);
                if (n > 0) (void)HAL_UART_Transmit(&huart2, (uint8_t*)line, (uint16_t)n, 100);
            }
        }

        HAL_Delay(300); 
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) { Error_Handler(); }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

