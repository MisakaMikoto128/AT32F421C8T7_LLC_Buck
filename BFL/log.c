#include "log.h"
#include <stdio.h>
#include <stdarg.h>

// 调试输出数据包最大长度
#define MAXDEBUGSEND 256
static uint8_t buffer[MAXDEBUGSEND + 1];
#define DEBUG_COM COM2

/**
 * @brief 指定终端的调试格式化输出方法。
 *
 * @param terminal_id 终端ID，仅在RTT模式下有效
 * @param format 格式化字符串
 * @param ... 可变参数
 */
void Debug_Printf_Terminal(uint8_t terminal_id, const void *format, ...)
{

    uint32_t uLen;
    va_list vArgs;
    va_start(vArgs, format);
    uLen = vsnprintf((char *)buffer, MAXDEBUGSEND, (char const *)format, vArgs);
    va_end(vArgs);
    if (uLen > MAXDEBUGSEND)
        uLen = MAXDEBUGSEND;
#if USING_RTT == 1
    SEGGER_RTT_Write(terminal_id, buffer, uLen);
#elif USING_USB_CDC == 1
    UNUSED(terminal_id);
    CDC_Transmit_FS(buffer, uLen);
#elif USING_UART == 1
    UNUSED(terminal_id);
    Uart_Write(DEBUG_COM, buffer, uLen);
#elif USING_SYS == 1
    UNUSED(terminal_id);
    printf("%s", buffer);
#endif // USING_RTT
#ifdef ULOG_ENABLED
#endif // ULOG_ENABLED
}

void Debug_Printf(const void *format, ...)
{
#ifdef ULOG_ENABLED
    uint32_t uLen;
    va_list vArgs;
    va_start(vArgs, format);
    uLen = vsnprintf((char *)buffer, MAXDEBUGSEND, (char const *)format, vArgs);
    va_end(vArgs);
    if (uLen > MAXDEBUGSEND)
        uLen = MAXDEBUGSEND;
#if USING_RTT == 1
    SEGGER_RTT_Write(0, buffer, uLen);
#elif USING_USB_CDC == 1
    CDC_Transmit_FS(buffer, uLen);
#elif USING_UART == 1
    Uart_Write(DEBUG_COM, buffer, uLen);
#elif USING_SYS == 1
    printf("%s", buffer);
#endif
#endif // ULOG_ENABLED
}

void my_console_logger(ulog_level_t severity, char *msg)
{
#ifdef ULOG_ENABLED
    switch (severity) {
        case ULOG_WARNING_LEVEL:
            Debug_Printf(RTT_CTRL_TEXT_YELLOW "[%s]: %s\n" RTT_CTRL_RESET,
                         ulog_level_name(severity),
                         msg);
            break;
        case ULOG_ERROR_LEVEL:
            Debug_Printf(RTT_CTRL_TEXT_RED "[%s]: %s\n" RTT_CTRL_RESET,
                         ulog_level_name(severity),
                         msg);
            break;
        case ULOG_CRITICAL_LEVEL:
            Debug_Printf(RTT_CTRL_BG_RED "[%s]: %s\n" RTT_CTRL_RESET,
                         ulog_level_name(severity),
                         msg);
            break;
        default:
            Debug_Printf("[%s]: %s\n",
                         ulog_level_name(severity),
                         msg);
            break;
    }
#endif // ULOG_ENABLED
}

void ulog_init_user()
{

#ifdef ULOG_ENABLED
    ULOG_INIT();

#if USING_RTT == 1
    /* 配置通道 0，上行配置 */
    SEGGER_RTT_ConfigUpBuffer(0, "RTTUP", NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);
    /* 配置通道 0，下行配置 */
    SEGGER_RTT_ConfigDownBuffer(0, "RTTDOWN", NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);
    SEGGER_RTT_SetTerminal(0);
#elif USING_USB_CDC == 1
#elif USING_UART == 1
    Uart_Init(DEBUG_COM, 1500000, LL_USART_DATAWIDTH_8B, LL_USART_STOPBITS_1, LL_USART_PARITY_NONE);
#elif USING_SYS == 1
    // do nothing
#endif
    // dynamically change the threshold for a specific logger
    ULOG_SUBSCRIBE(my_console_logger, ULOG_DEBUG_LEVEL);
#endif // ULOG_ENABLED
}
