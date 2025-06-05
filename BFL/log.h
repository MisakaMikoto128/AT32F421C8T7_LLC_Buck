/**
 * @file log.h
 * @author Liu Yuanlin (liuyuanlins@outlook.com)
 * @brief
 * @version 0.1
 * @date 2023-01-10
 * @last modified 2023-01-10
 *
 * @copyright Copyright (c) 2023 Liu Yuanlin Personal.
 *
 */
#ifndef LOG_H
#define LOG_H

#include "ulog.h"

#ifdef _WIN32
// Windows-specific code
#define USING_SYS     1
#define USING_RTT     0
#define USING_USB_CDC 0
#define USING_UART    0
#elif defined(__linux__)
// Linux-specific code
#define USING_SYS 1
#elif defined(__APPLE__)
// macOS-specific code
#define USING_SYS 1
#else
// Other systems
#define USING_SYS     0
#define USING_RTT     1
#define USING_USB_CDC 0
#define USING_UART    0
#endif // _WIN32

#if USING_RTT == 1
#include "SEGGER_RTT.h"
#elif USING_USB_CDC == 1
#include "usbd_cdc_if.h"
#elif USING_SYS == 1
#include <stdio.h>
#endif

#if USING_RTT == 0
//
// Control sequences, based on ANSI.
// Can be used to control color, and clear the screen
//
#define RTT_CTRL_RESET               "\x1B[0m" // Reset to default colors
#define RTT_CTRL_CLEAR               "\x1B[2J" // Clear screen, reposition cursor to top left

#define RTT_CTRL_TEXT_BLACK          "\x1B[2;30m"
#define RTT_CTRL_TEXT_RED            "\x1B[2;31m"
#define RTT_CTRL_TEXT_GREEN          "\x1B[2;32m"
#define RTT_CTRL_TEXT_YELLOW         "\x1B[2;33m"
#define RTT_CTRL_TEXT_BLUE           "\x1B[2;34m"
#define RTT_CTRL_TEXT_MAGENTA        "\x1B[2;35m"
#define RTT_CTRL_TEXT_CYAN           "\x1B[2;36m"
#define RTT_CTRL_TEXT_WHITE          "\x1B[2;37m"

#define RTT_CTRL_TEXT_BRIGHT_BLACK   "\x1B[1;30m"
#define RTT_CTRL_TEXT_BRIGHT_RED     "\x1B[1;31m"
#define RTT_CTRL_TEXT_BRIGHT_GREEN   "\x1B[1;32m"
#define RTT_CTRL_TEXT_BRIGHT_YELLOW  "\x1B[1;33m"
#define RTT_CTRL_TEXT_BRIGHT_BLUE    "\x1B[1;34m"
#define RTT_CTRL_TEXT_BRIGHT_MAGENTA "\x1B[1;35m"
#define RTT_CTRL_TEXT_BRIGHT_CYAN    "\x1B[1;36m"
#define RTT_CTRL_TEXT_BRIGHT_WHITE   "\x1B[1;37m"

#define RTT_CTRL_BG_BLACK            "\x1B[24;40m"
#define RTT_CTRL_BG_RED              "\x1B[24;41m"
#define RTT_CTRL_BG_GREEN            "\x1B[24;42m"
#define RTT_CTRL_BG_YELLOW           "\x1B[24;43m"
#define RTT_CTRL_BG_BLUE             "\x1B[24;44m"
#define RTT_CTRL_BG_MAGENTA          "\x1B[24;45m"
#define RTT_CTRL_BG_CYAN             "\x1B[24;46m"
#define RTT_CTRL_BG_WHITE            "\x1B[24;47m"

#define RTT_CTRL_BG_BRIGHT_BLACK     "\x1B[4;40m"
#define RTT_CTRL_BG_BRIGHT_RED       "\x1B[4;41m"
#define RTT_CTRL_BG_BRIGHT_GREEN     "\x1B[4;42m"
#define RTT_CTRL_BG_BRIGHT_YELLOW    "\x1B[4;43m"
#define RTT_CTRL_BG_BRIGHT_BLUE      "\x1B[4;44m"
#define RTT_CTRL_BG_BRIGHT_MAGENTA   "\x1B[4;45m"
#define RTT_CTRL_BG_BRIGHT_CYAN      "\x1B[4;46m"
#define RTT_CTRL_BG_BRIGHT_WHITE     "\x1B[4;47m"
#endif // USING_RTT

void ulog_init_user();
void Debug_Printf(const void *format, ...);
void Debug_Printf_Terminal(uint8_t terminal_id, const void *format, ...);

#endif //! LOG_H