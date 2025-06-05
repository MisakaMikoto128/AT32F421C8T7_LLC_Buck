/**
* @file log_disable.h
* @author Liu Yuanlin (liuyuanlins@outlook.com)
* @brief
* @version 0.1
* @date 2025-04-12
* @last modified 2025-04-12
*
* @copyright Copyright (c) 2025 Liu Yuanlin Personal.
*
*/
#ifndef LOG_CTRL_H
#define LOG_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#undef ULOG_INFO
#define ULOG_INFO(...)

#undef ULOG_DEBUG
#define ULOG_DEBUG(...)

#undef ULOG_WARNING
#define ULOG_WARNING(...)

#undef ULOG_ERROR
#define ULOG_ERROR(...)

#undef ULOG_CRITICAL
#define ULOG_CRITICAL(...)

#ifdef __cplusplus
}
#endif
#endif //!LOG_CTRL_H
