/**
 * @file communication.h
 * @author Liu Yuanlin (liuyuanlins@outlook.com)
 * @brief
 * @version 0.1
 * @date 2025-06-16
 * @last modified 2025-06-16
 *
 * @copyright Copyright (c) 2025 Liu Yuanlin Personal.
 *
 */
#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
void scope_init();

void communication_init();

void communication_poll();

#ifdef __cplusplus
}
#endif
#endif //! COMMUNICATION_H
