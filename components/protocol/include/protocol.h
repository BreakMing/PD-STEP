/*
 * Protocol - Communication Stack public header
 *
 * UART1-based line protocol with real-time characters (? / !).
 *
 * Lifecycle:
 *   protocol_start() — creates comm_task on CPU0 (prio 5, stack >= 4KB).
 *   The task subscribes to the task watchdog and resets it every loop.
 *
 * Callbacks (all optional, default placeholder implementations built-in):
 *   line_handler  — called for each complete line.  Default: placeholder
 *                   dispatcher (empty/comment → ok, else → error:1).
 *                   T4 G-code parser registers to replace the placeholder.
 *   status_cb     — called when '?' real-time char is received.
 *                   Default: sends a placeholder status frame
 *                   <Idle|C:0.00|E:0.00>.
 *                   T9 monitor component registers to supply real values.
 *   stop_cb       — called when '!' real-time char is received.
 *                   Default: logs the event only (does NOT stop motion).
 *                   T6 step_engine registers to implement the actual stop.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Line handler callback type.
 *
 * Called for every complete line received on UART1.
 * The line is null-terminated and does NOT include the trailing \r or \n.
 *
 * @param line  Null-terminated line string (mutable buffer, valid only
 *              for the duration of the call).
 *
 * @note T4 G-code parser will register this callback to replace the
 *       placeholder dispatcher.
 */
typedef void (*protocol_line_handler_t)(const char *line);

/**
 * @brief Status query callback type.
 *
 * Called when the '?' real-time character is received (even in the middle
 * of a line).  The implementation should send a status frame via
 * protocol_send_* (all TX paths share a single mutex, so it's safe to
 * call from here).
 *
 * @note T9 monitor component will register this callback to supply real
 *       position / velocity / TMC flag values.  The default placeholder
 *       sends <Idle|C:0.00|E:0.00>.
 */
typedef void (*protocol_status_cb_t)(void);

/**
 * @brief Stop event callback type.
 *
 * Called when the '!' real-time character is received (even in the middle
 * of a line).  The implementation should stop motion immediately and
 * clear the segment queue.
 *
 * @note T6 step_engine will register this callback to implement the
 *       actual feed hold / clear queue logic.  The default placeholder
 *       just logs the event (motion is not stopped — no step engine yet).
 */
typedef void (*protocol_stop_cb_t)(void);

/**
 * @brief Start the communication task.
 *
 * Creates comm_task pinned to CPU0 with 4KB stack and priority 5.
 * The task subscribes to the task watchdog and feeds it every loop.
 *
 * UART1 is re-initialized with an event queue (bsp installs the driver
 * without a queue; we re-install here with queue so we can use event
 * driven RX).  Pin config and baud rate come from BSP Kconfig.
 *
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t protocol_start(void);

/**
 * @brief Register a line handler callback.
 *
 * If NULL, the built-in placeholder dispatcher is used.
 * Replaces any previously registered handler.
 *
 * @param cb  Line handler callback, or NULL for default.
 */
void protocol_register_line_handler(protocol_line_handler_t cb);

/**
 * @brief Register a status query callback.
 *
 * If NULL, the built-in placeholder status frame is sent on '?'.
 * Replaces any previously registered callback.
 *
 * @param cb  Status callback, or NULL for default.
 */
void protocol_register_status_cb(protocol_status_cb_t cb);

/**
 * @brief Register a stop event callback.
 *
 * If NULL, the built-in placeholder (log only) is used on '!'.
 * Replaces any previously registered callback.
 *
 * @param cb  Stop callback, or NULL for default.
 */
void protocol_register_stop_cb(protocol_stop_cb_t cb);

/**
 * @brief Send "ok\r\n" on UART1.
 *
 * Thread-safe (uses the TX mutex).
 */
void protocol_send_ok(void);

/**
 * @brief Send "error:<code>\r\n" on UART1.
 *
 * Thread-safe (uses the TX mutex).
 *
 * @param code  Error code number (see PLAN §4.6).
 */
void protocol_send_error(int code);

/**
 * @brief Send a raw string on UART1 (TX mutex protected).
 *
 * Lower-level helper used by protocol_send_ok / protocol_send_error
 * and available for status frames / other output.
 * Does NOT append \r\n — caller must include it if needed.
 *
 * @param data  Pointer to data buffer.
 * @param len   Number of bytes to send.
 */
void protocol_send_raw(const char *data, int len);

#ifdef __cplusplus
}
#endif
