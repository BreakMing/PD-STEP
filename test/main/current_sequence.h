/* UART0 空载测流的固定参数与时序；不依赖 ESP-IDF。 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

enum {
    CURRENT_RUN_MA = 300,
    CURRENT_HOLD_MA = 150,
    CURRENT_MICROSTEP = 8,
    CURRENT_SPEED_STEPS_S = 500,
    CURRENT_RUN_US = 4000000,
    CURRENT_HOLD_US = 2000000,
};

typedef enum {
    CURRENT_CMD_INVALID,
    CURRENT_CMD_INFO,
    CURRENT_CMD_START,
    CURRENT_CMD_STOP,
    CURRENT_CMD_PD20,
    CURRENT_CMD_PD9,
} current_command_t;

typedef enum {
    CURRENT_IDLE,
    CURRENT_RUN,
    CURRENT_HOLD,
} current_phase_t;

typedef enum {
    CURRENT_ACTION_NONE,
    CURRENT_ACTION_HOLD,
    CURRENT_ACTION_DISABLE,
} current_action_t;

typedef struct {
    current_phase_t phase;
    int64_t started_us;
} current_sequence_t;

/* 输入是不含换行的完整命令；接受的命令枚举集中定义于 current_command_t。 */
current_command_t current_command_parse(const char *line);
/* 仅空闲时接受开始；重复开始不得重置正在运行的截止时间。 */
bool current_sequence_start(current_sequence_t *s, int64_t now_us);
/* 从开始计时：4 秒停脉冲，6 秒关闭使能；跨过两阶段直接关闭。 */
current_action_t current_sequence_tick(current_sequence_t *s, int64_t now_us);
/* STOP 与硬件检查失败都终止序列，之后的 tick 不得重新启动。 */
void current_sequence_stop(current_sequence_t *s);
