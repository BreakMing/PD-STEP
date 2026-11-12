#include "current_sequence.h"
#include <string.h>

current_command_t current_command_parse(const char *line)
{
    if (line == NULL) return CURRENT_CMD_INVALID;
    if (strcmp(line, "CURRENT INFO") == 0) return CURRENT_CMD_INFO;
    if (strcmp(line, "CURRENT START") == 0) return CURRENT_CMD_START;
    if (strcmp(line, "CURRENT STOP") == 0) return CURRENT_CMD_STOP;
    if (strcmp(line, "CURRENT PD20") == 0) return CURRENT_CMD_PD20;
    if (strcmp(line, "CURRENT PD9") == 0) return CURRENT_CMD_PD9;
    return CURRENT_CMD_INVALID;
}

bool current_sequence_start(current_sequence_t *s, int64_t now_us)
{
    if (s->phase != CURRENT_IDLE) return false;
    s->started_us = now_us;
    s->phase = CURRENT_RUN;
    return true;
}

current_action_t current_sequence_tick(current_sequence_t *s, int64_t now_us)
{
    if (s->phase == CURRENT_IDLE) return CURRENT_ACTION_NONE;
    int64_t elapsed = now_us - s->started_us;
    if (elapsed >= CURRENT_RUN_US + CURRENT_HOLD_US) {
        s->phase = CURRENT_IDLE;
        return CURRENT_ACTION_DISABLE;
    }
    if (s->phase == CURRENT_RUN && elapsed >= CURRENT_RUN_US) {
        s->phase = CURRENT_HOLD;
        return CURRENT_ACTION_HOLD;
    }
    return CURRENT_ACTION_NONE;
}

void current_sequence_stop(current_sequence_t *s)
{
    s->phase = CURRENT_IDLE;
}
