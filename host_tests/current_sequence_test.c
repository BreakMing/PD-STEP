/* 固定测量时序的边界、重复命令与中止行为；预期来自 4 秒/2 秒契约。 */
#include <assert.h>
#include <stdio.h>
#include "current_sequence.h"

static void test_commands(void)
{
    assert(current_command_parse("CURRENT INFO") == CURRENT_CMD_INFO);
    assert(current_command_parse("CURRENT START") == CURRENT_CMD_START);
    assert(current_command_parse("CURRENT STOP") == CURRENT_CMD_STOP);
    assert(current_command_parse("CURRENT PD20") == CURRENT_CMD_PD20);
    assert(current_command_parse("CURRENT PD9") == CURRENT_CMD_PD9);
    const char *bad[] = {NULL, "", "START", "ENABLE", "VEL 1", "current start",
                         "CURRENT START extra", "CURRENT START ", "CURRENT", "CURRENT STOPPED",
                         "CURRENT PD28", "CURRENT PD20 extra", "CURRENT PD9 ", "CURRENT PD 20"};
    for (unsigned i = 0; i < sizeof(bad)/sizeof(bad[0]); ++i)
        assert(current_command_parse(bad[i]) == CURRENT_CMD_INVALID);
}

static void test_deadlines(void)
{
    current_sequence_t s = {0};
    assert(s.phase == CURRENT_IDLE);
    assert(current_sequence_tick(&s, 9000000) == CURRENT_ACTION_NONE);
    assert(current_sequence_start(&s, 10000000));
    assert(s.phase == CURRENT_RUN);
    assert(current_sequence_tick(&s, 13999999) == CURRENT_ACTION_NONE);
    assert(!current_sequence_start(&s, 13999999));
    assert(s.started_us == 10000000);
    assert(current_sequence_tick(&s, 14000000) == CURRENT_ACTION_HOLD);
    assert(s.phase == CURRENT_HOLD);
    assert(!current_sequence_start(&s, 14000001));
    assert(current_sequence_tick(&s, 14000001) == CURRENT_ACTION_NONE);
    assert(current_sequence_tick(&s, 15999999) == CURRENT_ACTION_NONE);
    assert(current_sequence_tick(&s, 16000000) == CURRENT_ACTION_DISABLE);
    assert(s.phase == CURRENT_IDLE);
    assert(current_sequence_tick(&s, 16000001) == CURRENT_ACTION_NONE);
    assert(current_sequence_start(&s, 20000000));
    assert(current_sequence_tick(&s, 27000000) == CURRENT_ACTION_DISABLE);
    assert(s.phase == CURRENT_IDLE);
}

static void test_stop_each_phase(void)
{
    for (int phase = CURRENT_IDLE; phase <= CURRENT_HOLD; ++phase) {
        current_sequence_t s = {.phase = phase, .started_us = 1000000};
        current_sequence_stop(&s);
        assert(s.phase == CURRENT_IDLE);
        assert(current_sequence_tick(&s, 5000000) == CURRENT_ACTION_NONE);
        assert(current_sequence_tick(&s, 7000000) == CURRENT_ACTION_NONE);
        assert(current_sequence_start(&s, 8000000));
        assert(current_sequence_tick(&s, 11999999) == CURRENT_ACTION_NONE);
    }
}

int main(void)
{
    test_commands();
    test_deadlines();
    test_stop_each_phase();
    puts("current_sequence_test: 命令、阶段边界、重复开始和各阶段中止全部通过");
    return 0;
}
