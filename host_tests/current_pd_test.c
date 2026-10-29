/* PD 调档契约：从禁用准入、独立时钟与电压样本验证，不访问硬件。 */
#include <assert.h>
#include <stdio.h>
#include "current_pd.h"

static void test_admission(void)
{
    current_pd_t s = {0};
    assert(current_pd_start_allowed(&s));
    assert(!current_pd_begin(&s, CURRENT_PD_20V, false, true, 0));
    assert(!current_pd_begin(&s, CURRENT_PD_20V, true, false, 0));
    assert(!current_pd_begin(&s, CURRENT_PD_NONE, true, true, 0));
    assert(s.state == CURRENT_PD_UNCHANGED);
    assert(current_pd_begin(&s, CURRENT_PD_20V, true, true, 0));
    assert(!current_pd_start_allowed(&s));
    assert(!current_pd_begin(&s, CURRENT_PD_9V, true, true, 1));
    assert(s.target == CURRENT_PD_20V && s.started_us == 0);
}

static void test_stable_voltage_and_spacing(void)
{
    current_pd_t s = {0};
    assert(current_pd_begin(&s, CURRENT_PD_20V, true, true, 1000000));
    for (int64_t t = 1000000; t < 4000000; t += 100000)
        assert(current_pd_sample(&s, t, true, 20000) == CURRENT_PD_PENDING);
    assert(current_pd_sample(&s, 4000000, true, 19000) == CURRENT_PD_PENDING);
    for (int i = 1; i < 100; ++i)
        assert(current_pd_sample(&s, 4000000 + i, true, 20000) == CURRENT_PD_PENDING);
    assert(current_pd_sample(&s, 4100000, true, 21000) == CURRENT_PD_PENDING);
    assert(current_pd_sample(&s, 4200000, true, 20000) == CURRENT_PD_PENDING);
    assert(current_pd_sample(&s, 4300000, true, 20000) == CURRENT_PD_PENDING);
    assert(current_pd_sample(&s, 4400000, true, 20000) == CURRENT_PD_SUCCESS);
    assert(s.state == CURRENT_PD_VERIFIED && current_pd_start_allowed(&s));
    assert(current_pd_sample(&s, 9000000, false, -1) == CURRENT_PD_PENDING);
}

static void test_invalid_samples_reset_stability(void)
{
    const struct { bool pd; int mv; } bad[] = {
        {false, 20000}, {true, -1}, {true, 18999}, {true, 21001}, {true, 9000}
    };
    for (unsigned j = 0; j < sizeof(bad) / sizeof(bad[0]); ++j) {
        current_pd_t s = {0};
        assert(current_pd_begin(&s, CURRENT_PD_20V, true, true, 0));
        for (int64_t t = 3000000; t <= 3300000; t += 100000)
            assert(current_pd_sample(&s, t, true, 20000) == CURRENT_PD_PENDING);
        assert(current_pd_sample(&s, 3400000, bad[j].pd, bad[j].mv) == CURRENT_PD_PENDING);
        for (int64_t t = 3500000; t <= 3800000; t += 100000)
            assert(current_pd_sample(&s, t, true, 20000) == CURRENT_PD_PENDING);
        assert(current_pd_sample(&s, 3900000, true, 20000) == CURRENT_PD_SUCCESS);
    }
}

static void test_timeout_cancel_and_recovery(void)
{
    current_pd_t s = {0};
    assert(current_pd_begin(&s, CURRENT_PD_20V, true, true, 0));
    assert(current_pd_sample(&s, 5999999, true, 9000) == CURRENT_PD_PENDING);
    assert(current_pd_sample(&s, 6000000, true, 20000) == CURRENT_PD_TIMEOUT);
    assert(s.state == CURRENT_PD_UNCERTAIN && !current_pd_start_allowed(&s));
    assert(current_pd_begin(&s, CURRENT_PD_9V, true, true, 7000000));
    current_pd_cancel(&s);
    assert(s.state == CURRENT_PD_UNCERTAIN && !current_pd_start_allowed(&s));
    assert(current_pd_sample(&s, 12000000, true, 9000) == CURRENT_PD_PENDING);
    assert(!current_pd_start_allowed(&s));
    assert(current_pd_begin(&s, CURRENT_PD_9V, true, true, 13000000));
    assert(current_pd_sample(&s, 16000000, true, 8549) == CURRENT_PD_PENDING);
    for (int i = 0; i < 4; ++i)
        assert(current_pd_sample(&s, 16100000 + 100000*i, true, 8550) == CURRENT_PD_PENDING);
    assert(current_pd_sample(&s, 16500000, true, 9450) == CURRENT_PD_SUCCESS);
    current_pd_cancel(&s); /* 空闲停止不抹去已经完成的验证。 */
    assert(current_pd_start_allowed(&s));
}

int main(void)
{
    test_admission();
    test_stable_voltage_and_spacing();
    test_invalid_samples_reset_stability();
    test_timeout_cancel_and_recovery();
    puts("current_pd_test: 禁用准入、连续稳定采样、时限、中止和恢复全部通过");
}
