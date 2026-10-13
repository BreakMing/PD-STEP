/*
 * telemetry 单元测试（官方 test/ 子目录约定，仅测试工程编入）
 *
 * 覆盖组帧向量与换算。行率（200 Hz±1）、x_um 漂移、手推方向一致性
 * 需从端侧/USB 串口观察，归入最终上板清单。
 */

#include "telemetry.h"
#include "as5600.h"

#include <string.h>

#include "unity.h"

/* 组帧向量（与宿主测试同源） */
TEST_CASE("组帧向量", "[telemetry]")
{
    char buf[64];
    int n = telemetry_format_state(buf, sizeof(buf), 123456, -3141592, -200, 4500);
    TEST_ASSERT_TRUE_MESSAGE(n > 0 && strcmp(buf, "STATE 123456 -3141592 -200 4500") == 0,
                             "基本格式");
    TEST_ASSERT_NULL_MESSAGE(strchr(buf, '\n'), "不含换行");

    telemetry_format_state(buf, sizeof(buf), 0, 0, 0, 0);
    TEST_ASSERT_EQUAL_STRING_MESSAGE("STATE 0 0 0 0", buf, "零值");

    char small[16];
    TEST_ASSERT_TRUE_MESSAGE(telemetry_format_state(small, sizeof(small), 1, 1, 1, 1) < 0,
                             "小缓冲负值");
}

/* counts→µm 换算 */
TEST_CASE("counts→µm 换算", "[telemetry]")
{
    TEST_ASSERT_EQUAL_INT(625, as5600_counts_to_um(64));
    TEST_ASSERT_EQUAL_INT(-625, as5600_counts_to_um(-64));
    TEST_ASSERT_EQUAL_INT(40000, as5600_counts_to_um(4096));
}
