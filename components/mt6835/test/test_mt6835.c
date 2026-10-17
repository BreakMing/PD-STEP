/*
 * mt6835 纯函数向量测试（官方 test/ 子目录约定，仅测试工程编入）
 *
 * 与宿主测试同源的换算向量。多圈累计（原 unwrap_diff）已上提为
 * magnetic_encoder 组件参数化 tracker（宿主测试
 * magnetic_encoder_tracker_test.c）。硬件用例（静止稳定性、theta
 * 合法域与解释层对拍、A10 手动段）已随设备面移至 magnetic_encoder
 * 测试组件（test_magnetic_encoder.c 的 [mt6835] 段）。
 */

#include "mt6835_urad.h"

#include "unity.h"

/* 纯函数向量（与宿主测试同源） */
TEST_CASE("纯函数向量", "[mt6835]")
{
    TEST_ASSERT_EQUAL_INT(0, mt6835_raw_to_urad(0, 0, false));
    TEST_ASSERT_EQUAL_INT(3141592, mt6835_raw_to_urad(1048576u, 0, false));
    TEST_ASSERT_EQUAL_INT(-3, mt6835_raw_to_urad(2097151u, 0, false));
    TEST_ASSERT_EQUAL_INT(-1570796, mt6835_raw_to_urad(524288u, 0, true));
}
