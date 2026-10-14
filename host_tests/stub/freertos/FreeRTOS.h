/* 宿主测试桩：cartaxis.c 的临界区宏（单线程宿主测试退化为空操作） */
#pragma once
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(l) do { (void)(l); } while (0)
#define portEXIT_CRITICAL(l)  do { (void)(l); } while (0)
