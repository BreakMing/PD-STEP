/* 空载测试的 PD 调档状态与验证条件；不依赖 ESP-IDF。 */
#pragma once
#include <stdbool.h>
#include <stdint.h>

enum {
    CURRENT_PD_SETTLE_US = 3000000,
    CURRENT_PD_TIMEOUT_US = 6000000,
    CURRENT_PD_SAMPLE_US = 100000,
    CURRENT_PD_STABLE_SAMPLES = 5,
};

typedef enum {
    CURRENT_PD_NONE = 0,
    CURRENT_PD_9V = 9,
    CURRENT_PD_20V = 20,
} current_pd_target_t;

typedef enum {
    CURRENT_PD_UNCHANGED, /* 启动后尚未通过本入口请求调档。 */
    CURRENT_PD_WAITING,
    CURRENT_PD_VERIFIED,
    CURRENT_PD_UNCERTAIN, /* 写入失败、中止或超时后，必须重新确认档位。 */
} current_pd_state_t;

typedef enum {
    CURRENT_PD_PENDING,
    CURRENT_PD_SUCCESS,
    CURRENT_PD_TIMEOUT,
} current_pd_result_t;

typedef struct {
    current_pd_state_t state;
    current_pd_target_t target;
    int64_t started_us;
    int64_t last_sample_us;
    unsigned stable_samples;
} current_pd_t;

/* 调档写入前调用；拒绝非法档位、忙碌及 EN 不为高，拒绝时不修改状态。 */
bool current_pd_begin(current_pd_t *s, current_pd_target_t target,
                      bool motor_idle, bool en_disabled, int64_t now_us);
/* 等待结束后按最小采样间隔验证 PD 与目标 ±5%；超时优先于成功。 */
current_pd_result_t current_pd_sample(current_pd_t *s, int64_t now_us,
                                     bool pd_valid, int vbus_mv);
/* 不撤销硬件电压请求；只把尚未完成的验证标记为未确认。 */
void current_pd_cancel(current_pd_t *s);
bool current_pd_start_allowed(const current_pd_t *s);
