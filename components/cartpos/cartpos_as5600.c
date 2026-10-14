/*
 * cartpos_as5600 — cartpos 域接口的 AS5600 实现（链接期绑定）
 *
 * 四个域函数逐条转发 as5600 设备 API；init 注入接线 config。
 */
#include "cartpos.h"
#include "cartpos_as5600.h"
#include "as5600.h"

esp_err_t cartpos_as5600_init(const cartpos_as5600_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    as5600_config_t dev = {
        .bus          = cfg->bus,
        .dev_addr     = cfg->dev_addr,
        .scl_speed_hz = cfg->scl_speed_hz,
        .invert_dir   = cfg->invert_dir,
    };
    return as5600_init(&dev);
}

bool cartpos_as5600_invert_dir_default(void)
{
    return AS5600_INVERT_DIR_DEFAULT != 0;
}

int64_t cartpos_position_um(void)   { return as5600_position_um(); }
float   cartpos_velocity_mm_s(void) { return as5600_velocity_mm_s(); }
bool    cartpos_is_ok(void)         { return as5600_is_ok(); }
void    cartpos_set_origin_mm(float new_pos_mm) { as5600_set_origin(new_pos_mm); }
