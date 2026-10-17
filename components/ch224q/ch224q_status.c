/*
 * ch224q_status — 0x09 状态位解析实现（契约见 ch224q_status.h；
 * 独立 TU，宿主测试可单独编译）
 */

#include "ch224q_status.h"

#include <stddef.h>

void ch224q_decode_status(uint8_t raw, ch224q_status_t *st)
{
    if (st == NULL) {
        return;
    }
    st->bc12        = (raw & 0x01) != 0;
    st->qc2         = (raw & 0x02) != 0;
    st->qc3         = (raw & 0x04) != 0;
    st->pd          = (raw & 0x08) != 0;
    st->epr_active  = (raw & 0x10) != 0;
    st->epr_present = (raw & 0x20) != 0;
    st->avs_present = (raw & 0x40) != 0;
}
