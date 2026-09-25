/*
 * cartproto_parse — 命令行纯解析（无 ESP 依赖，宿主可测）
 *
 * 词法契约见 cartproto.h。非法行一律返回 false，不产生任何输出。
 */

#include "cartproto.h"

#include <stddef.h>
#include <ctype.h>

/* 行尾空白（空格/制表符）的长度 */
static size_t trailing_ws(const char *s, size_t len)
{
    size_t n = 0;
    while (n < len && (s[len - 1 - n] == ' ' || s[len - 1 - n] == '\t')) {
        n++;
    }
    return n;
}

/* 解析纯十进制整数（仅容前导 '-'）：合法填 *out 返回 true */
static bool parse_long(const char *s, size_t len, long *out)
{
    if (len == 0) {
        return false;
    }

    size_t i = 0;
    bool neg = false;
    if (s[0] == '-') {
        neg = true;
        i = 1;
        if (len == 1) {
            return false;
        }
    }

    long v = 0;
    for (; i < len; i++) {
        if (!isdigit((unsigned char)s[i])) {
            return false;
        }
        /* 限幅防溢出：合法域 ±32767 远小于 LONG_MAX，中途超界即非法 */
        if (v > 100000000L) {
            return false;
        }
        v = v * 10 + (s[i] - '0');
    }

    *out = neg ? -v : v;
    return true;
}

/* NAME ∈ [A-Z0-9_]{1,15} */
static bool valid_name(const char *s, size_t len)
{
    if (len < 1 || len > 15) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        char c = s[i];
        if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_')) {
            return false;
        }
    }
    return true;
}

bool cartproto_parse_line(const char *line, cart_req_t *out)
{
    if (line == NULL || out == NULL) {
        return false;
    }

    /* 先剥行尾空白（空格/制表符，protocol.md 行尾仅容空白），
     * 再扫描剩余部分——行内控制字符（含 \r、\n）非法。 */
    size_t len = 0;
    while (line[len] != '\0') {
        len++;
    }
    len -= trailing_ws(line, len);
    for (size_t i = 0; i < len; i++) {
        unsigned char c = (unsigned char)line[i];
        if (c < 0x20 || c == 0x7F) {
            return false;
        }
    }
    if (len == 0) {
        return false;
    }

    /* 命令词 */
    size_t cmd_len = 0;
    while (cmd_len < len && line[cmd_len] != ' ') {
        cmd_len++;
    }

    /* 无参数命令 */
    if (cmd_len == len) {
        if (cmd_len == 5) {
            if (line[0] == 'R' && line[1] == 'E' && line[2] == 'S'
                && line[3] == 'E' && line[4] == 'T') {
                out->cmd = CART_CMD_RESET;
                return true;
            }
            if (line[0] == 'Q' && line[1] == 'U' && line[2] == 'E'
                && line[3] == 'R' && line[4] == 'Y') {
                out->cmd = CART_CMD_QUERY;
                return true;
            }
        }
        if (cmd_len == 6 && line[0] == 'E' && line[1] == 'N'
            && line[2] == 'A' && line[3] == 'B' && line[4] == 'L'
            && line[5] == 'E') {
            out->cmd = CART_CMD_ENABLE;
            return true;
        }
        if (cmd_len == 7 && line[0] == 'C' && line[1] == 'O'
            && line[2] == 'N' && line[3] == 'F' && line[4] == 'I'
            && line[5] == 'G' && line[6] == '?') {
            out->cmd = CART_CMD_CONFIG_DUMP;
            return true;
        }
        return false;
    }

    /* 带参数命令：参数区必须无前导空格（单空格分隔） */
    size_t rest = cmd_len + 1;
    if (rest >= len) {
        return false;   /* 命令词后直接结束（如 "VEL "）——行尾空白已剥 */
    }

    /* MARK <LEFT|RIGHT>：命令词 + 字面方向词（规格 §2） */
    if (cmd_len == 4 && line[0] == 'M' && line[1] == 'A'
        && line[2] == 'R' && line[3] == 'K') {
        size_t rl = len - rest;
        if (rl == 4 && line[rest] == 'L' && line[rest + 1] == 'E'
            && line[rest + 2] == 'F' && line[rest + 3] == 'T') {
            out->cmd = CART_CMD_MARK_LEFT;
            return true;
        }
        if (rl == 5 && line[rest] == 'R' && line[rest + 1] == 'I'
            && line[rest + 2] == 'G' && line[rest + 3] == 'H'
            && line[rest + 4] == 'T') {
            out->cmd = CART_CMD_MARK_RIGHT;
            return true;
        }
        return false;
    }

    if (cmd_len == 3 && line[0] == 'V' && line[1] == 'E' && line[2] == 'L') {
        long v;
        if (!parse_long(&line[rest], len - rest, &v)) {
            return false;
        }
        if (v > 32767 || v < -32767) {
            return false;   /* 防溢出丢弃；正常钳位在 cartfsm */
        }
        out->cmd = CART_CMD_VEL;
        out->v_mm_s = (int32_t)v;
        return true;
    }

    if (cmd_len == 6 && line[0] == 'C' && line[1] == 'O' && line[2] == 'N'
        && line[3] == 'F' && line[4] == 'I' && line[5] == 'G') {
        /* CONFIG NAME value：NAME 与 value 以单空格分隔 */
        const char *args = &line[rest];
        size_t args_len = len - rest;

        size_t name_len = 0;
        while (name_len < args_len && args[name_len] != ' ') {
            name_len++;
        }
        if (name_len == args_len) {
            return false;   /* 只有 NAME 缺值 */
        }
        if (!valid_name(args, name_len)) {
            return false;
        }

        size_t vstart = name_len + 1;
        if (vstart >= args_len) {
            return false;   /* "NAME " 形态（值缺失） */
        }
        long value;
        if (!parse_long(&args[vstart], args_len - vstart, &value)) {
            return false;
        }

        out->cmd = CART_CMD_CONFIG_SET;
        for (size_t i = 0; i < name_len; i++) {
            out->name[i] = args[i];
        }
        out->name[name_len] = '\0';
        out->value = value;
        return true;
    }

    return false;   /* 未知命令 */
}
