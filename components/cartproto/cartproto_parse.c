/*
 * cartproto_parse — 命令行纯解析（无 ESP 依赖，宿主可测）
 *
 * 词法契约见 cartproto.h。非法行一律返回 false，不产生任何输出。
 */

#include "cartproto.h"

#include <stddef.h>
#include <string.h>
#include <ctype.h>

/* s 的前 len 字符与字面量 lit 完全相等（长度相等且逐位相等） */
static bool streq_n(const char *s, size_t len, const char *lit)
{
    return strlen(lit) == len && strncmp(s, lit, len) == 0;
}

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
        if (streq_n(line, cmd_len, "RESET")) {
            out->cmd = CART_CMD_RESET;
            return true;
        }
        if (streq_n(line, cmd_len, "QUERY")) {
            out->cmd = CART_CMD_QUERY;
            return true;
        }
        if (streq_n(line, cmd_len, "ENABLE")) {
            out->cmd = CART_CMD_ENABLE;
            return true;
        }
        if (streq_n(line, cmd_len, "CONFIG?")) {
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
    if (streq_n(line, cmd_len, "MARK")) {
        size_t rl = len - rest;
        if (streq_n(&line[rest], rl, "LEFT")) {
            out->cmd = CART_CMD_MARK_LEFT;
            return true;
        }
        if (streq_n(&line[rest], rl, "RIGHT")) {
            out->cmd = CART_CMD_MARK_RIGHT;
            return true;
        }
        return false;
    }

    if (streq_n(line, cmd_len, "VEL")) {
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

    if (streq_n(line, cmd_len, "CONFIG")) {
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
