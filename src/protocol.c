/* protocol.c */

#include "protocol.h"
#include "kinematics.h"
#include "hal.h"
#include "grbl.h"
#include <string.h>

/* ---- internal types ---- */

struct protocol {
    proto_config_t cfg;

    proto_line_cb_t on_line;
    proto_rt_cb_t   on_rt;
    void           *user;

    /* Current assembling line */
    char     cur[PROTOCOL_LINE_MAX + 1];
    uint16_t cur_len;
    bool     cur_overflow;
    bool     in_paren_comment;

    /* Completed line queue */
    char     q[PROTOCOL_LINE_QUEUE_DEPTH][PROTOCOL_LINE_MAX + 1];
    proto_line_status_t qst[PROTOCOL_LINE_QUEUE_DEPTH];
    uint8_t  q_head;
    uint8_t  q_tail;
    uint8_t  q_count;
};

/* ---- helpers ---- */

static bool is_printable_ascii(uint8_t c) {
    return (c >= 0x20u && c <= 0x7Eu);
}

static char to_upper(char c) {
    if (c >= 'a' && c <= 'z') return (char)(c - ('a' - 'A'));
    return c;
}

static void queue_push(protocol_t *p, const char *line, proto_line_status_t st) {
    if (p->q_count >= PROTOCOL_LINE_QUEUE_DEPTH) {
        /* Drop oldest (or newest). Here: drop newest by ignoring push. */
        return;
    }
    strncpy(p->q[p->q_tail], line, PROTOCOL_LINE_MAX);
    p->q[p->q_tail][PROTOCOL_LINE_MAX] = '\0';
    p->qst[p->q_tail] = st;

    p->q_tail = (uint8_t)((p->q_tail + 1u) % PROTOCOL_LINE_QUEUE_DEPTH);
    p->q_count++;
}

static bool queue_pop(protocol_t *p, char *out, size_t out_cap, proto_line_status_t *st) {
    if (p->q_count == 0u) return false;

    if (out && out_cap) {
        strncpy(out, p->q[p->q_head], out_cap - 1u);
        out[out_cap - 1u] = '\0';
    }
    if (st) *st = p->qst[p->q_head];

    p->q_head = (uint8_t)((p->q_head + 1u) % PROTOCOL_LINE_QUEUE_DEPTH);
    p->q_count--;
    return true;
}

/* Trim leading/trailing whitespace in-place. Returns new length. */
static size_t trim_ws(char *s) {
    size_t n = strlen(s);
    size_t start = 0;
    while (start < n && (s[start] == ' ' || s[start] == '\t')) start++;
    if (start > 0) memmove(s, s + start, n - start + 1);

    n = strlen(s);
    while (n > 0 && (s[n - 1] == ' ' || s[n - 1] == '\t')) {
        s[n - 1] = '\0';
        n--;
    }
    return n;
}

static void emit_line(protocol_t *p) {
    /* Finalize current buffer -> normalized line */
    p->cur[p->cur_len] = '\0';

    proto_line_status_t st = PROTO_LINE_OK;

    if (p->cur_overflow) {
        st = PROTO_LINE_OVERFLOW;
    } else {
        /* Trim */
        trim_ws(p->cur);

        /* Empty after trim? */
        if (p->cur[0] == '\0') {
            st = PROTO_LINE_EMPTY;
        } else if (!p->cfg.allow_dollar_commands && p->cur[0] == '$') {
            /* Treat as "empty/ignored" at protocol layer */
            st = PROTO_LINE_EMPTY;
            p->cur[0] = '\0';
        }
    }

    /* Reset assembly state for next line */
    p->cur_len = 0;
    p->cur_overflow = false;
    p->in_paren_comment = false;

    if (st == PROTO_LINE_EMPTY) {
        return; /* don't enqueue empty/ignored lines */
    }

    /* Deliver line: either immediate callback or queue */
    if (p->on_line) {
        p->on_line(p->cur, st, p->user);
    } else {
        queue_push(p, p->cur, st);
    }
}

static void emit_rt(protocol_t *p, proto_rt_cmd_t cmd) {
    if (p->on_rt) p->on_rt(cmd, p->user);
    /* If no callback, user can wire this to a shared flag elsewhere. */
}

/* ---- public API ---- */

void protocol_init(protocol_t *p,
                   const proto_config_t *cfg,
                   proto_line_cb_t on_line,
                   proto_rt_cb_t on_rt,
                   void *user)
{
    if (!p) return;
    memset(p, 0, sizeof(*p));
    if (cfg) p->cfg = *cfg;

    p->on_line = on_line;
    p->on_rt   = on_rt;
    p->user    = user;
}

void protocol_reset(protocol_t *p) {
    if (!p) return;
    p->cur_len = 0;
    p->cur_overflow = false;
    p->in_paren_comment = false;

    p->q_head = p->q_tail = p->q_count = 0;
    memset(p->q, 0, sizeof(p->q));
}

void protocol_feed_bytes(protocol_t *p, const uint8_t *data, size_t len) {
    if (!p || !data) return;

    for (size_t i = 0; i < len; i++) {
        uint8_t c = data[i];

        /* ---- realtime commands (handled immediately) ---- */
        if (c == 0x18u) { /* Ctrl-X soft reset */
            emit_rt(p, PROTO_RT_RESET);
            protocol_reset(p);
            continue;
        }
        if (c == (uint8_t)'?') { emit_rt(p, PROTO_RT_STATUS_QUERY); continue; }
        if (c == (uint8_t)'!') { emit_rt(p, PROTO_RT_FEED_HOLD);    continue; }
        if (c == (uint8_t)'~') { emit_rt(p, PROTO_RT_CYCLE_START);  continue; }

        /* ---- line termination ---- */
        if (c == '\n') {
            emit_line(p);
            continue;
        }
        if (c == '\r') {
            /* ignore CR, treat LF as terminator */
            continue;
        }

        /* ---- ignore non-printable (except tab/space) ---- */
        if (!(is_printable_ascii(c) || c == '\t')) {
            /* Mark bad char but keep consuming until newline. */
            if (!p->cur_overflow && p->cur_len < PROTOCOL_LINE_MAX) {
                /* We can choose to record an error state by forcing overflow-like behavior. */
            }
            continue;
        }

        char ch = (char)c;

        /* ---- comment stripping ---- */
        if (p->cfg.strip_paren_comments) {
            if (p->in_paren_comment) {
                if (ch == ')') p->in_paren_comment = false;
                continue;
            } else if (ch == '(') {
                p->in_paren_comment = true;
                continue;
            }
        }

        if (p->cfg.strip_semicolon_comments) {
            if (ch == ';') {
                /* ignore rest of line until newline */
                /* easiest: set overflow-like mode and just stop appending */
                p->cur_overflow = p->cur_overflow; /* no-op */
                /* consume bytes until newline without appending */
                /* We can do this by entering a paren-comment-like mode, but keep it simple:
                   just skip appends while leaving cur_len as-is; the loop continues. */
                for (i = i + 1; i < len; i++) {
                    if (data[i] == '\n') { emit_line(p); }
                    else if (data[i] == 0x18u) { emit_rt(p, PROTO_RT_RESET); protocol_reset(p); }
                    else if (data[i] == (uint8_t)'?') { emit_rt(p, PROTO_RT_STATUS_QUERY); }
                    else if (data[i] == (uint8_t)'!') { emit_rt(p, PROTO_RT_FEED_HOLD); }
                    else if (data[i] == (uint8_t)'~') { emit_rt(p, PROTO_RT_CYCLE_START); }
                }
                break;
            }
        }

        if (p->cfg.to_uppercase) ch = to_upper(ch);

        /* ---- append to current line ---- */
        if (p->cur_len < PROTOCOL_LINE_MAX) {
            p->cur[p->cur_len++] = ch;
        } else {
            p->cur_overflow = true; /* keep consuming until newline, then report overflow */
        }
    }
}

bool protocol_pop_line(protocol_t *p, char *out, size_t out_cap, proto_line_status_t *st) {
    if (!p) return false;
    return queue_pop(p, out, out_cap, st);
}

bool protocol_has_line(const protocol_t *p) {
    if (!p) return false;
    return (p->q_count != 0u);
}