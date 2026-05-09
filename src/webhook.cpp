// Outbound webhook worker — see include/webhook.h for design notes.
#include "webhook.h"
#include "main.h"     // Configuration, config
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <string.h>

// Forward decls from main.cpp / webservice.cpp
extern Configuration config;

// Queue depth: small.  If RX bursts faster than the network task can flush,
// packets are dropped rather than queued indefinitely (firmware RAM is tight).
#define WH_QUEUE_DEPTH 8
#define WH_RAW_MAX 260

struct WebhookEvent {
    char raw[WH_RAW_MAX];
    int channel;
    int audio;
    time_t ts;
    int8_t only_slot;   // -1 = all enabled slots; otherwise fire just this one
};

static QueueHandle_t wh_queue = nullptr;
static TaskHandle_t  wh_task  = nullptr;

// ---------- helpers ------------------------------------------------------

// Best-effort parse of TNC2 raw to pull out src callsign, addressee, and
// message text (when the packet is a message packet).  All output buffers
// must be NUL-terminated by the caller.
static void parse_for_template(const char *raw, char *src, size_t srcCap,
                                char *addressee, size_t addrCap,
                                char *message, size_t msgCap)
{
    src[0] = addressee[0] = message[0] = 0;
    const char *gt = strchr(raw, '>');
    if (gt && gt - raw < (long)srcCap) {
        size_t n = gt - raw; if (n >= srcCap) n = srcCap - 1;
        memcpy(src, raw, n); src[n] = 0;
    }
    const char *colon = strchr(raw, ':');
    if (!colon) return;
    const char *info = colon + 1;
    if (info[0] == ':' && strlen(info) >= 11) {
        // Message packet ":<addressee 9 chars>:<text>{msgid}"
        size_t cap = addrCap > 10 ? 10 : addrCap;
        strlcpy(addressee, info + 1, cap);
        // strip trailing spaces
        for (int i = (int)strlen(addressee) - 1; i >= 0 && addressee[i] == ' '; i--)
            addressee[i] = 0;
        // text starts at info + 11 (info[0]=':' + 9 char addr + ':')
        if (info[10] == ':') {
            strlcpy(message, info + 11, msgCap);
            char *lc = strrchr(message, '{');
            if (lc) *lc = 0;
        }
    }
}

// In-place templating: replaces {key} tokens. Unknown keys pass through.
// Variables are substituted verbatim — for JSON bodies the user controls
// the surrounding quoting; we only JSON-escape if `jsonEscape` is true.
static void substitute(char *dst, size_t cap, const char *tmpl,
                       const char *src, const char *addressee, const char *message,
                       const char *raw, int channel, time_t ts, bool jsonEscape)
{
    auto putEsc = [&](const char *v, size_t &j) {
        for (; *v && j + 2 < cap; v++) {
            unsigned char c = (unsigned char)*v;
            if (jsonEscape) {
                if (c == '"' || c == '\\') { dst[j++] = '\\'; dst[j++] = (char)c; }
                else if (c == '\n')        { dst[j++] = '\\'; dst[j++] = 'n'; }
                else if (c < 0x20)         { /* drop */ }
                else                       { dst[j++] = (char)c; }
            } else {
                dst[j++] = (char)c;
            }
        }
    };
    char tsBuf[24], chBuf[8];
    snprintf(tsBuf, sizeof(tsBuf), "%lld", (long long)ts);
    snprintf(chBuf, sizeof(chBuf), "%d", channel);

    size_t j = 0;
    for (size_t i = 0; tmpl[i] && j + 1 < cap; ) {
        if (tmpl[i] == '{') {
            const char *e = strchr(tmpl + i + 1, '}');
            if (e && (e - (tmpl + i + 1)) < 16) {
                char k[16];
                size_t kl = e - (tmpl + i + 1);
                memcpy(k, tmpl + i + 1, kl); k[kl] = 0;
                const char *v = nullptr;
                if      (!strcmp(k, "src"))       v = src;
                else if (!strcmp(k, "raw"))       v = raw;
                else if (!strcmp(k, "payload"))   v = raw;
                else if (!strcmp(k, "addressee")) v = addressee;
                else if (!strcmp(k, "message"))   v = message;
                else if (!strcmp(k, "ts"))        v = tsBuf;
                else if (!strcmp(k, "channel"))   v = chBuf;
                if (v) {
                    putEsc(v, j);
                    i = (e - tmpl) + 1;
                    continue;
                }
            }
        }
        dst[j++] = tmpl[i++];
    }
    dst[j] = 0;
}

// Decide whether this event matches a webhook's filters.
static bool match_filters(const Configuration::WebhookEntry &w,
                           const char *src, const char *addressee, const char *raw)
{
    if (!w.enabled || !w.url[0]) return false;
    if (w.filter_callsign[0] &&
        strncasecmp(src, w.filter_callsign, strlen(w.filter_callsign)) != 0)
        return false;
    if (w.event_mask == 0) return true; // unset = any
    if (w.event_mask & WEBHOOK_EVT_ANY) return true;
    if (w.event_mask & WEBHOOK_EVT_MSG_TO_ME) {
        if (addressee[0] && strcasecmp(addressee, config.aprs_mycall) == 0) return true;
    }
    if (w.event_mask & WEBHOOK_EVT_POSITION) {
        const char *colon = strchr(raw, ':');
        if (colon && (colon[1] == '!' || colon[1] == '=' ||
                      colon[1] == '/' || colon[1] == '@')) return true;
    }
    if (w.event_mask & WEBHOOK_EVT_STATUS) {
        const char *colon = strchr(raw, ':');
        if (colon && colon[1] == '>') return true;
    }
    return false;
}

// Default JSON body when body_template is empty.
static const char *DEFAULT_BODY_TEMPLATE =
    "{\"src\":\"{src}\",\"addressee\":\"{addressee}\","
    "\"message\":\"{message}\",\"raw\":\"{raw}\","
    "\"ts\":{ts},\"channel\":{channel}}";

// Fire one webhook synchronously (called only from the worker task).
static int post_one(const Configuration::WebhookEntry &w, const WebhookEvent &ev,
                    char *errBuf, size_t errCap)
{
    char src[16], addressee[12], message[200];
    parse_for_template(ev.raw, src, sizeof(src),
                                addressee, sizeof(addressee),
                                message, sizeof(message));
    if (!match_filters(w, src, addressee, ev.raw)) return -2;

    char url[256];
    char body[1024];
    substitute(url, sizeof(url), w.url, src, addressee, message,
               ev.raw, ev.channel, ev.ts, /*jsonEscape*/false);
    const char *tmpl = w.body_template[0] ? w.body_template : DEFAULT_BODY_TEMPLATE;
    substitute(body, sizeof(body), tmpl, src, addressee, message,
               ev.raw, ev.channel, ev.ts, /*jsonEscape*/true);

    HTTPClient http;
    http.setConnectTimeout(3000);
    http.setTimeout(5000);
    http.setReuse(false);
    if (!http.begin(url)) {
        if (errBuf) strlcpy(errBuf, "begin() failed", errCap);
        return -1;
    }
    http.addHeader("Content-Type", "application/json");
    http.addHeader("User-Agent", "ESP32APRS/1.7");
    int code = http.POST((uint8_t *)body, strlen(body));
    if (errBuf) snprintf(errBuf, errCap, "%d", code);
    log_d("webhook[%s] POST %s -> %d", w.name, url, code);
    http.end();
    return code;
}

static void wh_task_fn(void *)
{
    WebhookEvent ev;
    for (;;) {
        if (xQueueReceive(wh_queue, &ev, portMAX_DELAY) != pdTRUE) continue;
        if (WiFi.status() != WL_CONNECTED) continue; // no link, drop
        for (int i = 0; i < WEBHOOK_SLOTS; i++) {
            if (ev.only_slot >= 0 && ev.only_slot != i) continue;
            post_one(config.webhooks[i], ev, nullptr, 0);
        }
    }
}

// ---------- public API ---------------------------------------------------

void webhook_init()
{
    if (wh_queue) return;
    wh_queue = xQueueCreate(WH_QUEUE_DEPTH, sizeof(WebhookEvent));
    xTaskCreatePinnedToCore(wh_task_fn, "webhook", 6144, nullptr,
                             tskIDLE_PRIORITY + 1, &wh_task, 0);
}

void webhook_enqueue(const char *raw, int channel, int audioLvl)
{
    if (!wh_queue || !raw || !raw[0]) return;
    // Fast-path: skip if no slot is enabled (saves the queue copy).
    bool anyOn = false;
    for (int i = 0; i < WEBHOOK_SLOTS; i++) {
        if (config.webhooks[i].enabled && config.webhooks[i].url[0]) { anyOn = true; break; }
    }
    if (!anyOn) return;

    WebhookEvent ev = {};
    strlcpy(ev.raw, raw, sizeof(ev.raw));
    ev.channel   = channel;
    ev.audio     = audioLvl;
    ev.ts        = time(NULL);
    ev.only_slot = -1;
    xQueueSend(wh_queue, &ev, 0); // non-blocking; drop if full
}

// Synchronous fire of one slot is unsafe from inside an HTTP handler
// because TLS + DNS can block tens of seconds and starve the
// AsyncWebServer task.  Instead we enqueue a synthetic packet pinned
// to a single slot; the worker task picks it up and fires it.
// Returns 0 on enqueue success, <0 on bad slot or queue full.
int webhook_test(int slotIdx, char *errBuf, size_t errCap)
{
    if (slotIdx < 0 || slotIdx >= WEBHOOK_SLOTS) {
        if (errBuf) strlcpy(errBuf, "bad slot", errCap);
        return -1;
    }
    if (!config.webhooks[slotIdx].enabled || !config.webhooks[slotIdx].url[0]) {
        if (errBuf) strlcpy(errBuf, "slot disabled or empty url", errCap);
        return -3;
    }
    if (!wh_queue) {
        if (errBuf) strlcpy(errBuf, "worker not started", errCap);
        return -4;
    }
    WebhookEvent ev = {};
    snprintf(ev.raw, sizeof(ev.raw),
             "%s>APE32L,WIDE1-1::%-9s:webhook test from ESP32APRS{1",
             (config.aprs_mycall[0] ? config.aprs_mycall : "TEST"),
             (config.aprs_mycall[0] ? config.aprs_mycall : "TEST"));
    ev.ts = time(NULL);
    ev.only_slot = (int8_t)slotIdx;
    if (xQueueSend(wh_queue, &ev, 0) != pdTRUE) {
        if (errBuf) strlcpy(errBuf, "queue full", errCap);
        return -5;
    }
    if (errBuf) strlcpy(errBuf, "queued", errCap);
    return 0;
}
