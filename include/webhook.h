// Outbound HTTP webhook worker.
//
// publishRawPacket() in webservice.cpp enqueues every received APRS packet
// into a small FreeRTOS queue.  A dedicated low-priority task drains the
// queue and POSTs each packet to up to WEBHOOK_SLOTS configured destinations
// (e.g. a Telegram bot, Discord webhook, n8n flow, etc.).
//
// All HTTP work happens on the worker task so the radio RX path can never
// block on TLS or DNS.  If the queue is full, packets are silently dropped.
//
// Templating: webhook URL and body may contain {src}, {raw}, {addressee},
// {message}, {ts}, {channel} placeholders. Variable values are NOT
// JSON-escaped automatically — for JSON bodies the user controls the
// quoting in their template (the firmware substitutes verbatim).

#pragma once
#include <stddef.h>

void webhook_init();
void webhook_enqueue(const char *raw, int channel, int audioLvl);

// Synchronous test fire of a single slot. `errBuf` receives a short status
// string. Returns the HTTP status code (or <0 on transport error / disabled).
int webhook_test(int slotIdx, char *errBuf, size_t errCap);
