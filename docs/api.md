# ESP32APRS HTTP, SSE, and WebSocket API

This document records the device API surface as implemented in
`src/webservice.cpp` and consumed by `data/app.js`. Use the `/api/*` endpoints
as the app-facing contract for new clients. The older HTML routes are listed
separately because they are tightly coupled to the built-in settings UI and may
return HTML instead of JSON.

## Base URL and Auth

- HTTP server: `http://<device-ip>:80`
- Audio/GNSS WebSocket server: `ws://<device-ip>:81` for `/ws` and `/ws_gnss`
- Audio WebSocket is registered on the HTTP server as `ws://<device-ip>/ws_audio`.
  Note: the current browser code constructs `ws://<device-ip>:81/ws_audio`, which
  does not match the route registration in `webService()`.
- Most `/api/*` endpoints currently do not enforce Basic Auth.
- Some legacy settings/file routes call `request->authenticate()` and require
  `config.http_username` / `config.http_password`.
- POST bodies for app-facing endpoints use
  `application/x-www-form-urlencoded`, not JSON.

## Common Response Patterns

Most mutating `/api/*` endpoints return JSON with `ok`.

```json
{"ok": true}
```

Error responses usually use HTTP `400`, `500`, or `503` and include:

```json
{"ok": false, "err": "missing field"}
```

## App-Facing REST Endpoints

### `GET /api/me`

Returns station identity and device counters.

Response fields:

| Field | Type | Notes |
| --- | --- | --- |
| `callsign` | string | `config.aprs_mycall` |
| `ssid` | number | APRS SSID, `0`-`15` |
| `version` | string | firmware `VERSION` |
| `ip` | string | Wi-Fi local IP |
| `free_heap` | number | bytes from `ESP.getFreeHeap()` |
| `uptime` | number | seconds since boot |
| `rx_count` | number | packets published as RX since boot |
| `tx_count` | number | packets accepted for TX since boot |

Example:

```json
{
  "callsign": "M0ABC",
  "ssid": 7,
  "version": "1.7",
  "ip": "192.168.1.42",
  "free_heap": 123456,
  "uptime": 3600,
  "rx_count": 128,
  "tx_count": 4
}
```

### `GET /api/radio`

Returns current RF and modem settings plus live receive diagnostics.

Response fields:

| Field | Type | Notes |
| --- | --- | --- |
| `freq_rx` | number | RX frequency in MHz |
| `freq_tx` | number | TX frequency in MHz |
| `tone_rx` | number | CTCSS RX value, SR110 convention |
| `tone_tx` | number | CTCSS TX value, SR110 convention |
| `sql_level` | number | squelch level |
| `rf_power` | boolean | false = low, true = high |
| `band` | number | configured RF band |
| `rf_en` | boolean | RF module enabled |
| `volume` | number | configured volume |
| `rf_type` | number | configured RF module type |
| `modem` | number | `0` 300 baud, `1` Bell 202 APRS, `2` V.23, `3` 9600 baud |
| `adc_en` | number | ADC task state, `1` sampling, `-1` halted |
| `sql_active` | number | squelch active level config |
| `mvrms` | number | current demod input RMS in mV |
| `sql_pin` | number | current SQL GPIO read, or `-1` |
| `dcd` | number | demodulator carrier-detect counter |

### `POST /api/radio`

Updates one or more radio fields and queues RF reinitialization.

Form fields:

| Field | Required | Notes |
| --- | --- | --- |
| `freq` | no | MHz. Sets RX and TX together. Accepted range: `100.0`-`530.0` |
| `freq_rx` | no | MHz. Accepted range: `100.0`-`530.0` |
| `freq_tx` | no | MHz. Accepted range: `100.0`-`530.0` |
| `tone_rx` | no | integer |
| `tone_tx` | no | integer |
| `sql_level` | no | integer |
| `volume` | no | integer |
| `rf_power` | no | truthy when first char is `1`, `t`, `y`, or `h` |
| `rf_en` | no | truthy when first char is `1`, `t`, or `y` |
| `modem_type` | no | `0`-`3` |

At least one recognized field must be present.

Success response:

```json
{
  "ok": true,
  "freq_rx": 144.8000,
  "freq_tx": 144.8000,
  "tone_rx": 0,
  "tone_tx": 0,
  "sql_level": 0,
  "rf_power": true
}
```

### `POST /api/identity`

Sets the callsign and/or SSID across APRS, message, tracker, and digipeater
roles.

Form fields:

| Field | Required | Notes |
| --- | --- | --- |
| `callsign` | no | uppercased, whitespace stripped, only `A-Z`, `0-9`, and `-` retained |
| `ssid` | no | clamped to `0`-`15` |

At least one field is required.

Success response:

```json
{"ok": true, "callsign": "M0ABC", "ssid": 7}
```

### `GET /api/packets/recent`

Returns the current packet ring, newest first. The browser reverses this list
before rendering so the newest packet appears at the bottom of the chat feed.

Response:

```json
[
  {
    "ts": 1715260000,
    "ch": 1,
    "audio": 0,
    "raw": "M0ABC>APE32L,WIDE1-1:>status"
  }
]
```

Fields:

| Field | Type | Notes |
| --- | --- | --- |
| `ts` | number | Unix timestamp, seconds |
| `ch` | number | packet channel, firmware-specific |
| `audio` | number | raw audio level, not dBV |
| `raw` | string | TNC2 packet line |

### `GET /api/packets/stream`

Server-Sent Events stream for live packets.

Event type: `packet`

Event payload:

```json
{
  "ts": 1715260000,
  "ch": 1,
  "audio": 0,
  "dir": "rx",
  "raw": "M0ABC>APE32L,WIDE1-1:>status"
}
```

Notes:

- `dir` is `rx` or `tx`.
- TX packets sent through `/api/tx/message` and `/api/tx/position` are echoed to
  this stream.
- If no SSE clients are connected, the firmware skips event formatting for
  receive packets to save work.

Browser example:

```js
const es = new EventSource('/api/packets/stream');
es.addEventListener('packet', (event) => {
  const packet = JSON.parse(event.data);
});
```

### `POST /api/tx/message`

Queues an APRS message.

Form fields:

| Field | Required | Notes |
| --- | --- | --- |
| `to` | yes | destination callsign, uppercased by firmware |
| `text` | yes | message body, max stored input length is 199 bytes |

Success response:

```json
{"ok": true}
```

Side effects:

- Increments `tx_count`.
- Publishes a representative TX packet to `/api/packets/stream`.

### `POST /api/tx/position`

Queues an APRS position beacon.

Form fields:

| Field | Required | Notes |
| --- | --- | --- |
| `lat` | yes | decimal latitude |
| `lon` | yes | decimal longitude |
| `comment` | no | max stored input length is 63 bytes |
| `symbol_table` | no | one character, defaults to `/` |
| `symbol_code` | no | one character, defaults to `>` |
| `dest` | no | `rf`, `inet`, or blank. Blank uses tracker config and falls back to RF |

The current implementation rejects `lat=0` and `lon=0` together as invalid.

Success response:

```json
{"ok": true, "raw": "(see /api/packets/stream)"}
```

Failure response when the TX queue cannot accept the packet:

```json
{"ok": false, "raw": "(see /api/packets/stream)"}
```

### `GET /api/webhooks`

Returns configured outbound webhook slots.

Response:

```json
[
  {
    "slot": 0,
    "enabled": true,
    "event_mask": 1,
    "name": "telegram",
    "url": "https://example.invalid/hook",
    "body_template": "{\"text\":\"{src}: {message}\"}",
    "filter_callsign": ""
  }
]
```

### `POST /api/webhooks`

Creates or updates one outbound webhook slot and persists `/default.cfg`.

Form fields:

| Field | Required | Notes |
| --- | --- | --- |
| `slot` | yes | `0` to `WEBHOOK_SLOTS - 1` |
| `enabled` | no | truthy when first char is `1`, `t`, or `y` |
| `event_mask` | no | bit mask described below |
| `name` | no | display name |
| `url` | no | destination URL, supports template variables |
| `body_template` | no | JSON request body template |
| `filter_callsign` | no | prefix filter on source callsign |

Event mask values:

| Value | Meaning |
| --- | --- |
| `0` | unset, any event |
| `1` | any RX |
| `2` | messages addressed to this station |
| `4` | positions only |
| `8` | status packets only |

Masks are bitwise, though the current UI offers selected combinations only.

Success response:

```json
{"ok": true}
```

### `POST /api/webhooks/test`

Queues a synthetic APRS message event for a single webhook slot. The actual
HTTP request is performed asynchronously by the webhook worker task.

Form fields:

| Field | Required | Notes |
| --- | --- | --- |
| `slot` | yes | `0` to `WEBHOOK_SLOTS - 1` |

Queued response:

```json
{"ok": true, "queued": true, "err": "queued"}
```

Synchronous validation failures return HTTP `400`.

## WebSocket Endpoints

### `ws://<device-ip>/ws_audio`

Full-duplex FM voice monitor and PTT audio channel. This is the server-side
route intended for listen/PTT.

Server-to-client on connect:

```json
{"type": "cfg", "codec": "mulaw", "rate": 8000, "tx": 144.8000, "rx": 144.8000}
```

Server-to-client binary frames:

- Codec: G.711 mu-law
- Sample rate: 8 kHz mono
- Nominal chunk: 320 bytes, about 40 ms at 8 kHz

Client-to-server text commands:

| Command | Response |
| --- | --- |
| `ping` | `pong` |
| `tx_start` | `{"type":"tx","ok":1,"state":"on"}` or error reason |
| `tx_stop` | `{"type":"tx","ok":1,"state":"off"}` |
| `set_freq:<tx>,<rx>` | `{"type":"freq","ok":1,"tx":...,"rx":...}` or `{"type":"freq","ok":0,"reason":"invalid"}` |

Possible `tx_start` failure reasons:

| Reason | Meaning |
| --- | --- |
| `rf_disabled` | RF disabled or RF type is none |
| `busy` | another WebSocket client owns TX |
| `modem_busy` | APRS modem is transmitting |

Client-to-server binary frames:

- G.711 mu-law audio samples at 8 kHz mono.
- Samples are accepted only while the same client owns an active TX session.

### `ws://<device-ip>:81/ws`

Legacy TNC2-style packet WebSocket. The firmware registers the socket and
provides `handle_ws(char *Raw, size_t len, uint16_t mVrms)` to send packet text
to connected clients. This endpoint is not currently used by the new browser
app.

Server-to-client:

- Text packet payloads, as produced by `handle_ws`.

### `ws://<device-ip>:81/ws_gnss`

Legacy GNSS WebSocket. The firmware registers the socket and provides
`handle_ws_gnss(char *nmea, size_t size)` to send GNSS/NMEA text to connected
clients. The `/gnss` legacy page uses it.

Server-to-client:

- Text GNSS/NMEA payloads.

## Legacy SSE Endpoints

### `GET /eventHeard`

Legacy Server-Sent Events stream for the settings dashboard's last-heard table.

Event type: `lastHeard`

Payload:

- JSON array encoded as an SSE data string.
- Fields include `time`, `icon`, `callsign`, `path`, `dx`, `packet`, and
  `audio`.

### `GET /eventMsg`

Legacy Server-Sent Events stream for the settings message table.

Event type: `chatMsg`

Payload:

- HTML table rows, not JSON.
- A connected client receives an immediate `chatMsg` event containing the
  current table.

## Static App Assets

| Method | Path | Notes |
| --- | --- | --- |
| `GET` | `/` | mobile chat UI from `/index.html` in LittleFS |
| `GET` | `/app.js` | browser app script |
| `GET` | `/app.css` | browser app stylesheet |
| `GET` | `/manifest.webmanifest` | PWA manifest |
| `GET` | `/settings` | legacy multi-tab settings UI |
| `GET` | `/style.css` | legacy UI stylesheet |
| `GET` | `/jquery-3.7.1.js` | gzipped built-in jQuery |

Static app assets prefer a `*.gz` sibling when present and set
`Content-Encoding: gzip`.

## Legacy Settings and Maintenance Routes

These endpoints are useful for understanding the current firmware surface, but
they should not be treated as the long-term mobile app API without adding JSON
wrappers. Most return HTML pages on `GET`; many `POST` branches return plain
text such as `OK`, `Setup completed successfully`, or an HTML fragment.

| Methods | Path | Purpose |
| --- | --- | --- |
| `GET` | `/symbol` | APRS symbol reference page |
| `GET` | `/logout` | returns `401` to clear Basic Auth |
| `GET, POST` | `/radio` | legacy radio/TNC configuration |
| `GET, POST` | `/vpn` | WireGuard VPN configuration |
| `GET, POST` | `/mqtt` | MQTT configuration, compiled only with `MQTT` |
| `GET, POST` | `/msg` | APRS message configuration and message send settings |
| `GET, POST` | `/mod` | UART, GNSS, Modbus, RF IO, counters, AT command, PPPoS configuration |
| `GET, POST` | `/default` | reset/load default configuration handler |
| `GET, POST` | `/igate` | iGate configuration and filters |
| `GET, POST` | `/digi` | digipeater configuration |
| `GET, POST` | `/tracker` | tracker configuration |
| `GET, POST` | `/wx` | weather configuration |
| `GET, POST` | `/tlm` | telemetry configuration |
| `GET, POST` | `/sensor` | sensor configuration |
| `GET` | `/audio` | legacy voice monitor/PTT page |
| `GET, POST` | `/system` | web auth, APRS path, power, log, display settings |
| `GET, POST` | `/wireless` | Wi-Fi AP/client and Bluetooth settings |
| `GET` | `/tnc2` | legacy test/TNC2 WebSocket page |
| `GET` | `/gnss` | legacy GNSS WebSocket page |
| `GET, POST` | `/about` | about page and OTA upload form |
| `GET` | `/dashboard` | dashboard fragment for settings UI |
| `GET` | `/sidebarInfo` | sidebar/status fragment for settings UI |
| `GET` | `/sysinfo` | system information fragment |
| `GET, POST` | `/storage` | LittleFS file browser; Basic Auth |
| `GET, POST` | `/download` | file download helper |
| `GET, POST` | `/delete` | file deletion helper |
| `GET, POST` | `/format` | LittleFS format helper |
| `POST` | `/upload` | LittleFS file upload |
| `POST` | `/update` | firmware OTA upload |

Legacy POST commit flags currently include:

| Path | Commit flags |
| --- | --- |
| `/radio` | `commitRadio`, `commitTNC` |
| `/vpn` | `commitVPN` |
| `/mqtt` | `commitMQTT` |
| `/msg` | `commitChat`, `commitMSG` |
| `/mod` | `commitGNSS`, `commitUART0`, `commitUART1`, `commitMODBUS`, `commitTNC`, `commitONEWIRE`, `commitRF`, `commitI2C0`, `commitI2C1`, `commitCOUNTER0`, `commitCOUNTER1`, `commitCMD`, `commitPPPoS` when compiled |
| `/system` | `commitWebAuth`, `commitPath`, `commitPWR`, `commitLOG`, `commitDISP` |
| `/igate` | `commitIGATE`, `commitIGATEfilter` |
| `/digi` | `commitDIGI` |
| `/wx` | `commitWX` |
| `/tlm` | `commitTLM` |
| `/sensor` | `commitSENSOR` |
| `/tracker` | `commitTRACKER` |
| `/wireless` | `commitWiFiAP`, `commitWiFiClient`, `commitBluetooth` |

## Outbound Webhook Calls

When webhooks are enabled, incoming RX packets are queued by
`publishRawPacket()` and sent by a low-priority worker task. TX echoes are not
sent to outbound webhooks.

HTTP behavior:

- Method: `POST`
- Content-Type: `application/json`
- User-Agent: `ESP32APRS/1.7`
- Connect timeout: 3 seconds
- Total HTTP timeout: 5 seconds
- Queue depth: 8 events; events are dropped when full

Template variables supported in webhook URL and body:

| Variable | Meaning |
| --- | --- |
| `{src}` | source callsign parsed from TNC2 |
| `{raw}` | raw TNC2 packet |
| `{payload}` | alias of `{raw}` |
| `{addressee}` | APRS message addressee when present |
| `{message}` | APRS message text when present |
| `{ts}` | Unix timestamp, seconds |
| `{channel}` | packet channel |

If `body_template` is empty, the default body is:

```json
{
  "src": "{src}",
  "addressee": "{addressee}",
  "message": "{message}",
  "raw": "{raw}",
  "ts": "{ts}",
  "channel": "{channel}"
}
```

## Browser-App External Calls

The current `data/app.js` browser client also calls third-party services
directly:

| URL | Purpose |
| --- | --- |
| `https://unpkg.com/leaflet@1.9.4/dist/leaflet.css` | lazy-loaded map CSS |
| `https://unpkg.com/leaflet@1.9.4/dist/leaflet.js` | lazy-loaded map JS |
| `https://tile.openstreetmap.org/{z}/{x}/{y}.png` | map tiles |
| `https://www.openstreetmap.org/?mlat=...&mlon=...` | external map links in packet list |

These calls are browser-side only and will fail on an air-gapped LAN unless the
assets are hosted locally or replaced.

## Compatibility Notes for Future Apps

- Prefer `/api/packets/recent` plus `/api/packets/stream` for packet history
  and live updates.
- Parse APRS/TNC2 in the app. The firmware intentionally streams raw TNC2 and
  keeps structured parsing in the browser.
- Treat legacy routes as implementation details until JSON APIs are added for
  the same settings.
- Use form encoding for all current POSTs. JSON request bodies are not parsed by
  the firmware handlers today.
- The firmware escapes JSON responses for strings, but request values are stored
  in fixed-size C buffers. Keep user-facing input limits conservative.
