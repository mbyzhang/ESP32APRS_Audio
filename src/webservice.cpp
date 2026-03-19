/*
 Name:		ESP32APRS_Audio
 Created:	13-10-2023 14:27:23
 Author:	HS5TQA/Atten
 Github:	https://github.com/nakhonthai
 Facebook:	https://www.facebook.com/atten
 Support IS: host:aprs.dprns.com port:14580 or aprs.hs5tqa.ampr.org:14580
 Support IS monitor: http://aprs.dprns.com:14501 or http://aprs.hs5tqa.ampr.org:14501
*/
#include <Arduino.h>
#include "webservice.h"
#include "base64.hpp"
#include "wireguard_vpn.h"
#include <LibAPRSesp.h>
#include <parse_aprs.h>
#include "jquery_min_js.h"
#include <ESPCPUTemp.h>
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include <memory>
#include <strings.h>
#include <ArduinoJson.h>

extern SemaphoreHandle_t psramMutex;
extern bool psramLock(TickType_t timeout = portMAX_DELAY);
extern void psramUnlock();
extern int mVrms;

// Helper function to allocate memory with PSRAM support
char *allocateStringMemory(size_t size)
{
#ifdef BOARD_HAS_PSRAM
	// Try to allocate in PSRAM first
	size*=2; // overallocation to reduce fragmentation
	char *ptr = (char *)ps_calloc(size, sizeof(char));
	if (ptr != NULL)
	{
		return ptr;
	}
	// If PSRAM allocation fails, fall back to regular heap
#endif
	// Regular heap allocation
	return (char *)calloc(size, sizeof(char));
}

// Helper function to format integers to string using allocateStringMemory
char *intToString(int value)
{
	char *str = allocateStringMemory(12); // Enough for a 32-bit integer + null terminator
	if (str != NULL)
	{
		sprintf(str, "%d", value);
	}
	return str;
}

// Helper function to format floats to string using allocateStringMemory
char *floatToString(float value, int decimals)
{
	char *str = allocateStringMemory(20); // Enough for most float values
	if (str != NULL)
	{
		switch (decimals)
		{
		case 0:
			sprintf(str, "%.0f", value);
			break;
		case 1:
			sprintf(str, "%.1f", value);
			break;
		case 2:
			sprintf(str, "%.2f", value);
			break;
		case 3:
			sprintf(str, "%.3f", value);
			break;
		default:
			sprintf(str, "%.2f", value);
			break;
		}
	}
	return str;
}

// Helper function to convert Arduino String to char* using allocateStringMemory
char *StringToCharPtr(const String &str)
{
	size_t len = str.length() + 1; // +1 for null terminator
	char *charPtr = allocateStringMemory(len);
	if (charPtr != NULL)
	{
		strcpy(charPtr, str.c_str());
	}
	return charPtr;
}

// Build a streamed response from an owned heap buffer to avoid duplicating
// large HTML pages into AsyncBasicResponse::String on low-memory targets.
AsyncWebServerResponse *beginOwnedHtmlResponse(AsyncWebServerRequest *request, char *html)
{
	if (html == nullptr)
	{
		return nullptr;
	}

	const size_t htmlLen = strlen(html);
	auto htmlHolder = std::shared_ptr<char>(html, [](char *ptr)
											 { free(ptr); });

	return request->beginResponse(
		"text/html",
		htmlLen,
		[htmlHolder, htmlLen](uint8_t *buffer, size_t maxLen, size_t index) -> size_t
		{
			if (index >= htmlLen)
			{
				return 0;
			}
			size_t chunkLen = htmlLen - index;
			if (chunkLen > maxLen)
			{
				chunkLen = maxLen;
			}
			memcpy(buffer, htmlHolder.get() + index, chunkLen);
			return chunkLen;
		});
}

#ifdef PPPOS
#include <PPP.h>
#endif

#ifdef SH1106
#include <Adafruit_SH1106.h>
#else
#include "Adafruit_SSD1306.h"
#endif // SH1106

#define SCREEN_ADDRESS 0x3C

AsyncWebServer async_server(80);
AsyncWebServer async_websocket(81);
AsyncWebSocket ws("/ws");
AsyncWebSocket ws_gnss("/ws_gnss");
AsyncWebSocket ws_audio("/ws_audio");

#ifdef MQTT
#include <PubSubClient.h>
extern PubSubClient clientMQTT;
#endif

#ifdef PPPOS
extern pppType pppStatus;
#endif

// Create an Event Source on /events
AsyncEventSource lastheard_events("/eventHeard");
AsyncEventSource message_events("/eventMsg");

char *webString;

static constexpr uint16_t AUDIO_MONITOR_RATE = 8000;
static constexpr size_t AUDIO_MONITOR_CHUNK = 320; // 40ms at 8kHz
static uint8_t audioMonitorBuffer[AUDIO_MONITOR_CHUNK];
static size_t audioMonitorBufferLen = 0;
static uint32_t audioMonitorAccumulator = 0;
static constexpr uint32_t AUDIO_TUNE_IDLE_TIMEOUT_MS = 120000; // 2 minutes
static bool audioTuneActive = false;
static float audioTuneAprsFreqRx = 0.0F;
static uint32_t audioTuneLastUserMs = 0;
static bool webPttActive = false;
static uint32_t webPttExpireMs = 0;

static inline float absFloat(float v)
{
	return (v < 0.0F) ? -v : v;
}

static void serviceWebPttTimeout()
{
	if (webPttActive && (int32_t)(millis() - webPttExpireMs) >= 0)
	{
		setPtt(false);
		webPttActive = false;
		webPttExpireMs = 0;
	}
}

static void getRfRangeForType(uint8_t rfType, float &freqMin, float &freqMax)
{
	switch (rfType)
	{
	case RF_SA868_VHF:
		freqMin = 134.0F;
		freqMax = 174.0F;
		break;
	case RF_SR_1WV:
	case RF_SR_2WVS:
		freqMin = 136.0F;
		freqMax = 174.0F;
		break;
	case RF_SA868_350:
		freqMin = 320.0F;
		freqMax = 400.0F;
		break;
	case RF_SR_1W350:
		freqMin = 350.0F;
		freqMax = 390.0F;
		break;
	case RF_SA868_UHF:
	case RF_SR_1WU:
	case RF_SR_2WUS:
		freqMin = 400.0F;
		freqMax = 470.0F;
		break;
	default:
		freqMin = 134.0F;
		freqMax = 500.0F;
		break;
	}
}

static void audioTuneTouch()
{
	audioTuneLastUserMs = millis();
}

static void audioTuneResetToAprs(const char *reason)
{
	if (!audioTuneActive)
	{
		return;
	}

	const float targetFreq = audioTuneAprsFreqRx;
	audioTuneActive = false;
	audioTuneTouch();

	if (absFloat(config.freq_rx - targetFreq) > 0.00005F)
	{
		config.freq_rx = targetFreq;
		RF_MODULE(false);
	}

	log_i("Audio tuner returned to APRS RX %.4f (%s)", targetFreq, reason ? reason : "manual");
}

static inline int16_t clampToInt16(int32_t value)
{
	if (value > 32767)
	{
		return 32767;
	}
	if (value < -32768)
	{
		return -32768;
	}
	return (int16_t)value;
}

static uint8_t linearToMuLaw(int16_t pcm)
{
	static constexpr int16_t MULAW_BIAS = 0x84;
	static constexpr int16_t MULAW_CLIP = 32635;
	uint8_t sign = (pcm < 0) ? 0x80 : 0x00;
	if (pcm < 0)
	{
		pcm = -pcm;
	}
	if (pcm > MULAW_CLIP)
	{
		pcm = MULAW_CLIP;
	}
	pcm += MULAW_BIAS;

	uint8_t exponent = 7;
	for (uint16_t expMask = 0x4000; (pcm & expMask) == 0 && exponent > 0; expMask >>= 1)
	{
		exponent--;
	}
	const uint8_t mantissa = (pcm >> (exponent + 3)) & 0x0F;
	return (uint8_t)(~(sign | (exponent << 4) | mantissa));
}

extern unsigned long waitISRetry;
extern volatile int8_t adcEn;
extern volatile int8_t dacEn;
extern unsigned long upTimeStamp;
extern double VBat;
extern bool VBat_Flag;

#ifdef OLED
#ifdef SH1106
extern Adafruit_SH1106 display;
#else
extern Adafruit_SSD1306 display;
#endif
#endif // OLED

bool defaultSetting = false;

void saveConfig(AsyncWebServerRequest *request)
{
	String html;
	if (saveConfiguration("/default.cfg", config))
	{
		html = "Setup completed successfully";
		request->send(200, "text/html", html); // send to someones browser when asked
	}
	else
	{
		html = "Save config failed.";
		request->send(501, "text/html", html); // Not Implemented
	}
	html.clear();
}

void serviceHandle()
{
	// server.handleClient();
}

void notFound(AsyncWebServerRequest *request)
{
	request->send(404, "text/plain", "Not found");
}

void handle_logout(AsyncWebServerRequest *request)
{
	char *webString = allocateStringMemory(64); // Small buffer for "Log out"
	if (!webString)
	{
		return; // Memory allocation failed
	}
	strcpy(webString, "Log out");
	request->send(200, "text/html", webString);
	free(webString); // Free the allocated memory
}

void setMainPage(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}

	// Using dynamic memory allocation instead of String
	char *webString = allocateStringMemory(12000); // Initial buffer size, adjust as needed
	if (!webString)
	{
		return; // Memory allocation failed
	}

	strcpy(webString, "<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n");
	strcat(webString, "<meta name=\"robots\" content=\"index\" />\n");
	strcat(webString, "<meta name=\"robots\" content=\"follow\" />\n");
	strcat(webString, "<meta name=\"language\" content=\"English\" />\n");
	strcat(webString, "<meta http-equiv=\"Content-Type\" content=\"text/html; charset=utf-8\" />\n");
	strcat(webString, "<meta name=\"GENERATOR\" content=\"configure 20230924\" />\n");
	strcat(webString, "<meta name=\"Author\" content=\"Mr.Somkiat Nakhonthai (HS5TQA)\" />\n");
	strcat(webString, "<meta name=\"Description\" content=\"Web Embedded Configuration\" />\n");
	strcat(webString, "<meta name=\"KeyWords\" content=\"ESP32,ESP32C3,AFSK,APRS\" />\n");
	strcat(webString, "<meta http-equiv=\"Cache-Control\" content=\"no-cache, no-store, must-revalidate\" />\n");
	strcat(webString, "<meta http-equiv=\"pragma\" content=\"no-cache\" />\n");
	strcat(webString, "<link rel=\"shortcut icon\" href=\"http://aprs.dprns.com/favicon.ico\" type=\"image/x-icon\" />\n");
	strcat(webString, "<meta http-equiv=\"Expires\" content=\"0\" />\n");

	char temp_buffer[512];
	if (strlen(config.host_name) > 0)
	{
		snprintf(temp_buffer, sizeof(temp_buffer), "<title>%s</title>\n", config.host_name);
		strcat(webString, temp_buffer);
	}
	else
	{
		strcat(webString, "<title>ESP32APRS_Audio</title>\n");
	}

	strcat(webString, "<link rel=\"stylesheet\" type=\"text/css\" href=\"style.css\" />\n");
	strcat(webString, "<script src=\"/jquery-3.7.1.js\"></script>\n");
	strcat(webString, "<script type=\"text/javascript\">\n");
	strcat(webString, "function selectTab(evt, tabName) {\n");
	strcat(webString, "var i, tabcontent, tablinks;\n");
	strcat(webString, "tablinks = document.getElementsByClassName(\"nav-tabs\");\n");
	strcat(webString, "for (i = 0; i < tablinks.length; i++) {\n");
	strcat(webString, "tablinks[i].className = tablinks[i].className.replace(\" active\", \"\");\n");
	strcat(webString, "}\n");
	strcat(webString, "\n");
	strcat(webString, "//document.getElementById(tabName).style.display = \"block\";\n");
	strcat(webString, "if (tabName == 'DashBoard') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/dashboard\");\n");
	strcat(webString, "} else if (tabName == 'Radio') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/radio\");\n");
	strcat(webString, "} else if (tabName == 'IGATE') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/igate\");\n");
	strcat(webString, "} else if (tabName == 'DIGI') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/digi\");\n");
	strcat(webString, "} else if (tabName == 'TRACKER') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/tracker\");\n");
	strcat(webString, "} else if (tabName == 'WX') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/wx\");\n");
	strcat(webString, "} else if (tabName == 'TLM') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/tlm\");\n");
	strcat(webString, "} else if (tabName == 'SENSOR') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/sensor\");\n");
	strcat(webString, "} else if (tabName == 'Audio') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/audio\");\n");
	strcat(webString, "} else if (tabName == 'VPN') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/vpn\");\n");
#ifdef MQTT
	strcat(webString, "} else if (tabName == 'MQTT') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/mqtt\");\n");
#endif
	strcat(webString, "} else if (tabName == 'MSG') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/msg\");\n");
	strcat(webString, "} else if (tabName == 'WiFi') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/wireless\");\n");
	strcat(webString, "} else if (tabName == 'MOD') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/mod\");\n");
	strcat(webString, "} else if (tabName == 'System') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/system\");\n");
	strcat(webString, "} else if (tabName == 'File') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/storage\");\n");
	strcat(webString, "} else if (tabName == 'About') {\n");
	strcat(webString, "$(\"#contentmain\").load(\"/about\");\n");
	strcat(webString, "}\n");
	strcat(webString, "\n");
	strcat(webString, "if (evt != null) evt.currentTarget.className += \" active\";\n");
	strcat(webString, "}\n");
	strcat(webString, "if (!!window.EventSource) {");
	strcat(webString, "var source = new EventSource('/eventHeard');");

	strcat(webString, "source.addEventListener('open', function(e) {");
	strcat(webString, "console.log(\"Events Connected\");");
	strcat(webString, "}, false);");
	strcat(webString, "source.addEventListener('error', function(e) {");
	strcat(webString, "if (e.target.readyState != EventSource.OPEN) {");
	strcat(webString, "console.log(\"Events Disconnected\");");
	strcat(webString, "}\n}, false);");
	strcat(webString, "source.addEventListener('lastHeard', function(e) {");
	// strcat(webString, "console.log(\"lastHeard\", e.data);");
	strcat(webString, "var lh=document.getElementById(\"aprsTable\");");
	strcat(webString, "if(lh != null) {renderTable(e.data);}");
	strcat(webString, "}, false);\n}");
	strcat(webString, "if (!!window.EventSource) {");
	strcat(webString, "var source = new EventSource('/eventMsg');");

	strcat(webString, "source.addEventListener('open', function(e) {");
	strcat(webString, "console.log(\"Events MSG Connected\");");
	strcat(webString, "}, false);");
	strcat(webString, "source.addEventListener('error', function(e) {");
	strcat(webString, "if (e.target.readyState != EventSource.OPEN) {");
	strcat(webString, "console.log(\"Events MSG Disconnected\");");
	strcat(webString, "}\n}, false);");
	strcat(webString, "source.addEventListener('chatMsg', function(e) {");
	// strcat(webString, "console.log(\"lastHeard\", e.data);");
	strcat(webString, "var lh=document.getElementById(\"chatMsg\");");
	strcat(webString, "if(lh != null) {lh.innerHTML = e.data;}");
	strcat(webString, "}, false);\n}\n");
	//strcat(webString, "</script>\n");

	strcat(webString, "let sortDirection = {};\n");
	strcat(webString, "let currentSortKey = \"time\";\n\n");
	strcat(webString, "function renderTable(raw) {\n");
	// strcat(webString, "const tableBody = document.getElementById(\"aprsTableBody\");\n");
	// //strcat(webString, "const tableBody = document.querySelector(\"#aprsTable tbody\");\n");
	// strcat(webString, "if(tableBody == null) {return;}\n");
	strcat(webString, "var data=JSON.parse(raw);\n");	
	strcat(webString, "lastHeardSort(data);\n");
	strcat(webString, "document.querySelectorAll(\"#aprsTable th[data-sort]\")\n");
	strcat(webString, ".forEach(header => {\n\n");
	strcat(webString, "header.addEventListener(\"click\", () => {\n\n");
	strcat(webString, "const key = header.dataset.sort;\n\n");
	strcat(webString, "sortDirection[key] = !sortDirection[key];\n");
	strcat(webString, "currentSortKey = key;\n\n");	
	strcat(webString, "clearArrows();\n\n");
	strcat(webString, "const arrowSpan = header.querySelector(\".arrow\");\n");
	strcat(webString, "arrowSpan.textContent = sortDirection[key] ? \"▲\" : \"▼\";\n\n");
	strcat(webString, "lastHeardSort(data);\n");
	strcat(webString, "printLastHeard(data);\n");
	strcat(webString, "});\n\n");
	strcat(webString, "});\n\n");
	strcat(webString, "printLastHeard(data);\n");
	strcat(webString, "}\n\n");

	strcat(webString, "function printLastHeard(data) {\n");
	strcat(webString, "const tableBody = document.getElementById(\"aprsTableBody\");\n");
	//strcat(webString, "const tableBody = document.querySelector(\"#aprsTable tbody\");\n");
	strcat(webString, "if(tableBody == null) {return;}\n");
	strcat(webString, "tableBody.innerHTML = \"\";\n");
	strcat(webString, "data.forEach(row => {\n");
	strcat(webString, "const tr = document.createElement(\"tr\");\n");
	strcat(webString, "tr.innerHTML = `\n");
	strcat(webString, "<td>${row.time}</td>\n");
	strcat(webString, "<td><img src=\"http://aprs.nakhonthai.net/symbols/icons/${row.icon}\"></td>\n");
	strcat(webString, "<td>${row.callsign}</td>\n");
	strcat(webString, "<td align=\"left\">${row.path}</td>\n");
	strcat(webString, "<td>${row.dx !== null ? row.dx : \"-\"}</td>\n");
	strcat(webString, "<td>${row.packet}</td>\n");
	strcat(webString, "<td style=\"color:green;\">${row.audio !== '-' ? row.audio + \"dBV\" : \"-\"}</td>\n");
	strcat(webString, "`;\n");
	strcat(webString, "tableBody.appendChild(tr);\n");
	strcat(webString, "});\n");
	strcat(webString, "}\n\n");

	strcat(webString, "function clearArrows() {\n");
	strcat(webString, "document.querySelectorAll(\".arrow\").forEach(a => a.textContent = \"\");\n");
	strcat(webString, "}\n\n");
	strcat(webString, "function lastHeardSort(data) {\nvar key=currentSortKey;\n");
	strcat(webString, "data.sort((a, b) => {\n\n");
	strcat(webString, "let valA = a[key];\n");
	strcat(webString, "let valB = b[key];\n\n");
	strcat(webString, "if (key === \"time\") {\n");
	//strcat(webString, "// Parse time in dd hh:mm:ss format\n");
	strcat(webString, "const [dayTime, timePart] = valA.split(' ');\n");
	strcat(webString, "const [hours, minutes, seconds] = timePart.split(':');\n");
	strcat(webString, "valA = parseInt(dayTime) * 86400 + parseInt(hours) * 3600 + parseInt(minutes) * 60 + parseInt(seconds);\n");
	//strcat(webString, "                \n");
	strcat(webString, "const [dayTimeB, timePartB] = valB.split(' ');\n");
	strcat(webString, "const [hoursB, minutesB, secondsB] = timePartB.split(':');\n");
	strcat(webString, "valB = parseInt(dayTimeB) * 86400 + parseInt(hoursB) * 3600 + parseInt(minutesB) * 60 + parseInt(secondsB);\n");
	strcat(webString, "}\n\n");
	strcat(webString, "if (valA === null) return 1;\n");
	strcat(webString, "if (valB === null) return -1;\n\n");
	strcat(webString, "if (valA < valB) return sortDirection[key] ? -1 : 1;\n");
	strcat(webString, "if (valA > valB) return sortDirection[key] ? 1 : -1;\n");
	strcat(webString, "return 0;\n");
	strcat(webString, "});\n\n");
	strcat(webString, "}\n\n");
	strcat(webString, "</script>\n");
	strcat(webString, "</head>\n");
	//strcat(webString, "\n");
	strcat(webString, "<body onload=\"selectTab(event, 'DashBoard')\">\n");
	strcat(webString, "\n");
	strcat(webString, "<div class=\"container\">\n");
	strcat(webString, "<div class=\"header\">\n");
	// strcat(webString, "<div style=\"font-size: 8px; text-align: right; padding-right: 8px;\">ESP32IGate Firmware V" + String(VERSION) + "</div>\n");
	// strcat(webString, "<div style=\"font-size: 8px; text-align: right; padding-right: 8px;\"><a href=\"/logout\">[LOG OUT]</a></div>\n");
	if (strlen(config.host_name) > 0)
	{
		snprintf(temp_buffer, sizeof(temp_buffer), "<h1>%s</h1>\n", config.host_name);
		strcat(webString, temp_buffer);
	}
	else
	{
		strcat(webString, "<h1>ESP32APRS_Audio</h1>\n");
	}
	strcat(webString, "<div class=\"top-actions\"><a href=\"/logout\">Log Out</a></div>\n");
	strcat(webString, "<div class=\"row\">\n");
	strcat(webString, "<ul class=\"nav nav-tabs\">\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'DashBoard')\">DashBoard</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'Radio')\" id=\"btnRadio\">Radio</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'IGATE')\">IGATE</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'DIGI')\">DIGI</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'TRACKER')\">TRACKER</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'WX')\">WX</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'TLM')\">TLM</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'SENSOR')\">SENSOR</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'Audio')\">Audio</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'VPN')\">VPN</button>\n");
#ifdef MQTT
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'MQTT')\">MQTT</button>\n");
#endif
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'MSG')\">MSG</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'WiFi')\">WiFi</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'MOD')\">MOD</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'System')\">System</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'File')\">File</button>\n");
	strcat(webString, "<button class=\"nav-tabs\" onclick=\"selectTab(event, 'About')\">About</button>\n");
	strcat(webString, "</ul>\n");
	strcat(webString, "</div>\n");
	strcat(webString, "</div>\n");
	strcat(webString, "\n");

	strcat(webString, "<div class=\"contentwide\" id=\"contentmain\">\n");
	strcat(webString, "\n");
	strcat(webString, "</div>\n");
	strcat(webString, "<br />\n");
	strcat(webString, "<div class=\"footer\">\n");
	strcat(webString, "ESP32APRS_Audio Web Configuration<br />Copy right ©2023.\n");
	strcat(webString, "<br />\n");
	strcat(webString, "</div>\n");
	strcat(webString, "</div>\n");
	strcat(webString, "<!-- <script type=\"text/javascript\" src=\"/nice-select.min.js\"></script> -->\n");
	strcat(webString, "<script type=\"text/javascript\">\n");
	strcat(webString, "var selectize = document.querySelectorAll('select')\n");
	strcat(webString, "var options = { searchable: true };\n");
	strcat(webString, "selectize.forEach(function (select) {\n");
	strcat(webString, "if (select.length > 30 && null === select.onchange && !select.name.includes(\"ExtendedId\")) {\n");
	strcat(webString, "select.classList.add(\"small\", \"selectize\");\n");
	strcat(webString, "tabletd = select.closest('td');\n");
	strcat(webString, "tabletd.style.cssText = 'overflow-x:unset';\n");
	strcat(webString, "NiceSelect.bind(select, options);\n");
	strcat(webString, "}\n");
	strcat(webString, "});\n");
	strcat(webString, "</script>\n");
	strcat(webString, "</body>\n");
	strcat(webString, "</html>");


	AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, webString);
	response->addHeader("Sensor", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
	lastHeardTimeout = 0;
	lastHeard_Flag = true;
}

////////////////////////////////////////////////////////////
// handler for web server request: http://IpAddress/      //
////////////////////////////////////////////////////////////

void handle_css(AsyncWebServerRequest *request)
{
	static const char css[] PROGMEM = R"CSS(
:root{
	--bg:#e8f1f6;
	--panel:#ffffff;
	--panel-2:#f7fbfe;
	--line:#b7d3e2;
	--ink:#11202b;
	--ink-soft:#415f72;
	--primary:#0b7fb3;
	--primary-2:#0a5f95;
	--accent:#0da96b;
	--danger:#c23e3e;
}
*{box-sizing:border-box}
body,font{
	margin:0;
	padding:16px 10px 24px;
	color:var(--ink);
	font:14px/1.45 "Trebuchet MS","Verdana","Tahoma",sans-serif;
	background:
		radial-gradient(circle at 12% 0%,rgba(11,127,179,.20) 0,transparent 42%),
		radial-gradient(circle at 90% 5%,rgba(13,169,107,.16) 0,transparent 38%),
		linear-gradient(180deg,#f5fbff 0%,var(--bg) 100%);
}
a,a:link,a:visited{text-decoration:none;color:#0f5e86}
a:hover{text-decoration:underline}
.container{
	width:min(1160px,96vw);
	margin:auto;
	border:1px solid var(--line);
	border-radius:14px;
	overflow:hidden;
	background:var(--panel);
	box-shadow:0 22px 36px rgba(15,35,48,.18);
}
.header{
	padding:14px 16px 10px;
	background:
		linear-gradient(135deg,var(--primary) 0%,var(--primary-2) 72%),
		linear-gradient(45deg,rgba(255,255,255,.12) 0%,rgba(255,255,255,0) 60%);
	color:#ecf8ff;
}
h1{
	margin:4px 0 8px;
	text-align:center;
	font-size:1.55rem;
	letter-spacing:.02em;
	color:#ecf8ff;
}
.top-actions{
	text-align:right;
	margin-bottom:8px;
	font-size:12px;
}
.top-actions a{
	display:inline-block;
	padding:4px 9px;
	border-radius:999px;
	border:1px solid rgba(255,255,255,.45);
	color:#fff !important;
	background:rgba(255,255,255,.10);
}
.top-actions a:hover{
	background:rgba(255,255,255,.2);
	text-decoration:none;
}
.row{padding:0}
.nav{
	margin:0;
	padding:0;
	list-style:none;
}
ul.nav.nav-tabs{
	display:flex;
	flex-wrap:wrap;
	gap:6px;
	padding:0;
	margin:0;
}
.nav-tabs>button{
	border:1px solid rgba(255,255,255,.45);
	border-radius:9px;
	padding:7px 10px;
	background:rgba(255,255,255,.08);
	color:#e5f7ff;
	font-weight:700;
	letter-spacing:.01em;
	cursor:pointer;
	transition:.18s ease;
}
.nav-tabs>button:hover{
	background:rgba(255,255,255,.20);
	transform:translateY(-1px);
}
.nav-tabs>button.active,.nav-tabs>button.active:hover,.nav-tabs>button.active:focus{
	background:var(--accent);
	border-color:var(--accent);
	color:#fff;
	box-shadow:0 6px 14px rgba(13,169,107,.35);
}
.contentwide{
	padding:14px 12px 10px;
	background:linear-gradient(180deg,var(--panel) 0%,var(--panel-2) 100%);
	min-height:420px;
	font-size:14px !important;
}
.contentwide h2{
	margin:0 0 10px;
	color:var(--ink);
	font-size:1.08rem;
}
.footer{
	padding:10px 8px;
	font-size:12px;
	text-align:center;
	color:#d8f0fb;
	background:linear-gradient(135deg,var(--primary-2) 0%,var(--primary) 100%);
}
table{
	width:100%;
	border-collapse:separate;
	border-spacing:0;
	background:#fff;
	border:1px solid #9dc3d9;
	font-size:12px;
	color:var(--ink);
}
table th{
	position:sticky;
	top:0;
	background:linear-gradient(180deg,#0d84bd 0%,#0b6997 100%);
	color:#f7fdff;
	border:1px solid #8fc0da;
	padding:6px 7px;
	font-family:"Lucida Console","Menlo","Monaco","Courier New",monospace;
	text-shadow:none;
}
table td{
	border:1px solid #d0e3ef;
	padding:5px 6px;
	color:#112735;
	font-family:"Lucida Console","Menlo","Monaco","Courier New",monospace;
}
table tr:nth-child(odd){background:#f8fcff}
table tr:nth-child(even){background:#eff7fb}
input,select,textarea,button{
	font:13px/1.35 "Trebuchet MS","Verdana","Tahoma",sans-serif;
}
input[type=text],input[type=password],input[type=number],input[type=search],select,textarea{
	width:auto;
	max-width:100%;
	padding:5px 7px;
	border:1px solid #9ec3d8;
	border-radius:7px;
	background:#fff;
	color:#12232f;
}
input:focus,select:focus,textarea:focus{
	outline:none;
	border-color:#3fa4d3;
	box-shadow:0 0 0 3px rgba(63,164,211,.20);
}
button,.button,input[type=button],input[type=submit]{
	border:1px solid #0f7cb0;
	border-radius:8px;
	padding:6px 11px;
	font-weight:700;
	color:#fff;
	background:linear-gradient(180deg,#1692cf 0%,#0d6f9f 100%);
}
button:hover,.button:hover,input[type=button]:hover,input[type=submit]:hover{
	filter:brightness(1.06);
}
button:disabled,.button:disabled,button[disabled],input[type=submit]:disabled{
	border-color:#9aa8b1;
	color:#d7dce0;
	background:#b3bcc3;
}
fieldset{
	border:1px solid #b4d0df;
	border-radius:9px;
	background:#fafdff;
	padding:8px 10px;
}
legend{
	color:#1a6f98;
	font-weight:700;
	padding:0 6px;
}
#tail{
	height:420px;
	width:100%;
	overflow:auto;
	color:#7cff9b;
	background:#05131a;
	border:1px solid #184359;
	padding:8px;
	font-family:"Lucida Console","Menlo","Monaco","Courier New",monospace;
}
#bar,#prgbar{background:#cfe4f0;border-radius:14px}
#bar{background:linear-gradient(90deg,#0c7fb4 0%,#0ea878 100%);width:0%;height:14px}
.nav-status{
	margin:0;
	padding:6px;
	width:180px;
	font-weight:400;
	min-height:560px;
}
.arrow{margin-left:5px;font-size:12px}
.switch{position:relative;display:inline-block;width:34px;height:16px}
.switch input{opacity:0;width:0;height:0}
.slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background-color:#dc5a5a;transition:.3s}
.slider:before{position:absolute;content:"";height:12px;width:12px;left:2px;bottom:2px;background-color:#fff;transition:.3s}
input:checked+.slider{background-color:#31a962}
input:checked+.slider:before{transform:translateX(16px)}
.slider.round{border-radius:34px}
.slider.round:before{border-radius:50%}
.toggle{position:absolute;margin-left:-9999px;visibility:hidden}
.toggle+label{display:block;position:relative;cursor:pointer;outline:none}
input.toggle-round-flat+label{padding:1px;width:33px;height:18px;background-color:#d4dde2;border-radius:10px;transition:background .4s}
input.toggle-round-flat+label:before,input.toggle-round-flat+label:after{display:block;position:absolute;content:""}
input.toggle-round-flat+label:before{top:1px;left:1px;bottom:1px;right:1px;background-color:#fff;border-radius:10px;transition:background .4s}
input.toggle-round-flat+label:after{top:2px;left:2px;bottom:2px;width:16px;background-color:#c7d2da;border-radius:12px;transition:margin .4s,background .4s}
input.toggle-round-flat:checked+label{background-color:#25a06d}
input.toggle-round-flat:checked+label:after{margin-left:14px;background-color:#25a06d}
.nice-select.small,.nice-select-dropdown li.option{height:24px!important;min-height:24px!important;line-height:24px!important}
.nice-select.small ul li:nth-of-type(2){clear:both}
@media (max-width:960px){
	body{padding:8px}
	.container{width:100%;border-radius:10px}
	.header{padding:12px 10px 8px}
	h1{font-size:1.2rem}
	ul.nav.nav-tabs{gap:5px}
	.nav-tabs>button{padding:6px 8px;font-size:12px}
	.contentwide{padding:9px 7px;min-height:340px}
	.nav-status{width:100%;min-height:0}
	table{font-size:11px}
	table th,table td{padding:4px}
}
)CSS";
	request->send_P(200, "text/css", css);
}

void handle_jquery(AsyncWebServerRequest *request)
{
#if defined(CONFIG_IDF_TARGET_ESP32)
	adcEn = -1;
	dacEn = -1;
	delay(100);
#endif
	AsyncWebServerResponse *response = request->beginResponse_P(200, "application/javascript", (const uint8_t *)jquery_3_7_1_min_js_gz, jquery_3_7_1_min_js_gz_len);
	response->addHeader("Content-Encoding", "gzip");
	response->addHeader("Cache-Control", "no-cache");
	response->setContentLength(jquery_3_7_1_min_js_gz_len);
	request->send(response);
#if defined(CONFIG_IDF_TARGET_ESP32)
	delay(200);
	adcEn = 1;
	dacEn = 0;
#endif
}

void handle_dashboard(AsyncWebServerRequest *request)
{
	char temp_buffer[200];
	// if (!request->authenticate(config.http_username, config.http_password))
	// {
	// 	return request->requestAuthentication();
	// }
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	// Using dynamic memory allocation instead of String
	char *webString = allocateStringMemory(4096); // Initial buffer size, adjust as needed
	if (!webString)
	{
		return; // Memory allocation failed
	}

	strcpy(webString, "<script type=\"text/javascript\">\n");
	strcat(webString, "function reloadSysInfo() {\n");
	strcat(webString, "$(\"#sysInfo\").load(\"/sysinfo\", function () { setTimeout(reloadSysInfo, 60000) });\n");
	strcat(webString, "}\n");
	strcat(webString, "setTimeout(reloadSysInfo(), 100);\n");
	strcat(webString, "function reloadSidebarInfo() {\n");
	strcat(webString, "$(\"#sidebarInfo\").load(\"/sidebarInfo\", function () { setTimeout(reloadSidebarInfo, 10000) });\n");
	strcat(webString, "}\n");
	strcat(webString, "setTimeout(reloadSidebarInfo, 1000);\n");
	strcat(webString, "$(window).trigger('resize');\n");

	strcat(webString, "</script>\n");

	//strcat(webString, "<script>\n");
	// strcat(webString, "let aprsData = [\n");
	// strcat(webString, "  { time: \"21:54:23\", icon: \"91-1.png\", callsign: \"HS5TQA-7\", path: \"RF: WIDE1-1\", dx: 0.0, packet: 2, audio: -19.6 },\n");
	// strcat(webString, "  { time: \"22:13:33\", icon: \"66-1.png\", callsign: \"HS5TQA-3\", path: \"RF: DIRECT\", dx: null, packet: 9, audio: -14.1 }\n");
	// strcat(webString, "];\n\n");
	// strcat(webString, "const tableBody = document.querySelector(\"#aprsTable tbody\");\n");
	// strcat(webString, "let sortDirection = {};\n");
	// strcat(webString, "let currentSortKey = null;\n\n");
	// strcat(webString, "function renderTable(data) {\n");
	// strcat(webString, "    tableBody.innerHTML = \"\";\n");
	// strcat(webString, "    data.forEach(row => {\n");
	// strcat(webString, "        const tr = document.createElement(\"tr\");\n");
	// strcat(webString, "        tr.innerHTML = `\n");
	// strcat(webString, "            <td>${row.time}</td>\n");
	// strcat(webString, "            <td><img src=\\\"http://aprs.dprns.com/symbols/icons/${row.icon}\\\"></td>\n");
	// strcat(webString, "            <td>${row.callsign}</td>\n");
	// strcat(webString, "            <td>${row.path}</td>\n");
	// strcat(webString, "            <td>${row.dx !== null ? row.dx + \\\" km\\\" : \\\"-\\\"}</td>\n");
	// strcat(webString, "            <td>${row.packet}</td>\n");
	// strcat(webString, "            <td style=\\\"color:green;\\\">${row.audio !== null ? row.audio + \\\" dBV\\\" : \\\"-\\\"}</td>\n");
	// strcat(webString, "        `;\n");
	// strcat(webString, "        tableBody.appendChild(tr);\n");
	// strcat(webString, "    });\n");
	// strcat(webString, "}\n\n");
	// strcat(webString, "function clearArrows() {\n");
	// strcat(webString, "    document.querySelectorAll(\\\".arrow\\\").forEach(a => a.textContent = \\\"\\\");\n");
	// strcat(webString, "}\n\n");
	// strcat(webString, "document.querySelectorAll(\\\"#aprsTable th[data-sort]\\\")\n");
	// strcat(webString, ".forEach(header => {\n\n");
	// strcat(webString, "    header.addEventListener(\\\"click\\\", () => {\n\n");
	// strcat(webString, "        const key = header.dataset.sort;\n\n");
	// strcat(webString, "        sortDirection[key] = !sortDirection[key];\n");
	// strcat(webString, "        currentSortKey = key;\n\n");
	// strcat(webString, "        aprsData.sort((a, b) => {\n\n");
	// strcat(webString, "            let valA = a[key];\n");
	// strcat(webString, "            let valB = b[key];\n\n");
	// strcat(webString, "            if (key === \\\"time\\\") {\n");
	// strcat(webString, "                valA = new Date(\\\"1970-01-01T\\\" + valA);\n");
	// strcat(webString, "                valB = new Date(\\\"1970-01-01T\\\" + valB);\n");
	// strcat(webString, "            }\n\n");
	// strcat(webString, "            if (valA === null) return 1;\n");
	// strcat(webString, "            if (valB === null) return -1;\n\n");
	// strcat(webString, "            if (valA < valB) return sortDirection[key] ? -1 : 1;\n");
	// strcat(webString, "            if (valA > valB) return sortDirection[key] ? 1 : -1;\n");
	// strcat(webString, "            return 0;\n");
	// strcat(webString, "        });\n\n");
	// strcat(webString, "        clearArrows();\n\n");
	// strcat(webString, "        const arrowSpan = header.querySelector(\\\".arrow\\\");\n");
	// strcat(webString, "        arrowSpan.textContent = sortDirection[key] ? \\\"▲\\\" : \\\"▼\\\";\n\n");
	// strcat(webString, "        renderTable(aprsData);\n");
	// strcat(webString, "    });\n\n");
	// strcat(webString, "});\n\n");
	// strcat(webString, "renderTable(aprsData);\n");
	// strcat(webString, "</script>\n");


	strcat(webString, "<div id=\"sysInfo\">\n");
	strcat(webString, "</div>\n");

	strcat(webString, "<br />\n");
	strcat(webString, "<div class=\"nav-status\">\n");
	strcat(webString, "<div id=\"sidebarInfo\">\n");
	strcat(webString, "</div>\n");
	strcat(webString, "<br />\n");

	strcat(webString, "<table>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<th colspan=\"2\">Radio Info</th>\n");
	strcat(webString, "</tr>\n");
	if (config.rf_en)
	{
		strcat(webString, "<tr>\n");
		strcat(webString, "<td>Freq.TX</td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%.4f MHz</td>\n", config.freq_tx);
		strcat(webString, temp_buffer);

		strcat(webString, "</tr>\n");
		strcat(webString, "<tr>\n");
		strcat(webString, "<td>Freq.RX</td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%.4f MHz</td>\n", config.freq_rx);
		strcat(webString, temp_buffer);

		strcat(webString, "</tr>\n");
		strcat(webString, "<tr>\n");
		strcat(webString, "<td>TX PWR</td>\n");
		if (config.rf_power)
			strcat(webString, "<td>HIGH</td>\n");
		else
			strcat(webString, "<td>LOW</td>\n");
		strcat(webString, "</tr>\n");
	}
	strcat(webString, "<tr>\n");
	strcat(webString, "<td>MODEM</td>\n");

	
	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%s</td>\n", MODEM_TYPE[config.modem_type]);
	strcat(webString, temp_buffer);

	strcat(webString, "</tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<td>FX.25</td>\n");

	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%s</td>\n", FX25_MODE[config.fx25_mode]);
	strcat(webString, temp_buffer);

	strcat(webString, "</tr>\n");
	strcat(webString, "</table>\n");
	strcat(webString, "\n");
	if (config.igate_en)
	{
		strcat(webString, "<br />\n");
		strcat(webString, "<table>\n");
		strcat(webString, "<tr>\n");
		strcat(webString, "<th colspan=\"2\">APRS-IS SERVER</th>\n");
		strcat(webString, "</tr>\n");
		strcat(webString, "<tr>\n");
		strcat(webString, "<td>HOST</td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%s</td>\n", config.aprs_host);
		strcat(webString, temp_buffer);

		strcat(webString, "</tr>\n");
		strcat(webString, "<tr>\n");
		strcat(webString, "<td>PORT</td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%d</td>\n", config.aprs_port);
		strcat(webString, temp_buffer);

		strcat(webString, "</tr>\n");
		strcat(webString, "</table>\n");
	}
	strcat(webString, "<br />\n");
	strcat(webString, "<table>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<th colspan=\"2\">WiFi</th>\n");
	strcat(webString, "</tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<td>MODE</td>\n");
	const char *strWiFiMode = "OFF";
	if (config.wifi_mode == WIFI_STA_FIX)
	{
		strWiFiMode = "STA";
	}
	else if (config.wifi_mode == WIFI_AP_FIX)
	{
		strWiFiMode = "AP";
	}
	else if (config.wifi_mode == WIFI_AP_STA_FIX)
	{
		strWiFiMode = "AP+STA";
	}

	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%s</td>\n", strWiFiMode);
	strcat(webString, temp_buffer);

	strcat(webString, "</tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<td>SSID</td>\n");

	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%s</td>\n", WiFi.SSID().c_str());
	strcat(webString, temp_buffer);

	strcat(webString, "</tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<td>RSSI</td>\n");
	if (WiFi.isConnected())
	{
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%d dBm</td>\n", WiFi.RSSI());
		strcat(webString, temp_buffer);
	}
	else
		strcat(webString, "<td style=\"background:#606060; color:#b0b0b0;\" aria-disabled=\"true\">Disconnect</td>\n");
	strcat(webString, "</tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<td>STA IP</td>\n");
	IPAddress staIp = WiFi.localIP();
	bool staHasIp = WiFi.isConnected() && (staIp[0] || staIp[1] || staIp[2] || staIp[3]);
	if (staHasIp)
	{
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background:#d8ffd8; color:#034f03;\"><b>%s</b></td>\n", staIp.toString().c_str());
		strcat(webString, temp_buffer);
	}
	else if (config.wifi_mode & WIFI_STA_FIX)
	{
		strcat(webString, "<td style=\"background:#fff6cc; color:#7a5d00;\">Waiting DHCP</td>\n");
	}
	else
	{
		strcat(webString, "<td style=\"background:#606060; color:#b0b0b0;\" aria-disabled=\"true\">STA disabled</td>\n");
	}
	strcat(webString, "</tr>\n");
	strcat(webString, "</table>\n");
	strcat(webString, "<br />\n");
#ifdef BLUETOOTH
	strcat(webString, "<table>\n");
	strcat(webString, "<tr>\n");

	strcat(webString, "<th colspan=\"2\">Bluetooth</th>\n");
	strcat(webString, "</tr>\n");
	strcat(webString, "<td>Master</td>\n");
	if (config.bt_master)
		strcat(webString, "<td style=\"background:#0b0; color:#030; width:50%;\">Enabled</td>\n");
	else
		strcat(webString, "<td style=\"background:#606060; color:#b0b0b0;\" aria-disabled=\"true\">Disabled</td>\n");
	strcat(webString, "</tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<td>NAME</td>\n");

	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%s</td>\n", config.bt_name);
	strcat(webString, temp_buffer);

	strcat(webString, "</tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<td>MODE</td>\n");
	const char *btMode = "";
	if (config.bt_mode == 1)
	{
		btMode = "TNC2";
	}
	else if (config.bt_mode == 2)
	{
		btMode = "KISS";
	}
	else
	{
		btMode = "NONE";
	}

	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%s</td>\n", btMode);
	strcat(webString, temp_buffer);

	strcat(webString, "</tr>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "</table>\n");
#endif
	strcat(webString, "</div>\n");

	strcat(webString, "</div>\n");
	strcat(webString, "\n");
	strcat(webString, "<div class=\"content\">\n");
	strcat(webString, "<div id=\"lastHeard\">\n");
	//String lastHeardString = event_lastHeard(true);
	//strcat(webString, lastHeardString.c_str());
	//lastHeardString.clear();
	strcat(webString, "<table id=\"aprsTable\">\n<thread>\n");
	strcat(webString, "<th colspan=\"7\" style=\"background-color: #070ac2;\">LAST HEARD <a href=\"/tnc2\" target=\"_tnc2\" style=\"color: yellow;font-size:8pt\">[RAW]</a></th>\n");
	strcat(webString, "<tr>\n");
	strcat(webString, "<th data-sort=\"time\" style=\"min-width:10ch\"><span><b>Time (");
	if (config.timeZone >= 0)
		strcat(webString, "+");
	// else
	//	strcat(webString, "-");

	if (config.timeZone == (int)config.timeZone)
	{
		sprintf(temp_buffer, "%d", (int)config.timeZone);
		strcat(webString, temp_buffer);
		strcat(webString, ")</b></span><span class=\"arrow\"></span></th>\n");
	}
	else
	{
		sprintf(temp_buffer, "%.1f", config.timeZone);
		strcat(webString, temp_buffer);
		strcat(webString, ")</b></span><span class=\"arrow\"></span></th>\n");
	}
	strcat(webString, "<th style=\"min-width:16px\">ICON</th>\n");
	strcat(webString, "<th data-sort=\"callsign\" style=\"min-width:10ch\">Callsign<span class=\"arrow\"></span></th>\n");
	strcat(webString, "<th>VIA LAST PATH</th>\n");
	strcat(webString, "<th data-sort=\"dx\" style=\"min-width:5ch\">DX<span class=\"arrow\"></span></th>\n");
	strcat(webString, "<th data-sort=\"packet\" style=\"min-width:5ch\">PACKET<span class=\"arrow\"></span></th>\n");
	strcat(webString, "<th data-sort=\"audio\" style=\"min-width:5ch\">AUDIO<span class=\"arrow\"></span></th>\n");
	strcat(webString, "</tr></thread>\n<tbody id=\"aprsTableBody\"></tbody>\n");
	strcat(webString, "</table>\n");
	strcat(webString, "</div>\n");

	AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, webString);
	response->addHeader("dashboard", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
	lastHeardTimeout = millis() + 500;
	lastHeard_Flag = true;
}

void handle_sidebar(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}

	// Using dynamic memory allocation instead of String
	char *html = allocateStringMemory(4096); // Initial buffer size, adjust as needed
	if (!html)
	{
		return; // Memory allocation failed
	}
	char temp_buffer[64];

	strcpy(html, "<table style=\"background:white;border-collapse: unset;\">\n");
	strcat(html, "<tr>\n");
	strcat(html, "<th colspan=\"2\">Modes Enabled</th>\n");
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	if (config.igate_en)
		strcat(html, "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">IGATE</th>\n");
	else
		strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\">IGATE</th>\n");

	if (config.digi_en)
		strcat(html, "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">DIGI</th>\n");
	else
		strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\">DIGI</th>\n");
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	if (config.wx_en)
		strcat(html, "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">WX</th>\n");
	else
		strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\">WX</th>\n");
	if (config.trk_en)
		strcat(html, "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">TRACKER</th>\n");
	else
		strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\">TRACKER</th>\n");
	strcat(html, "</tr>\n");
	strcat(html, "</table>\n");
	strcat(html, "<br />\n");
	strcat(html, "<table style=\"background:white;border-collapse: unset;\">\n");
	strcat(html, "<tr>\n");
	strcat(html, "<th colspan=\"2\">Network Status</th>\n");
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	if (aprsClient.connected() == true)
		strcat(html, "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">APRS-IS</th>\n");
	else
		strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\" aria-disabled=\"true\">APRS-IS</th>\n");
	if (wireguard_active() == true)
		strcat(html, "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">VPN</th>\n");
	else
		strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\" aria-disabled=\"true\">VPN</th>\n");
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
// strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\" aria-disabled=\"true\">4G LTE</th>\n");
#ifdef PPPOS
	if (PPP.connected())
		strcat(html, "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">PPPoS</th>\n");
	else
#endif
		strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\" aria-disabled=\"true\">PPPoS</th>\n");
#ifdef MQTT
	if (clientMQTT.connected())
		strcat(html, "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">MQTT</th>\n");
	else
		strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\" aria-disabled=\"true\">MQTT</th>\n";);
#endif
	if (config.fx25_mode > 0)
		strcat(html, "<th style=\"background:#0b0; color:#030; width:50%;border-radius: 10px;border: 2px solid white;\">FX.25</th>\n");
	else
		strcat(html, "<th style=\"background:#606060; color:#b0b0b0;border-radius: 10px;border: 2px solid white;\" aria-disabled=\"true\">FX.25</th>\n");
	strcat(html, "</tr>\n");
	strcat(html, "</table>\n");
	strcat(html, "<br />\n");
	strcat(html, "<table>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<th colspan=\"2\">WiFi STA</th>\n");
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td style=\"width: 60px;text-align: right;\">LINK:</td>\n");
	IPAddress staIp = WiFi.localIP();
	bool staEnabled = (config.wifi_mode & WIFI_STA_FIX);
	bool staHasIp = WiFi.isConnected() && (staIp[0] || staIp[1] || staIp[2] || staIp[3]);
	if (staHasIp)
		strcat(html, "<td style=\"background:#0b0; color:#030;\"><b>Connected</b></td>\n");
	else if (staEnabled)
		strcat(html, "<td style=\"background:#fff6cc; color:#7a5d00;\">Connecting...</td>\n");
	else
		strcat(html, "<td style=\"background:#606060; color:#b0b0b0;\" aria-disabled=\"true\">Disabled</td>\n");
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td style=\"width: 60px;text-align: right;\">IP:</td>\n");
	if (staHasIp)
	{
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background:#d8ffd8; color:#034f03;\"><b>%s</b></td>\n", staIp.toString().c_str());
		strcat(html, temp_buffer);
	}
	else
	{
		strcat(html, "<td style=\"background:#ffffff; color:#999999;\">-</td>\n");
	}
	strcat(html, "</tr>\n");
	strcat(html, "</table>\n");
	strcat(html, "<br />\n");
	strcat(html, "<table>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<th colspan=\"2\">STATISTICS</th>\n");
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td style=\"width: 60px;text-align: right;\">RADIO RX:</td>\n");

	// Convert numeric values to strings using temporary buffers
	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%lu</td>\n", status.rxCount);
	strcat(html, temp_buffer);

	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td style=\"width: 60px;text-align: right;\">PACKET RX:</td>\n");
	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%lu</td>\n", status.allCount);
	strcat(html, temp_buffer);

	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td style=\"width: 60px;text-align: right;\">PACKET TX:</td>\n");
	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%lu</td>\n", status.txCount);
	strcat(html, temp_buffer);

	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td style=\"width: 60px;text-align: right;\">RF2INET:</td>\n");
	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%lu</td>\n", status.rf2inet);
	strcat(html, temp_buffer);

	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td style=\"width: 60px;text-align: right;\">INET2RF:</td>\n");
	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%lu</td>\n", status.inet2rf);
	strcat(html, temp_buffer);

	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td style=\"width: 60px;text-align: right;\">DIGI:</td>\n");
	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%lu</td>\n", status.digiCount);
	strcat(html, temp_buffer);

	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td style=\"width: 60px;text-align: right;\">DROP/ERR:</td>\n");
	snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;\">%lu/", status.dropCount);
	strcat(html, temp_buffer);
	snprintf(temp_buffer, sizeof(temp_buffer), "%lu</td>\n", status.errorCount);
	strcat(html, temp_buffer);

	strcat(html, "</tr>\n");
	strcat(html, "</table>\n");
	strcat(html, "<br />\n");
	if (config.gnss_enable)
	{
		strcat(html, "<table>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<th colspan=\"2\">GPS Info <a href=\"/gnss\" target=\"_gnss\" style=\"color: yellow;font-size:8pt\">[View]</a></th>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td>LAT:</td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;text-align: left;\">%f</td>\n", gps.location.lat());
		strcat(html, temp_buffer);

		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td>LON:</td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;text-align: left;\">%f</td>\n", gps.location.lng());
		strcat(html, temp_buffer);

		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td>ALT:</td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;text-align: left;\">%f</td>\n", gps.altitude.meters());
		strcat(html, temp_buffer);

		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td>SAT:</td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"background: #ffffff;text-align: left;\">%d</td>\n", gps.satellites.value());
		strcat(html, temp_buffer);

		strcat(html, "</tr>\n");
		strcat(html, "</table>\n");
	}
	strcat(html, "<script>\n");
	strcat(html, "$(window).trigger('resize');\n");
	strcat(html, "</script>\n");

	AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, html);
	response->addHeader("Sidebar", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
}

void handle_symbol(AsyncWebServerRequest *request)
{
	int i;
	int sel = -1;
	for (i = 0; i < request->args(); i++)
	{
		if (request->argName(i) == "sel")
		{
			if (request->arg(i) != "")
			{
				if (isValidNumber(request->arg(i)))
				{
					sel = request->arg(i).toInt();
				}
			}
		}
	}

	char *web = allocateStringMemory(22000); // Initial buffer size, adjust as needed
	if (web)
	{
		memset(web, 0, 22000);
		strcat(web, "<table border=\"1\" align=\"center\">\n");
		strcat(web, "<tr><th colspan=\"16\">Table '/'</th></tr>\n");
		strcat(web, "<tr>\n");
		char lnk[200];
		for (i = 33; i < 129; i++)
		{
			memset(lnk, 0, sizeof(lnk));
			//<td><img onclick="window.opener.setValue(113,2);" src="http://aprs.dprns.com/symbols/icons/113-2.png"></td>
			if (sel == -1)
				sprintf(lnk, "<td><img onclick=\"window.opener.setValue(%d,1);\" src=\"http://aprs.dprns.com/symbols/icons/%d-1.png\"></td>", i, i);
			else
				sprintf(lnk, "<td><img onclick=\"window.opener.setValue(%d,%d,1);\" src=\"http://aprs.dprns.com/symbols/icons/%d-1.png\"></td>", sel, i, i);
			strcat(web, lnk);

			if (((i % 16) == 0) && (i < 126))
				strcat(web, "</tr>\n<tr>\n");
		}
		strcat(web, "</tr>");
		strcat(web, "</table>\n<br />");
		strcat(web, "<table border=\"1\" align=\"center\">\n");
		strcat(web, "<tr><th colspan=\"16\">Table '\\'</th></tr>\n");
		strcat(web, "<tr>\n");
		for (i = 33; i < 129; i++)
		{
			memset(lnk, 0, sizeof(lnk));
			if (sel == -1)
				sprintf(lnk, "<td><img onclick=\"window.opener.setValue(%d,2);\" src=\"http://aprs.dprns.com/symbols/icons/%d-2.png\"></td>", i, i);
			else
				sprintf(lnk, "<td><img onclick=\"window.opener.setValue(%d,%d,2);\" src=\"http://aprs.dprns.com/symbols/icons/%d-2.png\"></td>", sel, i, i);
			strcat(web, lnk);
			if (((i % 16) == 0) && (i < 126))
				strcat(web, "</tr>\n<tr>\n");
		}
		strcat(web, "</tr>");
		strcat(web, "</table>\n");
		size_t len = strlen(web);
		AsyncWebServerResponse *response = request->beginResponse_P(200, String(F("text/html")), (const uint8_t *)web, len);
		response->addHeader("Symbol", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
		free(web);
	}
}

void handle_sysinfo(AsyncWebServerRequest *request)
{
	// Using dynamic memory allocation instead of String
	char *html = allocateStringMemory(1024); // Initial buffer size, adjust as needed
	if (!html)
	{
		return; // Memory allocation failed
	}

	strcpy(html, "<table style=\"table-layout: fixed;border-collapse: unset;border-radius: 10px;border-color: #ee800a;border-style: ridge;border-spacing: 1px;border-width: 4px;background: #ee800a;\">\n");
	strcat(html, "<tr>\n");
	strcat(html, "<th><span><b>Up Time</b></span></th>\n");
	strcat(html, "<th><span>RAM(KByte)</span></th>\n");
#ifdef BOARD_HAS_PSRAM
	strcat(html, "<th><span>PSRAM(KByte)</span></th>\n");
#endif
	strcat(html, "<th><span>SPIFFS(KByte)</span></th>\n");
	if (VBat_Flag)
		strcat(html, "<th><span>VBat(V)</span></th>\n");
	strcat(html, "<th><span>CPU(Mhz)</span></th>\n");
	strcat(html, "<th><span>CPU.Temp(°C)</span></th>\n");

	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	// time_t tn = time(NULL) - systemUptime;
	//  char* uptime = String(day(tn) - 1, DEC) + "D " + String(hour(tn), DEC) + ":" + String(minute(tn), DEC) + ":" + String(second(tn), DEC);
	// String uptime = String(day(tn) - 1, DEC) + "D " + String(hour(tn), DEC) + ":" + String(minute(tn), DEC);
	char strTime[20];
	convertSecondsToDHMS(strTime, (millis() / 1000) - upTimeStamp);

	char temp_buffer[512];
	snprintf(temp_buffer, sizeof(temp_buffer), "<td><b>%s</b></td>\n", strTime);
	strcat(html, temp_buffer);

	snprintf(temp_buffer, sizeof(temp_buffer), "<td><b>%.1f/%.1f</b></td>\n", (float)ESP.getFreeHeap() / 1000, (float)ESP.getHeapSize() / 1000);
	strcat(html, temp_buffer);

#ifdef BOARD_HAS_PSRAM
	snprintf(temp_buffer, sizeof(temp_buffer), "<td><b>%.1f/%.1f</b></td>\n", (float)ESP.getFreePsram() / 1000, (float)ESP.getPsramSize() / 1000);
	strcat(html, temp_buffer);
#endif

	unsigned long cardTotal = LITTLEFS.totalBytes();
	unsigned long cardUsed = LITTLEFS.usedBytes();
	snprintf(temp_buffer, sizeof(temp_buffer), "<td><b>%.1f/%.1f</b></td>\n", (double)cardUsed / 1024, (double)cardTotal / 1024);
	strcat(html, temp_buffer);

	if (VBat_Flag)
	{
		snprintf(temp_buffer, sizeof(temp_buffer), "<td><b>%.2f</b></td>\n", VBat);
		strcat(html, temp_buffer);
	}

	snprintf(temp_buffer, sizeof(temp_buffer), "<td><b>%d</b></td>\n", ESP.getCpuFreqMHz());
	strcat(html, temp_buffer);

	ESPCPUTemp tempSensor;
	if (tempSensor.begin())
	{
		snprintf(temp_buffer, sizeof(temp_buffer), "<td><b>%.1f</b></td>\n", tempSensor.getTemp());
		strcat(html, temp_buffer);
	}
	else
	{
		strcat(html, "<td><b>N/A</b></td>\n");
	}
	// html += "<td style=\"background: #f00\"><b>" + String(ESP.getCycleCount()) + "</b></td>\n";
	strcat(html, "</tr>\n");
	strcat(html, "</table>\n");

	// request->send(200, "text/html", html); // send to someones browser when asked
	AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, html);
	response->addHeader("Sysinfo", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
}

void event_lastHeard(bool gethtml)
{
	(void)gethtml;
	if (lastheard_events.count() > 0)
	{
		lastheard_events.send("[]", "lastHeard", millis() / 1000, 1000);
	}
}

String event_chatMessage(bool gethtml)
{
	if (gethtml)
	{
		return String("[]");
	}
	if (message_events.count() > 0)
	{
		message_events.send("[]", "chatMsg", time(NULL), 5000);
	}
	return String("[]");
}

void handle_storage(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	adcEn = -1;
	dacEn = -1;
	delay(100);	

	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	const char *dirname = "/";
	char strTime[100];

	unsigned long cardTotal = LITTLEFS.totalBytes();
	unsigned long cardUsed = LITTLEFS.usedBytes();

	if (request->hasArg("delete"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "FILE")
			{
				String path = request->arg(i);
#ifdef DEBUG
				Serial.println("Deleting file: " + path);
#endif
				if (LITTLEFS.remove("/" + path))
				{
					//html = "File deleted";
#ifdef DEBUG
					Serial.println("File deleted");
#endif
				}
				else
				{
					//html = "Delete failed";
#ifdef DEBUG
					Serial.println("Delete failed");
#endif
				}
				break;
			}
		}
	}else if (request->hasArg("download"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "FILE")
			{
				String path = request->arg(i);
				String dataType = "";
				if (path.endsWith(".src"))
					path = path.substring(0, path.lastIndexOf("."));
				else if (path.endsWith(".htm"))
					dataType = "text/html";
				else if (path.endsWith(".csv"))
					dataType = "text/csv";
				else if (path.endsWith(".css"))
					dataType = "text/css";
				else if (path.endsWith(".xml"))
					dataType = "text/xml";
				else if (path.endsWith(".png"))
					dataType = "image/png";
				else if (path.endsWith(".gif"))
					dataType = "image/gif";
				else if (path.endsWith(".jpg"))
					dataType = "image/jpeg";
				else if (path.endsWith(".ico"))
					dataType = "image/x-icon";
				else if (path.endsWith(".svg"))
					dataType = "image/svg+xml";
				else if (path.endsWith(".ico"))
					dataType = "image/x-icon";
				else if (path.endsWith(".js"))
					dataType = "application/javascript";
				else if (path.endsWith(".pdf"))
					dataType = "application/pdf";
				else if (path.endsWith(".zip"))
					dataType = "application/zip";
				else if (path.endsWith(".cfg"))
					dataType = "plain/text";
				else if (path.endsWith(".json"))
					dataType = "application/json";
				else if (path.endsWith(".gz"))
				{
					if (path.startsWith("/gz/htm"))
						dataType = "text/html";
					else if (path.startsWith("/gz/css"))
						dataType = "text/css";
					else if (path.startsWith("/gz/csv"))
						dataType = "text/csv";
					else if (path.startsWith("/gz/xml"))
						dataType = "text/xml";
					else if (path.startsWith("/gz/js"))
						dataType = "application/javascript";
					else if (path.startsWith("/gz/svg"))
						dataType = "image/svg+xml";
					else
						dataType = "application/x-gzip";
				}else{
					dataType = "application/octet-stream";
					path = path.substring(0, path.lastIndexOf("."));
					//html = "File type not support";
					//request->send_P(404, PSTR("text/plain"), PSTR("File type not support"));
					//break;
				}

				if (path != "" && dataType != "")
				{
					String file = "/" + path;
					//request->send(LITTLEFS, file, dataType, true);
					AsyncWebServerResponse *response = request->beginResponse(LITTLEFS, file, dataType, true);
					//response->addHeader("Content-Disposition","attachment");
					request->send(response);
				}
				else
				{
					if (dataType != "")
						request->send_P(404, PSTR("text/plain"), PSTR("ContentType Not Support"));
					else
						request->send_P(404, PSTR("text/plain"), PSTR("File Not found"));
				}
				return;
			}
		}
	}

	// Using dynamic memory allocation instead of String
	char *webString = allocateStringMemory(8192); // Initial buffer size, adjust as needed
	if (!webString)
	{
		return; // Memory allocation failed
	}

		strcat(webString, "<script type=\"text/javascript\">\n"
					  "function sub(obj){"
					  "var fileName = obj.value.split('\\\\');"
					  "document.getElementById('file-input').innerHTML = '   '+ fileName[fileName.length-1];"
					  "}\n"
					  //"var form = document.getElementById('upload_form');"
					  "$('form').submit(function(e){"
					  "e.preventDefault();"
					  "var data = new FormData(e.currentTarget);\n"
					  
					  "if(e.currentTarget.id === 'upload_form'){ document.getElementById('upload_sumbit').disabled = true;"
					  "var formUp = $('#upload_form')[0];"
					  "var dataUp = new FormData(formUp);"					  
					  //"document.getElementById('upload_sumbit').disabled = true;"
					  "$.ajax({"
					  "url: '/upload',"
					  "type: 'POST',"
					  "data: dataUp,"
					  "contentType: false,"
					  "processData:false,"
					  "xhr: function() {"
					  "var xhr = new window.XMLHttpRequest();"
					  "xhr.upload.addEventListener('progress', function(evt) {"
					  "if (evt.lengthComputable) {"
					  "var per = evt.loaded / evt.total;"
					  "$('#prg').html(Math.round(per*100) + '%');"
					  "$('#bar').css('width',Math.round(per*100) + '%');"
					  "}"
					  "}, false);"
					  "return xhr;"
					  "},"
					  "success:function(d, s) {"
					  "alert('Upload Success');"
					  "$(\"#contentmain\").load(\"/storage\");\n"
					  "},"
					  "error: function (a, b, c) {"
					  "}"
					  "});"
					  //"});"
					  "}else{"					  
					  "$.ajax({"
					  "url: '/storage',"
					  "type: 'POST',"
					  "data: data,"
					  "contentType: false,"
					  "processData:false,"					  
					  "success:function(d, s) {"
					  //"alert('Upload Success');"
					  "if(e.currentTarget.id===\"formDelete\") $(\"#contentmain\").load(\"/storage\");\n"
					  "},"
					  "error: function (a, b, c) {"
					  "}"
					  "});"
					  //"});"
					  "}"
					  "});"
					  "</script>");

	strcat(webString, "<div style=\"font-size: 8pt;text-align:left;\">");
	strcat(webString, "<b>Total space: </b>");

	char temp_buffer[512];
	if (cardTotal > 1000000)
	{
		snprintf(temp_buffer, sizeof(temp_buffer), "%.2f MByte ,", (double)cardTotal / 1048576);
		strcat(webString, temp_buffer);
	}
	else
	{
		snprintf(temp_buffer, sizeof(temp_buffer), "%.2f KByte ,", (double)cardTotal / 1024);
		strcat(webString, temp_buffer);
	}

	strcat(webString, "<b>Used space: </b>");
	snprintf(temp_buffer, sizeof(temp_buffer), "%.2f KByte", (double)cardUsed / 1024);
	strcat(webString, temp_buffer);

	snprintf(temp_buffer, sizeof(temp_buffer), "</br>Listing directory: </b>%s</div>\n", dirname);
	strcat(webString, temp_buffer);

	File root = LITTLEFS.open(dirname);
	if (!root)
	{
		strcat(webString, "Failed to open directory\n");
		// return;
	}
	if (!root.isDirectory())
	{
		strcat(webString, "Not a directory");
		// return;
	}

	File file = root.openNextFile();
	strcat(webString, "<table border=\"1\"><tr align=\"center\" bgcolor=\"#03DDFC\"><th width=\"100\"><b>DIRECTORY</b></th><th><b>FILE NAME</b></th><th width=\"100\"><b>SIZE(Byte)</b></th><th width=\"170\"><b>DATE TIME</b></th><th width=\"50\"><b>DELETE</b></th><th width=\"100\"><b>DOWNLOAD</b></th></tr>");
	while (file)
	{
		if (file.isDirectory())
		{
			// webString += "<tr><td>DIR : ");
			snprintf(temp_buffer, sizeof(temp_buffer), "<tr><td>%s</td>", file.name());
			strcat(webString, temp_buffer);
			time_t t = file.getLastWrite();
			struct tm *tmstruct = localtime(&t);
			sprintf(strTime, "<td></td><td></td><td align=\"right\">%d-%02d-%02d %02d:%02d:%02d</td>", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);
			strcat(webString, strTime);
			// if (levels) {
			//	listDir(fs, file.name(), levels - 1);
			// }
			strcat(webString, "<td></td></tr>\n");
		}
		else
		{
			/*Serial.print("  FILE: ");
			Serial.print(file.name());*/
			// char *fName = String(file.name()).substring(1).c_str(); // Not needed for full path
			const char *fName = file.name();
			snprintf(temp_buffer, sizeof(temp_buffer), "<tr><td>/</td><td align=\"left\"><a href=\"/download?FILE=%s\" target=\"_blank\">%s</a></td>", fName, fName);
			strcat(webString, temp_buffer);
			// Serial.print("  SIZE: ");
			snprintf(temp_buffer, sizeof(temp_buffer), "<td align=\"right\">%d</td>", file.size());
			strcat(webString, temp_buffer);
			time_t t = file.getLastWrite();
			struct tm *tmstruct = localtime(&t);
			sprintf(strTime, "<td align=\"center\">%d-%02d-%02d %02d:%02d:%02d</td>", (tmstruct->tm_year) + 1900, (tmstruct->tm_mon) + 1, tmstruct->tm_mday, tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);
			strcat(webString, strTime);
			snprintf(temp_buffer, sizeof(temp_buffer), "<td align=\"center\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formDelete\" method=\"post\"><input name=\"delete\" type=\"hidden\" /><input name=\"FILE\" type=\"hidden\" value=\"%s\" /><button name=\"commit\" id=\"btnDelete\" type=\"submit\" style=\"background-color:red;color:white\">X</button></form></td>\n", fName);
			strcat(webString, temp_buffer);
			snprintf(temp_buffer, sizeof(temp_buffer), "<td align=\"center\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formDownload\" method=\"post\"><input name=\"download\" type=\"hidden\" /><input name=\"FILE\" type=\"hidden\" value=\"%s\" /><button name=\"commit\" id=\"btnDownload\" type=\"submit\" style=\"background-color:green;color:white\">DOWNLOAD</button></form></td></tr>\n", fName);
			strcat(webString, temp_buffer);
		}
		file = root.openNextFile();
	}
	strcat(webString, "</table>\n");
	// strcat(webString, "<form accept-charset=\"UTF-8\" action=\"/format\" class=\"form-horizontal\" id=\"format_form\" method=\"post\">\n");
	// strcat(webString, "<br><div><button class=\"button\" type='submit' id='format_form_sumbit'  name=\"commit\"> FORMAT </button></div>\n");
	// strcat(webString, "</form><br/>\n");

	// UPLOAD CONFIGURATION FILE
	strcat(webString, "<br><form accept-charset=\"UTF-8\" action=\"#\" "
					  "class=\"form-horizontal\" id=\"upload_form\" method=\"post\" "
					  "enctype=\"multipart/form-data\">\n");

	strcat(webString, "<table border=\"1\" style=\"margin-top:10px;width:100%;\">\n");
	strcat(webString, "<tr align=\"center\" bgcolor=\"#03DDFC\">"
					  "<th colspan=\"3\"><b>UPLOAD FILE</b></th>"
					  "</tr>\n");

	strcat(webString, "<tr align=\"center\">"
					  "<td width=\"60\" align=\"right\"><b>File:</b></td>"
					  "<td align=\"left\"><input type=\"file\" name=\"data\" required></td>"
					  "<td width=\"120\"><button class=\"button\" type='submit' id='upload_sumbit'>UPLOAD</button></td>"
					  "</tr>\n");

	strcat(webString, "</table>\n");
	strcat(webString, "</form><br/>\n");

	strcat(webString, "</body>\n</html>\n");

	AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, webString);
	response->addHeader("Sensor", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
	adcEn = 1;
	dacEn = 0;
}

void handle_download(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	String dataType = "";
	String path = "";

	if (request->args() > 0)
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "FILE")
			{
				path = request->arg(i);
				break;
			}
		}
	}

	if (path.endsWith(".src"))
		path = path.substring(0, path.lastIndexOf("."));
	else if (path.endsWith(".htm"))
		dataType = "text/html";
	else if (path.endsWith(".csv"))
		dataType = "text/csv";
	else if (path.endsWith(".css"))
		dataType = "text/css";
	else if (path.endsWith(".xml"))
		dataType = "text/xml";
	else if (path.endsWith(".png"))
		dataType = "image/png";
	else if (path.endsWith(".gif"))
		dataType = "image/gif";
	else if (path.endsWith(".jpg"))
		dataType = "image/jpeg";
	else if (path.endsWith(".ico"))
		dataType = "image/x-icon";
	else if (path.endsWith(".svg"))
		dataType = "image/svg+xml";
	else if (path.endsWith(".ico"))
		dataType = "image/x-icon";
	else if (path.endsWith(".js"))
		dataType = "application/javascript";
	else if (path.endsWith(".pdf"))
		dataType = "application/pdf";
	else if (path.endsWith(".zip"))
		dataType = "application/zip";
	else if (path.endsWith(".cfg"))
		dataType = "text/html";
	else if (path.endsWith(".json"))
		dataType = "application/json";
	else if (path.endsWith(".gz"))
	{
		if (path.startsWith("/gz/htm"))
			dataType = "text/html";
		else if (path.startsWith("/gz/css"))
			dataType = "text/css";
		else if (path.startsWith("/gz/csv"))
			dataType = "text/csv";
		else if (path.startsWith("/gz/xml"))
			dataType = "text/xml";
		else if (path.startsWith("/gz/js"))
			dataType = "application/javascript";
		else if (path.startsWith("/gz/svg"))
			dataType = "image/svg+xml";
		else
			dataType = "application/x-gzip";
	}

	if (path != "" && dataType != "")
	{
		String file = "/" + path;
		request->send(LITTLEFS, file, dataType, true);
		// AsyncWebServerResponse *response = request->beginResponse(LITTLEFS, file, dataType, true);
		// response->addHeader("Content-Disposition","attachment");
		// request->send(response);
	}
	else
	{
		if (dataType != "")
			request->send_P(404, PSTR("text/plain"), PSTR("ContentType Not Support"));
		else
			request->send_P(404, PSTR("text/plain"), PSTR("File Not found"));
	}
}

void handle_delete(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	String html = "FAIL";
	String dataType = "text/plain";
	String path;
	if (request->args() > 0)
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "FILE")
			{
				path = request->arg(i);
#ifdef DEBUG
				Serial.println("Deleting file: " + path);
#endif
				if (LITTLEFS.remove("/" + path))
				{
					html = "File deleted";
#ifdef DEBUG
					Serial.println("File deleted");
#endif
				}
				else
				{
					html = "Delete failed";
#ifdef DEBUG
					Serial.println("Delete failed");
#endif
				}
				break;
			}
		}
	}
	request->send(200, "text/html", html); // send to someones browser when asked
}

void handle_format(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	String html = "FAIL";
	if (request->args() > 0)
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "commit")
			{
				if (request->arg(i) == "FORMAT")
				{
					LITTLEFS.format();
					html = "OK";
					break;
				}
			}
		}
	}

	request->send(200, "text/html", html); // send to someones browser when asked
}

void handle_radio(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	// bool noiseEn=false;
	bool radioEnable = false;
	if (request->hasArg("commitRadio"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));
			if (request->argName(i) == "radioEnable")
			{
				if (request->arg(i) != "")
				{
					// Compare the argument directly without converting to String
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
					{
						radioEnable = true;
					}
				}
			}

			if (request->argName(i) == "nw_band")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
					{
						config.band = request->arg(i).toInt();
						// if (request->arg(i).toInt())
						// 	config.band = 1;
						// else
						// 	config.band = 0;
					}
				}
			}

			if (request->argName(i) == "volume")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.volume = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "rf_power")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
					{
						if (request->arg(i).toInt())
							config.rf_power = true;
						else
							config.rf_power = false;
					}
				}
			}

			if (request->argName(i) == "sql_level")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.sql_level = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "tx_freq")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.freq_tx = request->arg(i).toFloat();
				}
			}
			if (request->argName(i) == "rx_freq")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.freq_rx = request->arg(i).toFloat();
				}
			}

			if (request->argName(i) == "tx_offset")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.offset_tx = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "rx_offset")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.offset_rx = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "tx_ctcss")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.tone_tx = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "rx_ctcss")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.tone_rx = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "rf_type")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.rf_type = request->arg(i).toInt();
				}
			}
		}
		// config.noise=noiseEn;
		// config.agc=agcEn;
		config.rf_en = radioEnable;
		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(64); // Small buffer for "OK"
		if (html)
		{
			strcpy(html, "OK");
			request->send(200, "text/html", html); // send to someones browser when asked
			free(html);							   // Free the allocated memory
		}
		saveConfiguration("/default.cfg", config);
		delay(500);
		RF_MODULE(false);
	}
	else if (request->hasArg("commitTNC"))
	{
		bool hpf = 0;
		bool lpf = 0;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "HPF")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
					{
						hpf = true;
					}
				}
			}
			if (request->argName(i) == "LPF")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
					{
						lpf = true;
					}
				}
			}
			if (request->argName(i) == "timeSlot")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
					{
						config.tx_timeslot = request->arg(i).toInt();
					}
				}
			}
			if (request->argName(i) == "preamble")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
					{
						config.preamble = request->arg(i).toInt();
					}
				}
			}
			if (request->argName(i) == "modem_type")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.modem_type = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "fx25_mode")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.fx25_mode = request->arg(i).toInt();
				}
			}
		}
		config.audio_hpf = hpf;
		config.audio_lpf = lpf;
		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(64); // Small buffer for "OK"
		if (html)
		{
			strcpy(html, "OK");
			request->send(200, "text/html", html); // send to someones browser when asked
			free(html);							   // Free the allocated memory
		}
		saveConfiguration("/default.cfg", config);
		afskSetModem(config.modem_type, config.audio_lpf, config.tx_timeslot, config.preamble * 100, config.fx25_mode);
	}
	else
	{
		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(12000); // Initial buffer size, adjust as needed
		if (!html)
		{
			return; // Memory allocation failed
		}

		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "var sliderVol = document.getElementById(\"sliderVolume\");\n");
		strcat(html, "var outputVol = document.getElementById(\"volShow\");\n");
		strcat(html, "var sliderSql = document.getElementById(\"sliderSql\");\n");
		strcat(html, "var outputSql = document.getElementById(\"sqlShow\");\n");
		strcat(html, "outputVol.innerHTML = sliderVol.value;\n");
		strcat(html, "outputSql.innerHTML = sliderSql.value;\n");
		strcat(html, "\n");
		strcat(html, "sliderVol.oninput = function () {\n");
		strcat(html, "outputVol.innerHTML = this.value;\n");
		strcat(html, "}\n");
		strcat(html, "sliderSql.oninput = function () {\n");
		strcat(html, "outputSql.innerHTML = this.value;\n");
		strcat(html, "}\n");
		strcat(html, "\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "if(e.currentTarget.id===\"formRadio\") document.getElementById(\"submitRadio\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formTNC\") document.getElementById(\"submitTNC\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/radio',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "function rfType(){\n");
		strcat(html, "var type = document.getElementById(\"rf_type\").value;\n");
		strcat(html, "if(type==1||type==4||type==7){document.getElementById(\"tx_freq\").setAttribute(\"max\",174);document.getElementById(\"rx_freq\").setAttribute(\"max\",174);};\n");
		strcat(html, "if(type==1){document.getElementById(\"tx_freq\").setAttribute(\"min\",134);document.getElementById(\"rx_freq\").setAttribute(\"min\",134);};\n");
		strcat(html, "if(type==4||type==7){document.getElementById(\"tx_freq\").setAttribute(\"min\",136);document.getElementById(\"rx_freq\").setAttribute(\"min\",136);};\n");
		strcat(html, "if(type==2||type==5||type==8){document.getElementById(\"tx_freq\").setAttribute(\"max\",470);document.getElementById(\"rx_freq\").setAttribute(\"max\",470);};\n");
		strcat(html, "if(type==2||type==5||type==8){document.getElementById(\"tx_freq\").setAttribute(\"min\",400);document.getElementById(\"rx_freq\").setAttribute(\"min\",400);};\n");
		strcat(html, "if(type==3){document.getElementById(\"tx_freq\").setAttribute(\"min\",320);document.getElementById(\"rx_freq\").setAttribute(\"min\",320);};\n");
		strcat(html, "if(type==3){document.getElementById(\"tx_freq\").setAttribute(\"max\",400);document.getElementById(\"rx_freq\").setAttribute(\"max\",400);};\n");
		strcat(html, "if(type==6){document.getElementById(\"tx_freq\").setAttribute(\"min\",350);document.getElementById(\"rx_freq\").setAttribute(\"min\",350);};\n");
		strcat(html, "if(type==6){document.getElementById(\"tx_freq\").setAttribute(\"max\",390);document.getElementById(\"rx_freq\").setAttribute(\"max\",390);};\n");
		strcat(html, "if(type==1||type==4||type==7){document.getElementById(\"tx_freq\").setAttribute(\"value\",144.390);document.getElementById(\"rx_freq\").setAttribute(\"value\",144.390);};\n");
		strcat(html, "if(type==2||type==5||type==8){document.getElementById(\"tx_freq\").setAttribute(\"value\",432.5);document.getElementById(\"rx_freq\").setAttribute(\"value\",432.5);};\n");
		strcat(html, "\n");
		strcat(html, "}\n");
		strcat(html, "</script>\n");
		strcat(html, "<form id='formRadio' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>RF Analog Module</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Enable:</b></td>\n");

		// Handle radio enable flag
		char temp_buffer[256];
		if (config.rf_en)
		{
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"radioEnable\" value=\"OK\" checked><span class=\"slider round\"></span></label></td>\n");
		}
		else
		{
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"radioEnable\" value=\"OK\" ><span class=\"slider round\"></span></label></td>\n");
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Module Type:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"rf_type\" id=\"rf_type\" onchange=\"rfType()\">\n");
		for (int i = 0; i < 10; i++)
		{
			snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" ", i);
			strcat(html, temp_buffer);
			if (config.rf_type == i)
			{
				strcat(html, "selected>");
			}
			else
			{
				strcat(html, ">");
			}
			snprintf(temp_buffer, sizeof(temp_buffer), "%s</option>\n", RF_TYPE[i]);
			strcat(html, temp_buffer);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		float freqMin = 0;
		float freqMax = 0;
		switch (config.rf_type)
		{
		case RF_SA868_VHF:
			freqMin = 134.0F;
			freqMax = 174.0F;
			break;
		case RF_SR_1WV:
		case RF_SR_2WVS:
			freqMin = 136.0F;
			freqMax = 174.0F;
			break;
		case RF_SA868_350:
			freqMin = 320.0F;
			freqMax = 400.0F;
			break;
		case RF_SR_1W350:
			freqMin = 350.0F;
			freqMax = 390.0F;
			break;
		case RF_SA868_UHF:
		case RF_SR_1WU:
		case RF_SR_2WUS:
			freqMin = 400.0F;
			freqMax = 470.0F;
			break;
		default:
			freqMin = 134.0F;
			freqMax = 500.0F;
			break;
		}
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>TX Frequency:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input type=\"number\" id=\"tx_freq\" name=\"tx_freq\" min=\"%.4f\" max=\"%.4f\"\n", freqMin, freqMax);
		strcat(html, temp_buffer);
		snprintf(temp_buffer, sizeof(temp_buffer), "step=\"0.0001\" value=\"%.4f\" /> MHz</td>\n", config.freq_tx);
		strcat(html, temp_buffer);

		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>RX Frequency:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input type=\"number\" id=\"rx_freq\" name=\"rx_freq\" min=\"%.4f\" max=\"%.4f\"\n", freqMin, freqMax);
		strcat(html, temp_buffer);
		snprintf(temp_buffer, sizeof(temp_buffer), "step=\"0.0001\" value=\"%.4f\" /> Mhz</td>\n", config.freq_rx);
		strcat(html, temp_buffer);

		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>TX CTCSS:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"tx_ctcss\" id=\"tx_ctcss\">\n");
		for (int i = 0; i < 39; i++)
		{
			snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" ", i);
			strcat(html, temp_buffer);
			if (config.tone_tx == i)
			{
				strcat(html, "selected>");
			}
			else
			{
				strcat(html, ">");
			}
			snprintf(temp_buffer, sizeof(temp_buffer), "%.1f</option>\n", ctcss[i]);
			strcat(html, temp_buffer);
		}
		strcat(html, "</select> Hz\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>RX CTCSS:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"rx_ctcss\" id=\"rx_ctcss\">\n");
		strcat(html, "<option value=\"0\" selected>0.0</option>\n");
		for (int i = 0; i < 39; i++)
		{
			snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" ", i);
			strcat(html, temp_buffer);
			if (config.tone_rx == i)
			{
				strcat(html, "selected>");
			}
			else
			{
				strcat(html, ">");
			}
			snprintf(temp_buffer, sizeof(temp_buffer), "%.1f</option>\n", ctcss[i]);
			strcat(html, temp_buffer);
		}
		strcat(html, "</select> Hz\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Narrow/Wide:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"nw_band\" id=\"nw_band\">\n");

		if (config.band)
		{
			strcat(html, "<option value=\"0\" >12.5KHz</option>\n");
			strcat(html, "<option value=\"1\" selected>25.0KHz</option>\n");
		}
		else
		{
			strcat(html, "<option value=\"0\" selected>12.5KHz</option>\n");
			strcat(html, "<option value=\"1\" >25.0KHz</option>\n");
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>TX Power:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"rf_power\" id=\"rf_power\">\n");

		if (config.rf_power)
		{
			strcat(html, "<option value=\"1\" selected>HIGH</option>\n");
			strcat(html, "<option value=\"0\" >LOW</option>\n");
		}
		else
		{
			strcat(html, "<option value=\"1\" >HIGH</option>\n");
			strcat(html, "<option value=\"0\" selected>LOW</option>\n");
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>VOLUME:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input id=\"sliderVolume\" name=\"volume\" type=\"range\"\nmin=\"1\" max=\"8\" value=\"%d\" /><b><span style=\"font-size: 14pt;\" id=\"volShow\">%d</span></b></td>\n", config.volume, config.volume);
		strcat(html, temp_buffer);

		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>SQL Level:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input id=\"sliderSql\" name=\"sql_level\" type=\"range\"\nmin=\"0\" max=\"8\" value=\"%d\" /><b><span style=\"font-size: 14pt;\" id=\"sqlShow\">%d</span></b></td>\n", config.sql_level, config.sql_level);
		strcat(html, temp_buffer);

		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitRadio'  name=\"commitRadio\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitRadio\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form>");

		// AFSK,TNC Configuration
		strcat(html, "<form id='formTNC' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>AFSK/TNC Configuration</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Modem Type:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"modem_type\" id=\"modem_type\" \">\n");
		for (int i = 0; i < 3; i++)
		{
			snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" ", i);
			strcat(html, temp_buffer);
			if (config.modem_type == i)
			{
				strcat(html, "selected>");
			}
			else
			{
				strcat(html, ">");
			}
			snprintf(temp_buffer, sizeof(temp_buffer), "%s</option>\n", MODEM_TYPE[i]);
			strcat(html, temp_buffer);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>FX.25 Mode:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"fx25_mode\" id=\"fx25_mode\" \">\n");
		for (int i = 0; i < 3; i++)
		{
			snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" ", i);
			strcat(html, temp_buffer);
			if (config.fx25_mode == i)
			{
				strcat(html, "selected>");
			}
			else
			{
				strcat(html, ">");
			}
			snprintf(temp_buffer, sizeof(temp_buffer), "%s</option>\n", FX25_MODE[i]);
			strcat(html, temp_buffer);
		}
		strcat(html, "</select>  (FX.25 = AX.25 + FEC)\n");
		strcat(html, "</td>\n");
		strcat(html, "<tr>\n");
		// strcat(html, "<td align=\"right\"><b>Audio HPF:</b></td>\n");
		// char strFlag[32] = "";
		// if (config.audio_hpf)
		// 	strcpy(strFlag, "checked");
		// snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"HPF\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Audio high pass filter >1KHz cutoff 10Khz</i></label></td>\n", strFlag);
		// strcat(html, temp_buffer);
		// strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Deemphasis Audio:</b></td>\n");
		if (config.audio_lpf)
		{
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"LPF\" value=\"OK\" checked><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Audio low pass filter 1hz-2.5KHz</i></label></td>\n");
		}
		else
		{
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"LPF\" value=\"OK\" ><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Audio low pass filter 1hz-2.5KHz</i></label></td>\n");
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>TX Time Slot:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input type=\"number\" name=\"timeSlot\" min=\"0\" max=\"99999\"\nstep=\"100\" value=\"%d\" /> mSec.</td>\n", config.tx_timeslot);
		strcat(html, temp_buffer);

		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Preamble:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"preamble\">\n");
		for (int i = 1; i < 11; i++)
		{
			snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" ", i);
			strcat(html, temp_buffer);
			if (config.preamble == i)
			{
				strcat(html, "selected>");
			}
			else
			{
				strcat(html, ">");
			}
			snprintf(temp_buffer, sizeof(temp_buffer), "%d</option>\n", i * 100);
			strcat(html, temp_buffer);
		}
		strcat(html, "</select> mSec.\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitTNC'  name=\"commitTNC\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitTNC\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form>");
		// request->send(200, "text/html", html); // send to someones browser when asked
		//request->send_P(200, "text/html", html);

		AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, html);
		response->addHeader("Sysinfo", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
	}
}

void handle_vpn(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	if (request->hasArg("commitVPN"))
	{
		bool vpnEn = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "vpnEnable")
			{
				if (request->arg(i) != "")
				{
					// if (isValidNumber(request->arg(i)))
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						vpnEn = true;
				}
			}

			// if (request->argName(i) == "taretime") {
			//	if (request->arg(i) != "")
			//	{
			//		//if (isValidNumber(request->arg(i)))
			//		if (strcmp(request->arg(i).c_str(), "OK") == 0)
			//			taretime = true;
			//	}
			// }
			if (request->argName(i) == "wg_port")
			{
				if (request->arg(i) != "")
				{
					config.wg_port = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "wg_public_key")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.wg_public_key, request->arg(i).c_str());
					config.wg_public_key[44] = 0;
				}
			}

			if (request->argName(i) == "wg_private_key")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.wg_private_key, request->arg(i).c_str());
					config.wg_private_key[44] = 0;
				}
			}

			if (request->argName(i) == "wg_peer_address")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.wg_peer_address, request->arg(i).c_str());
				}
			}

			if (request->argName(i) == "wg_local_address")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.wg_local_address, request->arg(i).c_str());
				}
			}

			if (request->argName(i) == "wg_netmask_address")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.wg_netmask_address, request->arg(i).c_str());
				}
			}

			if (request->argName(i) == "wg_gw_address")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.wg_gw_address, request->arg(i).c_str());
				}
			}
		}

		config.vpn = vpnEn;
		saveConfig(request);
	}
	else
	{
		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(8192); // Initial buffer size, adjust as needed
		if (!html)
		{
			return; // Memory allocation failed
		}

		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "if(e.currentTarget.id===\"formVPN\") document.getElementById(\"submitVPN\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/vpn',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");

		// Get MAC address and remove colons
		String ESP32_ID = WiFi.macAddress();
		ESP32_ID.replace(":", "");
		char temp_buffer[512];
		snprintf(temp_buffer, sizeof(temp_buffer), "function loadVPNConfig() {\nconst url = \"http://hs1.hs5tqa.ampr.org:81/wg/create\";\nconst espID = {'name': '%s'};\n", ESP32_ID.c_str());
		strcat(html, temp_buffer);
		strcat(html, "fetch(url,{\n");
		strcat(html, "method: 'POST',\n");
		strcat(html, "body: JSON.stringify(espID),\n");
		strcat(html, "headers: { 'Content-Type': 'application/json', 'Access-Control-Allow-Headers': 'Content-Type', 'Access-Control-Allow-Origin': '*','Access-Control-Allow-Methods': 'POST,GET,OPTIONS'}\n");
		strcat(html, "})\n");
		strcat(html, ".then(response => response.json())\n");
		strcat(html, ".then(data => {\n");
		strcat(html, "console.log(\"VPN Data:\", data);\n");
		strcat(html, "document.getElementById(\"wg_enable\").checked = true;\n");
		strcat(html, "document.getElementById(\"wg_peer_address\").value = data.Enpoint.split(\":\")[0];\n");
		strcat(html, "document.getElementById(\"wg_port\").value = data.Enpoint.split(\":\")[1];\n");
		strcat(html, "document.getElementById(\"wg_local_address\").value = data.Address;\n");
		strcat(html, "document.getElementById(\"wg_netmask_address\").value = \"255.255.255.0\";\n");
		strcat(html, "document.getElementById(\"wg_gw_address\").value = data.Gateway;\n");
		strcat(html, "document.getElementById(\"wg_public_key\").value = data.PublicKey;\n");
		strcat(html, "document.getElementById(\"wg_private_key\").value = data.PrivateKey;\n");
		strcat(html, "})\n");
		strcat(html, ".catch(err => console.error(\"VPN API Error:\", err));\n}\n");
		strcat(html, "</script>\n");
		// ===== JavaScript AJAX =====
		// strcat(html, "<script>\n");
		// strcat(html, "function loadVPNConfig() {\n");
		// strcat(html, "  $.ajax({\n");
		// strcat(html, "    url: '/api/vpnreq',\n");
		// strcat(html, "    method: 'GET',\n");
		// strcat(html, "    dataType: 'json',\n");
		// strcat(html, "    success: function(data) {\n");
		// strcat(html, "       console.log(data);\n");
		// strcat(html, "       let ep = data.Enpoint.split(':');\n");
		// strcat(html, "       $('#wg_peer_address').val(ep[0]);\n");
		// strcat(html, "       $('#wg_port').val(ep[1]);\n");
		// strcat(html, "       $('#wg_local_address').val(data.Address);\n");
		// strcat(html, "       $('#wg_public_key').val(data.PublicKey);\n");
		// strcat(html, "       $('#wg_private_key').val(data.PrivateKey);\n");
		// strcat(html, "    },\n");
		// strcat(html, "    error: function(e) {\n");
		// strcat(html, "       alert('โหลดข้อมูล VPN ไม่สำเร็จ');\n");
		// strcat(html, "       console.log(e);\n");
		// strcat(html, "    }\n");
		// strcat(html, "  });\n");
		// strcat(html, "}\n");
		// strcat(html, "</script>");

		// strcat(html, "<h2>System Setting</h2>\n");
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromVPN\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>Wireguard Configuration</b></span></th>\n");
		strcat(html, "<tr>");

		// Handle sync flag
		if (config.vpn)
		{
			strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" id=\"wg_enable\" name=\"vpnEnable\" value=\"OK\" checked><span class=\"slider round\"></span></label></td>\n");
		}
		else
		{
			strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" id=\"wg_enable\" name=\"vpnEnable\" value=\"OK\" ><span class=\"slider round\"></span></label></td>\n");
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Server Address</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input  size=\"20\" maxlength=\"32\" id=\"wg_peer_address\" name=\"wg_peer_address\" type=\"text\" value=\"%s\" /></td>\n", config.wg_peer_address);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Server Port</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input id=\"wg_port\" size=\"5\" name=\"wg_port\" type=\"number\" value=\"%d\" /></td>\n", config.wg_port);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Local Address</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input id=\"wg_local_address\" name=\"wg_local_address\" type=\"text\" value=\"%s\" /></td>\n", config.wg_local_address);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Netmask</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input id=\"wg_netmask_address\" name=\"wg_netmask_address\" type=\"text\" value=\"%s\" /></td>\n", config.wg_netmask_address);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Gateway</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input id=\"wg_gw_address\" name=\"wg_gw_address\" type=\"text\" value=\"%s\" /></td>\n", config.wg_gw_address);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Public Server Key</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input size=\"50\" maxlength=\"44\" id=\"wg_public_key\" name=\"wg_public_key\" type=\"text\" value=\"%s\" /></td>\n", config.wg_public_key);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Private Client Key</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input size=\"50\" maxlength=\"44\" id=\"wg_private_key\" name=\"wg_private_key\" type=\"text\" value=\"%s\" /></td>\n", config.wg_private_key);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitVPN'  name=\"commitVPN\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitVPN\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br /><br />");

		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromGetVPN\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>Helper: Free VPN Wireguard for Web Service</b></span></th>\n");
		strcat(html, "<tr><td align=\"left\">1. Click New Register button to get VPN config from web service.</td></tr>\n");
		strcat(html, "<tr><td align=\"left\">2. The VPN config will fill in the form automatically.</td></tr>\n");
		strcat(html, "<tr><td align=\"left\">3. Click Apply Change and reboot again.</td></tr>\n");
		strcat(html, "<tr><td align=\"left\">4. Enjoy your free VPN service!</td></tr>\n");

		// Check if local address starts with "10.44."
		String wg_local_addr = String(config.wg_local_address);
		if (wg_local_addr.startsWith("10.44."))
		{
			int lastoct = wg_local_addr.substring(wg_local_addr.lastIndexOf('.') + 1).toInt();
			// String url="http://"+String(config.wg_peer_address)+":"+String(8000+lastoct);
			// strcat(html, "<tr><td>Your External Host IP: <a href=\"");
			// snprintf(temp_buffer, sizeof(temp_buffer), "%s\">%s</a></td></tr>\n", url.c_str(), url.c_str());
			// strcat(html, temp_buffer);
			int thirdoct = wg_local_addr.substring(wg_local_addr.indexOf('.', wg_local_addr.indexOf('.') + 1) + 1, wg_local_addr.lastIndexOf('.')).toInt();
			String url_base = "http://hs" + String(thirdoct) + ".hs5tqa.ampr.org:" + String((thirdoct * 10000) + 8000 + lastoct);
			snprintf(temp_buffer, sizeof(temp_buffer), "<tr><td>Your External by AMPR URL: <a href=\"%s\" target=\"_blank\">%s</a></td></tr>\n", url_base.c_str(), url_base.c_str());
			strcat(html, temp_buffer);
			url_base = "http://vpn.nakhonthai.net:" + String((thirdoct * 10000) + 8000 + lastoct);
			snprintf(temp_buffer, sizeof(temp_buffer), "<tr><td>Fast Direct URL: <a href=\"%s\" target=\"_blank\">%s</a></td></tr>\n", url_base.c_str(), url_base.c_str());
			strcat(html, temp_buffer);
		}
		strcat(html, "<tr><td><button type=\"button\" onclick=\"loadVPNConfig()\">New Register</button></td></tr>\n");
		strcat(html, "</table><br />\n");
		strcat(html, "</form>");

		// request->send(200, "text/html", html); // send to someones browser when asked
		AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, html);
		response->addHeader("VPN", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
	}
}

#ifdef MQTT
void handle_mqtt(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	if (request->hasArg("commitMQTT"))
	{
		bool mqttEn = false;
		config.mqtt_topic_flag = 0;
		config.mqtt_subscribe_flag = 0;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "enable")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						mqttEn = true;
				}
			}

			if (request->argName(i) == "host")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.mqtt_host, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "port")
			{
				if (request->arg(i) != "")
				{
					config.mqtt_port = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "user")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.mqtt_user, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "pass")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.mqtt_pass, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "topic")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.mqtt_topic, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "subscribe")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.mqtt_subscribe, request->arg(i).c_str());
				}
			}

			if (request->argName(i) == "TopicTNC")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.mqtt_topic_flag |= MQTT_TOPIC_TNC;
				}
			}
			if (request->argName(i) == "TopicSts")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.mqtt_topic_flag |= MQTT_TOPIC_STATUS;
				}
			}
			if (request->argName(i) == "TopicTlm")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.mqtt_topic_flag |= MQTT_TOPIC_TELEMETRY;
				}
			}
			if (request->argName(i) == "TopicWX")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.mqtt_topic_flag |= MQTT_TOPIC_WX;
				}
			}
			if (request->argName(i) == "TopicSensor")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.mqtt_topic_flag |= MQTT_TOPIC_SENSOR;
				}
			}

			if (request->argName(i) == "subCMD")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.mqtt_topic_flag |= MQTT_SUBSCRIBE_CMD;
				}
			}
			if (request->argName(i) == "subTNC")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.mqtt_topic_flag |= MQTT_SUBSCRIBE_TNC;
				}
			}
			if (request->argName(i) == "subMsg")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.mqtt_topic_flag |= MQTT_SUBSCRIBE_MESSAGE;
				}
			}
		}

		config.en_mqtt = mqttEn;
		clientMQTT.disconnect();
		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(256); // Buffer for response message
		if (html)
		{
			if (saveConfiguration("/default.cfg", config))
			{
				strcpy(html, "Setup completed successfully");
				request->send(200, "text/html", html); // send to someones browser when asked
			}
			else
			{
				strcpy(html, "Save config failed.");
				request->send(501, "text/html", html); // Not Implemented
			}
			free(html); // Free the allocated memory
		}
	}
	else
	{
		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(8192); // Initial buffer size, adjust as needed
		if (!html)
		{
			return; // Memory allocation failed
		}

		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "if(e.currentTarget.id===\"formVPN\") document.getElementById(\"submitMQTT\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/mqtt',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "</script>\n");

		// strcat(html, "<h2>System Setting</h2>\n");
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromMQTT\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>MQTT Configuration</b></span></th>\n");
		strcat(html, "<tr>");

		// Handle sync flag
		if (config.en_mqtt)
		{
			strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"enable\" value=\"OK\" checked><span class=\"slider round\"></span></label></td>\n");
		}
		else
		{
			strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"enable\" value=\"OK\" ><span class=\"slider round\"></span></label></td>\n");
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Server Address:</b></td>\n");
		char temp_buffer[512];
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input  size=\"30\" maxlength=\"32\" name=\"host\" type=\"text\" value=\"%s\" /></td>\n", config.mqtt_host);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Server Port:</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input size=\"5\"  maxlength=\"5\"  name=\"port\" type=\"number\" value=\"%d\" /></td>\n", config.mqtt_port);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>User:</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input maxlength=\"32\" name=\"user\" type=\"text\" value=\"%s\" /></td>\n", config.mqtt_user);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Password:</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input size=\"40\" maxlength=\"63\" name=\"pass\" type=\"password\" value=\"%s\" /></td>\n", config.mqtt_pass);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Topic:</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input size=\"50\" maxlength=\"32\" id=\"topic\" name=\"topic\" type=\"text\" value=\"%s\" /></td>\n", config.mqtt_topic);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Topic Flag:</b></td>\n");

		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<fieldset id=\"TopicGrp\">\n");
		strcat(html, "<legend>Topic Flags Send out MQTT</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">");
		strcat(html, "<tr style=\"background:unset;\">");

		// Handle topic flags
		if (config.mqtt_topic_flag & MQTT_TOPIC_TNC)
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicTNC\" type=\"checkbox\" value=\"OK\" checked/>TNC</td>\n");
		}
		else
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicTNC\" type=\"checkbox\" value=\"OK\" />TNC</td>\n");
		}

		if (config.mqtt_topic_flag & MQTT_TOPIC_STATUS)
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicSts\" type=\"checkbox\" value=\"OK\" checked/>Status</td>\n");
		}
		else
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicSts\" type=\"checkbox\" value=\"OK\" />Status</td>\n");
		}

		if (config.mqtt_topic_flag & MQTT_TOPIC_TELEMETRY)
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicTlm\" type=\"checkbox\" value=\"OK\" checked/>Telemetry</td>\n");
		}
		else
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicTlm\" type=\"checkbox\" value=\"OK\" />Telemetry</td>\n");
		}

		if (config.mqtt_topic_flag & MQTT_TOPIC_WX)
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicWX\" type=\"checkbox\" value=\"OK\" checked/>Weather</td>\n");
		}
		else
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicWX\" type=\"checkbox\" value=\"OK\" />Weather</td>\n");
		}

		if (config.mqtt_topic_flag & MQTT_TOPIC_SENSOR)
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicSensor\" type=\"checkbox\" value=\"OK\" checked/>Sensor</td>\n");
		}
		else
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"TopicSensor\" type=\"checkbox\" value=\"OK\" />Sensor</td>\n");
		}

		strcat(html, "<td style=\"border:unset;\"></td>");
		strcat(html, "</tr></table></fieldset>\n");
		strcat(html, "</td></tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Subscription:</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input size=\"50\" maxlength=\"32\" name=\"subscribe\" type=\"text\" value=\"%s\" /></td>\n", config.mqtt_subscribe);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Subscription Flag:</b></td>\n");

		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<fieldset id=\"SubGrp\">\n");
		strcat(html, "<legend>Subscription Flags Receive</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">");
		strcat(html, "<tr style=\"background:unset;\">");

		// Handle subscription flags
		if (config.mqtt_subscribe_flag & MQTT_SUBSCRIBE_CMD)
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"subCMD\" type=\"checkbox\" value=\"OK\" checked/>AT-Command</td>\n");
		}
		else
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"subCMD\" type=\"checkbox\" value=\"OK\" />AT-Command</td>\n");
		}

		if (config.mqtt_subscribe_flag & MQTT_SUBSCRIBE_TNC)
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"subTNC\" type=\"checkbox\" value=\"OK\" checked/>TNC</td>\n");
		}
		else
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"subTNC\" type=\"checkbox\" value=\"OK\" />TNC</td>\n");
		}

		if (config.mqtt_subscribe_flag & MQTT_SUBSCRIBE_MESSAGE)
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"subMsg\" type=\"checkbox\" value=\"OK\" checked/>Message</td>\n");
		}
		else
		{
			strcat(html, "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"subMsg\" type=\"checkbox\" value=\"OK\" />Message</td>\n");
		}

		strcat(html, "<td style=\"border:unset;\"></td>");
		strcat(html, "</tr></table></fieldset>\n");
		strcat(html, "</td></tr>\n");

		strcat(html, "</table><br />\n");
		strcat(html, "<td><input class=\"button\" id=\"submitMQTT\" name=\"commitMQTT\" type=\"submit\" value=\"Save Config\" maxlength=\"80\"/></td>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitMQTT\"/>\n");
		strcat(html, "</form>\n");

		// request->send(200, "text/html", html); // send to someones browser when asked
		AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, html);
		response->addHeader("MQTT", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
	}
}
#endif

void handle_msg(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	if (request->hasArg("commitChat"))
	{
		// Using char arrays instead of String
		char toCall[10];
		char msg[256];
		memset(toCall, 0, sizeof(toCall));
		memset(msg, 0, sizeof(msg));

		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "toCall")
			{
				if (request->arg(i) != "")
				{
					strncpy(toCall, request->arg(i).c_str(), sizeof(toCall) - 1);
				}
			}
			if (request->argName(i) == "msg")
			{
				if (request->arg(i) != "")
				{
					strncpy(msg, request->arg(i).c_str(), sizeof(msg) - 1);
				}
			}
		}
		log_d("Chat to %s | msg %s", toCall, msg);
		sendAPRSMessage(String(toCall), String(msg), config.msg_encrypt);
		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(64); // Small buffer for "Send completed"
		if (html)
		{
			strcpy(html, "Send completed");
			request->send(200, "text/html", html); // send to someones browser when asked
			free(html);							   // Free the allocated memory
		}
	}
	else if (request->hasArg("commitMSG"))
	{
		bool msgEn = false;
		bool msgRf = false;
		bool msgInet = false;
		bool msgEncrypt = false;

		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "enable")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						msgEn = true;
				}
			}
			if (request->argName(i) == "msgRf")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						msgRf = true;
				}
			}
			if (request->argName(i) == "msgInet")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						msgInet = true;
				}
			}
			if (request->argName(i) == "encrypt")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						msgEncrypt = true;
				}
			}

			if (request->argName(i) == "mycall")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.msg_mycall, request->arg(i).c_str());
					config.msg_mycall[9] = 0;
				}
			}

			if (request->argName(i) == "key")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.msg_key, request->arg(i).c_str());
					config.msg_key[32] = 0;
				}
			}

			if (request->argName(i) == "retry")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.msg_retry = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "path")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.msg_path = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "timeout")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.msg_interval = request->arg(i).toInt();
				}
			}
		}

		config.msg_enable = msgEn;
		config.msg_rf = msgRf;
		config.msg_inet = msgInet;
		config.msg_encrypt = msgEncrypt;

		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(256); // Buffer for response message
		if (html)
		{
			if (saveConfiguration("/default.cfg", config))
			{
				strcpy(html, "Setup completed successfully");
				request->send(200, "text/html", html); // send to someones browser when asked
			}
			else
			{
				strcpy(html, "Save config failed.");
				request->send(501, "text/html", html); // Not Implemented
			}
			free(html); // Free the allocated memory
		}
	}
	else
	{
		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(8192); // Initial buffer size, adjust as needed
		if (!html)
		{
			return; // Memory allocation failed
		}

		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "if(e.currentTarget.id===\"formMSG\") document.getElementById(\"submitMSG\").disabled=true;\n");
		// strcat(html, "if(e.currentTarget.id===\"formChat\") document.getElementById(\"submitI2C0\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/msg',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "if(e.currentTarget.id===\"formMSG\") alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "if(e.currentTarget.id===\"formMSG\") alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");

		// strcat(html, "if (!!window.EventSource) {";
		// strcat(html, "var source = new EventSource('/eventMsg');");

		// strcat(html, "source.addEventListener('open', function(e) {";
		// strcat(html, "console.log(\"Events MSG Connected\");";
		// strcat(html, "}, false);";
		// strcat(html, "source.addEventListener('error', function(e) {";
		// strcat(html, "if (e.target.readyState != EventSource.OPEN) {";
		// strcat(html, "console.log(\"Events MSG Disconnected\");";
		// strcat(html, "}\n}, false);";
		// strcat(html, "source.addEventListener('chatMsg', function(e) {";
		// // strcat(html, "console.log(\"lastHeard\", e.data);";
		// strcat(html, "var lh=document.getElementById(\"chatMsg\");";
		// strcat(html, "if(lh != null) {lh.innerHTML = e.data;}";
		// strcat(html, "}, false);\n}";
		strcat(html, "</script>\n");

		// strcat(html, "<h2>System Setting</h2>\n");
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"formMSG\" method=\"post\">\n");
		strcat(html, "<table width=\"90%\">\n");
		strcat(html, "<th colspan=\"2\"><span><b>Message Configuration</b></span></th>\n");
		strcat(html, "<tr>");

		// Handle sync flag
		if (config.msg_enable)
		{
			strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"enable\" value=\"OK\" checked><span class=\"slider round\"></span></label></td>\n");
		}
		else
		{
			strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"enable\" value=\"OK\" ><span class=\"slider round\"></span></label></td>\n");
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>My Callsign:</b></td>\n");
		char temp_buffer[512];
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input  size=\"20\" maxlength=\"9\" name=\"mycall\" type=\"text\" value=\"%s\" /> *<i>Callsign with SSID (Ex. HS5TQA-12)</i></td>\n", config.msg_mycall);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		// Handle RF and Internet flags
		if (config.msg_rf && config.msg_inet)
		{
			strcat(html, "<tr><td style=\"text-align: right;\"><b>TX Channel:</b></td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"msgRf\" value=\"OK\" checked/>RF <input type=\"checkbox\" name=\"msgInet\" value=\"OK\" checked/>Internet </td></tr>\n");
		}
		else if (config.msg_rf)
		{
			strcat(html, "<tr><td style=\"text-align: right;\"><b>TX Channel:</b></td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"msgRf\" value=\"OK\" checked/>RF <input type=\"checkbox\" name=\"msgInet\" value=\"OK\" />Internet </td></tr>\n");
		}
		else if (config.msg_inet)
		{
			strcat(html, "<tr><td style=\"text-align: right;\"><b>TX Channel:</b></td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"msgRf\" value=\"OK\" />RF <input type=\"checkbox\" name=\"msgInet\" value=\"OK\" checked/>Internet </td></tr>\n");
		}
		else
		{
			strcat(html, "<tr><td style=\"text-align: right;\"><b>TX Channel:</b></td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"msgRf\" value=\"OK\" />RF <input type=\"checkbox\" name=\"msgInet\" value=\"OK\" />Internet </td></tr>\n");
		}

		strcat(html, "<tr>");
		if (config.msg_encrypt)
		{
			strcat(html, "<td align=\"right\"><b>Encryption</b></td>\n");
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"encrypt\" value=\"OK\" checked><span class=\"slider round\"></span></label></td>\n");
		}
		else
		{
			strcat(html, "<td align=\"right\"><b>Encryption</b></td>\n");
			strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"encrypt\" value=\"OK\" ><span class=\"slider round\"></span></label></td>\n");
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>AES Key:</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input  size=\"40\" maxlength=\"33\" name=\"key\" type=\"text\" value=\"%s\" /> *<i>ASCII HEX 16Byte</i></td>\n", config.msg_key);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Send Retry:</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input  min=\"0\" max=\"99\"   name=\"retry\" type=\"number\" value=\"%d\" /></td>\n", config.msg_retry);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Send Timeout:</b></td>\n");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input  min=\"1\" max=\"9999\"   name=\"timeout\" type=\"number\" value=\"%d\" /> Sec.</td>\n", config.msg_interval);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PATH:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"path\" id=\"path\">\n");
		for (uint8_t pthIdx = 0; pthIdx < PATH_LEN; pthIdx++)
		{
			snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" ", pthIdx);
			strcat(html, temp_buffer);
			if (config.msg_path == pthIdx)
			{
				strcat(html, "selected>");
			}
			else
			{
				strcat(html, ">");
			}
			snprintf(temp_buffer, sizeof(temp_buffer), "%s</option>\n", PATH_NAME[pthIdx]);
			strcat(html, temp_buffer);
		}
		strcat(html, "</select></td>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitMSG'  name=\"commitMSG\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitMSG\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br /><br />");

		strcat(html, "<table width=\"90%\">\n");
		strcat(html, "<th style=\"background-color: #070ac2;\">CHAT MESSAGE</th>\n");

		strcat(html, "<tr><td>\n");
		strcat(html, "<table id=\"chatMsg\">\n");
		strcat(html, event_chatMessage(true).c_str());
		strcat(html, "</table>\n");

		strcat(html, "</td></tr><tr><td colspan=\"5\">");

		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"formChat\" method=\"post\">\n");
		strcat(html, "<table>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"left\"><b>TO:</b><input size=\"10\" name=\"toCall\" id=\"toCall\" type=\"text\" value=\"\" oninput=\"this.value=this.value.toUpperCase();\" /> <b>MSG:</b><input size=\"80\" name=\"msg\" id=\"msg\" type=\"text\" value=\"\" /></td>\n");
		strcat(html, "<td align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitChat\" name=\"commitChat\" type=\"submit\" value=\"Send\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitChat\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form><br />\n");

		strcat(html, "</td></tr></table>");

		// request->send(200, "text/html", html); // send to someones browser when asked
		AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, html);
		response->addHeader("MSG", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
	}
}

void handle_mod(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	if (request->hasArg("commitGNSS"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					// if (isValidNumber(request->arg(i)))
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "atc")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.gnss_at_command, request->arg(i).c_str());
				}
				else
				{
					memset(config.gnss_at_command, 0, sizeof(config.gnss_at_command));
				}
			}

			if (request->argName(i) == "Host")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.gnss_tcp_host, request->arg(i).c_str());
				}
			}

			if (request->argName(i) == "Port")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.gnss_tcp_port = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "channel")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.gnss_channel = request->arg(i).toInt();
				}
			}
		}

		config.gnss_enable = En;
		saveConfig(request);
	}
	else if (request->hasArg("commitUART0"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "baudrate")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.uart0_baudrate = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "rx")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.uart0_rx_gpio = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "tx")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.uart0_tx_gpio = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "rts")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.uart0_rts_gpio = request->arg(i).toInt();
				}
			}
		}

		config.uart0_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	else if (request->hasArg("commitUART1"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "baudrate")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.uart1_baudrate = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "rx")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.uart1_rx_gpio = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "tx")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.uart1_tx_gpio = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "rts")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.uart1_rts_gpio = request->arg(i).toInt();
				}
			}
		}

		config.uart1_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	// else if (request->hasArg("commitUART2"))
	// {
	// 	bool En = false;
	// 	for (uint8_t i = 0; i < request->args(); i++)
	// 	{
	// 		// Serial.print("SERVER ARGS ");
	// 		// Serial.print(request->argName(i));
	// 		// Serial.print("=");
	// 		// Serial.println(request->arg(i));

	// 		if (request->argName(i) == "Enable")
	// 		{
	// 			if (request->arg(i) != "")
	// 			{
	// 				if (String(request->arg(i)) == "OK")
	// 					En = true;
	// 			}
	// 		}

	// 	// 	if (request->argName(i) == "baudrate")
	// 	// 	{
	// 	// 		if (isValidNumber(request->arg(i)))
	// 	// 		{
	// 	// 			config.uart2_baudrate = request->arg(i).toInt();
	// 	// 		}
	// 	// 	}

	// 	// 	if (request->argName(i) == "rx")
	// 	// 	{
	// 	// 		if (isValidNumber(request->arg(i)))
	// 	// 		{
	// 	// 			config.uart2_rx_gpio = request->arg(i).toInt();
	// 	// 		}
	// 	// 	}

	// 	// 	if (request->argName(i) == "tx")
	// 	// 	{
	// 	// 		if (isValidNumber(request->arg(i)))
	// 	// 		{
	// 	// 			config.uart2_tx_gpio = request->arg(i).toInt();
	// 	// 		}
	// 	// 	}

	// 	// 	if (request->argName(i) == "rts")
	// 	// 	{
	// 	// 		if (isValidNumber(request->arg(i)))
	// 	// 		{
	// 	// 			config.uart2_rts_gpio = request->arg(i).toInt();
	// 	// 		}
	// 	// 	}
	// 	// }

	// 	// config.uart2_enable = En;
	// 	saveConfiguration("/default.cfg", config);
	// 	String html = "OK";
	// 	request->send(200, "text/html", html);
	// }
	else if (request->hasArg("commitMODBUS"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					// if (isValidNumber(request->arg(i)))
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "channel")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.modbus_channel = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "address")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.modbus_address = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "de")
			{
				if (request->arg(i) != "")
				{
					config.modbus_de_gpio = request->arg(i).toInt();
				}
			}
		}

		config.modbus_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	else if (request->hasArg("commitTNC"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					// if (isValidNumber(request->arg(i)))
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "channel")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.ext_tnc_channel = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "mode")
			{
				if (isValidNumber(request->arg(i)))
				{
					config.ext_tnc_mode = request->arg(i).toInt();
				}
			}
		}

		config.ext_tnc_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	else if (request->hasArg("commitONEWIRE"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					// if (isValidNumber(request->arg(i)))
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "data")
			{
				if (request->arg(i) != "")
				{
					config.onewire_gpio = request->arg(i).toInt();
				}
			}
		}

		config.onewire_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	else if (request->hasArg("commitRF"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "sql_active")
			{
				if (request->arg(i) != "")
				{
					config.rf_sql_active = (bool)request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "pd_active")
			{
				if (request->arg(i) != "")
				{
					config.rf_pd_active = (bool)request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "pwr_active")
			{
				if (request->arg(i) != "")
				{
					config.rf_pwr_active = (bool)request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "ptt_active")
			{
				if (request->arg(i) != "")
				{
					config.rf_ptt_active = (bool)request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "baudrate")
			{
				if (request->arg(i) != "")
				{
					config.rf_baudrate = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "rx")
			{
				if (request->arg(i) != "")
				{
					config.rf_rx_gpio = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "tx")
			{
				if (request->arg(i) != "")
				{
					config.rf_tx_gpio = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "pd")
			{
				if (request->arg(i) != "")
				{
					config.rf_pd_gpio = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "pwr")
			{
				if (request->arg(i) != "")
				{
					config.rf_pwr_gpio = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "ptt")
			{
				if (request->arg(i) != "")
				{
					config.rf_ptt_gpio = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "sql")
			{
				if (request->arg(i) != "")
				{
					config.rf_sql_gpio = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "atten")
			{
				if (request->arg(i) != "")
				{
					config.adc_atten = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "offset")
			{
				if (request->arg(i) != "")
				{
					config.adc_dc_offset = request->arg(i).toInt();
				}
			}
		}
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	else if (request->hasArg("commitI2C0"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "sda")
			{
				if (request->arg(i) != "")
				{
					config.i2c_sda_pin = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "sck")
			{
				if (request->arg(i) != "")
				{
					config.i2c_sck_pin = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "freq")
			{
				if (request->arg(i) != "")
				{
					config.i2c_freq = request->arg(i).toInt();
				}
			}
		}

		config.i2c_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	else if (request->hasArg("commitI2C1"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "sda")
			{
				if (request->arg(i) != "")
				{
					config.i2c1_sda_pin = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "sck")
			{
				if (request->arg(i) != "")
				{
					config.i2c1_sck_pin = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "freq")
			{
				if (request->arg(i) != "")
				{
					config.i2c1_freq = request->arg(i).toInt();
				}
			}
		}

		config.i2c1_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	else if (request->hasArg("commitCOUNTER0"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "gpio")
			{
				if (request->arg(i) != "")
				{
					config.counter0_gpio = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "active")
			{
				if (request->arg(i) != "")
				{
					config.counter0_active = (bool)request->arg(i).toInt();
				}
			}
		}

		config.counter0_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	else if (request->hasArg("commitCOUNTER1"))
	{
		bool En = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			// Serial.print("SERVER ARGS ");
			// Serial.print(request->argName(i));
			// Serial.print("=");
			// Serial.println(request->arg(i));

			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}

			if (request->argName(i) == "gpio")
			{
				if (request->arg(i) != "")
				{
					config.counter1_gpio = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "active")
			{
				if (request->arg(i) != "")
				{
					config.counter1_active = (bool)request->arg(i).toInt();
				}
			}
		}

		config.counter0_enable = En;
		saveConfiguration("/default.cfg", config);
		String html = "OK";
		request->send(200, "text/html", html);
	}
	else if (request->hasArg("commitCMD"))
	{
		bool mqtt = false;
		bool msg = false;
		bool bluetooth = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "mqtt")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						mqtt = true;
					}
				}
			}

			if (request->argName(i) == "msg")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						msg = true;
					}
				}
			}

			if (request->argName(i) == "bluetooth")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						bluetooth = true;
					}
				}
			}

			if (request->argName(i) == "uart")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.at_cmd_uart = request->arg(i).toInt();
				}
			}
		}
		config.at_cmd_mqtt = mqtt;
		config.at_cmd_msg = msg;
		config.at_cmd_bluetooth = bluetooth;
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
#ifdef PPPOS
	else if (request->hasArg("commitPPPoS"))
	{
		bool pppEn = false;
		bool pppGnss = false;
		bool pppNapt = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "pppEn")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						pppEn = true;
					}
				}
			}

			if (request->argName(i) == "pppGnss")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						pppGnss = true;
					}
				}
			}

			if (request->argName(i) == "pppNapt")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						pppNapt = true;
					}
				}
			}

			if (request->argName(i) == "pppAPN")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.ppp_apn, request->arg(i).c_str());
				}
			}

			if (request->argName(i) == "pppPin")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.ppp_pin, request->arg(i).c_str());
				}
			}

			if (request->argName(i) == "rstDly")
			{
				if (request->arg(i) != "")
				{
					config.ppp_rst_delay = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "baudrate")
			{
				if (request->arg(i) != "")
				{
					config.ppp_serial_baudrate = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "port")
			{
				if (request->arg(i) != "")
				{
					config.ppp_serial = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "rx")
			{
				if (request->arg(i) != "")
				{
					config.ppp_rx_gpio = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "tx")
			{
				if (request->arg(i) != "")
				{
					config.ppp_tx_gpio = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "rst")
			{
				if (request->arg(i) != "")
				{
					config.ppp_rst_gpio = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "rst_active")
			{
				if (request->arg(i) != "")
				{
					config.ppp_rst_active = (bool)request->arg(i).toInt();
				}
			}

			// if (request->argName(i) == "pppSerial")
			// {
			// 	if (request->arg(i) != "")
			// 	{
			// 		if (isValidNumber(request->arg(i)))
			// 			config.ppp_serial = request->arg(i).toInt();
			// 	}
			// }
		}
		config.ppp_enable = pppEn;
		config.ppp_gnss = pppGnss;
		config.ppp_napt = pppNapt;
		if (config.ppp_enable)
		{
			if (config.ppp_serial == 0)
			{
				config.uart0_enable = false;
			}
			else if (config.ppp_serial == 1)
			{
				config.uart1_enable = false;
			}
			// else if (config.ppp_serial == 2)
			// {
			// 	config.uart2_enable = false;
			// }
		}
		char *html = allocateStringMemory(256); // Small buffer for success/failure message
		if (html != NULL)
		{
			if (saveConfiguration("/default.cfg", config))
			{
				strcpy(html, "Setup completed successfully");
				request->send(200, "text/html", html); // send to someones browser when asked
			}
			else
			{
				strcpy(html, "Save config failed.");
				request->send(501, "text/html", html); // Not Implemented
			}
			free(html);
		}
	}
#endif
	else
	{
		// Allocate memory for the HTML string
		char *html = allocateStringMemory(22000); // Start with 8KB, adjust as needed
		if (html == NULL)
		{
			request->send(500, "text/html", "Memory allocation failed");
			return;
		}

		// Initialize the HTML string with the JavaScript code
		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "if(e.currentTarget.id===\"formUART0\") document.getElementById(\"submitURAT0\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formUART1\") document.getElementById(\"submitURAT1\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formUART1\") document.getElementById(\"submitURAT1\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formGNSS\") document.getElementById(\"submitGNSS\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formMODBUS\") document.getElementById(\"submitMODBUS\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formTNC\") document.getElementById(\"submitTNC\").disabled=true;\n");
		// strcat(html, "if(e.currentTarget.id===\"formONEWIRE\") document.getElementById(\"submitONEWIRE\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formRF\") document.getElementById(\"submitRF\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formI2C0\") document.getElementById(\"submitI2C0\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formI2C1\") document.getElementById(\"submitI2C1\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formCOUNT0\") document.getElementById(\"submitCOUNT0\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formCOUNT1\") document.getElementById(\"submitCOUNT1\").disabled=true;\n");
#ifdef PPPOS
		strcat(html, "if(e.currentTarget.id===\"formPPPoS\") document.getElementById(\"submitPPPoS\").disabled=true;\n");
#endif
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/mod',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\\nRequire hardware RESET!\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "</script>\n");

		strcat(html, "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;vertical-align:top\"><td width=\"32%\" style=\"border:unset;\">\n");
		// strcat(html, "<h2>System Setting</h2>\n");
		/**************UART0(USB) Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromUART0\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>UART0 Modify</b></span></th>\n");
		strcat(html, "<tr>\n");

		char enFlage[20] = "";
		if (config.uart0_enable)
			strcpy(enFlage, "checked");
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		{
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ") + strlen(enFlage) + strlen("\"><span class=\"slider round\"></span></label></td>\n") + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", enFlage);
			strcat(html, temp_html);
			free(temp_html);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>RX GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rx_gpio_str = StringToCharPtr(String(config.uart0_rx_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"rx\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(rx_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"rx\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, rx_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rx_gpio_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>TX GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *tx_gpio_str = StringToCharPtr(String(config.uart0_tx_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"tx\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(tx_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"tx\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, tx_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(tx_gpio_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>RTS/DE GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rts_gpio_str = StringToCharPtr(String(config.uart0_rts_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\"  name=\"rts\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(rts_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\"  name=\"rts\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, rts_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rts_gpio_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Baudrate:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"baudrate\" id=\"baudrate\">\n");
		for (int i = 0; i < 13; i++)
		{
			char *baudrate_str = StringToCharPtr(String(baudrate[i]));
			if (config.uart0_baudrate == baudrate[i])
			{
				char *temp_html = allocateStringMemory(strlen("<option value=\"\" selected>  </option>\n") + 2 * strlen(baudrate_str) + 1);
				sprintf(temp_html, "<option value=\"%s\" selected>%s </option>\n", baudrate_str, baudrate_str);
				strcat(html, temp_html);
				free(temp_html);
			}
			else
			{
				char *temp_html = allocateStringMemory(strlen("<option value=\"\" >  </option>\n") + 2 * strlen(baudrate_str) + 1);
				sprintf(temp_html, "<option value=\"%s\" >%s </option>\n", baudrate_str, baudrate_str);
				strcat(html, temp_html);
				free(temp_html);
			}
			free(baudrate_str);
		}
		strcat(html, "</select> bps\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitUART0\" name=\"commitUART0\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitUART0\"/>\n");
		strcat(html, "</td></tr></table>\n");

		strcat(html, "</form><br />\n");
		strcat(html, "</td><td width=\"32%\" style=\"border:unset;\">");

		/**************UART1 Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromUART1\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>UART1 Modify</b></span></th>\n");
		strcat(html, "<tr>");

		if (config.uart1_enable)
			strcpy(enFlage, "checked");
		else
			strcpy(enFlage, "");
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		{
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ") + strlen(enFlage) + strlen("\"><span class=\"slider round\"></span></label></td>\n") + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", enFlage);
			strcat(html, temp_html);
			free(temp_html);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>RX GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rx_gpio_str = StringToCharPtr(String(config.uart1_rx_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"rx\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(rx_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"rx\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, rx_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rx_gpio_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>TX GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *tx_gpio_str = StringToCharPtr(String(config.uart1_tx_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"tx\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(tx_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"tx\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, tx_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(tx_gpio_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>RTS/DE GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rts_gpio_str = StringToCharPtr(String(config.uart1_rts_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\"  name=\"rts\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(rts_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\"  name=\"rts\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, rts_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rts_gpio_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Baudrate:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"baudrate\" id=\"baudrate\">\n");
		for (int i = 0; i < 13; i++)
		{
			char *baudrate_str = StringToCharPtr(String(baudrate[i]));
			if (config.uart1_baudrate == baudrate[i])
			{
				char *temp_html = allocateStringMemory(strlen("<option value=\"\" selected>  </option>\n") + 2 * strlen(baudrate_str) + 1);
				sprintf(temp_html, "<option value=\"%s\" selected>%s </option>\n", baudrate_str, baudrate_str);
				strcat(html, temp_html);
				free(temp_html);
			}
			else
			{
				char *temp_html = allocateStringMemory(strlen("<option value=\"\" >  </option>\n") + 2 * strlen(baudrate_str) + 1);
				sprintf(temp_html, "<option value=\"%s\" >%s </option>\n", baudrate_str, baudrate_str);
				strcat(html, temp_html);
				free(temp_html);
			}
			free(baudrate_str);
		}
		strcat(html, "</select> bps\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitUART1\" name=\"commitUART1\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitUART1\"/>\n");
		strcat(html, "</td></tr></table>\n");

		strcat(html, "</form><br />\n");
		// strcat(html, "</td><td width=\"32%\" style=\"border:unset;\">");

		/**************UART2 Modify******************/
		// strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromUART2\" method=\"post\">\n");
		// html += "<table>\n";
		// html += "<th colspan=\"2\"><span><b>UART2 Modify</b></span></th>\n";
		// html += "<tr>";

		// enFlage = "";
		// if (config.uart2_enable)
		// 	enFlage = "checked";
		// html += "<td align=\"right\"><b>Enable</b></td>\n";
		// html += "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" " + enFlage + "><span class=\"slider round\"></span></label></td>\n";
		// html += "</tr>\n";

		// html += "<tr>\n";
		// html += "<td align=\"right\"><b>RX GPIO:</b></td>\n";
		// html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"rx\" type=\"number\" value=\"" + String(config.uart2_rx_gpio) + "\" /></td>\n";
		// html += "</tr>\n";

		// html += "<tr>\n";
		// html += "<td align=\"right\"><b>TX GPIO:</b></td>\n";
		// html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\" name=\"tx\" type=\"number\" value=\"" + String(config.uart2_tx_gpio) + "\" /></td>\n";
		// html += "</tr>\n";

		// html += "<tr>\n";
		// html += "<td align=\"right\"><b>RTS/DE GPIO:</b></td>\n";
		// html += "<td style=\"text-align: left;\"><input min=\"-1\" max=\""+String(GPIO_NUM_MAX)+"\"  name=\"rts\" type=\"number\" value=\"" + String(config.uart2_rts_gpio) + "\" /></td>\n";
		// html += "</tr>\n";

		// html += "<tr>\n";
		// html += "<td align=\"right\"><b>Baudrate:</b></td>\n";
		// html += "<td style=\"text-align: left;\">\n";
		// html += "<select name=\"baudrate\" id=\"baudrate\">\n";
		// for (int i = 0; i < 13; i++)
		// {
		// 	if (config.uart2_baudrate == baudrate[i])
		// 		html += "<option value=\"" + String(baudrate[i]) + "\" selected>" + String(baudrate[i]) + " </option>\n";
		// 	else
		// 		html += "<option value=\"" + String(baudrate[i]) + "\" >" + String(baudrate[i]) + " </option>\n";
		// }
		// html += "</select> bps\n";
		// html += "</td>\n";
		// html += "</tr>\n";
		// html += "<tr><td colspan=\"2\" align=\"right\">\n";
		// html += "<input class=\"btn btn-primary\" id=\"submitUART2\" name=\"commitUART2\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n";
		// html += "<input type=\"hidden\" name=\"commitUART2\"/>\n";
		// html += "</td></tr></table>\n";

		// html += "</form><br />\n";
		// html += "</td></tr></table>\n";

		strcat(html, "</td><td width=\"32%\" style=\"border:unset;\">");

		/**************1-Wire Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromONEWIRE\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>1-Wire Bus Modify</b></span></th>\n");
		strcat(html, "<tr>");

		String syncFlage = "";
		if (config.onewire_enable)
			syncFlage = "checked";
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		{
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ") + strlen(syncFlage.c_str()) + strlen("\"><span class=\"slider round\"></span></label></td>\n") + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", syncFlage.c_str());
			strcat(html, temp_html);
			free(temp_html);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *onewire_gpio_str = StringToCharPtr(String(config.onewire_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"data\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(onewire_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"data\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, onewire_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(onewire_gpio_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitONEWIRE\" name=\"commitONEWIRE\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitONEWIRE\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form><br />\n");

		strcat(html, "</td></tr></table>\n");

		strcat(html, "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;vertical-align:top\"><td width=\"50%\" style=\"border:unset;vertical-align:top\">");
		/**************RF GPIO******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromRF\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>RF GPIO Modify</b></span></th>\n");
		strcat(html, "<tr>");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>ADC Attenuation:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"atten\" id=\"atten\">\n");
		for (int i = 0; i < 5; i++)
		{
			char *i_str = StringToCharPtr(String(i));
			char *atten_str = StringToCharPtr(String(ADC_ATTEN[i]));
			if (config.adc_atten == i)
			{
				char *temp_html = allocateStringMemory(strlen("<option value=\"\" selected>  </option>\n") + strlen(i_str) + strlen(atten_str) + 1);
				sprintf(temp_html, "<option value=\"%s\" selected>%s </option>\n", i_str, atten_str);
				strcat(html, temp_html);
				free(temp_html);
			}
			else
			{
				char *temp_html = allocateStringMemory(strlen("<option value=\"\" >  </option>\n") + strlen(i_str) + strlen(atten_str) + 1);
				sprintf(temp_html, "<option value=\"%s\" >%s </option>\n", i_str, atten_str);
				strcat(html, temp_html);
				free(temp_html);
			}
			free(i_str);
			free(atten_str);
		}
		{
			char *offset_str = StringToCharPtr(String(offset));
			char *temp_html = allocateStringMemory(strlen("</select> DC-Offset:  mV\n") + strlen(offset_str) + 1);
			sprintf(temp_html, "</select> DC-Offset: %s mV\n", offset_str);
			strcat(html, temp_html);
			free(temp_html);
			free(offset_str);
		}
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>UART2 Baudrate:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"baudrate\" id=\"baudrate\">\n");
		for (int i = 0; i < 13; i++)
		{
			char *baudrate_str = StringToCharPtr(String(baudrate[i]));
			if (config.rf_baudrate == baudrate[i])
			{
				char *temp_html = allocateStringMemory(strlen("<option value=\"\" selected>  </option>\n") + 2 * strlen(baudrate_str) + 1);
				sprintf(temp_html, "<option value=\"%s\" selected>%s </option>\n", baudrate_str, baudrate_str);
				strcat(html, temp_html);
				free(temp_html);
			}
			else
			{
				char *temp_html = allocateStringMemory(strlen("<option value=\"\" >  </option>\n") + 2 * strlen(baudrate_str) + 1);
				sprintf(temp_html, "<option value=\"%s\" >%s </option>\n", baudrate_str, baudrate_str);
				strcat(html, temp_html);
				free(temp_html);
			}
			free(baudrate_str);
		}
		strcat(html, "</select> bps\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		// html += "<tr>\n";
		// html += "<td align=\"right\"><b>ADC DC OFFSET:</b></td>\n";
		// html += "<td style=\"text-align: left;\"><input min=\"100\" max=\"2500\" name=\"offset\" type=\"number\" value=\"" + String(config.adc_dc_offset) + "\" /> mV     (Current: " + String(offset) + " mV)</td>\n";
		// strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>UART2 RX GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rf_rx_gpio_str = StringToCharPtr(String(config.rf_rx_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"rx\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(rf_rx_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"rx\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, rf_rx_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rf_rx_gpio_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>UART2 TX GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rf_tx_gpio_str = StringToCharPtr(String(config.rf_tx_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"tx\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(rf_tx_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"tx\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, rf_tx_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rf_tx_gpio_str);
		}
		strcat(html, "</tr>\n");

		char LowFlag[20] = "", HighFlag[20] = "";
		strcpy(LowFlag, "");
		strcpy(HighFlag, "");
		if (config.rf_pd_active)
			strcpy(HighFlag, "checked=\"checked\"");
		else
			strcpy(LowFlag, "checked=\"checked\"");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PD GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rf_pd_gpio_str = StringToCharPtr(String(config.rf_pd_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\"  name=\"pd\" type=\"number\" value=\"\" /> Active:<input type=\"radio\" name=\"pd_active\" value=\"0\"  />LOW <input type=\"radio\" name=\"pd_active\" value=\"1\"  />HIGH </td>\n") + strlen(gpio_max_str) + strlen(rf_pd_gpio_str) + strlen(LowFlag) + strlen(HighFlag) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\"  name=\"pd\" type=\"number\" value=\"%s\" /> Active:<input type=\"radio\" name=\"pd_active\" value=\"0\" %s/>LOW <input type=\"radio\" name=\"pd_active\" value=\"1\" %s/>HIGH </td>\n", gpio_max_str, rf_pd_gpio_str, LowFlag, HighFlag);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rf_pd_gpio_str);
		}
		strcat(html, "</tr>\n");

		sprintf(LowFlag, "");
		sprintf(HighFlag, "");
		if (config.rf_pwr_active)
			strcpy(HighFlag, "checked=\"checked\"");
		else
			strcpy(LowFlag, "checked=\"checked\"");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>H/L GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rf_pwr_gpio_str = StringToCharPtr(String(config.rf_pwr_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\"  name=\"pwr\" type=\"number\" value=\"\" /> Active:<input type=\"radio\" name=\"pwr_active\" value=\"0\"  />LOW <input type=\"radio\" name=\"pwr_active\" value=\"1\"  />HIGH </td>\n") + strlen(gpio_max_str) + strlen(rf_pwr_gpio_str) + strlen(LowFlag) + strlen(HighFlag) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\"  name=\"pwr\" type=\"number\" value=\"%s\" /> Active:<input type=\"radio\" name=\"pwr_active\" value=\"0\" %s/>LOW <input type=\"radio\" name=\"pwr_active\" value=\"1\" %s/>HIGH </td>\n", gpio_max_str, rf_pwr_gpio_str, LowFlag, HighFlag);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rf_pwr_gpio_str);
		}
		strcat(html, "</tr>\n");

		sprintf(LowFlag, "");
		sprintf(HighFlag, "");
		if (config.rf_sql_active)
			strcpy(HighFlag, "checked=\"checked\"");
		else
			strcpy(LowFlag, "checked=\"checked\"");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>SQL GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rf_sql_gpio_str = StringToCharPtr(String(config.rf_sql_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\"  name=\"sql\" type=\"number\" value=\"\" /> Active:<input type=\"radio\" name=\"sql_active\" value=\"0\"  />LOW <input type=\"radio\" name=\"sql_active\" value=\"1\"  />HIGH </td>\n") + strlen(gpio_max_str) + strlen(rf_sql_gpio_str) + strlen(LowFlag) + strlen(HighFlag) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\"  name=\"sql\" type=\"number\" value=\"%s\" /> Active:<input type=\"radio\" name=\"sql_active\" value=\"0\" %s/>LOW <input type=\"radio\" name=\"sql_active\" value=\"1\" %s/>HIGH </td>\n", gpio_max_str, rf_sql_gpio_str, LowFlag, HighFlag);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rf_sql_gpio_str);
		}
		strcat(html, "</tr>\n");

		sprintf(LowFlag, "");
		sprintf(HighFlag, "");
		if (config.rf_ptt_active)
			strcpy(HighFlag, "checked=\"checked\"");
		else
			strcpy(LowFlag, "checked=\"checked\"");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PTT GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *rf_ptt_gpio_str = StringToCharPtr(String(config.rf_ptt_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\"  name=\"ptt\" type=\"number\" value=\"\" /> Active:<input type=\"radio\" name=\"ptt_active\" value=\"0\"  />LOW <input type=\"radio\" name=\"ptt_active\" value=\"1\"  />HIGH </td>\n") + strlen(gpio_max_str) + strlen(rf_ptt_gpio_str) + strlen(LowFlag) + strlen(HighFlag) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\"  name=\"ptt\" type=\"number\" value=\"%s\" /> Active:<input type=\"radio\" name=\"ptt_active\" value=\"0\" %s/>LOW <input type=\"radio\" name=\"ptt_active\" value=\"1\" %s/>HIGH </td>\n", gpio_max_str, rf_ptt_gpio_str, LowFlag, HighFlag);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(rf_ptt_gpio_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitRF\" name=\"commitRF\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitRF\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form>\n");

		strcat(html, "</td><td width=\"23%\" style=\"border:unset;\">");

		/**************I2C_0 Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromI2C0\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>I2C_0(OLED) Modify</b></span></th>\n");
		strcat(html, "<tr>");

		syncFlage = "";
		if (config.i2c_enable)
			syncFlage = "checked";
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		{
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ") + strlen(syncFlage.c_str()) + strlen("\"><span class=\"slider round\"></span></label></td>\n") + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", syncFlage.c_str());
			strcat(html, temp_html);
			free(temp_html);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>SDA GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *i2c_sda_pin_str = StringToCharPtr(String(config.i2c_sda_pin));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"sda\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(i2c_sda_pin_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"sda\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, i2c_sda_pin_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(i2c_sda_pin_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>SCK GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *i2c_sck_pin_str = StringToCharPtr(String(config.i2c_sck_pin));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"sck\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(i2c_sck_pin_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"sck\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, i2c_sck_pin_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(i2c_sck_pin_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Frequency:</b></td>\n");
		{
			char *freq_str = StringToCharPtr(String(config.i2c_freq));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"1000\" max=\"800000\" name=\"freq\" type=\"number\" value=\"\" /></td>\n") + strlen(freq_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"1000\" max=\"800000\" name=\"freq\" type=\"number\" value=\"%s\" /></td>\n", freq_str);
			strcat(html, temp_html);
			free(temp_html);
			free(freq_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitI2C0\" name=\"commitI2C0\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitI2C0\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form>\n");

		/**************Counter_0 Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromCOUNTER0\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>Counter_0 Modify</b></span></th>\n");
		strcat(html, "<tr>");

		syncFlage = "";
		if (config.counter0_enable)
			syncFlage = "checked";
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		{
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ") + strlen(syncFlage.c_str()) + strlen("\"><span class=\"slider round\"></span></label></td>\n") + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", syncFlage.c_str());
			strcat(html, temp_html);
			free(temp_html);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>INPUT GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *counter0_gpio_str = StringToCharPtr(String(config.counter0_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"gpio\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(counter0_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"gpio\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, counter0_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(counter0_gpio_str);
		}
		strcat(html, "</tr>\n");

		sprintf(LowFlag, "");
		sprintf(HighFlag, "");
		if (config.counter0_active)
			strcpy(HighFlag, "checked=\"checked\"");
		else
			strcpy(LowFlag, "checked=\"checked\"");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\">Active</td>\n");
		{
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input type=\"radio\" name=\"active\" value=\"0\"  />LOW <input type=\"radio\" name=\"active\" value=\"1\"  />HIGH </td>\n") + strlen(LowFlag) + strlen(HighFlag) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input type=\"radio\" name=\"active\" value=\"0\" %s/>LOW <input type=\"radio\" name=\"active\" value=\"1\" %s/>HIGH </td>\n", LowFlag, HighFlag);
			strcat(html, temp_html);
			free(temp_html);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitCOUNTER0\" name=\"commitCOUNTER0\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitCOUNTER0\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form>\n");

		strcat(html, "</td><td width=\"23%\" style=\"border:unset;\">");
		/**************I2C_1 Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromI2C1\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>I2C_1 Modify</b></span></th>\n");
		strcat(html, "<tr>");

		syncFlage = "";
		if (config.i2c1_enable)
			syncFlage = "checked";
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		{
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ") + strlen(syncFlage.c_str()) + strlen("\"><span class=\"slider round\"></span></label></td>\n") + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", syncFlage.c_str());
			strcat(html, temp_html);
			free(temp_html);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>SDA GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *i2c1_sda_pin_str = StringToCharPtr(String(config.i2c1_sda_pin));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"sda\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(i2c1_sda_pin_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"sda\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, i2c1_sda_pin_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(i2c1_sda_pin_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>SCK GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *i2c1_sck_pin_str = StringToCharPtr(String(config.i2c1_sck_pin));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"sck\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(i2c1_sck_pin_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"sck\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, i2c1_sck_pin_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(i2c1_sck_pin_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Frequency:</b></td>\n");
		{
			char *freq_str = StringToCharPtr(String(config.i2c1_freq));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"1000\" max=\"800000\" name=\"freq\" type=\"number\" value=\"\" /></td>\n") + strlen(freq_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"1000\" max=\"800000\" name=\"freq\" type=\"number\" value=\"%s\" /></td>\n", freq_str);
			strcat(html, temp_html);
			free(temp_html);
			free(freq_str);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitI2C1\" name=\"commitI2C1\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitI2C1\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form>\n");

		/**************Counter_1 Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromCOUNTER1\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>Counter_1 Modify</b></span></th>\n");
		strcat(html, "<tr>");

		syncFlage = "";
		if (config.counter1_enable)
			syncFlage = "checked";
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		{
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ") + strlen(syncFlage.c_str()) + strlen("\"><span class=\"slider round\"></span></label></td>\n") + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", syncFlage.c_str());
			strcat(html, temp_html);
			free(temp_html);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>INPUT GPIO:</b></td>\n");
		{
			char *gpio_max_str = StringToCharPtr(String(GPIO_NUM_MAX));
			char *counter1_gpio_str = StringToCharPtr(String(config.counter1_gpio));
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input min=\"-1\" max=\"\" name=\"gpio\" type=\"number\" value=\"\" /></td>\n") + strlen(gpio_max_str) + strlen(counter1_gpio_str) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"%s\" name=\"gpio\" type=\"number\" value=\"%s\" /></td>\n", gpio_max_str, counter1_gpio_str);
			strcat(html, temp_html);
			free(temp_html);
			free(gpio_max_str);
			free(counter1_gpio_str);
		}
		strcat(html, "</tr>\n");

		sprintf(LowFlag, "");
		sprintf(HighFlag, "");
		if (config.counter1_active)
			strcpy(HighFlag, "checked=\"checked\"");
		else
			strcpy(LowFlag, "checked=\"checked\"");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\">Active</td>\n");
		{
			char *temp_html = allocateStringMemory(strlen("<td style=\"text-align: left;\"><input type=\"radio\" name=\"active\" value=\"0\"  />LOW <input type=\"radio\" name=\"active\" value=\"1\"  />HIGH </td>\n") + strlen(LowFlag) + strlen(HighFlag) + 1);
			sprintf(temp_html, "<td style=\"text-align: left;\"><input type=\"radio\" name=\"active\" value=\"0\" %s/>LOW <input type=\"radio\" name=\"active\" value=\"1\" %s/>HIGH </td>\n", LowFlag, HighFlag);
			strcat(html, temp_html);
			free(temp_html);
		}
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitCOUNTER1\" name=\"commitCOUNTER1\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitCOUNTER1\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form>\n");

		strcat(html, "</td></tr></table>\n");
		strcat(html, "<br />\n");

		//******************
		strcat(html, "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;vertical-align:top\"><td width=\"50%\" style=\"border:unset;vertical-align:top\">");
		/**************GNSS Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromGNSS\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>GNSS Modify</b></span></th>\n");
		strcat(html, "<tr>\n");

		strcpy(enFlage, "");
		if (config.gnss_enable)
			strcpy(enFlage, "checked");
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ");
		strcat(html, enFlage);
		strcat(html, "><span class=\"slider round\"></span></label></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PORT:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"channel\" id=\"channel\">\n");
		for (int i = 0; i < 5; i++)
		{
			strcat(html, "<option value=\"");
			char *iStr2 = intToString(i);
			strcat(html, iStr2);
			strcat(html, "\" ");
			if (config.gnss_channel == i)
			{
				strcat(html, "selected>");
				strcat(html, GNSS_PORT[i]);
				strcat(html, " </option>\n");
			}
			else
			{
				strcat(html, ">");
				strcat(html, GNSS_PORT[i]);
				strcat(html, " </option>\n");
			}
			// Free temporary string
			free(iStr2);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<td align=\"right\"><b>AT Command:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input maxlength=\"30\" size=\"20\" id=\"atc\" name=\"atc\" type=\"text\" value=\"");
		strcat(html, config.gnss_at_command);
		strcat(html, "\" /></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<td align=\"right\"><b>TCP Host:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input maxlength=\"20\" size=\"15\" id=\"Host\" name=\"Host\" type=\"text\" value=\"");
		strcat(html, config.gnss_tcp_host);
		strcat(html, "\" /></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>TCP Port:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input min=\"1024\" max=\"65535\"  id=\"Port\" name=\"Port\" type=\"number\" value=\"");
		char *gnssTcpPortStr = intToString(config.gnss_tcp_port);
		strcat(html, gnssTcpPortStr);
		strcat(html, "\" /></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitGNSS\" name=\"commitGNSS\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitGNSS\"/>\n");
		strcat(html, "</td></tr></table>\n");

		strcat(html, "</form><br />\n");

		strcat(html, "</td><td width=\"23%\" style=\"border:unset;\">\n");

		/**************MODBUS Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromMODBUS\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>MODBUS Modify</b></span></th>\n");
		strcat(html, "<tr>\n");

		strcpy(enFlage, "");
		if (config.modbus_enable)
			strcpy(enFlage, "checked");
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ");
		strcat(html, enFlage);
		strcat(html, "><span class=\"slider round\"></span></label></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PORT:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"channel\" id=\"channel\">\n");
		for (int i = 0; i < 5; i++)
		{
			strcat(html, "<option value=\"");
			char *iStr3 = intToString(i);
			strcat(html, iStr3);
			strcat(html, "\" ");
			if (config.modbus_channel == i)
			{
				strcat(html, "selected>");
				strcat(html, GNSS_PORT[i]);
				strcat(html, " </option>\n");
			}
			else
			{
				strcat(html, ">");
				strcat(html, GNSS_PORT[i]);
				strcat(html, " </option>\n");
			}
			// Free temporary string
			free(iStr3);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Address:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"");
		char *gpioMaxStr20 = intToString(GPIO_NUM_MAX);
		strcat(html, gpioMaxStr20);
		strcat(html, "\" name=\"address\" type=\"number\" value=\"");
		char *modbusAddrStr = intToString(config.modbus_address);
		strcat(html, modbusAddrStr);
		strcat(html, "\" /></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>DE:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"");
		char *gpioMaxStr21 = intToString(GPIO_NUM_MAX);
		strcat(html, gpioMaxStr21);
		strcat(html, "\" name=\"de\" type=\"number\" value=\"");
		char *modbusDeStr = intToString(config.modbus_de_gpio);
		strcat(html, modbusDeStr);
		strcat(html, "\" /></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitMODBUS\" name=\"commitMODBUS\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitMODBUS\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form>\n");

		strcat(html, "</td><td width=\"23%\" style=\"border:unset;\">\n");

		/**************External TNC Modify******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"fromTNC\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>External TNC Modify</b></span></th>\n");
		strcat(html, "<tr>\n");

		strcpy(enFlage, "");
		if (config.ext_tnc_enable)
			strcpy(enFlage, "checked");
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" ");
		strcat(html, enFlage);
		strcat(html, "><span class=\"slider round\"></span></label></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PORT:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"channel\" id=\"channel\">\n");
		for (int i = 0; i < 5; i++)
		{
			strcat(html, "<option value=\"");
			char *iStr4 = intToString(i);
			strcat(html, iStr4);
			strcat(html, "\" ");
			if (config.ext_tnc_channel == i)
			{
				strcat(html, "selected>");
				strcat(html, TNC_PORT[i]);
				strcat(html, " </option>\n");
			}
			else
			{
				strcat(html, ">");
				strcat(html, TNC_PORT[i]);
				strcat(html, " </option>\n");
			}
			// Free temporary string
			free(iStr4);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>MODE:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"mode\" id=\"mode\">\n");
		for (int i = 0; i < 4; i++)
		{
			strcat(html, "<option value=\"");
			char *iStr5 = intToString(i);
			strcat(html, iStr5);
			strcat(html, "\" ");
			if (config.ext_tnc_mode == i)
			{
				strcat(html, "selected>");
				strcat(html, TNC_MODE[i]);
				strcat(html, " </option>\n");
			}
			else
			{
				strcat(html, ">");
				strcat(html, TNC_MODE[i]);
				strcat(html, " </option>\n");
			}
			// Free temporary string
			free(iStr5);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<input class=\"button\" id=\"submitTNC\" name=\"commitTNC\" type=\"submit\" value=\"Apply\" maxlength=\"80\"/>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitTNC\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "<br />\n");

		strcat(html, "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;vertical-align:top\"><td width=\"50%\" style=\"border:unset;vertical-align:top\">\n");

		/************************ AT-COMMAND **************************/
		strcat(html, "<form id='formATCommand' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>AT-COMMAND CHANNEL</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td width=\"150\" align=\"right\"><b>MQTT:</b></td>\n");
		char cmdFlag[20] = "";
		if (config.at_cmd_mqtt)
			strcpy(cmdFlag, "checked");
		strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"mqtt\" value=\"OK\" ");
		strcat(html, cmdFlag);
		strcat(html, "><span class=\"slider round\"></span></label></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>MESSAGE:</b></td>\n");
		strcpy(cmdFlag, "");
		if (config.at_cmd_msg)
			strcpy(cmdFlag, "checked");
		strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"msg\" value=\"OK\" ");
		strcat(html, cmdFlag);
		strcat(html, "><span class=\"slider round\"></span></label></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>BLUETOOTH:</b></td>\n");
		strcpy(cmdFlag, "");
		if (config.at_cmd_bluetooth)
			strcpy(cmdFlag, "checked");
		strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"bluetooth\" value=\"OK\" ");
		strcat(html, cmdFlag);
		strcat(html, "><span class=\"slider round\"></span></label></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>UART PORT:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"uart\" id=\"cmdUart\">\n");
		for (int i = 0; i < 5; i++)
		{
			strcat(html, "<option value=\"");
			char *iStr6 = intToString(i);
			strcat(html, iStr6);
			strcat(html, "\" ");
			if (config.at_cmd_uart == i)
			{
				strcat(html, "selected>");
				strcat(html, TNC_PORT[i]);
				strcat(html, " </option>\n");
			}
			else
			{
				strcat(html, ">");
				strcat(html, TNC_PORT[i]);
				strcat(html, " </option>\n");
			}
			// Free temporary string
			free(iStr6);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitCMD'  name=\"commit\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitCMD\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br />\n");

#ifdef PPPOS
		strcat(html, "<br />\n");

		strcat(html, "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;vertical-align:top\"><td width=\"50%\" style=\"border:unset;vertical-align:top\">\n");

		/************************ PPPoS **************************/

		strcat(html, "<form id='formPPPoS' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>PPP Over Serial (GSM/4G-LTE)</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Enable:</b></td>\n");
		char pppEnFlag[20] = "";
		if (config.ppp_enable)
			strcpy(pppEnFlag, "checked");
		strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"pppEn\" value=\"OK\" ");
		strcat(html, pppEnFlag);
		strcat(html, "><span class=\"slider round\"></span></label></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<td align=\"right\"><b>GNSS:</b></td>\n");
		strcpy(pppEnFlag, "");
		if (config.ppp_gnss)
			strcpy(pppEnFlag, "checked");
		strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"pppGnss\" value=\"OK\" ");
		strcat(html, pppEnFlag);
		strcat(html, "><span class=\"slider round\"></span></label></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>NAPT:</b></td>\n");
		strcpy(pppEnFlag, "");
		if (config.ppp_napt)
			strcpy(pppEnFlag, "checked");
		strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"pppNapt\" value=\"OK\" ");
		strcat(html, pppEnFlag);
		strcat(html, "><span class=\"slider round\"></span></label> *WiFi NAT</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>APN:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input maxlength=\"20\" name=\"pppAPN\" type=\"text\" value=\"");
		strcat(html, config.ppp_apn);
		strcat(html, "\" /></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");

		strcat(html, "<td align=\"right\"><b>PIN:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input min=\"0\" max=\"999999\" name=\"pppPin\" type=\"number\" value=\"");
		char *pppPinStr = intToString(atoi(config.ppp_pin));
		strcat(html, pppPinStr);
		strcat(html, "\" /> <i>*PIN of SIM</i></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");

		strcat(html, "<td align=\"right\"><b>RX GPIO:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"50\" name=\"rx\" type=\"number\" value=\"");
		char *pppRxStr = intToString(config.ppp_rx_gpio);
		strcat(html, pppRxStr);
		strcat(html, "\" /></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>TX GPIO:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"50\" name=\"tx\" type=\"number\" value=\"");
		char *pppTxStr = intToString(config.ppp_tx_gpio);
		strcat(html, pppTxStr);
		strcat(html, "\" /></td>\n");
		strcat(html, "</tr>\n");

		strcpy(LowFlag, "");
		strcpy(HighFlag, "");
		if (config.ppp_rst_active)
			strcpy(HighFlag, "checked=\"checked\"");
		else
			strcpy(LowFlag, "checked=\"checked\"");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>RESET GPIO:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input min=\"-1\" max=\"50\"  name=\"rst\" type=\"number\" value=\"");
		char *pppRstStr = intToString(config.ppp_rst_gpio);
		strcat(html, pppRstStr);
		strcat(html, "\" /> Active:<input type=\"radio\" name=\"rst_active\" value=\"0\" ");
		strcat(html, LowFlag);
		strcat(html, "/>LOW <input type=\"radio\" name=\"rst_active\" value=\"1\" ");
		strcat(html, HighFlag);
		strcat(html, "/>HIGH </td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<td align=\"right\"><b>RESET DELAY:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input min=\"0\" max=\"999999\" name=\"rstDly\" type=\"number\" value=\"");
		char *pppRstDelayStr = intToString(config.ppp_rst_delay);
		strcat(html, pppRstDelayStr);
		strcat(html, "\" /> mSec.</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PORT:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"port\" id=\"port\">\n");
		for (int i = 0; i < 2; i++)
		{
			strcat(html, "<option value=\"");
			char *iStr7 = intToString(i);
			strcat(html, iStr7);
			strcat(html, "\" ");
			if (config.ppp_serial == i)
			{
				strcat(html, "selected>");
				strcat(html, GNSS_PORT[i + 1]);
				strcat(html, " </option>\n");
			}
			else
			{
				strcat(html, ">");
				strcat(html, GNSS_PORT[i + 1]);
				strcat(html, " </option>\n");
			}
			// Free temporary string
			free(iStr7);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Baudrate:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"baudrate\" id=\"baudrate\">\n");
		for (int i = 0; i < 13; i++)
		{
			strcat(html, "<option value=\"");
			char *baudrateStr3 = intToString(baudrate[i]);
			strcat(html, baudrateStr3);
			strcat(html, "\" ");
			if (config.ppp_serial_baudrate == baudrate[i])
			{
				strcat(html, "selected>");
				strcat(html, baudrateStr3);
				strcat(html, " </option>\n");
			}
			else
			{
				strcat(html, ">");
				strcat(html, baudrateStr3);
				strcat(html, " </option>\n");
			}
			// Free temporary string
			free(baudrateStr3);
		}
		strcat(html, "</select> bps\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitPPPoS'  name=\"commitPPPoS\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitPPPoS\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form>\n");

		strcat(html, "</td></tr></table>\n");
#endif

		AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, html);
		response->addHeader("MOD", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
	}
}

void handle_system(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	if (request->hasArg("updateTimeZone"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "SetTimeZone")
			{
				if (request->arg(i) != "")
				{
					config.timeZone = request->arg(i).toFloat();
					// Serial.println("WEB Config Time Zone);
					configTime(3600 * config.timeZone, 0, config.ntp_host);
				}
				break;
			}
		}
		saveConfig(request);
	}
	else if (request->hasArg("updateHostName"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			log_d("%s", request->arg(i).c_str());
			if (request->argName(i) == "SetHostName")
			{
				if (request->arg(i) != "")
				{
					strncpy(config.host_name, request->arg(i).c_str(), sizeof(config.host_name) - 1);
					config.host_name[sizeof(config.host_name) - 1] = '\0'; // Null terminate
				}
				break;
			}
		}
		saveConfig(request);
	}
	else if (request->hasArg("updateTimeNtp"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "SetTimeNtp")
			{
				if (request->arg(i) != "")
				{
					// Serial.println("WEB Config NTP");
					strcpy(config.ntp_host, request->arg(i).c_str());
					configTime(3600 * config.timeZone, 0, config.ntp_host);
				}
				break;
			}
		}
		saveConfig(request);
	}
	else if (request->hasArg("updateAutoReset"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{

			if (request->argName(i) == "SetAutoReset")
			{
				if (request->arg(i) != "")
				{
					config.reset_timeout = request->arg(i).toInt();
				}
				break;
			}
		}
		saveConfig(request);
	}
	else if (request->hasArg("updateTime"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "SetTime")
			{
				if (request->arg(i) != "")
				{
					// struct tm tmn;
					String date = getValue(request->arg(i), ' ', 0);
					String time = getValue(request->arg(i), ' ', 1);
					int yyyy = getValue(date, '-', 0).toInt();
					int mm = getValue(date, '-', 1).toInt();
					int dd = getValue(date, '-', 2).toInt();
					int hh = getValue(time, ':', 0).toInt();
					int ii = getValue(time, ':', 1).toInt();
					int ss = getValue(time, ':', 2).toInt();
					// int ss = 0;

					tmElements_t timeinfo;
					timeinfo.Year = yyyy - 1970;
					timeinfo.Month = mm;
					timeinfo.Day = dd;
					timeinfo.Hour = hh;
					timeinfo.Minute = ii;
					timeinfo.Second = ss;
					time_t timeStamp = makeTime(timeinfo);

					// tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec

					time_t rtc = timeStamp - (config.timeZone * 3600);
					timeval tv = {rtc, 0};
					timezone tz = {(0) + DST_MN, 0};
					settimeofday(&tv, &tz);

					// Serial.println("Update TIME " + request->arg(i));
					// Serial.print("Set New Time at ");
					// Serial.print(dd);
					// Serial.print("/");
					// Serial.print(mm);
					// Serial.print("/");
					// Serial.print(yyyy);
					// Serial.print(" ");
					// Serial.print(hh);
					// Serial.print(":");
					// Serial.print(ii);
					// Serial.print(":");
					// Serial.print(ss);
					// Serial.print(" ");
					// Serial.println(timeStamp);
				}
				break;
			}
		}
		saveConfig(request);
	}
	else if (request->hasArg("REBOOT"))
	{
		TLM_SEQ = 0;
		IGATE_TLM_SEQ = 0;
		DIGI_TLM_SEQ = 0;
		esp_restart();
	}
	else if (request->hasArg("Factory"))
	{
		defaultConfig();
	}
	else if (request->hasArg("LoadCFG"))
	{
		if (loadConfiguration("/default.cfg", config))
		{
			String html = "OK";
			request->send(200, "text/html", html);
		}
	}
	else if (request->hasArg("commitWebAuth"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "webauth_user")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.http_username, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "webauth_pass")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.http_password, request->arg(i).c_str());
				}
			}
		}
		saveConfig(request);
	}
	else if (request->hasArg("commitPath"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "path1")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.path[0], request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "path2")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.path[1], request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "path3")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.path[2], request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "path4")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.path[3], request->arg(i).c_str());
				}
			}
		}
		saveConfig(request);
	}
	else if (request->hasArg("commitPWR"))
	{
		bool PwrEn = false;
		config.pwr_sleep_activate = 0;

		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "pwr_active")
			{
				if (request->arg(i) != "")
				{
					config.pwr_active = (bool)request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
					{
						PwrEn = true;
					}
				}
			}
			if (request->argName(i) == "pwr")
			{
				if (request->arg(i) != "")
				{
					config.pwr_gpio = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "sleep")
			{
				if (request->arg(i) != "")
				{
					config.pwr_sleep_interval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "stb")
			{
				if (request->arg(i) != "")
				{
					config.pwr_stanby_delay = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "mode")
			{
				if (request->arg(i) != "")
				{
					config.pwr_mode = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "FilterTelemetry")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.pwr_sleep_activate |= ACTIVATE_TELEMETRY;
				}
			}

			if (request->argName(i) == "FilterStatus")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.pwr_sleep_activate |= ACTIVATE_STATUS;
				}
			}

			if (request->argName(i) == "FilterWeather")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.pwr_sleep_activate |= ACTIVATE_WX;
				}
			}

			if (request->argName(i) == "FilterTracker")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.pwr_sleep_activate |= ACTIVATE_TRACKER;
				}
			}

			if (request->argName(i) == "FilterIGate")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.pwr_sleep_activate |= ACTIVATE_IGATE;
				}
			}

			if (request->argName(i) == "FilterDigi")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.pwr_sleep_activate |= ACTIVATE_DIGI;
				}
			}

			if (request->argName(i) == "FilterQuery")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.pwr_sleep_activate |= ACTIVATE_QUERY;
				}
			}

			if (request->argName(i) == "FilterWifi")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.pwr_sleep_activate |= ACTIVATE_WIFI;
				}
			}
		}
		config.pwr_en = PwrEn;
		saveConfig(request);
	}
	else if (request->hasArg("commitLOG"))
	{
		bool PwrEn = false;
		config.log = 0;

		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "logStatus")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.log |= LOG_STATUS;
				}
			}

			if (request->argName(i) == "logWeather")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.log |= LOG_WX;
				}
			}

			if (request->argName(i) == "logTracker")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.log |= LOG_TRACKER;
				}
			}

			if (request->argName(i) == "logIgate")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.log |= LOG_IGATE;
				}
			}

			if (request->argName(i) == "logDigi")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.log |= LOG_DIGI;
				}
			}
		}
		saveConfig(request);
	}
	else if (request->hasArg("commitDISP"))
	{
		bool dispRX = false;
		bool dispTX = false;
		bool dispRF = false;
		bool dispINET = false;
		bool oledEN = false;
		bool dispFlip = false;

		config.dispFilter = 0;

		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "oledEnable")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
					{
						oledEN = true;
					}
				}
			}
			if (request->argName(i) == "dispFlip")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
					{
						dispFlip = true;
					}
				}
			}
			if (request->argName(i) == "filterMessage")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.dispFilter |= FILTER_MESSAGE;
				}
			}

			if (request->argName(i) == "filterTelemetry")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.dispFilter |= FILTER_TELEMETRY;
				}
			}

			if (request->argName(i) == "filterStatus")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.dispFilter |= FILTER_STATUS;
				}
			}

			if (request->argName(i) == "filterWeather")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.dispFilter |= FILTER_WX;
				}
			}

			if (request->argName(i) == "filterObject")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.dispFilter |= FILTER_OBJECT;
				}
			}

			if (request->argName(i) == "filterItem")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.dispFilter |= FILTER_ITEM;
				}
			}

			if (request->argName(i) == "filterQuery")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.dispFilter |= FILTER_QUERY;
				}
			}
			if (request->argName(i) == "filterBuoy")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.dispFilter |= FILTER_BUOY;
				}
			}
			if (request->argName(i) == "filterPosition")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.dispFilter |= FILTER_POSITION;
				}
			}

			if (request->argName(i) == "dispRF")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						dispRF = true;
				}
			}

			if (request->argName(i) == "dispINET")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						dispINET = true;
				}
			}
			if (request->argName(i) == "txdispEnable")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						dispTX = true;
				}
			}
			if (request->argName(i) == "rxdispEnable")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						dispRX = true;
				}
			}

			if (request->argName(i) == "dispBright")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
					{
						config.disp_brightness = request->arg(i).toInt();
#ifdef ST7735_LED_K_Pin
						ledcWrite(0, (uint32_t)config.disp_brightness);
#endif
					}
				}
			}

			if (request->argName(i) == "dispDelay")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
					{
						config.dispDelay = request->arg(i).toInt();
						if (config.dispDelay < 0)
							config.dispDelay = 0;
					}
				}
			}

			if (request->argName(i) == "oled_timeout")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
					{
						config.oled_timeout = request->arg(i).toInt();
						if (config.oled_timeout < 0)
							config.oled_timeout = 0;
					}
				}
			}
			if (request->argName(i) == "filterDX")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
					{
						config.filterDistant = request->arg(i).toInt();
					}
				}
			}
		}

#ifdef OLED
		if (oledEN && !config.oled_enable)
		{
			// display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false); // initialize with the I2C addr 0x3C (for the 128x64)
			//  Initialising the UI will init the display too.
			// #ifdef SH1106
			// 			display.begin(SH1106_SWITCHCAPVCC, SCREEN_ADDRESS, false);
			// #else
			// 			display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS, false, false);
			// #endif
			//			display.clearDisplay();
		}
		config.oled_enable = oledEN;
		config.dispINET = dispINET;
		config.dispRF = dispRF;
		config.rx_display = dispRX;
		config.tx_display = dispTX;
		config.disp_flip = dispFlip;
#endif // OLED
	   // config.filterMessage = filterMessage;
	   // config.filterStatus = filterStatus;
	   // config.filterTelemetry = filterTelemetry;
	   // config.filterWeather = filterWeather;
	   // config.filterTracker = filterTracker;
	   // config.filterMove = filterMove;
	   // config.filterPosition = filterPosition;
		saveConfig(request);
	}
	else
	{
		struct tm tmstruct;
		char strTime[30];
		tmstruct.tm_year = 0;
		getLocalTime(&tmstruct, 100);
		sprintf(strTime, "%d-%02d-%02d %02d:%02d:%02d", (tmstruct.tm_year) + 1900, (tmstruct.tm_mon) + 1, tmstruct.tm_mday, tmstruct.tm_hour, tmstruct.tm_min, tmstruct.tm_sec);

		// Using dynamic memory allocation instead of String
		char *html = allocateStringMemory(20000); // Initial buffer size, adjust as needed
		if (!html)
		{
			return; // Memory allocation failed
		}

		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "if(e.currentTarget.id===\"formHostName\") document.getElementById(\"updateHostName\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formTime\") document.getElementById(\"updateTime\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formNTP\") document.getElementById(\"updateTimeNtp\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formTimeZone\") document.getElementById(\"updateTimeZone\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formReboot\") document.getElementById(\"REBOOT\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formDisp\") document.getElementById(\"submitDISP\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formWebAuth\") document.getElementById(\"submitWebAuth\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formPWR\") document.getElementById(\"submitPWR\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formPath\") document.getElementById(\"submitPath\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/system',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "</script>\n");

		// html += "<h2>System Setting</h2>\n";
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>System Setting</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\">Host Name:</td>\n");

		// Building form with snprintf to avoid string concatenation
		char temp_buffer[300];
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formHostName\" method=\"post\"><input name=\"SetHostName\" type=\"text\" value=\"%s\" />\n", config.host_name);
		strcat(html, temp_buffer);
		strcat(html, "<button type='submit' id='updateHostName'  name=\"updateHostName\"> Apply </button>\n");
		strcat(html, "<input type=\"hidden\" name=\"updateHostName\"/></form>\n</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>");
		// html += "<form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formTime\" method=\"post\">\n";
		strcat(html, "<td style=\"text-align: right;\">LOCAL DATE/TIME </td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formTime\" method=\"post\">\n<input name=\"SetTime\" type=\"text\" value=\"%s\" />\n", strTime);
		strcat(html, temp_buffer);
		strcat(html, "<span class=\"input-group-addon\">\n<span class=\"glyphicon glyphicon-calendar\">\n</span></span>\n");
		// html += "<div class=\"col-sm-3 col-xs-6\"><button class=\"btn btn-primary\" data-args=\"[true]\" data-method=\"getDate\" type=\"button\" data-related-target=\"#SetTime\" />Get Date</button></div>\n");
		strcat(html, "<button type='submit' id='updateTime'  name=\"commit\"> Time Update </button>\n");
		strcat(html, "<input type=\"hidden\" name=\"updateTime\"/></form>\n</td>\n");
		// html += "<input class=\"btn btn-primary\" id=\"updateTime\" name=\"updateTime\" type=\"submit\" value=\"Time Update\" maxlength=\"80\"/></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\">NTP Host </td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formNTP\" method=\"post\"><input name=\"SetTimeNtp\" type=\"text\" value=\"%s\" />\n", config.ntp_host);
		strcat(html, temp_buffer);
		strcat(html, "<button type='submit' id='updateTimeNtp'  name=\"commit\"> NTP Update </button>\n");
		strcat(html, "<input type=\"hidden\" name=\"updateTimeNtp\"/></form>\n</td>\n");
		// html += "<input class=\"btn btn-primary\" id=\"updateTimeNtp\" name=\"updateTimeNtp\" type=\"submit\" value=\"NTP Update\" maxlength=\"80\"/></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\">Auto REBOOT:</td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formAutoReset\" method=\"post\"><input  min=\"0\" max=\"65535\"  name=\"SetAutoReset\" type=\"number\" value=\"%d\" /> Minutes\n", config.reset_timeout);
		strcat(html, temp_buffer);
		strcat(html, "<button type='submit' id='updateAutoReset'  name=\"commit\"> Update </button> *<i>0=No reset</i>\n");
		strcat(html, "<input type=\"hidden\" name=\"updateAutoReset\"/></form>\n</td>\n");
		// html += "<input class=\"button\" id=\"updateTimeNtp\" name=\"updateTimeNtp\" type=\"submit\" value=\"NTP Update\" maxlength=\"80\"/></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\">Time Zone </td>\n");
		strcat(html, "<td style=\"text-align: left;\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formTimeZone\" method=\"post\">\n");
		strcat(html, "<select name=\"SetTimeZone\" id=\"SetTimeZone\">\n");
		for (int i = 0; i < 40; i++)
		{
			if (config.timeZone == tzList[i].tz)
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%.1f\" selected>%s Sec</option>\n", tzList[i].tz, tzList[i].name);
			}
			else
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%.1f\" >%s Sec</option>\n", tzList[i].tz, tzList[i].name);
			}
			strcat(html, temp_buffer);
		}
		strcat(html, "</select>");
		strcat(html, "<button type='submit' id='updateTimeZone'  name=\"commit\"> TZ Update </button>\n");
		strcat(html, "<input type=\"hidden\" name=\"updateTimeZone\"/></form>\n</td>\n");
		// html += "<input class=\"btn btn-primary\" id=\"updateTimeZone\" name=\"updateTimeZone\" type=\"submit\" value=\"TZ Update\" maxlength=\"80\"/></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\">SYSTEM CONTROL </td>\n");
		strcat(html, "<td style=\"text-align: left;\"><table><tr><td width=\"100\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formReboot\" method=\"post\"> <button type='submit' id='REBOOT'  name=\"commit\" style=\"background-color:red;color:white\"> REBOOT </button>\n");
		strcat(html, " <input type=\"hidden\" name=\"REBOOT\"/></form></td><td width=\"100\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formFactory\" method=\"post\"> <button type='submit' id='Factory'  name=\"commit\" style=\"background-color:orange;color:white\"> Factory Reset </button>\n");
		strcat(html, " <input type=\"hidden\" name=\"Factory\"/></form></td><td width=\"100\"><form accept-charset=\"UTF-8\" action=\"#\" enctype='multipart/form-data' id=\"formLoad\" method=\"post\"> <button type='submit' id='LoadCFG'  name=\"commit\" style=\"background-color:green;color:white\"> Load Default </button>\n");
		strcat(html, " <input type=\"hidden\" name=\"LoadCFG\"/></form></td></tr></table></td>\n");
		// html += "<td style=\"text-align: left;\"><input type='submit' class=\"btn btn-danger\" id=\"REBOOT\" name=\"REBOOT\" value='REBOOT'></td>\n");
		strcat(html, "</tr></table><br /><br />\n");

		/************************ WEB AUTH **************************/
		strcat(html, "<form id='formWebAuth' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>Web Authentication</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Web USER:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input size=\"32\" maxlength=\"32\" class=\"form-control\" name=\"webauth_user\" type=\"text\" value=\"%s\" /></td>\n", config.http_username);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Web PASSWORD:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input size=\"63\" maxlength=\"63\" class=\"form-control\" name=\"webauth_pass\" type=\"password\" value=\"%s\" /></td>\n", config.http_password);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitWebAuth'  name=\"commit\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitWebAuth\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br /><br />");

		/**************Power Mode******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"formPWR\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>Power Save Mode</b></span></th>\n");
		strcat(html, "<tr>");

		char enFlage[10] = "";
		if (config.pwr_en)
			strcpy(enFlage, "checked");
		strcat(html, "<td align=\"right\"><b>Enable</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", enFlage);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		char LowFlag[20] = "", HighFlag[20] = "";
		strcpy(LowFlag, "");
		strcpy(HighFlag, "");
		if (config.pwr_active)
			strcpy(HighFlag, "checked=\"checked\"");
		else
			strcpy(LowFlag, "checked=\"checked\"");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PWR GPIO:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input min=\"-1\" max=\"50\"  name=\"pwr\" type=\"number\" value=\"%d\" /> Output Active:<input type=\"radio\" name=\"pwr_active\" value=\"0\" %s/>LOW <input type=\"radio\" name=\"pwr_active\" value=\"1\" %s/>HIGH </td>\n", config.pwr_gpio, LowFlag, HighFlag);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Sleep Interval:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input min=\"0\" max=\"9999\" name=\"sleep\" type=\"number\" value=\"%d\" /></td>\n", config.pwr_sleep_interval);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>StandBy Delay:</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><input min=\"0\" max=\"9999\" name=\"stb\" type=\"number\" value=\"%d\" /></td>\n", config.pwr_stanby_delay);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Power Mode:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"mode\" id=\"mode\">\n");
		for (int i = 0; i < 3; i++)
		{
			if (config.pwr_mode == i)
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" selected>%s </option>\n", i, PWR_MODE[i]);
			}
			else
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" >%s </option>\n", i, PWR_MODE[i]);
			}
			strcat(html, temp_buffer);
		}
		strcat(html, "</select> A=Reduce Speed(PWR Off),B=Light Sleep(WiFi/PWR Off),C=Deep Sleep(All Off)\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Event Activate:</b><br/>(For Mode C)</td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<fieldset id=\"FilterGrp\">\n");
		strcat(html, "<legend>Events</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">\n");
		strcat(html, "<tr style=\"background:unset;\">");

		char filterFlageEn[10] = "";
		if (config.pwr_sleep_activate & ACTIVATE_TRACKER)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterTracker\" type=\"checkbox\" value=\"OK\" %s/>Tracker</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.pwr_sleep_activate & ACTIVATE_STATUS)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterStatus\" type=\"checkbox\" value=\"OK\" %s/>Status</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.pwr_sleep_activate & ACTIVATE_TELEMETRY)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterTelemetry\" type=\"checkbox\" value=\"OK\" %s/>Telemetry</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.pwr_sleep_activate & ACTIVATE_WX)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterWeather\" type=\"checkbox\" value=\"OK\" %s/>Weather</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.pwr_sleep_activate & ACTIVATE_IGATE)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterIGate\" type=\"checkbox\" value=\"OK\" %s/>IGate</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.pwr_sleep_activate & ACTIVATE_DIGI)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterDigi\" type=\"checkbox\" value=\"OK\" %s/>Digi</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.pwr_sleep_activate & ACTIVATE_QUERY)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterQuery\" type=\"checkbox\" value=\"OK\" %s/>Query</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.pwr_sleep_activate & ACTIVATE_WIFI)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterWifi\" type=\"checkbox\" value=\"OK\" %s/>WiFi</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		strcat(html, "<td style=\"border:unset;\"></td>\n");
		strcat(html, "</tr></table></fieldset>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitPWR'  name=\"commitPWR\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitPWR\"/>\n");
		strcat(html, "</td></tr></table>\n");

		strcat(html, "</form><br /><br />\n");

		/**************Log File******************/
		strcat(html, "<form accept-charset=\"UTF-8\" action=\"#\" class=\"form-horizontal\" id=\"formLOG\" method=\"post\">\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>Log File</b></span></th>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Activate:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<fieldset id=\"FilterGrp\">\n");
		strcat(html, "<legend>Events</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">\n");
		strcat(html, "<tr style=\"background:unset;\">");

		if (config.log & LOG_TRACKER)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"logTracker\" type=\"checkbox\" value=\"OK\" %s/>Tracker</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.log & LOG_IGATE)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"logIgate\" type=\"checkbox\" value=\"OK\" %s/>IGate</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.log & LOG_DIGI)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"logDigi\" type=\"checkbox\" value=\"OK\" %s/>DIGI</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		if (config.log & LOG_WX)
			strcpy(filterFlageEn, "checked");
		else
			strcpy(filterFlageEn, "");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"logWeather\" type=\"checkbox\" value=\"OK\" %s/>Weather</td>\n", filterFlageEn);
		strcat(html, temp_buffer);

		strcat(html, "<td style=\"border:unset;\"></td>\n");
		strcat(html, "</tr></table></fieldset>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitLOG'  name=\"commitLOG\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitLOG\"/>\n");
		strcat(html, "</td></tr></table>\n");

		strcat(html, "</form><br /><br />\n");

		/************************ PATH USER define **************************/
		strcat(html, "<form id='formPath' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>PATH USER Define</b></span></th>\n");
		strcat(html, "<tr>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td align=\"right\"><b>PATH_1:</b></td>\n<td style=\"text-align: left;\"><input size=\"72\" maxlength=\"72\" class=\"form-control\" name=\"path1\" type=\"text\" value=\"%s\" /></td>\n", config.path[0]);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td align=\"right\"><b>PATH_2:</b></td>\n<td style=\"text-align: left;\"><input size=\"72\" maxlength=\"72\" class=\"form-control\" name=\"path2\" type=\"text\" value=\"%s\" /></td>\n", config.path[1]);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td align=\"right\"><b>PATH_3:</b></td>\n<td style=\"text-align: left;\"><input size=\"72\" maxlength=\"72\" class=\"form-control\" name=\"path3\" type=\"text\" value=\"%s\" /></td>\n", config.path[2]);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td align=\"right\"><b>PATH_4:</b></td>\n<td style=\"text-align: left;\"><input size=\"72\" maxlength=\"72\" class=\"form-control\" name=\"path4\" type=\"text\" value=\"%s\" /></td>\n", config.path[3]);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitPath'  name=\"commitPath\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitPath\"/>\n");
		strcat(html, "</td></tr></table>\n");
		strcat(html, "</form><br /><br />");

#if defined OLED || defined ST7735_160x80
		strcat(html, "<form id='formDisp' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		// html += "<h2>Display Setting</h2>\n";
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>Display Setting</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\"><b>OLED/TFT Enable</b></td>\n");
		char oledFlageEn[10] = "";
		if (config.oled_enable == true)
			strcpy(oledFlageEn, "checked");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"oledEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", oledFlageEn);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		if (config.disp_flip == true)
			strcpy(oledFlageEn, "checked");
		else
			strcpy(oledFlageEn, "");
		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\"><b>Flip Rotate</b></td>\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"dispFlip\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", oledFlageEn);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\"><b>TX Display</b></td>\n");
		if (config.tx_display == true)
			strcpy(oledFlageEn, "checked");
		else
			strcpy(oledFlageEn, "");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"txdispEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"> <i>*All TX Packet for display affter filter.</i></label></td>\n", oledFlageEn);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\"><b>RX Display</b></td>\n");
		if (config.rx_display == true)
			strcpy(oledFlageEn, "checked");
		else
			strcpy(oledFlageEn, "");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"rxdispEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"> <i>*All RX Packet for display affter filter.</i></label></td>\n", oledFlageEn);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\"><b>Head Up</b></td>\n");
		if (config.h_up == true)
			strcpy(oledFlageEn, "checked");
		else
			strcpy(oledFlageEn, "");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"hupEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"> <i>*The compass will rotate in the direction of movement.</i></label></td>\n", oledFlageEn);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\"><b>TFT Brightness</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"dispBright\" id=\"dispBright\">\n");
		for (int i = 0; i < 255; i += 25)
		{
			if (config.disp_brightness == i)
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" selected>%d</option>\n", i, i);
			}
			else
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" >%d</option>\n", i, i);
			}
			strcat(html, temp_buffer);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td></tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\"><b>Popup Delay</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"dispDelay\" id=\"dispDelay\">\n");
		for (int i = 0; i < 16; i += 1)
		{
			if (config.dispDelay == i)
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" selected>%d Sec</option>\n", i, i);
			}
			else
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" >%d Sec</option>\n", i, i);
			}
			strcat(html, temp_buffer);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td></tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td style=\"text-align: right;\"><b>OLED/TFT Sleep</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"oled_timeout\" id=\"oled_timeout\">\n");
		for (int i = 0; i <= 600; i += 30)
		{
			if (config.oled_timeout == i)
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" selected>%d Sec</option>\n", i, i);
			}
			else
			{
				snprintf(temp_buffer, sizeof(temp_buffer), "<option value=\"%d\" >%d Sec</option>\n", i, i);
			}
			strcat(html, temp_buffer);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td></tr>\n");
		char rfFlageEn[20] = "";
		if (config.dispRF == true)
			strcpy(rfFlageEn, "checked");
		char inetFlageEn[20] = "";
		if (config.dispINET == true)
			strcpy(inetFlageEn, "checked");
		snprintf(temp_buffer, sizeof(temp_buffer), "<tr><td style=\"text-align: right;\"><b>RX Channel</b></td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"dispRF\" value=\"OK\" %s/>RF <input type=\"checkbox\" name=\"dispINET\" value=\"OK\" %s/>Internet </td></tr>\n", rfFlageEn, inetFlageEn);
		strcat(html, temp_buffer);
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Filter DX:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\"><input type=\"number\" name=\"filterDX\" min=\"0\" max=\"9999\"\n");

		snprintf(temp_buffer, sizeof(temp_buffer), "step=\"1\" value=\"%d\" /> Km.  <label style=\"vertical-align: bottom;font-size: 8pt;\"> <i>*Value 0 is all distant allow.</i></label></td>\n", config.filterDistant);
		strcat(html, temp_buffer);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Filter:</b></td>\n");

		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<fieldset id=\"filterDispGrp\">\n");
		strcat(html, "<legend>Filter popup display</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">\n");
		strcat(html, "<tr style=\"background:unset;\">");

		// html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"dispTNC\" name=\"dispTNC\" type=\"checkbox\" value=\"OK\" " + rfFlageEn + "/>From RF</td>\n";

		// html += "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"dispINET\" name=\"dispINET\" type=\"checkbox\" value=\"OK\" " + inetFlageEn + "/>From INET</td>\n";

		if (config.dispFilter & FILTER_MESSAGE)
			strcpy(oledFlageEn, "checked");
		else
			strcpy(oledFlageEn, "");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterMessage\" name=\"filterMessage\" type=\"checkbox\" value=\"OK\" %s/>Message</td>\n", oledFlageEn);
		strcat(html, temp_buffer);
		if (config.dispFilter & FILTER_STATUS)
			strcpy(oledFlageEn, "checked");
		else
			strcpy(oledFlageEn, "");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterStatus\" name=\"filterStatus\" type=\"checkbox\" value=\"OK\" %s/>Status</td>\n", oledFlageEn);
		strcat(html, temp_buffer);

		if (config.dispFilter & FILTER_TELEMETRY)
			strcpy(oledFlageEn, "checked");
		else
			strcpy(oledFlageEn, "");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterTelemetry\" name=\"filterTelemetry\" type=\"checkbox\" value=\"OK\" %s/>Telemetry</td>\n", oledFlageEn);
		strcat(html, temp_buffer);

		if (config.dispFilter & FILTER_WX)
			strcpy(oledFlageEn, "checked");
		else
			strcpy(oledFlageEn, "");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterWeather\" name=\"filterWeather\" type=\"checkbox\" value=\"OK\" %s/>Weather</td>\n", oledFlageEn);
		strcat(html, temp_buffer);

		if (config.dispFilter & FILTER_OBJECT)
			strcpy(oledFlageEn, "checked");
		else
			strcpy(oledFlageEn, "");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterObject\" name=\"filterObject\" type=\"checkbox\" value=\"OK\" %s/>Object</td>\n", oledFlageEn);
		strcat(html, temp_buffer);

		if (config.dispFilter & FILTER_ITEM)
			strcpy(oledFlageEn, "checked");
		else
			strcpy(oledFlageEn, "");
		strcat(html, "</tr><tr style=\"background:unset;\"><td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterItem\" name=\"filterItem\" type=\"checkbox\" value=\"OK\" ");
		strcat(html, oledFlageEn);
		strcat(html, "/>Item</td>\n");

		if (config.dispFilter & FILTER_QUERY)
			strcpy(oledFlageEn, "checked");
		else
			strcpy(oledFlageEn, "");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterQuery\" name=\"filterQuery\" type=\"checkbox\" value=\"OK\" %s/>Query</td>\n", oledFlageEn);
		strcat(html, temp_buffer);

		if (config.dispFilter & FILTER_BUOY)
			strcpy(oledFlageEn, "checked");
		else
			strcpy(oledFlageEn, "");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterBuoy\" name=\"filterBuoy\" type=\"checkbox\" value=\"OK\" %s/>Buoy</td>\n", oledFlageEn);
		strcat(html, temp_buffer);

		if (config.dispFilter & FILTER_POSITION)
			strcpy(oledFlageEn, "checked");
		else
			strcpy(oledFlageEn, "");
		snprintf(temp_buffer, sizeof(temp_buffer), "<td style=\"border:unset;\"><input class=\"field_checkbox\" id=\"filterPosition\" name=\"filterPosition\" type=\"checkbox\" value=\"OK\" %s/>Position</td>\n", oledFlageEn);
		strcat(html, temp_buffer);

		strcat(html, "<td style=\"border:unset;\"></td>\n");
		strcat(html, "</tr></table></fieldset>\n");

		strcat(html, "</td></tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitDISP'  name=\"commitDISP\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitDISP\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br />");
#endif

		AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, html);
		response->addHeader("System", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
	}
}

void handle_igate(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	bool aprsEn = false;
	bool rf2inetEn = false;
	bool inet2rfEn = false;
	bool posGPS = false;
	bool bcnEN = false;
	bool pos2RF = false;
	bool pos2INET = false;
	bool timeStamp = false;

	if (request->hasArg("commitIGATE"))
	{

		for (int i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "igateEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						aprsEn = true;
				}
			}
			if (request->argName(i) == "myCall")
			{
				if (request->arg(i) != "")
				{
					String name = request->arg(i);
					name.trim();
					name.toUpperCase();
					strcpy(config.aprs_mycall, name.c_str());
				}
			}
			if (request->argName(i) == "igateObject")
			{
				if (request->arg(i) != "")
				{
					String name = request->arg(i);
					name.trim();
					strcpy(config.igate_object, name.c_str());
				}
				else
				{
					memset(config.igate_object, 0, sizeof(config.igate_object));
				}
			}
			if (request->argName(i) == "mySSID")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.aprs_ssid = request->arg(i).toInt();
					if (config.aprs_ssid > 15)
						config.aprs_ssid = 13;
				}
			}
			if (request->argName(i) == "igatePosInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.igate_interval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "igateSTSInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.igate_sts_interval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "igatePosLat")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.igate_lat = request->arg(i).toFloat();
				}
			}

			if (request->argName(i) == "igatePosLon")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.igate_lon = request->arg(i).toFloat();
				}
			}
			if (request->argName(i) == "igatePosAlt")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.igate_alt = request->arg(i).toFloat();
				}
			}
			if (request->argName(i) == "igatePosSel")
			{
				if (request->arg(i) != "")
				{
					if (request->arg(i).toInt() == 1)
						posGPS = true;
				}
			}

			if (request->argName(i) == "igateTable")
			{
				if (request->arg(i) != "")
				{
					config.igate_symbol[0] = request->arg(i).charAt(0);
				}
			}
			if (request->argName(i) == "igateSymbol")
			{
				if (request->arg(i) != "")
				{
					config.igate_symbol[1] = request->arg(i).charAt(0);
				}
			}
			if (request->argName(i) == "aprsHost")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.aprs_host, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "aprsPort")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.aprs_port = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "aprsFilter")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.aprs_filter, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "igatePath")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.igate_path = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "igateComment")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.igate_comment, request->arg(i).c_str());
				}
				else
				{
					memset(config.igate_comment, 0, sizeof(config.igate_comment));
				}
			}
			if (request->argName(i) == "igateStatus")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.igate_status, request->arg(i).c_str());
				}
				else
				{
					memset(config.igate_comment, 0, sizeof(config.igate_comment));
				}
			}
			if (request->argName(i) == "texttouse")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.igate_phg, request->arg(i).c_str());
				}
			}

			if (request->argName(i) == "rf2inetEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						rf2inetEn = true;
				}
			}
			if (request->argName(i) == "inet2rfEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						inet2rfEn = true;
				}
			}
			if (request->argName(i) == "igatePos2RF")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						pos2RF = true;
				}
			}
			if (request->argName(i) == "igatePos2INET")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						pos2INET = true;
				}
			}
			if (request->argName(i) == "igateBcnEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						bcnEN = true;
				}
			}
			if (request->argName(i) == "igateTimeStamp")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						timeStamp = true;
				}
			}
			if (request->argName(i) == "igateTlmInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.igate_tlm_interval = request->arg(i).toInt();
				}
			}

			String arg;
			for (int x = 0; x < 5; x++)
			{
				arg = "sensorCH" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.igate_tlm_sensor[x] = request->arg(i).toInt();
				}
				arg = "param" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.igate_tlm_PARM[x], request->arg(i).c_str());
					}
				}
				arg = "unit" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.igate_tlm_UNIT[x], request->arg(i).c_str());
					}
				}
				arg = "precision" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.igate_tlm_precision[x] = request->arg(i).toInt();
				}
				arg = "offset" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.igate_tlm_offset[x] = request->arg(i).toFloat();
				}
				for (int y = 0; y < 3; y++)
				{
					arg = "eqns" + String(x) + String((char)(y + 'a'));
					if (request->argName(i) == arg)
					{
						if (isValidNumber(request->arg(i)))
							config.igate_tlm_EQNS[x][y] = request->arg(i).toFloat();
					}
				}
			}
		}

		// waitISRetry = millis() + 10000; // Retry connect 5Sec
		config.igate_en = aprsEn;
		config.rf2inet = rf2inetEn;
		config.inet2rf = inet2rfEn;
		config.igate_gps = posGPS;
		config.igate_bcn = bcnEN;
		config.igate_loc2rf = pos2RF;
		config.igate_loc2inet = pos2INET;
		config.igate_timestamp = timeStamp;

		initInterval = true;
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
		aprsClient.stop();
	}
	else if (request->hasArg("commitIGATEfilter"))
	{
		config.rf2inetFilter = 0;
		config.inet2rfFilter = 0;
		for (int i = 0; i < request->args(); i++)
		{
			// config rf2inet filter
			if (request->argName(i) == "rf2inetFilterMessage")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.rf2inetFilter |= FILTER_MESSAGE;
				}
			}

			if (request->argName(i) == "rf2inetFilterTelemetry")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.rf2inetFilter |= FILTER_TELEMETRY;
				}
			}

			if (request->argName(i) == "rf2inetFilterStatus")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.rf2inetFilter |= FILTER_STATUS;
				}
			}

			if (request->argName(i) == "rf2inetFilterWeather")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.rf2inetFilter |= FILTER_WX;
				}
			}

			if (request->argName(i) == "rf2inetFilterObject")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.rf2inetFilter |= FILTER_OBJECT;
				}
			}

			if (request->argName(i) == "rf2inetFilterItem")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.rf2inetFilter |= FILTER_ITEM;
				}
			}

			if (request->argName(i) == "rf2inetFilterQuery")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.rf2inetFilter |= FILTER_QUERY;
				}
			}
			if (request->argName(i) == "rf2inetFilterBuoy")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.rf2inetFilter |= FILTER_BUOY;
				}
			}
			if (request->argName(i) == "rf2inetFilterPosition")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.rf2inetFilter |= FILTER_POSITION;
				}
			}
			// config inet2rf filter

			if (request->argName(i) == "inet2rfFilterMessage")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.inet2rfFilter |= FILTER_MESSAGE;
				}
			}

			if (request->argName(i) == "inet2rfFilterTelemetry")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.inet2rfFilter |= FILTER_TELEMETRY;
				}
			}

			if (request->argName(i) == "inet2rfFilterStatus")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.inet2rfFilter |= FILTER_STATUS;
				}
			}

			if (request->argName(i) == "inet2rfFilterWeather")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.inet2rfFilter |= FILTER_WX;
				}
			}

			if (request->argName(i) == "inet2rfFilterObject")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.inet2rfFilter |= FILTER_OBJECT;
				}
			}

			if (request->argName(i) == "inet2rfFilterItem")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.inet2rfFilter |= FILTER_ITEM;
				}
			}

			if (request->argName(i) == "inet2rfFilterQuery")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.inet2rfFilter |= FILTER_QUERY;
				}
			}
			if (request->argName(i) == "inet2rfFilterBuoy")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.inet2rfFilter |= FILTER_BUOY;
				}
			}
			if (request->argName(i) == "inet2rfFilterPosition")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						config.inet2rfFilter |= FILTER_POSITION;
				}
			}
		}
		String html;
		if (saveConfiguration("/default.cfg", config))
		{
			html = "Setup completed successfully";
			request->send(200, "text/html", html); // send to someones browser when asked
		}
		else
		{
			html = "Save config failed.";
			request->send(501, "text/html", html); // Not Implemented
		}
	}
	else
	{
		// Allocate initial memory for HTML content
		char tempHtml[512];
		char *html = allocateStringMemory(25000); // Start with 8KB buffer
		if (!html)
		{
			request->send(500, "text/html", "Memory allocation failed");
			return;
		}
		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		// strcat(html, "document.getElementById(\"submitIGATE\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formIgate\") document.getElementById(\"submitIGATE\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formIgateFilter\") document.getElementById(\"submitIGATEfilter\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/igate',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "</script>\n<script type=\"text/javascript\">\n");

		strcat(html, "function openWindowSymbol() {\n");
		strcat(html, "var i, l, options = [{\n");
		strcat(html, "value: 'first',\n");
		strcat(html, "text: 'First'\n");
		strcat(html, "}, {\n");
		strcat(html, "value: 'second',\n");
		strcat(html, "text: 'Second'\n");
		strcat(html, "}],\n");
		strcat(html, "newWindow = window.open(\"/symbol\", null, \"height=400,width=400,status=no,toolbar=no,menubar=no,location=no\");\n");
		strcat(html, "}\n");

		strcat(html, "function setValue(symbol,table) {\n");
		strcat(html, "document.getElementById('igateSymbol').value = String.fromCharCode(symbol);\n");
		strcat(html, "if(table==1){\n document.getElementById('igateTable').value='/';\n");
		strcat(html, "}else if(table==2){\n document.getElementById('igateTable').value='\\\\';\n}\n");
		strcat(html, "document.getElementById('igateImgSymbol').src = \"http://aprs.dprns.com/symbols/icons/\"+symbol.toString()+'-'+table.toString()+'.png';\n");
		strcat(html, "\n}\n");
		strcat(html, "function calculatePHGR(){document.forms.formIgate.texttouse.value=\"PHG\"+calcPower(document.forms.formIgate.power.value)+calcHeight(document.forms.formIgate.haat.value)+calcGain(document.forms.formIgate.gain.value)+calcDirection(document.forms.formIgate.direction.selectedIndex)}function Log2(e){return Math.log(e)/Math.log(2)}function calcPerHour(e){return e<10?e:String.fromCharCode(65+(e-10))}function calcHeight(e){return String.fromCharCode(48+Math.round(Log2(e/10),0))}function calcPower(e){if(e<1)return 0;if(e>=1&&e<4)return 1;if(e>=4&&e<9)return 2;if(e>=9&&e<16)return 3;if(e>=16&&e<25)return 4;if(e>=25&&e<36)return 5;if(e>=36&&e<49)return 6;if(e>=49&&e<64)return 7;if(e>=64&&e<81)return 8;if(e>=81)return 9}function calcDirection(e){if(e==\"0\")return\"0\";if(e==\"1\")return\"1\";if(e==\"2\")return\"2\";if(e==\"3\")return\"3\";if(e==\"4\")return\"4\";if(e==\"5\")return\"5\";if(e==\"6\")return\"6\";if(e==\"7\")return\"7\";if(e==\"8\")return\"8\"}function calcGain(e){return e>9?\"9\":e<0?\"0\":Math.round(e,0)}\n");
		strcat(html, "function onRF2INETCheck() {\n");
		strcat(html, "if (document.querySelector('#rf2inetEnable').checked) {\n");
		// Checkbox has been checked
		strcat(html, "document.getElementById(\"rf2inetFilterGrp\").disabled=false;\n");
		strcat(html, "} else {\n");
		// Checkbox has been unchecked
		strcat(html, "document.getElementById(\"rf2inetFilterGrp\").disabled=true;\n");
		strcat(html, "}\n}\n");
		strcat(html, "function onINET2RFCheck() {\n");
		strcat(html, "if (document.querySelector('#inet2rfEnable').checked) {\n");
		// Checkbox has been checked
		strcat(html, "document.getElementById(\"inet2rfFilterGrp\").disabled=false;\n");
		strcat(html, "} else {\n");
		// Checkbox has been unchecked
		strcat(html, "document.getElementById(\"inet2rfFilterGrp\").disabled=true;\n");
		strcat(html, "}\n}\n");

		strcat(html, "function selPrecision(idx) {\n");
		strcat(html, "var x=0;\n");
		strcat(html, "x = document.getElementsByName(\"precision\"+idx)[0].value;\n");
		strcat(html, "document.getElementsByName(\"eqns\"+idx+\"b\")[0].value=1/Math.pow(10,x);\n");
		strcat(html, "}\n");
		strcat(html, "function selOffset(idx) {\n");
		strcat(html, "var x=0;\n");
		strcat(html, "x = document.getElementsByName(\"offset\"+idx)[0].value;\n");
		strcat(html, "document.getElementsByName(\"eqns\"+idx+\"c\")[0].value=x*(-1);\n");
		strcat(html, "}\n");
		strcat(html, "</script>\n");
		delay(1);
		/************************ IGATE Mode **************************/
		strcat(html, "<form id='formIgate' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		// strcat(html, "<h2>[IGATE] Internet Gateway Mode</h2>\n");
		strcat(html, "<table>\n");
		// strcat(html, "<tr>\n");
		// strcat(html, "<th width=\"200\"><span><b>Setting</b></span></th>\n");
		// strcat(html, "<th><span><b>Value</b></span></th>\n");
		// strcat(html, "</tr>\n");
		strcat(html, "<th colspan=\"2\"><span><b>[IGATE] Internet Gateway Mode</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Enable:</b></td>\n");
		char igateEnFlag[10] = "";
		if (config.igate_en)
			strcpy(igateEnFlag, "checked");
		else
			strcpy(igateEnFlag, "");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"igateEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", igateEnFlag);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Station Callsign:</b></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"7\" size=\"6\" id=\"myCall\" name=\"myCall\" type=\"text\" value=\"%s\" /></td>\n", config.aprs_mycall);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Station SSID:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"mySSID\" id=\"mySSID\">\n");
		for (uint8_t ssid = 0; ssid <= 15; ssid++)
		{
			if (config.aprs_ssid == ssid)
			{
				snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>%d</option>\n", ssid, ssid);
				}
				else
				{
				snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\">%d</option>\n", ssid, ssid);
				}
			strcat(html, tempHtml);
		}
		strcat(html, "</select></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Station Symbol:</b></td>\n");
		const char *table = "1";
		if (config.igate_symbol[0] == 47)
			table = "1";
		if (config.igate_symbol[0] == 92)
			table = "2";
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\">Table:<input maxlength=\"1\" size=\"1\" id=\"igateTable\" name=\"igateTable\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> Symbol:<input maxlength=\"1\" size=\"1\" id=\"igateSymbol\" name=\"igateSymbol\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> <img border=\"1\" style=\"vertical-align: middle;\" id=\"igateImgSymbol\" onclick=\"openWindowSymbol();\" src=\"http://aprs.dprns.com/symbols/icons/%d-%s.png\"> <i>*Click icon for select symbol</i></td>\n",
						 config.igate_symbol[0], config.igate_symbol[1], (int)config.igate_symbol[1], table);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Item/Obj Name:</b></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"9\" size=\"9\" id=\"igateObject\" name=\"igateObject\" type=\"text\" value=\"%s\" /><i> *If not used, leave it blank.In use 3-9 charactor</i></td>\n", config.igate_object);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PATH:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"igatePath\" id=\"igatePath\">\n");
		for (uint8_t pthIdx = 0; pthIdx < PATH_LEN; pthIdx++)
			{
				if (config.igate_path == pthIdx)
				{
				snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
				}
				else
				{
				snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\">%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
				}
			strcat(html, tempHtml);
		}
		strcat(html, "</select></td>\n");
		// strcat(html, "<td style=\"text-align: left;\"><input maxlength=\"72\" size=\"72\" id=\"igatePath\" name=\"igatePath\" type=\"text\" value=\"" + String(config.igate_path) + "\" /></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Server Host:</b></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"20\" size=\"20\" id=\"aprsHost\" name=\"aprsHost\" type=\"text\" value=\"%s\" /> *APRS-IS by T2THAI at <a href=\"http://aprs.dprns.com:14501\" target=\"_t2thai\">aprs.dprns.com:14580</a>,CBAPRS at <a href=\"http://aprs.dprns.com:24501\" target=\"_t2thai\">aprs.dprns.com:24580</a></td>\n", config.aprs_host);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Server Port:</b></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input min=\"1\" max=\"65535\" step=\"1\" id=\"aprsPort\" name=\"aprsPort\" type=\"number\" value=\"%d\" /> *AMPR Host at <a href=\"http://aprs.hs5tqa.ampr.org:14501\" target=\"_t2thai\">aprs.hs5tqa.ampr.org:14580</a></td>\n", config.aprs_port);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Server Filter:</b></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"30\" size=\"30\" id=\"aprsFilter\" name=\"aprsFilter\" type=\"text\" value=\"%s\" /> *Filter: <a target=\"_blank\" href=\"http://www.aprs-is.net/javAPRSFilter.aspx\">http://www.aprs-is.net/javAPRSFilter.aspx</a></td>\n", config.aprs_filter);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Text Comment:</b></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"25\" size=\"30\" id=\"igateComment\" name=\"igateComment\" type=\"text\" value=\"%s\" /></td>\n", config.igate_comment);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Text Status:</b></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"50\" size=\"60\" id=\"igateStatus\" name=\"igateStatus\" type=\"text\" value=\"%s\" />  Interval:<input min=\"0\" max=\"3600\" step=\"1\" name=\"igateSTSInv\" type=\"number\" value=\"%d\" />Sec.</td>\n", config.igate_status, config.igate_sts_interval);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");

		char rf2inetFlag[10];
		if (config.rf2inet)
			strcpy(rf2inetFlag, "checked");
		else
			strcpy(rf2inetFlag, "");
		snprintf(tempHtml, sizeof(tempHtml), "<td align=\"right\"><b>RF2INET:</b></td>\n<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" id=\"rf2inetEnable\" name=\"rf2inetEnable\" onclick=\"onRF2INETCheck()\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Switch RF to Internet gateway</i></label></td>\n", rf2inetFlag);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		char inet2rfEnFlag[10];
		if (config.inet2rf)
			strcpy(inet2rfEnFlag, "checked");
		else
			strcpy(inet2rfEnFlag, "");
		snprintf(tempHtml, sizeof(tempHtml), "<td align=\"right\"><b>INET2RF:</b></td>\n<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" id=\"inet2rfEnable\" name=\"inet2rfEnable\" onclick=\"onINET2RFCheck()\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Switch Internet to RF gateway</i></label></td>\n", inet2rfEnFlag);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
			const char *timeStampFlag = config.igate_timestamp ? "checked" : "";
		snprintf(tempHtml, sizeof(tempHtml), "<td align=\"right\"><b>Time Stamp:</b></td>\n<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"igateTimeStamp\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", timeStampFlag);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n<tr>");

		strcat(html, "<td align=\"right\"><b>POSITION:</b></td>\n");
		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<table>");
			const char *igateBcnEnFlag = config.igate_bcn ? "checked" : "";
		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Beacon:</td><td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"igateBcnEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\">  Interval:<input min=\"0\" max=\"3600\" step=\"1\" id=\"igatePosInv\" name=\"igatePosInv\" type=\"number\" value=\"%d\" />Sec.</label></td></tr>", igateBcnEnFlag, config.igate_interval);
		strcat(html, tempHtml);
		const char *igatePosFixFlag = config.igate_gps ? "" : "checked=\"checked\"";
		const char *igatePosGPSFlag = config.igate_gps ? "checked=\"checked\"" : "";
		const char *igatePos2RFFlag = config.igate_loc2rf ? "checked" : "";
		const char *igatePos2INETFlag = config.igate_loc2inet ? "checked" : "";

		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Location:</td><td style=\"text-align: left;\"><input type=\"radio\" name=\"igatePosSel\" value=\"0\" %s/>Fix <input type=\"radio\" name=\"igatePosSel\" value=\"1\" %s/>GPS </td></tr>\n", igatePosFixFlag, igatePosGPSFlag);
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">TX Channel:</td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"igatePos2RF\" value=\"OK\" %s/>RF <input type=\"checkbox\" name=\"igatePos2INET\" value=\"OK\" %s/>Internet </td></tr>\n", igatePos2RFFlag, igatePos2INETFlag);
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Latitude:</td><td style=\"text-align: left;\"><input min=\"-90\" max=\"90\" step=\"0.00001\" id=\"igatePosLat\" name=\"igatePosLat\" type=\"number\" value=\"%.5f\" />degrees (positive for North, negative for South)</td></tr>\n", config.igate_lat);
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Longitude:</td><td style=\"text-align: left;\"><input min=\"-180\" max=\"180\" step=\"0.00001\" id=\"igatePosLon\" name=\"igatePosLon\" type=\"number\" value=\"%.5f\" />degrees (positive for East, negative for West)</td></tr>\n", config.igate_lon);
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Altitude:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"10000\" step=\"0.1\" id=\"igatePosAlt\" name=\"igatePosAlt\" type=\"number\" value=\"%.2f\" /> meter. *Value 0 is not send height</td></tr>\n", config.igate_alt);
		strcat(html, tempHtml);

		strcat(html, "</table></td>");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PHG:</b></td>\n");
		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<table>");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\">Radio TX Power</td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"power\" id=\"power\">\n");
		strcat(html, "<option value=\"1\" selected>1</option>\n");
		strcat(html, "<option value=\"5\">5</option>\n");
		strcat(html, "<option value=\"10\">10</option>\n");
		strcat(html, "<option value=\"15\">15</option>\n");
		strcat(html, "<option value=\"25\">25</option>\n");
		strcat(html, "<option value=\"35\">35</option>\n");
		strcat(html, "<option value=\"50\">50</option>\n");
		strcat(html, "<option value=\"65\">65</option>\n");
		strcat(html, "<option value=\"80\">80</option>\n");
		strcat(html, "</select> Watts</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td style=\"text-align: right;\">Antenna Gain</td><td style=\"text-align: left;\"><input size=\"3\" min=\"0\" max=\"100\" step=\"0.1\" id=\"gain\" name=\"gain\" type=\"number\" value=\"6\" /> dBi</td></tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\">Height</td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"haat\" id=\"haat\">\n");
		int k = 10;
		for (uint8_t w = 0; w < 10; w++)
		{
			char *temp_opt = allocateStringMemory(256);
			if (temp_opt)
			{
				if (w == 0)
				{
					snprintf(temp_opt, 256, "<option value=\"%d\" selected>%d</option>\n", k, k);
				}
				else
				{
					snprintf(temp_opt, 256, "<option value=\"%d\">%d</option>\n", k, k);
				}
				strcat(html, temp_opt);
				free(temp_opt);
			}
			k += k;
		}
		strcat(html, "</select> Feet</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\">Antenna/Direction</td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"direction\" id=\"direction\">\n");
		strcat(html, "<option>Omni</option><option>NE</option><option>E</option><option>SE</option><option>S</option><option>SW</option><option>W</option><option>NW</option><option>N</option>\n");
		strcat(html, "</select></td>\n");
		strcat(html, "</tr>\n");

		snprintf(tempHtml, sizeof(tempHtml), "<tr><td align=\"right\"><b>PHG Text</b></td><td align=\"left\"><input name=\"texttouse\" type=\"text\" size=\"6\" style=\"background-color: rgb(97, 239, 170);\" value=\"%s\"/> <input type=\"button\" value=\"Calculate PHG\" onclick=\"javascript:calculatePHGR()\" /></td></tr>\n", config.igate_phg);
		strcat(html, tempHtml);
		strcat(html, "</table></td>");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Telemetry:</b><br />(v=0->8280)</td>\n");
		strcat(html, "<td align=\"center\"><table>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"1000\" step=\"1\" id=\"igateTlmInv\" name=\"igateTlmInv\" type=\"number\" value=\"%d\" /> *Number of packets interval,<i>Example: 0 not send,1 send every packet</i></label></td></tr>", config.igate_tlm_interval);
		strcat(html, tempHtml);
		for (int ax = 0; ax < 5; ax++)
		{
			snprintf(tempHtml, sizeof(tempHtml), "<tr><td align=\"right\"><b>CH A%d:</b></td>\n", ax + 1);
			strcat(html, tempHtml);

			strcat(html, "<td align=\"center\">\n");
			strcat(html, "<table>");

			strcat(html, "<tr><td style=\"text-align: right;\">Sensor:</td>\n");
			strcat(html, "<td style=\"text-align: left;\">CH: ");

			snprintf(tempHtml, sizeof(tempHtml), "<select name=\"sensorCH%d\" id=\"sensorCH%d\">\n", ax, ax);
			strcat(html, tempHtml);

			for (uint8_t idx = 0; idx < 11; idx++)
				{
					if (idx == 0)
					{
						if (config.igate_tlm_sensor[ax] == idx)
						{
						snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>NONE</option>\n", idx);
						}
						else
						{
						snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\">NONE</option>\n", idx);
						}
					}
					else
					{
						if (config.igate_tlm_sensor[ax] == idx)
						{
						snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>SENSOR#%d</option>\n", idx, idx);
						}
						else
						{
						snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\">SENSOR#%d</option>\n", idx, idx);
						}
					}
				strcat(html, tempHtml);
			}
			strcat(html, "</select></td>\n");

			snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\">Name: <input maxlength=\"10\" size=\"8\" name=\"param%d\" type=\"text\" value=\"%s\" /></td>\n", ax, config.igate_tlm_PARM[ax]);
			strcat(html, tempHtml);

			snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\">Unit: <input maxlength=\"8\" size=\"5\" name=\"unit%d\" type=\"text\" value=\"%s\" /></td>\n", ax, config.igate_tlm_UNIT[ax]);
			strcat(html, tempHtml);

			snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\">Precision: <input min=\"0\" max=\"5\" step=\"1\" type=\"number\" style=\"width: 2em\" name=\"precision%d\" type=\"text\" value=\"%d\" onchange=\"selPrecision(%d)\"/></td></tr>\n", ax, config.igate_tlm_precision[ax], ax);
			strcat(html, tempHtml);

			snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">EQNS:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns%da\" type=\"number\" value=\"%.5f\" />  b:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns%db\" type=\"number\" value=\"%.5f\" /> c:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns%dc\" type=\"number\" value=\"%.5f\" /> (av<sup>2</sup>+bv+c) </td>\n",
							 ax, config.igate_tlm_EQNS[ax][0], ax, config.igate_tlm_EQNS[ax][1], ax, config.igate_tlm_EQNS[ax][2]);
			strcat(html, tempHtml);

			snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\">Offset: <input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" type=\"number\" name=\"offset%d\" type=\"text\" value=\"%.5f\" onchange=\"selOffset(%d)\"/></td></tr>\n", ax, config.igate_tlm_offset[ax], ax);
			strcat(html, tempHtml);

			strcat(html, "</table></td>");
			strcat(html, "</tr>\n");
		}
		strcat(html, "</table></td></tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitIGATE'  name=\"commitIGATE\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitIGATE\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br /><br />");

		strcat(html, "<form id='formIgateFilter' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>[IGATE] Filter</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>RF2INET Filter:</b></td>\n");

		strcat(html, "<td align=\"center\">\n");
		if (config.rf2inet)
			strcat(html, "<fieldset id=\"rf2inetFilterGrp\">\n");
		else
			strcat(html, "<fieldset id=\"rf2inetFilterGrp\" disabled>\n");
		strcat(html, "<legend>Filter RF to Internet</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">");
		strcat(html, "<tr style=\"background:unset;\">");

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterMessage\" type=\"checkbox\" value=\"OK\" %s/>Message</td>\n",
						 (config.rf2inetFilter & FILTER_MESSAGE) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterStatus\" type=\"checkbox\" value=\"OK\" %s/>Status</td>\n",
						 (config.rf2inetFilter & FILTER_STATUS) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterTelemetry\" type=\"checkbox\" value=\"OK\" %s/>Telemetry</td>\n",
						 (config.rf2inetFilter & FILTER_TELEMETRY) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterWeather\" type=\"checkbox\" value=\"OK\" %s/>Weather</td>\n",
						 (config.rf2inetFilter & FILTER_WX) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterObject\" type=\"checkbox\" value=\"OK\" %s/>Object</td>\n",
						 (config.rf2inetFilter & FILTER_OBJECT) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "</tr><tr style=\"background:unset;\"><td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterItem\" type=\"checkbox\" value=\"OK\" %s/>Item</td>\n",
						 (config.rf2inetFilter & FILTER_ITEM) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterQuery\" type=\"checkbox\" value=\"OK\" %s/>Query</td>\n",
						 (config.rf2inetFilter & FILTER_QUERY) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterBuoy\" type=\"checkbox\" value=\"OK\" %s/>Buoy</td>\n",
						 (config.rf2inetFilter & FILTER_BUOY) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"rf2inetFilterPosition\" type=\"checkbox\" value=\"OK\" %s/>Position</td>\n",
						 (config.rf2inetFilter & FILTER_POSITION) ? "checked" : "");
		strcat(html, tempHtml);

		strcat(html, "<td style=\"border:unset;\"></td>");
		strcat(html, "</tr></table></fieldset>\n");
		strcat(html, "</td></tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>INET2RF Filter:</b></td>\n");

		strcat(html, "<td align=\"center\">\n");
		if (config.inet2rf)
			strcat(html, "<fieldset id=\"inet2rfFilterGrp\">\n");
		else
			strcat(html, "<fieldset id=\"inet2rfFilterGrp\" disabled>\n");
		strcat(html, "<legend>Filter Internet to RF</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">");
		strcat(html, "<tr style=\"background:unset;\">");

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterMessage\" type=\"checkbox\" value=\"OK\" %s/>Message</td>\n",
						 (config.inet2rfFilter & FILTER_MESSAGE) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterStatus\" type=\"checkbox\" value=\"OK\" %s/>Status</td>\n",
						 (config.inet2rfFilter & FILTER_STATUS) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterTelemetry\" type=\"checkbox\" value=\"OK\" %s/>Telemetry</td>\n",
						 (config.inet2rfFilter & FILTER_TELEMETRY) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterWeather\" type=\"checkbox\" value=\"OK\" %s/>Weather</td>\n",
						 (config.inet2rfFilter & FILTER_WX) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterObject\" type=\"checkbox\" value=\"OK\" %s/>Object</td>\n",
						 (config.inet2rfFilter & FILTER_OBJECT) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "</tr><tr style=\"background:unset;\"><td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterItem\" type=\"checkbox\" value=\"OK\" %s/>Item</td>\n",
						 (config.inet2rfFilter & FILTER_ITEM) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterQuery\" type=\"checkbox\" value=\"OK\" %s/>Query</td>\n",
						 (config.inet2rfFilter & FILTER_QUERY) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterBuoy\" type=\"checkbox\" value=\"OK\" %s/>Buoy</td>\n",
						 (config.inet2rfFilter & FILTER_BUOY) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"inet2rfFilterPosition\" type=\"checkbox\" value=\"OK\" %s/>Position</td>\n",
						 (config.inet2rfFilter & FILTER_POSITION) ? "checked" : "");
		strcat(html, tempHtml);

		strcat(html, "<td style=\"border:unset;\"></td>");
		strcat(html, "</tr></table></fieldset>\n");
		strcat(html, "</td></tr>\n");

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitIGATEfilter'  name=\"commitIGATEfilter\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitIGATEfilter\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br />");

		AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, html);
		response->addHeader("IGATE", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
	}
}

void handle_digi(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	bool digiEn = false;
	bool digiAuto = false;
	bool posGPS = false;
	bool bcnEN = false;
	bool pos2RF = false;
	bool pos2INET = false;
	bool timeStamp = false;

	if (request->hasArg("commitDIGI"))
	{
		config.digiFilter = 0;
		for (int i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "digiEnable")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						digiEn = true;
				}
			}
			if (request->argName(i) == "digiAuto")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						digiAuto = true;
				}
			}
			if (request->argName(i) == "myCall")
			{
				if (request->arg(i) != "")
				{
					String name = request->arg(i);
					name.trim();
					name.toUpperCase();
					strcpy(config.digi_mycall, name.c_str());
				}
			}
			if (request->argName(i) == "mySSID")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.digi_ssid = request->arg(i).toInt();
					if (config.digi_ssid > 15)
						config.digi_ssid = 3;
				}
			}
			if (request->argName(i) == "digiDelay")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.digi_delay = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "digiPosInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.digi_interval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "digiSTSInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.digi_sts_interval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "digiPosLat")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.digi_lat = request->arg(i).toFloat();
				}
			}

			if (request->argName(i) == "digiPosLon")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.digi_lon = request->arg(i).toFloat();
				}
			}
			if (request->argName(i) == "digiPosAlt")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.digi_alt = request->arg(i).toFloat();
				}
			}
			if (request->argName(i) == "digiPosSel")
			{
				if (request->arg(i) != "")
				{
					if (request->arg(i).toInt() == 1)
						posGPS = true;
				}
			}

			if (request->argName(i) == "digiTable")
			{
				if (request->arg(i) != "")
				{
					config.digi_symbol[0] = request->arg(i).charAt(0);
				}
			}
			if (request->argName(i) == "digiSymbol")
			{
				if (request->arg(i) != "")
				{
					config.digi_symbol[1] = request->arg(i).charAt(0);
				}
			}
			if (request->argName(i) == "digiPath")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.digi_path = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "digiComment")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.digi_comment, request->arg(i).c_str());
				}
				else
				{
					memset(config.digi_comment, 0, sizeof(config.digi_comment));
				}
			}
			if (request->argName(i) == "texttouse")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.digi_phg, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "digiStatus")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.digi_status, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "digiPos2RF")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						pos2RF = true;
				}
			}
			if (request->argName(i) == "digiPos2INET")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						pos2INET = true;
				}
			}
			if (request->argName(i) == "digiBcnEnable")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						bcnEN = true;
				}
			}
			// Filter
			if (request->argName(i) == "FilterMessage")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.digiFilter |= FILTER_MESSAGE;
				}
			}

			if (request->argName(i) == "FilterTelemetry")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.digiFilter |= FILTER_TELEMETRY;
				}
			}

			if (request->argName(i) == "FilterStatus")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.digiFilter |= FILTER_STATUS;
				}
			}

			if (request->argName(i) == "FilterWeather")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.digiFilter |= FILTER_WX;
				}
			}

			if (request->argName(i) == "FilterObject")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.digiFilter |= FILTER_OBJECT;
				}
			}

			if (request->argName(i) == "FilterItem")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.digiFilter |= FILTER_ITEM;
				}
			}

			if (request->argName(i) == "FilterQuery")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.digiFilter |= FILTER_QUERY;
				}
			}
			if (request->argName(i) == "FilterBuoy")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.digiFilter |= FILTER_BUOY;
				}
			}
			if (request->argName(i) == "FilterPosition")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						config.digiFilter |= FILTER_POSITION;
				}
			}
			if (request->argName(i) == "digiTimeStamp")
			{
				if (request->arg(i) != "")
				{
					if (strcmp(request->arg(i).c_str(), "OK") == 0)
						timeStamp = true;
				}
			}
			if (request->argName(i) == "digiTlmInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.digi_tlm_interval = request->arg(i).toInt();
				}
			}

			for (int x = 0; x < 5; x++)
			{
				{
					char *arg = allocateStringMemory(32);
					if (arg)
					{
						snprintf(arg, 32, "sensorCH%d", x);
						if (request->argName(i) == String(arg))
						{
							if (isValidNumber(request->arg(i)))
								config.digi_tlm_sensor[x] = request->arg(i).toInt();
						}
						free(arg);
					}
				}
				{
					char *arg = allocateStringMemory(32);
					if (arg)
					{
						snprintf(arg, 32, "param%d", x);
						if (request->argName(i) == String(arg))
						{
							if (request->arg(i) != "")
							{
								strcpy(config.digi_tlm_PARM[x], request->arg(i).c_str());
							}
						}
						free(arg);
					}
				}
				{
					char *arg = allocateStringMemory(32);
					if (arg)
					{
						snprintf(arg, 32, "unit%d", x);
						if (request->argName(i) == String(arg))
						{
							if (request->arg(i) != "")
							{
								strcpy(config.digi_tlm_UNIT[x], request->arg(i).c_str());
							}
						}
						free(arg);
					}
				}
				{
					char *arg = allocateStringMemory(32);
					if (arg)
					{
						snprintf(arg, 32, "precision%d", x);
						if (request->argName(i) == String(arg))
						{
							if (isValidNumber(request->arg(i)))
								config.digi_tlm_precision[x] = request->arg(i).toInt();
						}
						free(arg);
					}
				}
				{
					char *arg = allocateStringMemory(32);
					if (arg)
					{
						snprintf(arg, 32, "offset%d", x);
						if (request->argName(i) == String(arg))
						{
							if (isValidNumber(request->arg(i)))
								config.digi_tlm_offset[x] = request->arg(i).toFloat();
						}
						free(arg);
					}
				}
				for (int y = 0; y < 3; y++)
				{
					{
						char *arg = allocateStringMemory(32);
						if (arg)
						{
							snprintf(arg, 32, "eqns%d%c", x, (char)(y + 'a'));
							if (request->argName(i) == String(arg))
							{
								if (isValidNumber(request->arg(i)))
									config.digi_tlm_EQNS[x][y] = request->arg(i).toFloat();
							}
							free(arg);
						}
					}
				}
			}
		}
		config.digi_en = digiEn;
		config.digi_auto = digiAuto;
		config.digi_gps = posGPS;
		config.digi_bcn = bcnEN;
		config.digi_loc2rf = pos2RF;
		config.digi_loc2inet = pos2INET;
		config.digi_timestamp = timeStamp;

		initInterval = true;
		saveConfig(request);
	}
	else
	{
		// Allocate initial memory for HTML content
		char *html = allocateStringMemory(20000); // Start with 12KB buffer
		if (!html)
		{
			request->send(500, "text/html", "Memory allocation failed");
			return;
		}
		memset(html, 0, 20000);
		char tempHtml[512];
		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "document.getElementById(\"submitDIGI\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/digi',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "</script>\n<script type=\"text/javascript\">\n");
		strcat(html, "function openWindowSymbol() {\n");
		strcat(html, "var i, l, options = [{\n");
		strcat(html, "value: 'first',\n");
		strcat(html, "text: 'First'\n");
		strcat(html, "}, {\n");
		strcat(html, "value: 'second',\n");
		strcat(html, "text: 'Second'\n");
		strcat(html, "}],\n");
		strcat(html, "newWindow = window.open(\"/symbol\", null, \"height=400,width=400,status=no,toolbar=no,menubar=no,titlebar=no,location=no\");\n");
		strcat(html, "}\n");

		strcat(html, "function setValue(symbol,table) {\n");
		strcat(html, "document.getElementById('digiSymbol').value = String.fromCharCode(symbol);\n");
		strcat(html, "if(table==1){\n document.getElementById('digiTable').value='/';\n");
		strcat(html, "}else if(table==2){\n document.getElementById('digiTable').value='\\\\';\n}\n");
		strcat(html, "document.getElementById('digiImgSymbol').src = \"http://aprs.dprns.com/symbols/icons/\"+symbol.toString()+'-'+table.toString()+'.png';\n");
		strcat(html, "\n}\n");
		strcat(html, "function calculatePHGR(){document.forms.formDIGI.texttouse.value=\"PHG\"+calcPower(document.forms.formDIGI.power.value)+calcHeight(document.forms.formDIGI.haat.value)+calcGain(document.forms.formDIGI.gain.value)+calcDirection(document.forms.formDIGI.direction.selectedIndex)}function Log2(e){return Math.log(e)/Math.log(2)}function calcPerHour(e){return e<10?e:String.fromCharCode(65+(e-10))}function calcHeight(e){return String.fromCharCode(48+Math.round(Log2(e/10),0))}function calcPower(e){if(e<1)return 0;if(e>=1&&e<4)return 1;if(e>=4&&e<9)return 2;if(e>=9&&e<16)return 3;if(e>=16&&e<25)return 4;if(e>=25&&e<36)return 5;if(e>=36&&e<49)return 6;if(e>=49&&e<64)return 7;if(e>=64&&e<81)return 8;if(e>=81)return 9}function calcDirection(e){if(e==\"0\")return\"0\";if(e==\"1\")return\"1\";if(e==\"2\")return\"2\";if(e==\"3\")return\"3\";if(e==\"4\")return\"4\";if(e==\"5\")return\"5\";if(e==\"6\")return\"6\";if(e==\"7\")return\"7\";if(e==\"8\")return\"8\"}function calcGain(e){return e>9?\"9\":e<0?\"0\":Math.round(e,0)}\n");
		strcat(html, "function selPrecision(idx) {\n");
		strcat(html, "var x=0;\n");
		strcat(html, "x = document.getElementsByName(\"precision\"+idx)[0].value;\n");
		strcat(html, "document.getElementsByName(\"eqns\"+idx+\"b\")[0].value=1/Math.pow(10,x);\n");
		strcat(html, "}\n");
		strcat(html, "function selOffset(idx) {\n");
		strcat(html, "var x=0;\n");
		strcat(html, "x = document.getElementsByName(\"offset\"+idx)[0].value;\n");
		strcat(html, "document.getElementsByName(\"eqns\"+idx+\"c\")[0].value=x*(-1);\n");
		strcat(html, "}\n");
		strcat(html, "</script>\n");

		/************************ DIGI Mode **************************/
		strcat(html, "<form id='formDIGI' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		// strcat(html, "<h2>[DIGI] Digital Repeater Mode</h2>\n");
		strcat(html, "<table>\n");
		// strcat(html, "<tr>\n");
		// strcat(html, "<th width=\"200\"><span><b>Setting</b></span></th>\n");
		// strcat(html, "<th><span><b>Value</b></span></th>\n");
		// strcat(html, "</tr>\n");
		strcat(html, "<th colspan=\"2\"><span><b>[DIGI] Digital Repeater Mode</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Enable:</b></td>\n");
		char digiFlag[10] = "";
		if (config.digi_en)
			strcpy(digiFlag, "checked");
		else
			strcpy(digiFlag, "");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"digiEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", digiFlag);
		strcat(html, tempHtml);

		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Auto Enable:</b></td>\n");

		if (config.digi_auto)
			strcpy(digiFlag, "checked");
		else
			strcpy(digiFlag, "");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"digiAuto\" value=\"OK\" %s><span class=\"slider round\"></span></label> <i>*Automatic enable when APRS-IS disconnected</i></td>\n", digiFlag);
		strcat(html, tempHtml);

		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Station Callsign:</b></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"7\" size=\"6\" id=\"myCall\" name=\"myCall\" type=\"text\" value=\"%s\" /></td>\n", config.digi_mycall);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Station SSID:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"mySSID\" id=\"mySSID\">\n");
		for (uint8_t ssid = 0; ssid <= 15; ssid++)
		{
			if (config.digi_ssid == ssid)
			{
				snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>%d</option>\n", ssid, ssid);
			}
			else
			{
				snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\">%d</option>\n", ssid, ssid);
			}
			strcat(html, tempHtml);
		}
		strcat(html, "</select></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Station Symbol:</b></td>\n");
		char table[3] = "1";
		if (config.digi_symbol[0] == 47)
			strcpy(table, "1");
		if (config.digi_symbol[0] == 92)
			strcpy(table, "2");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\">Table:<input maxlength=\"1\" size=\"1\" id=\"digiTable\" name=\"digiTable\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> Symbol:<input maxlength=\"1\" size=\"1\" id=\"digiSymbol\" name=\"digiSymbol\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> <img border=\"1\" style=\"vertical-align: middle;\" id=\"digiImgSymbol\" onclick=\"openWindowSymbol();\" src=\"http://aprs.dprns.com/symbols/icons/%d-%s.png\"> <i>*Click icon for select symbol</i></td>\n",
				 config.digi_symbol[0], config.digi_symbol[1], (int)config.digi_symbol[1], table);
		strcat(html, tempHtml);

		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PATH:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"digiPath\" id=\"digiPath\">\n");
		for (uint8_t pthIdx = 0; pthIdx < PATH_LEN; pthIdx++)
		{
			if (config.digi_path == pthIdx)
			{
				snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
			}
			else
			{
				snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\">%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
			}
			strcat(html, tempHtml);
		}
		strcat(html, "</select></td>\n");
		// strcat(html, "<td style=\"text-align: left;\"><input maxlength=\"72\" size=\"72\" id=\"digiPath\" name=\"digiPath\" type=\"text\" value=\"" + String(config.digi_path) + "\" /></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Text Comment:</b></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"25\" size=\"30\" id=\"digiComment\" name=\"digiComment\" type=\"text\" value=\"%s\" /></td>\n", config.digi_comment);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Text Status:</b></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"50\" size=\"60\" id=\"digiStatus\" name=\"digiStatus\" type=\"text\" value=\"%s\" />  Interval:<input min=\"0\" max=\"3600\" step=\"1\" name=\"digiSTSInv\" type=\"number\" value=\"%d\" />Sec.</td>\n", config.digi_status, config.digi_sts_interval);
		strcat(html, tempHtml);

		strcat(html, "</tr>\n");

		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\"><b>Repeat Delay:</b></td><td style=\"text-align: left;\"><input min=\"0\" max=\"10000\" step=\"100\" id=\"digiDelay\" name=\"digiDelay\" type=\"number\" value=\"%d\" /> mSec. <i>*0 is auto,Other random of delay time</i></td></tr>", config.digi_delay);
		strcat(html, tempHtml);

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Time Stamp:</b></td>\n");
		char timeStampFlag[10];
		if (config.digi_timestamp)
			strcpy(timeStampFlag, "checked");
		else
			strcpy(timeStampFlag, "");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"digiTimeStamp\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", timeStampFlag);
		strcat(html, tempHtml);

		strcat(html, "</tr>\n");

		strcat(html, "<tr><td align=\"right\"><b>POSITION:</b></td>\n");
		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<table>");

		if (config.digi_bcn)
			strcpy(digiFlag, "checked");
		else
			strcpy(digiFlag, "");

		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Beacon:</td><td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"digiBcnEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\">  Interval:<input min=\"0\" max=\"3600\" step=\"1\" id=\"digiPosInv\" name=\"digiPosInv\" type=\"number\" value=\"%d\" />Sec.</label></td></tr>", digiFlag, config.digi_interval);
		strcat(html, tempHtml);

		String digiPosFixFlag = "";
		String digiPosGPSFlag = "";
		String digiPos2RFFlag = "";
		String digiPos2INETFlag = "";
		if (config.digi_gps)
			digiPosGPSFlag = "checked=\"checked\"";
		else
			digiPosFixFlag = "checked=\"checked\"";

		if (config.digi_loc2rf)
			digiPos2RFFlag = "checked";
		if (config.digi_loc2inet)
			digiPos2INETFlag = "checked";
		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Location:</td><td style=\"text-align: left;\"><input type=\"radio\" name=\"digiPosSel\" value=\"0\" %s/>Fix <input type=\"radio\" name=\"digiPosSel\" value=\"1\" %s/>GPS </td></tr>\n",
				 digiPosFixFlag.c_str(), digiPosGPSFlag.c_str());
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">TX Channel:</td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"digiPos2RF\" value=\"OK\" %s/>RF <input type=\"checkbox\" name=\"digiPos2INET\" value=\"OK\" %s/>Internet </td></tr>\n",
				 digiPos2RFFlag.c_str(), digiPos2INETFlag.c_str());
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Latitude:</td><td style=\"text-align: left;\"><input min=\"-90\" max=\"90\" step=\"0.00001\" id=\"digiPosLat\" name=\"digiPosLat\" type=\"number\" value=\"%.5f\" />degrees (positive for North, negative for South)</td></tr>\n", config.digi_lat);
		strcat(html, tempHtml);
		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Longitude:</td><td style=\"text-align: left;\"><input min=\"-180\" max=\"180\" step=\"0.00001\" id=\"digiPosLon\" name=\"digiPosLon\" type=\"number\" value=\"%.5f\" />degrees (positive for East, negative for West)</td></tr>\n", config.digi_lon);
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Altitude:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"10000\" step=\"0.1\" id=\"digiPosAlt\" name=\"digiPosAlt\" type=\"number\" value=\"%.2f\" /> meter. *Value 0 is not send height</td></tr>\n", config.digi_alt);
		strcat(html, tempHtml);
		strcat(html, "</table></td>");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PHG:</b></td>\n");
		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<table>");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\">Radio TX Power</td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"power\" id=\"power\">\n");
		strcat(html, "<option value=\"1\" selected>1</option>\n");
		strcat(html, "<option value=\"5\">5</option>\n");
		strcat(html, "<option value=\"10\">10</option>\n");
		strcat(html, "<option value=\"15\">15</option>\n");
		strcat(html, "<option value=\"25\">25</option>\n");
		strcat(html, "<option value=\"35\">35</option>\n");
		strcat(html, "<option value=\"50\">50</option>\n");
		strcat(html, "<option value=\"65\">65</option>\n");
		strcat(html, "<option value=\"80\">80</option>\n");
		strcat(html, "</select> Watts</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td style=\"text-align: right;\">Antenna Gain</td><td style=\"text-align: left;\"><input size=\"3\" min=\"0\" max=\"100\" step=\"0.1\" id=\"gain\" name=\"gain\" type=\"number\" value=\"6\" /> dBi</td></tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\">Height</td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"haat\" id=\"haat\">\n");
		int k = 10;
		for (uint8_t w = 0; w < 10; w++)
		{
			if (w == 0)
			{
				snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>%d</option>\n", k, k);
			}
			else
			{
				snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\">%d</option>\n", k, k);
			}
			strcat(html, tempHtml);
			k += k;
		}
		strcat(html, "</select> Feet</td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\">Antenna/Direction</td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"direction\" id=\"direction\">\n");
		strcat(html, "<option>Omni</option><option>NE</option><option>E</option><option>SE</option><option>S</option><option>SW</option><option>W</option><option>NW</option><option>N</option>\n");
		strcat(html, "</select></td>\n");
		strcat(html, "</tr>\n");

		snprintf(tempHtml, sizeof(tempHtml), "<tr><td align=\"right\"><b>PHG Text</b></td><td align=\"left\"><input name=\"texttouse\" type=\"text\" size=\"6\" style=\"background-color: rgb(97, 239, 170);\" value=\"%s\"/> <input type=\"button\" value=\"Calculate PHG\" onclick=\"javascript:calculatePHGR()\" /></td></tr>\n", config.digi_phg);
		strcat(html, tempHtml);

		strcat(html, "</table></tr>");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Filter:</b></td>\n");

		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<fieldset id=\"FilterGrp\">\n");
		strcat(html, "<legend>Filter repeater</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">");
		strcat(html, "<tr style=\"background:unset;\">");

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterMessage\" type=\"checkbox\" value=\"OK\" %s/>Message</td>\n",
				 (config.digiFilter & FILTER_MESSAGE) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterStatus\" type=\"checkbox\" value=\"OK\" %s/>Status</td>\n",
				 (config.digiFilter & FILTER_STATUS) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterTelemetry\" type=\"checkbox\" value=\"OK\" %s/>Telemetry</td>\n",
				 (config.digiFilter & FILTER_TELEMETRY) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterWeather\" type=\"checkbox\" value=\"OK\" %s/>Weather</td>\n",
				 (config.digiFilter & FILTER_WX) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterObject\" type=\"checkbox\" value=\"OK\" %s/>Object</td>\n",
				 (config.digiFilter & FILTER_OBJECT) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "</tr><tr style=\"background:unset;\"><td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterItem\" type=\"checkbox\" value=\"OK\" %s/>Item</td>\n",
				 (config.digiFilter & FILTER_ITEM) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterQuery\" type=\"checkbox\" value=\"OK\" %s/>Query</td>\n",
				 (config.digiFilter & FILTER_QUERY) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterBuoy\" type=\"checkbox\" value=\"OK\" %s/>Buoy</td>\n",
				 (config.digiFilter & FILTER_BUOY) ? "checked" : "");
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"border:unset;\"><input class=\"field_checkbox\" name=\"FilterPosition\" type=\"checkbox\" value=\"OK\" %s/>Position</td>\n",
				 (config.digiFilter & FILTER_POSITION) ? "checked" : "");
		strcat(html, tempHtml);

		strcat(html, "<td style=\"border:unset;\"></td>");
		strcat(html, "</tr></table></fieldset>\n");
		strcat(html, "</td></tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Telemetry:</b><br />(v=0->8280)</td>\n");
		strcat(html, "<td align=\"center\"><table>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"1000\" step=\"1\" id=\"digiTlmInv\" name=\"digiTlmInv\" type=\"number\" value=\"%d\" /> *Number of packets interval,<i>Example: 0 not send,1 send every packet</i></label></td></tr>", config.digi_tlm_interval);
		strcat(html, tempHtml);
		for (int ax = 0; ax < 5; ax++)
		{
			snprintf(tempHtml, sizeof(tempHtml), "<tr><td align=\"right\"><b>CH A%d:</b></td>\n", ax + 1);
			strcat(html, tempHtml);
			strcat(html, "<td align=\"center\">\n");
			strcat(html, "<table>");

			strcat(html, "<tr><td style=\"text-align: right;\">Sensor:</td>\n");
			strcat(html, "<td style=\"text-align: left;\">CH: ");

			snprintf(tempHtml, sizeof(tempHtml), "<select name=\"sensorCH%d\" id=\"sensorCH%d\">\n", ax, ax);
			strcat(html, tempHtml);

			for (uint8_t idx = 0; idx < 11; idx++)
			{
				if (idx == 0)
				{
					if (config.digi_tlm_sensor[ax] == idx)
					{
						snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>NONE</option>\n", idx);
					}
					else
					{
						snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\">NONE</option>\n", idx);
					}
				}
				else
				{
					if (config.digi_tlm_sensor[ax] == idx)
					{
						snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>SENSOR#%d</option>\n", idx, idx);
					}
					else
					{
						snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\">SENSOR#%d</option>\n", idx, idx);
					}
				}
				strcat(html, tempHtml);
			}
			strcat(html, "</select></td>\n");

			snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\">Name: <input maxlength=\"10\" size=\"8\" name=\"param%d\" type=\"text\" value=\"%s\" /></td>\n", ax, config.digi_tlm_PARM[ax]);
			strcat(html, tempHtml);
			snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\">Unit: <input maxlength=\"8\" size=\"5\" name=\"unit%d\" type=\"text\" value=\"%s\" /></td>\n", ax, config.digi_tlm_UNIT[ax]);
			strcat(html, tempHtml);

			snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\">Precision: <input min=\"0\" max=\"5\" step=\"1\" type=\"number\" style=\"width: 2em\" name=\"precision%d\" type=\"text\" value=\"%d\" onchange=\"selPrecision(%d)\"/></td></tr>\n", ax, config.digi_tlm_precision[ax], ax);
			strcat(html, tempHtml);

			snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">EQNS:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns%da\" type=\"number\" value=\"%.5f\" />  b:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns%db\" type=\"number\" value=\"%.5f\" /> c:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns%dc\" type=\"number\" value=\"%.5f\" /> (av<sup>2</sup>+bv+c) </td>\n",
					 ax, config.digi_tlm_EQNS[ax][0], ax, config.digi_tlm_EQNS[ax][1], ax, config.digi_tlm_EQNS[ax][2]);
			strcat(html, tempHtml);

			snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\">Offset: <input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" type=\"number\" name=\"offset%d\" type=\"text\" value=\"%.5f\" onchange=\"selOffset(%d)\" /></td></tr>\n", ax, config.digi_tlm_offset[ax], ax);
			strcat(html, tempHtml);

			strcat(html, "</table></td>");
			strcat(html, "</tr>\n");
		}
		strcat(html, "</table></td></tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitDIGI'  name=\"commitDIGI\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitDIGI\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br />");

		AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, html);
		response->addHeader("digi", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
	}
}

void handle_wx(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	bool En = false;
	bool posGPS = false;
	bool pos2RF = false;
	bool pos2INET = false;
	bool timeStamp = false;
	String arg = "";

	if (request->hasArg("commitWX"))
	{
		for (int x = 0; x < WX_SENSOR_NUM; x++)
			config.wx_sensor_enable[x] = false;

		for (int i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}
			if (request->argName(i) == "Object")
			{
				if (request->arg(i) != "")
				{
					String name = request->arg(i);
					name.trim();
					strcpy(config.wx_object, name.c_str());
				}
				else
				{
					config.wx_object[0] = 0;
				}
			}
			if (request->argName(i) == "myCall")
			{
				if (request->arg(i) != "")
				{
					String name = request->arg(i);
					name.trim();
					name.toUpperCase();
					strcpy(config.wx_mycall, name.c_str());
				}
			}
			if (request->argName(i) == "mySSID")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.wx_ssid = request->arg(i).toInt();
					if (config.wx_ssid > 15)
						config.wx_ssid = 3;
				}
			}
			// if (request->argName(i) == "channel")
			// {
			// 	if (request->arg(i) != "")
			// 	{
			// 		if (isValidNumber(request->arg(i)))
			// 			config.wx_channel = request->arg(i).toInt();
			// 	}
			// }
			if (request->argName(i) == "PosInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.wx_interval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "PosLat")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.wx_lat = request->arg(i).toFloat();
				}
			}

			if (request->argName(i) == "PosLon")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.wx_lon = request->arg(i).toFloat();
				}
			}
			if (request->argName(i) == "PosAlt")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.wx_alt = request->arg(i).toFloat();
				}
			}
			if (request->argName(i) == "PosSel")
			{
				if (request->arg(i) != "")
				{
					if (request->arg(i).toInt() == 1)
						posGPS = true;
				}
			}

			if (request->argName(i) == "Path")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.wx_path = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "Comment")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.wx_comment, request->arg(i).c_str());
				}
				else
				{
					memset(config.wx_comment, 0, sizeof(config.wx_comment));
				}
			}
			if (request->argName(i) == "Pos2RF")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						pos2RF = true;
				}
			}
			if (request->argName(i) == "Pos2INET")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						pos2INET = true;
				}
			}
			if (request->argName(i) == "wxTimeStamp")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						timeStamp = true;
				}
			}
			for (int x = 0; x < WX_SENSOR_NUM; x++)
			{
				arg = "senEn" + String(x);
				if (request->argName(i) == arg)
				{

					if (request->arg(i) != "")
					{
						if (String(request->arg(i)) == "OK")
							config.wx_sensor_enable[x] = true;
					}
				}
				arg = "sensorCH" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.wx_sensor_ch[x] = request->arg(i).toInt();
				}
				arg = "avgSel" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						if (request->arg(i).toInt() == 1)
							config.wx_sensor_avg[x] = true;
						else if (request->arg(i).toInt() == 0)
							config.wx_sensor_avg[x] = false;
					}
				}
			}
		}
		config.wx_en = En;
		config.wx_gps = posGPS;
		config.wx_2rf = pos2RF;
		config.wx_2inet = pos2INET;
		config.wx_timestamp = timeStamp;

		initInterval = true;
		String html_msg;
		if (saveConfiguration("/default.cfg", config))
		{
			html_msg = "Setup completed successfully";
			request->send(200, "text/html", html_msg); // send to someones browser when asked
		}
		else
		{
			html_msg = "Save config failed.";
			request->send(501, "text/html", html_msg); // Not Implemented
		}
	}
	else
	{
		// Allocate initial memory for HTML content
		char tempHtml[300];
		char *html = allocateStringMemory(26000); // Start with 8KB buffer
		if (!html)
		{
			request->send(500, "text/html", "Memory allocation failed");
			return;
		}
		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "document.getElementById(\"submitWX\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/wx',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "</script>\n");

		/************************ WX Mode **************************/
		strcat(html, "<form id='formWX' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>[WX] Weather Station</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Enable:</b></td>\n");
		char EnFlag[10] = "";
		if (config.wx_en)
			strcpy(EnFlag, "checked");

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", EnFlag);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Station Callsign:</b></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"7\" size=\"6\" id=\"myCall\" name=\"myCall\" type=\"text\" value=\"%s\" /></td>\n", config.wx_mycall);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Station SSID:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"mySSID\" id=\"mySSID\">\n");
		for (uint8_t ssid = 0; ssid <= 15; ssid++)
		{
			if (config.wx_ssid == ssid)
			{
				snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>%d</option>\n", ssid, ssid);
			}
			else
			{
				snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\">%d</option>\n", ssid, ssid);
			}
			strcat(html, tempHtml);
		}
		strcat(html, "</select></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Object Name:</b></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"9\" size=\"9\" name=\"Object\" type=\"text\" value=\"%s\" /><i> *If not used, leave it blank.In use 3-9 charactor</i></td>\n", config.wx_object);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PATH:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"Path\" id=\"Path\">\n");
		for (uint8_t pthIdx = 0; pthIdx < PATH_LEN; pthIdx++)
		{
			if (config.wx_path == pthIdx)
			{
				snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
			}
			else
			{
				snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\">%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
			}
			strcat(html, tempHtml);
		}
		strcat(html, "</select></td>\n");
		// strcat(html, "<td style=\"text-align: left;\"><input maxlength=\"72\" size=\"72\" name=\"Path\" type=\"text\" value=\"" + String(config.wx_path) + "\" /></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Text Comment:</b></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"50\" size=\"50\" name=\"Comment\" type=\"text\" value=\"%s\" /></td>\n", config.wx_comment);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Time Stamp:</b></td>\n");
		char timeStampFlag[10];
		if (config.wx_timestamp)
			strcpy(timeStampFlag, "checked");
		else
			strcpy(timeStampFlag, "");

		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"wxTimeStamp\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", timeStampFlag);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");

		strcat(html, "<tr><td align=\"right\"><b>POSITION:</b></td>\n");
		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<table>");
		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"3600\" step=\"1\" id=\"PosInv\" name=\"PosInv\" type=\"number\" value=\"%d\" />Sec.</td></tr>", config.wx_interval);
		strcat(html, tempHtml);
		String PosFixFlag = "";
		String PosGPSFlag = "";
		String Pos2RFFlag = "";
		String Pos2INETFlag = "";
		if (config.wx_gps)
			PosGPSFlag = "checked=\"checked\"";
		else
			PosFixFlag = "checked=\"checked\"";

		if (config.wx_2rf)
			Pos2RFFlag = "checked";
		if (config.wx_2inet)
			Pos2INETFlag = "checked";

		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Location:</td><td style=\"text-align: left;\"><input type=\"radio\" name=\"PosSel\" value=\"0\" %s/>Fix <input type=\"radio\" name=\"PosSel\" value=\"1\" %s/>GPS </td></tr>\n", PosFixFlag.c_str(), PosGPSFlag.c_str());
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">TX Channel:</td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"Pos2RF\" value=\"OK\" %s/>RF <input type=\"checkbox\" name=\"Pos2INET\" value=\"OK\" %s/>Internet </td></tr>\n", Pos2RFFlag.c_str(), Pos2INETFlag.c_str());
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Latitude:</td><td style=\"text-align: left;\"><input min=\"-90\" max=\"90\" step=\"0.00001\" name=\"PosLat\" type=\"number\" value=\"%.5f\" />degrees (positive for North, negative for South)</td></tr>\n", config.wx_lat);
		strcat(html, tempHtml);

		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Longitude:</td><td style=\"text-align: left;\"><input min=\"-180\" max=\"180\" step=\"0.00001\" name=\"PosLon\" type=\"number\" value=\"%.5f\" />degrees (positive for East, negative for West)</td></tr>\n", config.wx_lon);
		strcat(html, tempHtml);
		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Altitude:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"10000\" step=\"0.1\" name=\"PosAlt\" type=\"number\" value=\"%.2f\" /> meter. *The altitude in meters(m) above sea level</td></tr>\n", config.wx_alt);
		strcat(html, tempHtml);

		strcat(html, "</table></td>");
		strcat(html, "</tr>\n");

		// strcat(html, "<tr>\n");
		// strcat(html, "<td align=\"right\"><b>PORT:</b></td>\n");
		// strcat(html, "<td style=\"text-align: left;\">\n");
		// strcat(html, "<select name=\"channel\" id=\"channel\">\n");
		// for (int i = 0; i < 5; i++)
		// {
		// 	if (config.wx_channel == i)
		// 		strcat(html, "<option value=\"" + String(i) + "\" selected>" + String(WX_PORT[i]) + " </option>\n");
		// 	else
		// 		strcat(html, "<option value=\"" + String(i) + "\" >" + String(WX_PORT[i]) + " </option>\n");
		// }
		// strcat(html, "</select>\n");
		// strcat(html, "</td>\n");
		// strcat(html, "</tr>\n");
		/************************ Sensor Config Mode **************************/
		// strcat(html, "<form id='formSENSOR' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		// strcat(html, "<table>\n");
		// strcat(html, "<th colspan=\"2\"><span><b>Sensor Config</b></span></th>\n");

		strcat(html, "<tr><td align=\"right\"><b>SENSOR:<br />Selection</b></td>\n");
		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<table>");

		for (int ax = 0; ax < WX_SENSOR_NUM; ax++)
		{
			snprintf(tempHtml, sizeof(tempHtml), "<tr><td align=\"right\"><b>%s:</b> \n", WX_SENSOR[ax]);
			strcat(html, tempHtml);

			char EnFlag[16] = "";
			if (config.wx_sensor_enable[ax])
				snprintf(EnFlag, sizeof(EnFlag), "checked");

			snprintf(tempHtml, sizeof(tempHtml), "<label class=\"switch\"><input type=\"checkbox\" name=\"senEn%d\" value=\"OK\" %s><span class=\"slider round\"></span></label>", ax, EnFlag);
			strcat(html, tempHtml);

			strcat(html, "</td>\n");

			// strcat(html, "<td style=\"text-align: lefe;\">Sensor:</td>\n");
			strcat(html, "<td style=\"text-align: left;\">Sensor Channel: ");

			snprintf(tempHtml, sizeof(tempHtml), "<select name=\"sensorCH%d\" id=\"sensorCH%d\">\n", ax, ax);
			strcat(html, tempHtml);
			for (uint8_t idx = 0; idx < 11; idx++)
			{
				if (idx == 0)
				{
					if (config.wx_sensor_ch[ax] == idx)
					{
						snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>NONE</option>\n", idx);
					}
					else
					{
						snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\">NONE</option>\n", idx);
					}
				}
				else
				{
					if (config.wx_sensor_ch[ax] == idx)
					{
						snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>SENSOR#%d</option>\n", idx, idx);
					}
					else
					{
						snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\">SENSOR#%d</option>\n", idx, idx);
					}
				}
				strcat(html, tempHtml);
			}
			strcat(html, "</select>\n");
			String avgFlag = "";
			String sampleFlag = "";
			if (config.wx_sensor_avg[ax])
				avgFlag = "checked=\"checked\"";
			else
				sampleFlag = "checked=\"checked\"";

			snprintf(tempHtml, sizeof(tempHtml), "<input type=\"radio\" name=\"avgSel%d\" value=\"0\" %s/>Sample <input type=\"radio\" name=\"avgSel%d\" value=\"1\" %s/>Average", ax, sampleFlag, ax, avgFlag);
			strcat(html, tempHtml);
			strcat(html, "</td></tr>");
		}
		strcat(html, "</table></td></tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitWX'  name=\"commitWX\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitWX\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br />");

		AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, html);
		response->addHeader("Weather", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
	}
}

void handle_tlm(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	bool En = false;
	bool pos2RF = false;
	bool pos2INET = false;
	String arg = "";

	if (request->hasArg("commitTLM"))
	{
		for (int i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "Enable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						En = true;
				}
			}
			if (request->argName(i) == "myCall")
			{
				if (request->arg(i) != "")
				{
					String name = request->arg(i);
					name.trim();
					name.toUpperCase();
					strcpy(config.tlm0_mycall, name.c_str());
				}
			}
			if (request->argName(i) == "mySSID")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.tlm0_ssid = request->arg(i).toInt();
					if (config.tlm0_ssid > 15)
						config.tlm0_ssid = 3;
				}
			}
			if (request->argName(i) == "infoInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.tlm0_info_interval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "dataInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.tlm0_data_interval = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "Path")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.tlm0_path = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "Comment")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.tlm0_comment, request->arg(i).c_str());
				}
				else
				{
					memset(config.tlm0_comment, 0, sizeof(config.tlm0_comment));
				}
			}
			if (request->argName(i) == "Pos2RF")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						pos2RF = true;
				}
			}
			if (request->argName(i) == "Pos2INET")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						pos2INET = true;
				}
			}
			for (int x = 0; x < 13; x++)
			{
				arg = "sensorCH" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.tml0_data_channel[x] = request->arg(i).toInt();
				}
				arg = "param" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.tlm0_PARM[x], request->arg(i).c_str());
					}
				}
				arg = "unit" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.tlm0_UNIT[x], request->arg(i).c_str());
					}
				}
				if (x < 5)
				{
					for (int y = 0; y < 3; y++)
					{
						arg = "eqns" + String(x) + String((char)(y + 'a'));
						if (request->argName(i) == arg)
						{
							if (isValidNumber(request->arg(i)))
								config.tlm0_EQNS[x][y] = request->arg(i).toFloat();
						}
					}
				}
			}
			uint8_t b = 1;
			for (int x = 0; x < 8; x++)
			{
				arg = "bitact" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
					{
						if (request->arg(i).toInt() == 1)
						{
							config.tlm0_BITS_Active |= b;
						}
						else
						{
							config.tlm0_BITS_Active &= ~b;
						}
					}
				}
				b <<= 1;
			}
		}
		config.tlm0_en = En;
		config.tlm0_2rf = pos2RF;
		config.tlm0_2inet = pos2INET;

		initInterval = true;
		String html_msg;
		if (saveConfiguration("/default.cfg", config))
		{
			html_msg = "Setup completed successfully";
			request->send(200, "text/html", html_msg); // send to someones browser when asked
		}
		else
		{
			html_msg = "Save config failed.";
			request->send(501, "text/html", html_msg); // Not Implemented
		}
	}
	else
	{
		// Allocate initial memory for HTML content
		char *html = allocateStringMemory(18000); // Start with 8KB buffer
		if (!html)
		{
			request->send(500, "text/html", "Memory allocation failed");
			return;
		}
		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "document.getElementById(\"submitTLM\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/tlm',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "</script>\n");

		/************************ TLM Mode **************************/
		strcat(html, "<form id='formTLM' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>System Telemetry</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Enable:</b></td>\n");
		char EnFlag[10] = "";
		if (config.tlm0_en)
			strcpy(EnFlag, "checked");
		{
			char *temp_flag = allocateStringMemory(512);
			if (temp_flag)
			{
				snprintf(temp_flag, 512, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"Enable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", EnFlag);
				strcat(html, temp_flag);
				free(temp_flag);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		{
			char *temp_callsign = allocateStringMemory(512);
			if (temp_callsign)
			{
				snprintf(temp_callsign, 512, "<td align=\"right\"><b>Station Callsign:</b></td>\n<td style=\"text-align: left;\"><input maxlength=\"7\" size=\"6\" id=\"myCall\" name=\"myCall\" type=\"text\" value=\"%s\" /></td>\n", config.tlm0_mycall);
				strcat(html, temp_callsign);
				free(temp_callsign);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Station SSID:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"mySSID\" id=\"mySSID\">\n");
		for (uint8_t ssid = 0; ssid <= 15; ssid++)
		{
			char *temp_option = allocateStringMemory(256);
			if (temp_option)
			{
				if (config.tlm0_ssid == ssid)
				{
					snprintf(temp_option, 256, "<option value=\"%d\" selected>%d</option>\n", ssid, ssid);
				}
				else
				{
					snprintf(temp_option, 256, "<option value=\"%d\">%d</option>\n", ssid, ssid);
				}
				strcat(html, temp_option);
				free(temp_option);
			}
		}
		strcat(html, "</select></td>\n");
		strcat(html, "</tr>\n");

		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PATH:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"Path\" id=\"Path\">\n");
		for (uint8_t pthIdx = 0; pthIdx < PATH_LEN; pthIdx++)
		{
			{
				char *temp_path = allocateStringMemory(256);
				if (temp_path)
				{
					if (config.tlm0_path == pthIdx)
					{
						snprintf(temp_path, 256, "<option value=\"%d\" selected>%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
					}
					else
					{
						snprintf(temp_path, 256, "<option value=\"%d\">%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
					}
					strcat(html, temp_path);
					free(temp_path);
				}
			}
		}
		strcat(html, "</select></td>\n");
		// strcat(html, "<td style=\"text-align: left;\"><input maxlength=\"72\" size=\"72\" name=\"Path\" type=\"text\" value=\"" + String(config.tlm0_path) + "\" /></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		{
			char *temp_comment = allocateStringMemory(512);
			if (temp_comment)
			{
				snprintf(temp_comment, 512, "<td align=\"right\"><b>Text Comment:</b></td>\n<td style=\"text-align: left;\"><input maxlength=\"50\" size=\"50\" name=\"Comment\" type=\"text\" value=\"%s\" /></td>\n", config.tlm0_comment);
				strcat(html, temp_comment);
				free(temp_comment);
			}
		}
		strcat(html, "</tr>\n");

		{
			char *temp_intervals = allocateStringMemory(1024);
			if (temp_intervals)
			{
				snprintf(temp_intervals, 1024, "<tr><td style=\"text-align: right;\">Info Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"3600\" step=\"1\" name=\"infoInv\" type=\"number\" value=\"%d\" />Sec.</td></tr>", config.tlm0_info_interval);
				strcat(html, temp_intervals);
				snprintf(temp_intervals, 1024, "<tr><td style=\"text-align: right;\">Data Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"3600\" step=\"1\" name=\"dataInv\" type=\"number\" value=\"%d\" />Sec.</td></tr>", config.tlm0_data_interval);
				strcat(html, temp_intervals);
				free(temp_intervals);
			}
		}

		char Pos2RFFlag[10] = "";
		char Pos2INETFlag[10] = "";
		if (config.tlm0_2rf)
			strcpy(Pos2RFFlag, "checked");
		if (config.tlm0_2inet)
			strcpy(Pos2INETFlag, "checked");
		{
			char *temp_channels = allocateStringMemory(1024);
			if (temp_channels)
			{
				snprintf(temp_channels, 1024, "<tr><td style=\"text-align: right;\">TX Channel:</td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"Pos2RF\" value=\"OK\" %s/>RF <input type=\"checkbox\" name=\"Pos2INET\" value=\"OK\" %s/>Internet </td></tr>\n", Pos2RFFlag, Pos2INETFlag);
				strcat(html, temp_channels);
				free(temp_channels);
			}
		}

		// strcat(html, "<tr>\n");
		// strcat(html, "<td align=\"right\"><b>Time Stamp:</b></td>\n");
		// String timeStampFlag = "";
		// if (config.wx_timestamp)
		// 	timeStampFlag = "checked";
		// strcat(html, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"wxTimeStamp\" value=\"OK\" " + timeStampFlag + "><span class=\"slider round\"></span></label></td>\n");
		// strcat(html, "</tr>\n");
		for (int ax = 0; ax < 5; ax++)
		{
			{
				char *temp_channel_a = allocateStringMemory(512);
				if (temp_channel_a)
				{
					snprintf(temp_channel_a, 512, "<tr><td align=\"right\"><b>Channel A%d:</b></td>\n", ax + 1);
					strcat(html, temp_channel_a);
					free(temp_channel_a);
				}
			}
			strcat(html, "<td align=\"center\">\n");
			strcat(html, "<table>");

			// strcat(html, "<tr><td style=\"text-align: right;\">Name:</td><td style=\"text-align: center;\"><i>Sensor Type</i></td><td style=\"text-align: center;\"><i>Parameter</i></td><td style=\"text-align: center;\"><i>Unit</i></td></tr>\n");

			strcat(html, "<tr><td style=\"text-align: right;\">Type/Name:</td>\n");
			strcat(html, "<td style=\"text-align: left;\">Sensor Type: ");
			{
				char *temp_select = allocateStringMemory(256);
				if (temp_select)
				{
					snprintf(temp_select, 256, "<select name=\"sensorCH%d\" id=\"sensorCH%d\">\n", ax, ax);
					strcat(html, temp_select);
					free(temp_select);
				}
			}
			for (uint8_t idx = 0; idx < SYSTEM_LEN; idx++)
			{
				{
					char *temp_option = allocateStringMemory(256);
					if (temp_option)
					{
						if (config.tml0_data_channel[ax] == idx)
						{
							snprintf(temp_option, 256, "<option value=\"%d\" selected>%s</option>\n", idx, SYSTEM_NAME[idx]);
						}
						else
						{
							snprintf(temp_option, 256, "<option value=\"%d\">%s</option>\n", idx, SYSTEM_NAME[idx]);
						}
						strcat(html, temp_option);
						free(temp_option);
					}
				}
			}
			strcat(html, "</select></td>\n");

			{
				char *temp_param = allocateStringMemory(512);
				if (temp_param)
				{
					snprintf(temp_param, 512, "<td style=\"text-align: left;\">Parameter: <input maxlength=\"10\" size=\"8\" name=\"param%d\" type=\"text\" value=\"%s\" /></td>\n", ax, config.tlm0_PARM[ax]);
					strcat(html, temp_param);
					free(temp_param);
				}
			}
			{
				char *temp_unit = allocateStringMemory(512);
				if (temp_unit)
				{
					snprintf(temp_unit, 512, "<td style=\"text-align: left;\">Unit: <input maxlength=\"8\" size=\"5\" name=\"unit%d\" type=\"text\" value=\"%s\" /></td>\n", ax, config.tlm0_UNIT[ax]);
					strcat(html, temp_unit);
					free(temp_unit);
				}
			}
			strcat(html, "</tr>\n");
			{
				char *temp_eqns = allocateStringMemory(1024);
				if (temp_eqns)
				{
					snprintf(temp_eqns, 1024, "<tr><td style=\"text-align: right;\">EQNS:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-9999\" max=\"9999\" step=\"0.0001\" name=\"eqns%da\" type=\"number\" value=\"%.3f\" />  b:<input min=\"-9999\" max=\"9999\" step=\"0.0001\" name=\"eqns%db\" type=\"number\" value=\"%.3f\" /> c:<input min=\"-9999\" max=\"9999\" step=\"0.0001\" name=\"eqns%dc\" type=\"number\" value=\"%.3f\" /> (av<sup>2</sup>+bv+c)</td></tr>\n",
							 ax, config.tlm0_EQNS[ax][0], ax, config.tlm0_EQNS[ax][1], ax, config.tlm0_EQNS[ax][2]);
					strcat(html, temp_eqns);
					free(temp_eqns);
				}
			}
			strcat(html, "</table></td>");
			strcat(html, "</tr>\n");
		}

		uint8_t b = 1;
		for (int ax = 0; ax < 8; ax++)
		{
			{
				char *temp_channel_b = allocateStringMemory(512);
				if (temp_channel_b)
				{
					snprintf(temp_channel_b, 512, "<tr><td align=\"right\"><b>Channel B%d:</b></td>\n", ax + 1);
					strcat(html, temp_channel_b);
					free(temp_channel_b);
				}
			}
			strcat(html, "<td align=\"center\">\n");
			strcat(html, "<table>");

			// strcat(html, "<tr><td style=\"text-align: right;\">Type/Name:</td>\n");
			strcat(html, "<td style=\"text-align: left;\">Type: ");
			{
				char *temp_select_b = allocateStringMemory(256);
				if (temp_select_b)
				{
					snprintf(temp_select_b, 256, "<select name=\"sensorCH%d\" id=\"sensorCH%d\">\n", ax + 5, ax);
					strcat(html, temp_select_b);
					free(temp_select_b);
				}
			}
			for (uint8_t idx = 0; idx < SYSTEM_BIT_LEN; idx++)
			{
				{
					char *temp_option_b = allocateStringMemory(256);
					if (temp_option_b)
					{
						if (config.tml0_data_channel[ax + 5] == idx)
						{
							snprintf(temp_option_b, 256, "<option value=\"%d\" selected>%s</option>\n", idx, SYSTEM_BITS_NAME[idx]);
						}
						else
						{
							snprintf(temp_option_b, 256, "<option value=\"%d\">%s</option>\n", idx, SYSTEM_BITS_NAME[idx]);
						}
						strcat(html, temp_option_b);
						free(temp_option_b);
					}
				}
			}
			strcat(html, "</select></td>\n");

			{
				char *temp_param_b = allocateStringMemory(512);
				if (temp_param_b)
				{
					snprintf(temp_param_b, 512, "<td style=\"text-align: left;\">Parameter: <input maxlength=\"10\" size=\"8\" name=\"param%d\" type=\"text\" value=\"%s\" /></td>\n", ax + 5, config.tlm0_PARM[ax + 5]);
					strcat(html, temp_param_b);
					free(temp_param_b);
				}
			}
			{
				char *temp_unit_b = allocateStringMemory(512);
				if (temp_unit_b)
				{
					snprintf(temp_unit_b, 512, "<td style=\"text-align: left;\">Unit: <input maxlength=\"8\" size=\"5\" name=\"unit%d\" type=\"text\" value=\"%s\" /></td>\n", ax + 5, config.tlm0_UNIT[ax + 5]);
					strcat(html, temp_unit_b);
					free(temp_unit_b);
				}
			}
			char LowFlag[20] = "", HighFlag[20] = "";
			if (config.tlm0_BITS_Active & b)
				strcpy(HighFlag, "checked=\"checked\"");
			else
				strcpy(LowFlag, "checked=\"checked\"");
			{
				char *temp_radio_b = allocateStringMemory(512);
				if (temp_radio_b)
				{
					snprintf(temp_radio_b, 512, "<td style=\"text-align: left;\"> Active:<input type=\"radio\" name=\"bitact%d\" value=\"0\" %s/>LOW <input type=\"radio\" name=\"bitact%d\" value=\"1\" %s/>HIGH </td>\n", ax, LowFlag, ax, HighFlag);
					strcat(html, temp_radio_b);
					free(temp_radio_b);
				}
			}
			strcat(html, "</tr>\n");
			// strcat(html, "<tr><td style=\"text-align: right;\">EQNS:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-999\" max=\"999\" step=\"0.1\" name=\"eqns" + String(ax + 1) + "a\" type=\"number\" value=\"" + String(config.tlm0_EQNS[ax][0], 3) + "\" />  b:<input min=\"-999\" max=\"999\" step=\"0.1\" name=\"eqns" + String(ax + 1) + "b\" type=\"number\" value=\"" + String(config.tlm0_EQNS[ax][1], 3) + "\" /> c:<input min=\"-999\" max=\"999\" step=\"0.1\" name=\"eqns" + String(ax + 1) + "c\" type=\"number\" value=\"" + String(config.tlm0_EQNS[ax][2], 3) + "\" /> (av<sup>2</sup>+bv+c)</td></tr>\n";
			strcat(html, "</table></td>");
			strcat(html, "</tr>\n");
			b <<= 1;
		}

		// strcat(html, "<tr><td align=\"right\"><b>Parameter Name:</b></td>\n";
		// strcat(html, "<td align=\"center\">\n";
		// strcat(html, "<table>";

		// // strcat(html, "<tr><td style=\"text-align: right;\">Latitude:</td><td style=\"text-align: left;\"><input min=\"-90\" max=\"90\" step=\"0.00001\" name=\"PosLat\" type=\"number\" value=\"" + String(config.wx_lat, 5) + "\" />degrees (positive for North, negative for South)</td></tr>\n";
		// // strcat(html, "<tr><td style=\"text-align: right;\">Longitude:</td><td style=\"text-align: left;\"><input min=\"-180\" max=\"180\" step=\"0.00001\" name=\"PosLon\" type=\"number\" value=\"" + String(config.wx_lon, 5) + "\" />degrees (positive for East, negative for West)</td></tr>\n";
		// // strcat(html, "<tr><td style=\"text-align: right;\">Altitude:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"10000\" step=\"0.1\" name=\"PosAlt\" type=\"number\" value=\"" + String(config.wx_alt, 2) + "\" /> meter. *Value 0 is not send height</td></tr>\n";
		// strcat(html, "</table></td>";
		// strcat(html, "</tr>\n";
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitTLM'  name=\"commitTLM\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitTLM\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br />");

		AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, html);
		response->addHeader("Telemetry", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
	}
}

extern TaskHandle_t taskSensorHandle;

void handle_sensor(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);
	String arg = "";

	if (request->hasArg("commitSENSOR"))
	{
		// vTaskSuspend(taskSensorHandle);
		for (int x = 0; x < SENSOR_NUMBER; x++)
		{
			config.sensor[x].enable = false;
		}
		for (int i = 0; i < request->args(); i++)
		{
			// log_d("Arg %s: %s", request->argName(i).c_str(), request->arg(i).c_str());
			for (int x = 0; x < SENSOR_NUMBER; x++)
			{
				arg = "En" + String(x);
				if (request->argName(i) == arg)
				{

					if (request->arg(i) != "")
					{
						if (String(request->arg(i)) == "OK")
							config.sensor[x].enable = true;
					}
				}
				arg = "sensorCH" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.sensor[x].type = request->arg(i).toInt();
				}
				arg = "sensorP" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.sensor[x].port = request->arg(i).toInt();
				}
				arg = "address" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.sensor[x].address = request->arg(i).toInt();
				}
				arg = "sample" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.sensor[x].samplerate = request->arg(i).toInt();
				}
				arg = "avg" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.sensor[x].averagerate = request->arg(i).toInt();
				}
				arg = "param" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.sensor[x].parm, request->arg(i).c_str());
					}
				}
				arg = "unit" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.sensor[x].unit, request->arg(i).c_str());
					}
				}
				for (int y = 0; y < 3; y++)
				{
					arg = "eqns" + String(x) + String((char)(y + 'a'));
					if (request->argName(i) == arg)
					{
						if (isValidNumber(request->arg(i)))
							config.sensor[x].eqns[y] = request->arg(i).toFloat();
					}
				}
				//}
			}
		}

		log_d("Sensor Config Updated.");
		String html_msg;
		if (saveConfiguration("/default.cfg", config))
		{
			html_msg = "Setup completed successfully";
			request->send(200, "text/html", html_msg); // send to someones browser when asked
		}
		else
		{
			html_msg = "Save config failed.";
			request->send(501, "text/html", html_msg); // Not Implemented
		}
		// vTaskResume(taskSensorHandle);
	}
	else
	{
		// Allocate initial memory for HTML content
		char *html = allocateStringMemory(25000); // Start with 8KB buffer
		if (!html)
		{
			request->send(500, "text/html", "Memory allocation failed");
			return;
		}
		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "document.getElementById(\"submitSENSOR\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/sensor',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "function setElm(name,val) {\n");
		strcat(html, "document.getElementById(name).value=val;\n");
		strcat(html, "};\n");
		strcat(html, "function selSensorType(idx) {\n");
		strcat(html, "var x=0;\n");
		strcat(html, "var parm=\"param\"+idx;\n");
		strcat(html, "var unit=\"unit\"+idx;\n");
		strcat(html, "x = document.getElementById(\"sensorCH\"+idx).value;\n");
		strcat(html, "if (x==1) {\n");
		strcat(html, "setElm(parm,\"Co2\");");
		strcat(html, "setElm(unit,\"ppm\");\n");
		strcat(html, "}else if (x==2) {\n");
		strcat(html, "setElm(parm,\"CH2O\");");
		strcat(html, "setElm(unit,\"μg/m³\");\n");
		strcat(html, "}else if (x==3) {\n");
		strcat(html, "setElm(parm,\"TVOC\");");
		strcat(html, "setElm(unit,\"μg/m³\");\n");
		strcat(html, "}else if (x==4) {\n");
		strcat(html, "setElm(parm,\"PM2.5\");");
		strcat(html, "setElm(unit,\"μg/m³\");\n");
		strcat(html, "}else if (x==5) {\n");
		strcat(html, "setElm(parm,\"PM10.0\");");
		strcat(html, "setElm(unit,\"μg/m³\");\n");
		strcat(html, "}else if (x==6) {\n");
		strcat(html, "setElm(parm,\"Temperature\");");
		strcat(html, "setElm(unit,\"°C\");\n");
		strcat(html, "}else if (x==7) {\n");
		strcat(html, "setElm(parm,\"Humidity\");");
		strcat(html, "setElm(unit,\"%RH\");\n");
		strcat(html, "}else if (x==8) {\n");
		strcat(html, "setElm(parm,\"Pressure\");");
		strcat(html, "setElm(unit,\"hPa\");\n");
		strcat(html, "}else if (x==9) {\n");
		strcat(html, "setElm(parm,\"WindSpeed\");");
		strcat(html, "setElm(unit,\"kPh\");\n");
		strcat(html, "}else if (x==10) {\n");
		strcat(html, "setElm(parm,\"WindCourse\");");
		strcat(html, "setElm(unit,\"°\");\n");
		strcat(html, "}else if (x==11) {\n");
		strcat(html, "setElm(parm,\"Rain\");");
		strcat(html, "setElm(unit,\"mm\");\n");
		strcat(html, "}else if (x==12) {\n");
		strcat(html, "setElm(parm,\"Luminosity\");");
		strcat(html, "setElm(unit,\"W/m³\");\n");
		strcat(html, "}else if (x==13) {\n");
		strcat(html, "setElm(parm,\"SoilTemp\");");
		strcat(html, "setElm(unit,\"°C\");\n");
		strcat(html, "}else if (x==14) {\n");
		strcat(html, "setElm(parm,\"SoilMoisture\");");
		strcat(html, "setElm(unit,\"%VWC\");\n");
		strcat(html, "}else if (x==15) {\n");
		strcat(html, "setElm(parm,\"WaterTemp\");");
		strcat(html, "setElm(unit,\"°C\");\n");
		strcat(html, "}else if (x==16) {\n");
		strcat(html, "setElm(parm,\"WaterTDS\");");
		strcat(html, "setElm(unit,\" \");\n");
		strcat(html, "}else if (x==17) {\n");
		strcat(html, "setElm(parm,\"WaterLevel\");");
		strcat(html, "setElm(unit,\"mm\");\n");
		strcat(html, "}else if (x==18) {\n");
		strcat(html, "setElm(parm,\"WaterFlow\");");
		strcat(html, "setElm(unit,\"L/min\");\n");
		strcat(html, "}else if (x==19) {\n");
		strcat(html, "setElm(parm,\"Voltage\");");
		strcat(html, "setElm(unit,\"V\");\n");
		strcat(html, "}else if (x==20) {\n");
		strcat(html, "setElm(parm,\"Current\");");
		strcat(html, "setElm(unit,\"A\");\n");
		strcat(html, "}else if (x==21) {\n");
		strcat(html, "setElm(parm,\"Power\");");
		strcat(html, "setElm(unit,\"W\");\n");
		strcat(html, "}else if (x==22) {\n");
		strcat(html, "setElm(parm,\"Energy\");");
		strcat(html, "setElm(unit,\"Wh\");\n");
		strcat(html, "}else if (x==23) {\n");
		strcat(html, "setElm(parm,\"Frequency\");");
		strcat(html, "setElm(unit,\"Hz\");\n");
		strcat(html, "}else if (x==24) {\n");
		strcat(html, "setElm(parm,\"PF\");");
		strcat(html, "setElm(unit,\" \");\n");
		strcat(html, "}else if (x==25) {\n");
		strcat(html, "setElm(parm,\"Satellite\");");
		strcat(html, "setElm(unit,\" \");\n");
		strcat(html, "}else if (x==26) {\n");
		strcat(html, "setElm(parm,\"HDOP\");");
		strcat(html, "setElm(unit,\" \");\n");
		strcat(html, "}else if (x==27) {\n");
		strcat(html, "setElm(parm,\"Battery\");");
		strcat(html, "setElm(unit,\"V\");\n");
		strcat(html, "}else if (x==28) {\n");
		strcat(html, "setElm(parm,\"BattLevel\");");
		strcat(html, "setElm(unit,\"%\");\n");
		strcat(html, "}\n}\n");

		strcat(html, "function selSensor(idx) {\n");
		strcat(html, "var x=0;\n");
		strcat(html, "x = document.getElementById(\"sensorP\"+idx).value;\n");
		strcat(html, "if (x>=10 && x<=13) {\n");
#ifdef TTGO_T_Beam_S3_SUPREME_V3
		strcat(html, "document.getElementById(\"address\"+idx).value=119;\n");
#else
		strcat(html, "document.getElementById(\"address\"+idx).value=118;\n");
#endif
		strcat(html, "}else if (x==16 || x==17) {\n");
		strcat(html, "document.getElementById(\"address\"+idx).value=90;\n");
		strcat(html, "}else if (x==23) {\n");
		strcat(html, "document.getElementById(\"address\"+idx).value=1;\n");
		strcat(html, "}else if (x==24 || x==25) {\n");
		strcat(html, "document.getElementById(\"address\"+idx).value=1000;\n");
		strcat(html, "}else{\n");
		strcat(html, "document.getElementById(\"address\"+idx).value=0;\n");
		strcat(html, "}\n}\n");
		strcat(html, "</script>\n");

		/************************ Sensor Monitor **************************/
		// strcat(html, "<form id='formSENSOR' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"5\"><span><b>Sensor Monitor</b></span></th>\n");
		int ax = 0;
		for (int r = 0; r < 3; r++)
		{
			strcat(html, "<tr>\n");
			for (int c = 0; c < 5; c++)
			{
				strcat(html, "<td align=\"center\">\n");
				if (config.sensor[ax].enable)
				{
					{
						char *temp_fieldset = allocateStringMemory(256);
						if (temp_fieldset)
						{
							snprintf(temp_fieldset, 256, "<fieldset id=\"SenGrp%d\">\n", ax + 1);
							strcat(html, temp_fieldset);
							free(temp_fieldset);
						}
					}
				}
				else
				{
					{
						char *temp_fieldset = allocateStringMemory(256);
						if (temp_fieldset)
						{
							snprintf(temp_fieldset, 256, "<fieldset id=\"SenGrp%d\" disabled>\n", ax + 1);
							strcat(html, temp_fieldset);
							free(temp_fieldset);
						}
					}
				}

				{
					char *temp_legend = allocateStringMemory(512);
					if (temp_legend)
					{
						snprintf(temp_legend, 512, "<legend>SEN#%d-%s</legend>\n", ax + 1, config.sensor[ax].parm);
						strcat(html, temp_legend);
						free(temp_legend);
					}
				}
				{
					char *temp_input = allocateStringMemory(512);
					if (temp_input)
					{
						snprintf(temp_input, 512, "<input id=\"sVal%d\" style=\"text-align:right;\" size=\"5\" type=\"text\" value=\"%.2f\" readonly/> %s\n", ax, sen[ax].sample, config.sensor[ax].unit);
						strcat(html, temp_input);
						free(temp_input);
					}
				}
				strcat(html, "</td>\n");
				ax++;
				if (ax >= SENSOR_NUMBER)
					break;
			}
			strcat(html, "</tr>\n");
			if (ax >= SENSOR_NUMBER)
				break;
		}
		strcat(html, "</table>< /br>\n");

		/************************ Sensor Config Mode **************************/
		strcat(html, "<form id='formSENSOR' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>Sensor Config</b></span></th>\n");
		String EnFlag = "";

		for (int ax = 0; ax < SENSOR_NUMBER; ax++)
		{
			{
				char *temp_sensor = allocateStringMemory(256);
				if (temp_sensor)
				{
					snprintf(temp_sensor, 256, "<tr><td align=\"right\"><b>SENSOR#%d:</b><br />\n", ax + 1);
					strcat(html, temp_sensor);
					free(temp_sensor);
				}
			}
			EnFlag = "";
			if (config.sensor[ax].enable)
				EnFlag = "checked";
			{
				char *temp_checkbox = allocateStringMemory(256);
				if (temp_checkbox)
				{
					snprintf(temp_checkbox, 256, "<label class=\"switch\"><input type=\"checkbox\" name=\"En%d\" value=\"OK\" %s><span class=\"slider round\"></span></label>", ax, EnFlag);
					strcat(html, temp_checkbox);
					free(temp_checkbox);
				}
			}
			strcat(html, "</td><td align=\"center\">\n");
			strcat(html, "<table>");

			strcat(html, "<tr><td style=\"text-align: right;\">Type:</td>\n");
			strcat(html, "<td style=\"text-align: left;\">");
			{
				char *temp_select = allocateStringMemory(256);
				if (temp_select)
				{
					snprintf(temp_select, 256, "<select name=\"sensorCH%d\" id=\"sensorCH%d\" onchange=\"selSensorType(%d)\">\n", ax, ax, ax);
					strcat(html, temp_select);
					free(temp_select);
				}
			}
			// for (uint8_t idx = 0; idx < SENSOR_NAME_NUM; idx++)
			// {
			// 	if (config.sensor[ax].type == idx)
			// 	{
			// 		strcat(html, "<option value=\"" + String(idx) + "\" selected>" + String(SENSOR_NAME[idx]) + "</option>\n");
			// 	}
			// 	else
			// 	{
			// 		strcat(html, "<option value=\"" + String(idx) + "\">" + String(SENSOR_NAME[idx]) + "</option>\n");
			// 	}
			// }
			strcat(html, "</select></td>\n");

			{
				char *temp_name = allocateStringMemory(512);
				if (temp_name)
				{
					snprintf(temp_name, 512, "<td style=\"text-align: left;\">Name: <input maxlength=\"15\" size=\"15\" name=\"param%d\" id=\"param%d\" type=\"text\" value=\"%s\" /></td>\n", ax, ax, config.sensor[ax].parm);
					strcat(html, temp_name);
					free(temp_name);
				}
			}
			{
				char *temp_unit = allocateStringMemory(512);
				if (temp_unit)
				{
					snprintf(temp_unit, 512, "<td style=\"text-align: left;\">Unit: <input maxlength=\"10\" size=\"5\" name=\"unit%d\" id=\"unit%d\" type=\"text\" value=\"%s\" /></td>\n", ax, ax, config.sensor[ax].unit);
					strcat(html, temp_unit);
					free(temp_unit);
				}
			}
			strcat(html, "</tr>\n");
			// strcat(html, "<tr><td style=\"text-align: right;\">Port:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-999\" max=\"999\" step=\"0.1\" name=\"eqns" + String(ax) + "a\" type=\"number\" value=\"" + String(config.sensor[ax].eqns[0], 3) + "\" />  b:<input min=\"-999\" max=\"999\" step=\"0.1\" name=\"eqns" + String(ax) + "b\" type=\"number\" value=\"" + String(config.sensor[ax].eqns[1], 3) + "\" /> c:<input min=\"-999\" max=\"999\" step=\"0.1\" name=\"eqns" + String(ax) + "c\" type=\"number\" value=\"" + String(config.sensor[ax].eqns[2], 3) + "\" /> (av<sup>2</sup>+bv+c)</td></tr>\n";
			{
				char *temp_port = allocateStringMemory(256);
				if (temp_port)
				{
					snprintf(temp_port, 256, "<tr><td style=\"text-align: right;\">PORT:</td>\n<td style=\"text-align: left;\">\n<select name=\"sensorP%d\" id=\"sensorP%d\" onchange=\"selSensor(%d)\">\n", ax, ax, ax);
					strcat(html, temp_port);
					free(temp_port);
				}
			}
			// for (uint8_t idx = 0; idx < SENSOR_PORT_NUM; idx++)
			// {
			// 	if (config.sensor[ax].port == idx)
			// 	{
			// 		strcat(html, "<option value=\"" + String(idx) + "\" selected>" + String(SENSOR_PORT[idx]) + "</option>\n";
			// 	}
			// 	else
			// 	{
			// 		strcat(html, "<option value=\"" + String(idx) + "\">" + String(SENSOR_PORT[idx]) + "</option>\n";
			// 	}
			// }
			strcat(html, "</select></td>\n");
			{
				char *temp_addr = allocateStringMemory(512);
				if (temp_addr)
				{
					snprintf(temp_addr, 512, "<td style=\"text-align: left;\">Addr/Reg/GPIO: <input style=\"text-align:right;\" min=\"0\" max=\"6500\" step=\"1\" name=\"address%d\" id=\"address%d\" type=\"number\" value=\"%d\" /></td>\n", ax, ax, config.sensor[ax].address);
					strcat(html, temp_addr);
					free(temp_addr);
				}
			}
			{
				char *temp_sample = allocateStringMemory(512);
				if (temp_sample)
				{
					snprintf(temp_sample, 512, "<td style=\"text-align: left;\">Sample: <input style=\"text-align:right;\" min=\"0\" max=\"9999\" step=\"1\" name=\"sample%d\" type=\"number\" value=\"%d\" />Sec.\n", ax, config.sensor[ax].samplerate);
					strcat(html, temp_sample);
					snprintf(temp_sample, 512, "Average: <input style=\"text-align:right;\" min=\"0\" max=\"999\" step=\"1\" name=\"avg%d\" type=\"number\" value=\"%d\" />Sec.</td></tr>\n", ax, config.sensor[ax].averagerate);
					strcat(html, temp_sample);
					free(temp_sample);
				}
			}
			{
				char *temp_eqns = allocateStringMemory(1024);
				if (temp_eqns)
				{
					snprintf(temp_eqns, 1024, "<tr><td style=\"text-align: right;\">EQNS:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-999\" max=\"999\" step=\"0.00001\" name=\"eqns%da\" type=\"number\" value=\"%.5f\" />  b:<input min=\"-999\" max=\"999\" step=\"0.00001\" name=\"eqns%db\" type=\"number\" value=\"%.5f\" /> c:<input min=\"-999\" max=\"999\" step=\"0.00001\" name=\"eqns%dc\" type=\"number\" value=\"%.5f\" /> (av<sup>2</sup>+bv+c)</td></tr>\n",
							 ax, config.sensor[ax].eqns[0], ax, config.sensor[ax].eqns[1], ax, config.sensor[ax].eqns[2]);
					strcat(html, temp_eqns);
					free(temp_eqns);
				}
			}
			strcat(html, "</table></td>");
			strcat(html, "</tr>\n");
		}

		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitSENSOR'  name=\"commitSENSOR\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitSENSOR\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br />");

		strcat(html, "<script type=\"text/javascript\">\n");
		strcat(html, "if (typeof typeArry === 'undefined'){let typeArry = [];};\n");
		strcat(html, "typeArry = new Array(");
		for (uint8_t idx = 0; idx < SENSOR_NAME_NUM; idx++)
		{
			{
				char *temp_array_item = allocateStringMemory(256);
				if (temp_array_item)
				{
					snprintf(temp_array_item, 256, "'%s'", SENSOR_NAME[idx]);
					strcat(html, temp_array_item);
					if (idx < SENSOR_NAME_NUM - 1)
						strcat(html, ",");
					free(temp_array_item);
				}
			}
		}
		strcat(html, ");\n");
		strcat(html, "if (typeof portArry === 'undefined'){let portArry = [];};\n");
		strcat(html, "portArry = new Array(");
		for (uint8_t idx = 0; idx < SENSOR_PORT_NUM; idx++)
		{
			{
				char *temp_port_item = allocateStringMemory(256);
				if (temp_port_item)
				{
					snprintf(temp_port_item, 256, "'%s'", SENSOR_PORT[idx]);
					strcat(html, temp_port_item);
					if (idx < SENSOR_PORT_NUM - 1)
						strcat(html, ",");
					free(temp_port_item);
				}
			}
		}
		strcat(html, ");\n");
		// strcat(html, "delete typeSel;delete listType;delete portSel;delete listPort;\n";
		strcat(html, "if (typeof typeSel === 'undefined'){var typeSel = [];};\n");
		strcat(html, "if (typeof listType === 'undefined'){var listType = [];};\n");
		strcat(html, "if (typeof portSel === 'undefined'){var portSel = [];};\n");
		strcat(html, "if (typeof listPort === 'undefined'){var listPort = [];};\n");
		for (int i = 0; i < 10; i++)
		{
			{
				char *temp_list = allocateStringMemory(512);
				if (temp_list)
				{
					snprintf(temp_list, 512, "listType[%d] = document.querySelector('#sensorCH%d');typeSel[%d]=%d;\n", i, i, i, config.sensor[i].type);
					strcat(html, temp_list);
					snprintf(temp_list, 512, "listPort[%d] = document.querySelector('#sensorP%d');portSel[%d]=%d;\n", i, i, i, config.sensor[i].port);
					strcat(html, temp_list);
					free(temp_list);
				}
			}
		}

		strcat(html, "for (let n = 0; n < 10; n++){\n");
		strcat(html, "for (let i = 0; i < typeArry.length; i++) {\n");
		strcat(html, "const optionType = new Option(typeArry[i], i);\n");
		strcat(html, "listType[n].add(optionType, undefined);\n");
		strcat(html, "};\n");
		strcat(html, "listType[n].options[typeSel[n]].selected = true;\n");
		strcat(html, "for (let p = 0; p < portArry.length; p++) {\n");
		strcat(html, "const optionPort = new Option(portArry[p], p);\n");
		strcat(html, "listPort[n].add(optionPort, undefined);\n");
		strcat(html, "};\n");
		strcat(html, "listPort[n].options[portSel[n]].selected = true;\n");
		strcat(html, "};\n");

		strcat(html, "</script>\n");

		AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, html);
		response->addHeader("Sensor", "content");
		response->addHeader("Cache-Control", "no-cache");
		request->send(response);
	}
}

void handle_tracker(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	bool trakerEn = false;
	bool smartEn = false;
	bool compEn = false;

	bool posGPS = false;
	bool bcnEN = false;
	bool pos2RF = false;
	bool pos2INET = false;
	bool optCST = false;
	bool optAlt = false;
	bool optBat = false;
	bool optSat = false;
	bool timeStamp = false;

	if (request->hasArg("commitTRACKER"))
	{
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "trackerEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						trakerEn = true;
				}
			}
			if (request->argName(i) == "smartBcnEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						smartEn = true;
				}
			}
			if (request->argName(i) == "compressEnable")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						compEn = true;
				}
			}
			if (request->argName(i) == "trackerOptCST")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						optCST = true;
				}
			}
			if (request->argName(i) == "trackerOptAlt")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						optAlt = true;
				}
			}
			if (request->argName(i) == "trackerOptBat")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						optBat = true;
				}
			}
			if (request->argName(i) == "trackerOptSat")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						optSat = true;
				}
			}
			if (request->argName(i) == "myCall")
			{
				if (request->arg(i) != "")
				{
					String name = request->arg(i);
					name.trim();
					name.toUpperCase();
					strcpy(config.trk_mycall, name.c_str());
				}
			}
			if (request->argName(i) == "trackerObject")
			{
				if (request->arg(i) != "")
				{
					String name = request->arg(i);
					name.trim();
					strcpy(config.trk_item, name.c_str());
				}
				else
				{
					memset(config.trk_item, 0, sizeof(config.trk_item));
				}
			}
			if (request->argName(i) == "mySSID")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_ssid = request->arg(i).toInt();
					if (config.trk_ssid > 15)
						config.trk_ssid = 13;
				}
			}
			if (request->argName(i) == "trackerPosInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_interval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "trkSTSInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_sts_interval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "trackerPosLat")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_lat = request->arg(i).toFloat();
				}
			}

			if (request->argName(i) == "trackerPosLon")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_lon = request->arg(i).toFloat();
				}
			}
			if (request->argName(i) == "trackerPosAlt")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_alt = request->arg(i).toFloat();
				}
			}
			if (request->argName(i) == "trackerPosSel")
			{
				if (request->arg(i) != "")
				{
					if (request->arg(i).toInt() == 1)
						posGPS = true;
				}
			}
			if (request->argName(i) == "hspeed")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_hspeed = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "lspeed")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_lspeed = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "slowInterval")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_slowinterval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "maxInterval")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_maxinterval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "minInterval")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_mininterval = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "minAngle")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_minangle = request->arg(i).toInt();
				}
			}

			if (request->argName(i) == "trackerTable")
			{
				if (request->arg(i) != "")
				{
					config.trk_symbol[0] = request->arg(i).charAt(0);
				}
			}
			if (request->argName(i) == "trackerSymbol")
			{
				if (request->arg(i) != "")
				{
					config.trk_symbol[1] = request->arg(i).charAt(0);
				}
			}
			if (request->argName(i) == "moveTable")
			{
				if (request->arg(i) != "")
				{
					config.trk_symmove[0] = request->arg(i).charAt(0);
				}
			}
			if (request->argName(i) == "moveSymbol")
			{
				if (request->arg(i) != "")
				{
					config.trk_symmove[1] = request->arg(i).charAt(0);
				}
			}
			if (request->argName(i) == "stopTable")
			{
				if (request->arg(i) != "")
				{
					config.trk_symstop[0] = request->arg(i).charAt(0);
				}
			}
			if (request->argName(i) == "stopSymbol")
			{
				if (request->arg(i) != "")
				{
					config.trk_symstop[1] = request->arg(i).charAt(0);
				}
			}

			if (request->argName(i) == "trackerPath")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_path = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "trkMicEType")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_mice_type = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "trackerComment")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.trk_comment, request->arg(i).c_str());
				}
				else
				{
					memset(config.trk_comment, 0, sizeof(config.trk_comment));
				}
			}
			if (request->argName(i) == "trkStatus")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.trk_status, request->arg(i).c_str());
				}
				else
				{
					memset(config.trk_status, 0, sizeof(config.trk_status));
				}
			}

			if (request->argName(i) == "trackerPos2RF")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						pos2RF = true;
				}
			}
			if (request->argName(i) == "trackerPos2INET")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						pos2INET = true;
				}
			}
			if (request->argName(i) == "trackerTimeStamp")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
						timeStamp = true;
				}
			}
			if (request->argName(i) == "trkTlmInv")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.trk_tlm_interval = request->arg(i).toInt();
				}
			}
			String arg;
			for (int x = 0; x < 5; x++)
			{
				arg = "sensorCH" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.trk_tlm_sensor[x] = request->arg(i).toInt();
				}
				arg = "param" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.trk_tlm_PARM[x], request->arg(i).c_str());
					}
				}
				arg = "unit" + String(x);
				if (request->argName(i) == arg)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.trk_tlm_UNIT[x], request->arg(i).c_str());
					}
				}
				arg = "precision" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.trk_tlm_precision[x] = request->arg(i).toInt();
				}
				arg = "offset" + String(x);
				if (request->argName(i) == arg)
				{
					if (isValidNumber(request->arg(i)))
						config.trk_tlm_offset[x] = request->arg(i).toFloat();
				}
				for (int y = 0; y < 3; y++)
				{
					arg = "eqns" + String(x) + String((char)(y + 'a'));
					if (request->argName(i) == arg)
					{
						if (isValidNumber(request->arg(i)))
							config.trk_tlm_EQNS[x][y] = request->arg(i).toFloat();
					}
				}
			}
		}
		config.trk_en = trakerEn;
		config.trk_smartbeacon = smartEn;
		config.trk_compress = compEn;

		config.trk_gps = posGPS;
		config.trk_loc2rf = pos2RF;
		config.trk_loc2inet = pos2INET;

		config.trk_log = optCST;
		config.trk_altitude = optAlt;
		config.trk_rssi = optBat;
		config.trk_sat = optSat;
		config.trk_timestamp = timeStamp;

		initInterval = true;
		saveConfig(request);
	}

	// Allocate initial memory for HTML content
	char tempHtml[500];
	char *html = allocateStringMemory(22000); // Start with 8KB buffer
	if (!html)
	{
		request->send(500, "text/html", "Memory allocation failed");
		return;
	}
	strcpy(html, "<script type=\"text/javascript\">\n");
	strcat(html, "$('form').submit(function (e) {\n");
	strcat(html, "e.preventDefault();\n");
	strcat(html, "var data = new FormData(e.currentTarget);\n");
	strcat(html, "document.getElementById(\"submitTRACKER\").disabled=true;\n");
	strcat(html, "$.ajax({\n");
	strcat(html, "url: '/tracker',\n");
	strcat(html, "type: 'POST',\n");
	strcat(html, "data: data,\n");
	strcat(html, "contentType: false,\n");
	strcat(html, "processData: false,\n");
	strcat(html, "success: function (data) {\n");
	strcat(html, "alert(\"Submited Successfully\");\n");
	strcat(html, "},\n");
	strcat(html, "error: function (data) {\n");
	strcat(html, "alert(\"An error occurred.\");\n");
	strcat(html, "}\n");
	strcat(html, "});\n");
	strcat(html, "});\n");
	strcat(html, "</script>\n<script type=\"text/javascript\">\n");
	strcat(html, "function openWindowSymbol(sel) {\n");
	strcat(html, "var i, l, options = [{\n");
	strcat(html, "value: 'first',\n");
	strcat(html, "text: 'First'\n");
	strcat(html, "}, {\n");
	strcat(html, "value: 'second',\n");
	strcat(html, "text: 'Second'\n");
	strcat(html, "}],\n");
	strcat(html, "newWindow = window.open(\"/symbol?sel=\"+sel.toString(), null, \"height=400,width=400,status=no,toolbar=no,menubar=no,location=no\");\n");
	strcat(html, "}\n");

	strcat(html, "function setValue(sel,symbol,table) {\n");
	strcat(html, "var txtsymbol=document.getElementById('trackerSymbol');\n");
	strcat(html, "var txttable=document.getElementById('trackerTable');\n");
	strcat(html, "var imgicon=document.getElementById('trackerImgSymbol');\n");
	strcat(html, "if(sel==1){\n");
	strcat(html, "txtsymbol=document.getElementById('moveSymbol');\n");
	strcat(html, "txttable=document.getElementById('moveTable');\n");
	strcat(html, "imgicon= document.getElementById('moveImgSymbol');\n");
	strcat(html, "}else if(sel==2){\n");
	strcat(html, "txtsymbol=document.getElementById('stopSymbol');\n");
	strcat(html, "txttable=document.getElementById('stopTable');\n");
	strcat(html, "imgicon= document.getElementById('stopImgSymbol');\n");
	strcat(html, "}\n");
	strcat(html, "txtsymbol.value = String.fromCharCode(symbol);\n");
	strcat(html, "if(table==1){\n txttable.value='/';\n");
	strcat(html, "}else if(table==2){\n txttable.value='\\\\';\n}\n");
	strcat(html, "imgicon.src = \"http://aprs.dprns.com/symbols/icons/\"+symbol.toString()+'-'+table.toString()+'.png';\n");
	strcat(html, "\n}\n");
	strcat(html, "function onSmartCheck() {\n");
	strcat(html, "if (document.querySelector('#smartBcnEnable').checked) {\n");
	// Checkbox has been checked
	strcat(html, "document.getElementById(\"smartbcnGrp\").disabled=false;\n");
	strcat(html, "} else {\n");
	// Checkbox has been unchecked
	strcat(html, "document.getElementById(\"smartbcnGrp\").disabled=true;\n");
	strcat(html, "}\n}\n");

	strcat(html, "function selPrecision(idx) {\n");
	strcat(html, "var x=0;\n");
	strcat(html, "x = document.getElementsByName(\"precision\"+idx)[0].value;\n");
	strcat(html, "document.getElementsByName(\"eqns\"+idx+\"b\")[0].value=1/Math.pow(10,x);\n");
	strcat(html, "}\n");
	strcat(html, "function selOffset(idx) {\n");
	strcat(html, "var x=0;\n");
	strcat(html, "x = document.getElementsByName(\"offset\"+idx)[0].value;\n");
	strcat(html, "document.getElementsByName(\"eqns\"+idx+\"c\")[0].value=x*(-1);\n");
	strcat(html, "}\n");
	strcat(html, "</script>\n");

	delay(1);
	/************************ tracker Mode **************************/
	strcat(html, "<form id='formtracker' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
	// strcat(html, "<h2>[TRACKER] Tracker Position Mode</h2>\n");
	strcat(html, "<table>\n");
	// strcat(html, "<tr>\n");
	// strcat(html, "<th width=\"200\"><span><b>Setting</b></span></th>\n");
	// strcat(html, "<th><span><b>Value</b></span></th>\n");
	// strcat(html, "</tr>\n");
	strcat(html, "<th colspan=\"2\"><span><b>[TRACKER] Tracker Position Mode</b></span></th>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Enable:</b></td>\n");
	char trackerEnFlag[10] = "";
	if (config.trk_en)
		strcpy(trackerEnFlag, "checked");
	else
		strcpy(trackerEnFlag, "");

	snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"trackerEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", trackerEnFlag);
	strcat(html, tempHtml);

	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Station Callsign:</b></td>\n");
	snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"7\" size=\"6\" id=\"myCall\" name=\"myCall\" type=\"text\" value=\"%s\" /></td>\n", config.trk_mycall);
	strcat(html, tempHtml);

	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Station SSID:</b></td>\n");
	strcat(html, "<td style=\"text-align: left;\">\n");
	strcat(html, "<select name=\"mySSID\" id=\"mySSID\">\n");
	for (uint8_t ssid = 0; ssid <= 15; ssid++)
	{
		if (config.trk_ssid == ssid)
		{
			snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>%d</option>\n", ssid, ssid);
		}
		else
		{
			snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\">%d</option>\n", ssid, ssid);
		}
		strcat(html, tempHtml);
	}
	strcat(html, "</select></td>\n");
	strcat(html, "</tr>\n");

	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Item/Obj Name:</b></td>\n");
	snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"9\" size=\"9\" id=\"trackerObject\" name=\"trackerObject\" type=\"text\" value=\"%s\" /><i> *If not used, leave it blank.In use 3-9 charactor</i></td>\n", config.trk_item);
	strcat(html, tempHtml);
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>PATH:</b></td>\n");
	strcat(html, "<td style=\"text-align: left;\">\n");
	strcat(html, "<select name=\"trackerPath\" id=\"trackerPath\">\n");
	for (uint8_t pthIdx = 0; pthIdx < PATH_LEN; pthIdx++)
	{
		if (config.trk_path == pthIdx)
		{
			snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
		}
		else
		{
			snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\">%s</option>\n", pthIdx, PATH_NAME[pthIdx]);
		}
		strcat(html, tempHtml);
	}
	strcat(html, "</select></td>\n");
	// strcat(html, "<td style=\"text-align: left;\"><input maxlength=\"72\" size=\"72\" id=\"trackerPath\" name=\"trackerPath\" type=\"text\" value=\"" + String(config.trk_path) + "\" /></td>\n";
	strcat(html, "</tr>\n");

	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Text Comment:</b></td>\n");
	snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"25\" size=\"30\" id=\"trackerComment\" name=\"trackerComment\" type=\"text\" value=\"%s\" /></td>\n", config.trk_comment);
	strcat(html, tempHtml);
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Text Status:</b></td>\n");
	snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"50\" size=\"60\" id=\"trkStatus\" name=\"trkStatus\" type=\"text\" value=\"%s\" />  Interval:<input min=\"0\" max=\"3600\" step=\"1\" name=\"trkSTSInv\" type=\"number\" value=\"%d\" />Sec.</td>\n", config.trk_status, config.trk_sts_interval);
	strcat(html, tempHtml);
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Smart Beacon:</b></td>\n");
	char smartBcnEnFlag[10] = "";
	if (config.trk_smartbeacon)
		strcpy(smartBcnEnFlag, "checked");

	snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" id=\"smartBcnEnable\" name=\"smartBcnEnable\" onclick=\"onSmartCheck()\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Switch use to smart beacon mode</i></label></td>\n", smartBcnEnFlag);
	strcat(html, tempHtml);
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Compress:</b></td>\n");
	char compressEnFlag[10] = "";
	if (config.trk_compress)
		strcpy(compressEnFlag, "checked");

	snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"compressEnable\" value=\"OK\" %s><span class=\"slider round\"></span></label><label style=\"vertical-align: bottom;font-size: 8pt;\"><i> *Switch compress packet</i></label></td>\n", compressEnFlag);
	strcat(html, tempHtml);
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Mic-E Type:</b></td>\n");
	strcat(html, "<td style=\"text-align: left;\">\n");
	strcat(html, "<select name=\"trkMicEType\" id=\"trkMicEType\">\n");
	for (uint8_t micEIdx = 0; micEIdx < 8; micEIdx++)
	{
		if (config.trk_mice_type == micEIdx)
		{
			snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>%s</option>\n", micEIdx, MIC_E_MSG[micEIdx]);
		}
		else
		{
			snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\">%s</option>\n", micEIdx, MIC_E_MSG[micEIdx]);
		}
		strcat(html, tempHtml);
	}
	strcat(html, "</select><label style=\"vertical-align: bottom;font-size: 8pt;\"><i>*Support if Compress is enabled and not use Item/Obj,Time Stamp</i></label></td>\n");
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Time Stamp:</b></td>\n");

	char timeStampFlag[10] = "";
	if (config.trk_timestamp)
		strcpy(timeStampFlag, "checked");
	{
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"trackerTimeStamp\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", timeStampFlag);
		strcat(html, tempHtml);
	}
	strcat(html, "</tr>\n");
	char trackerPos2RFFlag[10] = "";
	char trackerPos2INETFlag[10] = "";
	if (config.trk_loc2rf)
		strcpy(trackerPos2RFFlag, "checked");
	if (config.trk_loc2inet)
		strcpy(trackerPos2INETFlag, "checked");
	snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\"><b>TX Channel:</b></td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"trackerPos2RF\" value=\"OK\" %s/>RF <input type=\"checkbox\" name=\"trackerPos2INET\" value=\"OK\" %s/>Internet </td></tr>\n", trackerPos2RFFlag, trackerPos2INETFlag);
	strcat(html, tempHtml);
	char trackerOptBatFlag[10] = "";
	char trackerOptSatFlag[10] = "";
	char trackerOptAltFlag[10] = "";
	char trackerOptCSTFlag[10] = "";
	if (config.trk_rssi)
		strcpy(trackerOptBatFlag, "checked");
	if (config.trk_sat)
		strcpy(trackerOptSatFlag, "checked");
	if (config.trk_altitude)
		strcpy(trackerOptAltFlag, "checked");
	if (config.trk_log)
		strcpy(trackerOptCSTFlag, "checked");

	snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\"><b>Option:</b></td><td style=\"text-align: left;\"><input type=\"checkbox\" name=\"trackerOptCST\" value=\"OK\" %s/>Telemetry <input type=\"checkbox\" name=\"trackerOptAlt\" value=\"OK\" %s/>Altutude <input type=\"checkbox\" name=\"trackerOptBat\" value=\"OK\" %s/>RSSI Request </td></tr>\n",
			 trackerOptCSTFlag, trackerOptAltFlag, trackerOptBatFlag);
	strcat(html, tempHtml);

	strcat(html, "<tr>");
	strcat(html, "<td align=\"right\"><b>POSITION:</b></td>\n");
	strcat(html, "<td align=\"center\">\n");
	strcat(html, "<table>");
	snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"3600\" step=\"1\" id=\"trackerPosInv\" name=\"trackerPosInv\" type=\"number\" value=\"%d\" />Sec.</label></td></tr>", config.trk_interval);
	strcat(html, tempHtml);
	char trackerPosFixFlag[20] = "";
	char trackerPosGPSFlag[20] = "";

	if (config.trk_gps)
		strcpy(trackerPosGPSFlag, "checked=\"checked\"");
	else
		strcpy(trackerPosFixFlag, "checked=\"checked\"");

	snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Location:</td><td style=\"text-align: left;\"><input type=\"radio\" name=\"trackerPosSel\" value=\"0\" %s/>Fix <input type=\"radio\" name=\"trackerPosSel\" value=\"1\" %s/>GPS </td></tr>\n",
			 trackerPosFixFlag, trackerPosGPSFlag);
	strcat(html, tempHtml);
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\">Symbol Icon:</td>\n");
	char table[5] = "1";
	if (config.trk_symbol[0] == 47)
		strcpy(table, "1");
	if (config.trk_symbol[0] == 92)
		strcpy(table, "2");
	snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\">Table:<input maxlength=\"1\" size=\"1\" id=\"trackerTable\" name=\"trackerTable\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> Symbol:<input maxlength=\"1\" size=\"1\" id=\"trackerSymbol\" name=\"trackerSymbol\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> <img border=\"1\" style=\"vertical-align: middle;\" id=\"trackerImgSymbol\" onclick=\"openWindowSymbol(0);\" src=\"http://aprs.dprns.com/symbols/icons/%d-%s.png\"> <i>*Click icon for select symbol</i></td>\n",
			 config.trk_symbol[0], config.trk_symbol[1], (int)config.trk_symbol[1], table);
	strcat(html, tempHtml);
	strcat(html, "</tr>\n");
	snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Latitude:</td><td style=\"text-align: left;\"><input min=\"-90\" max=\"90\" step=\"0.00001\" id=\"trackerPosLat\" name=\"trackerPosLat\" type=\"number\" value=\"%.5f\" />degrees (positive for North, negative for South)</td></tr>\n", config.trk_lat);
	strcat(html, tempHtml);
	snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Longitude:</td><td style=\"text-align: left;\"><input min=\"-180\" max=\"180\" step=\"0.00001\" id=\"trackerPosLon\" name=\"trackerPosLon\" type=\"number\" value=\"%.5f\" />degrees (positive for East, negative for West)</td></tr>\n", config.trk_lon);
	strcat(html, tempHtml);
	snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Altitude:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"10000\" step=\"0.1\" id=\"trackerPosAlt\" name=\"trackerPosAlt\" type=\"number\" value=\"%.2f\" /> meter. *Value 0 is not send height</td></tr>\n", config.trk_alt);
	strcat(html, tempHtml);
	strcat(html, "</table></td>");
	strcat(html, "</tr>\n");

	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Smart Beacon:</b></td>\n");
	strcat(html, "<td align=\"center\">\n");
	if (config.trk_smartbeacon)
		strcat(html, "<fieldset id=\"smartbcnGrp\">\n");
	else
		strcat(html, "<fieldset id=\"smartbcnGrp\" disabled>\n");
	strcat(html, "<legend>Smart beacon configuration</legend>\n<table>");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\">Move Symbol:</td>\n");
	strcpy(table, "1");
	if (config.trk_symmove[0] == 47)
		strcpy(table, "1");
	if (config.trk_symmove[0] == 92)
		strcpy(table, "2");
	snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\">Table:<input maxlength=\"1\" size=\"1\" id=\"moveTable\" name=\"moveTable\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> Symbol:<input maxlength=\"1\" size=\"1\" id=\"moveSymbol\" name=\"moveSymbol\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> <img border=\"1\" style=\"vertical-align: middle;\" id=\"moveImgSymbol\" onclick=\"openWindowSymbol(1);\" src=\"http://aprs.dprns.com/symbols/icons/%d-%s.png\"> <i>*Click icon for select MOVE symbol</i></td>\n",
			 config.trk_symmove[0], config.trk_symmove[1], (int)config.trk_symmove[1], table);
	strcat(html, tempHtml);
	strcat(html, "</tr>\n");
	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\">Stop Symbol:</td>\n");
	strcpy(table, "1");
	if (config.trk_symstop[0] == 47)
		strcpy(table, "1");
	if (config.trk_symstop[0] == 92)
		strcpy(table, "2");
	snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\">Table:<input maxlength=\"1\" size=\"1\" id=\"stopTable\" name=\"stopTable\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> Symbol:<input maxlength=\"1\" size=\"1\" id=\"stopSymbol\" name=\"stopSymbol\" type=\"text\" value=\"%c\" style=\"background-color: rgb(97, 239, 170);\" /> <img border=\"1\" style=\"vertical-align: middle;\" id=\"stopImgSymbol\" onclick=\"openWindowSymbol(2);\" src=\"http://aprs.dprns.com/symbols/icons/%d-%s.png\"> <i>*Click icon for select STOP symbol</i></td>\n",
			 config.trk_symstop[0], config.trk_symstop[1], (int)config.trk_symstop[1], table);
	strcat(html, tempHtml);
	strcat(html, "</tr>\n");
	snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">High Speed:</td><td style=\"text-align: left;\"><input size=\"3\" min=\"10\" max=\"1000\" step=\"1\" id=\"hspeed\" name=\"hspeed\" type=\"number\" value=\"%d\" /> km/h</td></tr>\n", config.trk_hspeed);
	strcat(html, tempHtml);
	snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Low Speed:</td><td style=\"text-align: left;\"><input size=\"3\" min=\"1\" max=\"250\" step=\"1\" id=\"lspeed\" name=\"lspeed\" type=\"number\" value=\"%d\" /> km/h</td></tr>\n", config.trk_lspeed);
	strcat(html, tempHtml);
	snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Slow Interval:</td><td style=\"text-align: left;\"><input size=\"3\" min=\"60\" max=\"3600\" step=\"1\" id=\"slowInterval\" name=\"slowInterval\" type=\"number\" value=\"%d\" /> Sec.</td></tr>\n", config.trk_slowinterval);
	strcat(html, tempHtml);
	snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Max Interval:</td><td style=\"text-align: left;\"><input size=\"3\" min=\"10\" max=\"255\" step=\"1\" id=\"maxInterval\" name=\"maxInterval\" type=\"number\" value=\"%d\" /> Sec.</td></tr>\n", config.trk_maxinterval);
	strcat(html, tempHtml);
	snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Min Interval:</td><td style=\"text-align: left;\"><input size=\"3\" min=\"1\" max=\"100\" step=\"1\" id=\"minInterval\" name=\"minInterval\" type=\"number\" value=\"%d\" /> Sec.</td></tr>\n", config.trk_mininterval);
	strcat(html, tempHtml);
	snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Min Angle:</td><td style=\"text-align: left;\"><input size=\"3\" min=\"1\" max=\"359\" step=\"1\" id=\"minAngle\" name=\"minAngle\" type=\"number\" value=\"%d\" /> Degree.</td></tr>\n", config.trk_minangle);
	strcat(html, tempHtml);

	strcat(html, "</table></fieldset></tr>");

	strcat(html, "<tr>\n");
	strcat(html, "<td align=\"right\"><b>Telemetry:</b><br />(v=0->8280)</td>\n");
	strcat(html, "<td align=\"center\"><table>\n");
	snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">Interval:</td><td style=\"text-align: left;\"><input min=\"0\" max=\"1000\" step=\"1\" id=\"trkTlmInv\" name=\"trkTlmInv\" type=\"number\" value=\"%d\" /> *Number of packets interval,<i>Example: 0 not send,1 send every packet</i></label></td></tr>", config.trk_tlm_interval);
	strcat(html, tempHtml);
	for (int ax = 0; ax < 5; ax++)
	{
		snprintf(tempHtml, sizeof(tempHtml), "<tr><td align=\"right\"><b>CH A%d:</b></td>\n", ax + 1);
		strcat(html, tempHtml);
		strcat(html, "<td align=\"center\">\n");
		strcat(html, "<table>");

		strcat(html, "<tr><td style=\"text-align: right;\">Sensor:</td>\n");
		strcat(html, "<td style=\"text-align: left;\">CH: ");
		snprintf(tempHtml, sizeof(tempHtml), "<select name=\"sensorCH%d\" id=\"sensorCH%d\">\n", ax, ax);
		strcat(html, tempHtml);
		for (uint8_t idx = 0; idx < 11; idx++)
		{
			if (idx == 0)
			{
				if (config.trk_tlm_sensor[ax] == idx)
				{
					snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>NONE</option>\n", idx);
				}
				else
				{
					snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\">NONE</option>\n", idx);
				}
			}
			else
			{
				if (config.trk_tlm_sensor[ax] == idx)
				{
					snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>SENSOR#%d</option>\n", idx, idx);
				}
				else
				{
					snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\">SENSOR#%d</option>\n", idx, idx);
				}
			}
			strcat(html, tempHtml);
		}
		strcat(html, "</select></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\">Name: <input maxlength=\"10\" size=\"8\" name=\"param%d\" type=\"text\" value=\"%s\" /></td>\n", ax, config.trk_tlm_PARM[ax]);
		strcat(html, tempHtml);
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\">Unit: <input maxlength=\"8\" size=\"5\" name=\"unit%d\" type=\"text\" value=\"%s\" /></td>\n", ax, config.trk_tlm_UNIT[ax]);
		strcat(html, tempHtml);
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\">Precision: <input min=\"0\" max=\"5\" step=\"1\" type=\"number\" style=\"width: 2em\" name=\"precision%d\" type=\"text\" value=\"%d\" onchange=\"selPrecision(%d)\" /></td></tr>\n", ax, config.trk_tlm_precision[ax], ax);
		strcat(html, tempHtml);
		snprintf(tempHtml, sizeof(tempHtml), "<tr><td style=\"text-align: right;\">EQNS:</td><td colspan=\"3\" style=\"text-align: left;\">a:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns%da\" type=\"number\" value=\"%.5f\" />  b:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns%db\" type=\"number\" value=\"%.5f\" /> c:<input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" name=\"eqns%dc\" type=\"number\" value=\"%.5f\" /> (av<sup>2</sup>+bv+c) </td>\n",
				 ax, config.trk_tlm_EQNS[ax][0], ax, config.trk_tlm_EQNS[ax][1], ax, config.trk_tlm_EQNS[ax][2]);
		strcat(html, tempHtml);
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\">Offset: <input min=\"-9999\" max=\"9999\" step=\"0.00001\" style=\"width: 5em\" type=\"number\" name=\"offset%d\" type=\"text\" value=\"%.5f\"  onchange=\"selOffset(%d)\" /></td></tr>\n", ax, config.trk_tlm_offset[ax], ax);
		strcat(html, tempHtml);
		strcat(html, "</table></td>");
		strcat(html, "</tr>\n");
	}
	strcat(html, "</table></td></tr>\n");
	strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
	strcat(html, "<div><button class=\"button\" type='submit' id='submitTRACKER'  name=\"commitTRACKER\"> Apply Change </button></div>\n");
	strcat(html, "<input type=\"hidden\" name=\"commitTRACKER\"/>\n");
	strcat(html, "</td></tr></table><br />\n");
	strcat(html, "</form><br />");

	AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, html);
	response->addHeader("Tracker", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
}

void handle_wireless(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	StandByTick = millis() + (config.pwr_stanby_delay * 1000);

	if (request->hasArg("commitWiFiAP"))
	{
		bool wifiAP = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "wifiAP")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						wifiAP = true;
					}
				}
			}

			if (request->argName(i) == "wifi_ssidAP")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.wifi_ap_ssid, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "wifi_passAP")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.wifi_ap_pass, request->arg(i).c_str());
				}
			}
		}
		if (wifiAP)
		{
			config.wifi_mode |= WIFI_AP_FIX;
		}
		else
		{
			config.wifi_mode &= ~WIFI_AP_FIX;
		}
		saveConfig(request);
	}
	else if (request->hasArg("commitWiFiClient"))
	{
		bool wifiSTA = false;
		for (int n = 0; n < 5; n++)
			config.wifi_sta[n].enable = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "wificlient")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						wifiSTA = true;
					}
				}
			}

			for (int n = 0; n < 5; n++)
			{
				String nameSSID = "wifiStation" + String(n);
				if (request->argName(i) == nameSSID)
				{
					if (request->arg(i) != "")
					{
						if (String(request->arg(i)) == "OK")
						{
							config.wifi_sta[n].enable = true;
						}
					}
				}
				nameSSID = "wifi_ssid" + String(n);
				if (request->argName(i) == nameSSID)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.wifi_sta[n].wifi_ssid, request->arg(i).c_str());
					}
				}
				String namePASS = "wifi_pass" + String(n);
				if (request->argName(i) == namePASS)
				{
					if (request->arg(i) != "")
					{
						strcpy(config.wifi_sta[n].wifi_pass, request->arg(i).c_str());
					}
				}
			}

			if (request->argName(i) == "wifi_pwr")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
					{
						config.wifi_power = (int8_t)request->arg(i).toInt();
						WiFi.setTxPower((wifi_power_t)config.wifi_power);
					}
				}
			}
		}
		if (wifiSTA)
		{
			config.wifi_mode |= WIFI_STA_FIX;
		}
		else
		{
			config.wifi_mode &= ~WIFI_STA_FIX;
		}
		saveConfig(request);
	}
	#ifdef BLUETOOTH
	else if (request->hasArg("commitBluetooth"))
	{
		bool btMaster = false;
		for (uint8_t i = 0; i < request->args(); i++)
		{
			if (request->argName(i) == "btMaster")
			{
				if (request->arg(i) != "")
				{
					if (String(request->arg(i)) == "OK")
					{
						btMaster = true;
					}
				}
			}

			if (request->argName(i) == "bt_name")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.bt_name, request->arg(i).c_str());
				}
			}
			#if !defined(CONFIG_IDF_TARGET_ESP32)
			if (request->argName(i) == "bt_uuid")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.bt_uuid, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "bt_uuid_rx")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.bt_uuid_rx, request->arg(i).c_str());
				}
			}
			if (request->argName(i) == "bt_uuid_tx")
			{
				if (request->arg(i) != "")
				{
					strcpy(config.bt_uuid_tx, request->arg(i).c_str());
				}
			}
			#endif
			if (request->argName(i) == "bt_mode")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.bt_mode = request->arg(i).toInt();
				}
			}
			if (request->argName(i) == "bt_pin")
			{
				if (request->arg(i) != "")
				{
					if (isValidNumber(request->arg(i)))
						config.bt_pin = request->arg(i).toInt();
				}
			}
		}
		config.bt_master = btMaster;
		saveConfig(request);
	}
	#endif
	else
	{
		// Allocate initial memory for HTML content
		char tempHtml[256] = "";
		char *html = allocateStringMemory(12000); // Start with 12KB buffer
		if (!html)
		{
			request->send(500, "text/html", "Memory allocation failed");
			return;
		}
		strcpy(html, "<script type=\"text/javascript\">\n");
		strcat(html, "$('form').submit(function (e) {\n");
		strcat(html, "e.preventDefault();\n");
		strcat(html, "var data = new FormData(e.currentTarget);\n");
		strcat(html, "if(e.currentTarget.id===\"formBluetooth\") document.getElementById(\"submitBluetooth\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formWiFiAP\") document.getElementById(\"submitWiFiAP\").disabled=true;\n");
		strcat(html, "if(e.currentTarget.id===\"formWiFiClient\") document.getElementById(\"submitWiFiClient\").disabled=true;\n");
		strcat(html, "$.ajax({\n");
		strcat(html, "url: '/wireless',\n");
		strcat(html, "type: 'POST',\n");
		strcat(html, "data: data,\n");
		strcat(html, "contentType: false,\n");
		strcat(html, "processData: false,\n");
		strcat(html, "success: function (data) {\n");
		strcat(html, "alert(\"Submited Successfully\");\n");
		strcat(html, "},\n");
		strcat(html, "error: function (data) {\n");
		strcat(html, "alert(\"An error occurred.\");\n");
		strcat(html, "}\n");
		strcat(html, "});\n");
		strcat(html, "});\n");
		strcat(html, "</script>\n");
		
		/************************ WiFi AP **************************/
		strcat(html, "<form id='formWiFiAP' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>WiFi Access Point</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\" width=\"120\"><b>Enable:</b></td>\n");
		const char *wifiAPEnFlagMode = (config.wifi_mode & WIFI_AP_FIX) ? "checked" : "";
		{
			char *temp_flag = allocateStringMemory(512);
			if (temp_flag)
			{
				snprintf(temp_flag, 512, "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"wifiAP\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", wifiAPEnFlagMode);
				strcat(html, temp_flag);
				free(temp_flag);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>WiFi AP SSID:</b></td>\n");
		{
			char *temp_ssid = allocateStringMemory(512);
			if (temp_ssid)
			{
				snprintf(temp_ssid, 512, "<td style=\"text-align: left;\"><input size=\"32\" maxlength=\"32\" class=\"form-control\" id=\"wifi_ssidAP\" name=\"wifi_ssidAP\" type=\"text\" value=\"%s\" /></td>\n", config.wifi_ap_ssid);
				strcat(html, temp_ssid);
				free(temp_ssid);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>WiFi AP PASSWORD:</b></td>\n");
		{
			char *temp_pass = allocateStringMemory(512);
			if (temp_pass)
			{
				snprintf(temp_pass, 512, "<td style=\"text-align: left;\"><input size=\"63\" maxlength=\"63\" class=\"form-control\" id=\"wifi_passAP\" name=\"wifi_passAP\" type=\"password\" value=\"%s\" /></td>\n", config.wifi_ap_pass);
				strcat(html, temp_pass);
				free(temp_pass);
			}
		}
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitWiFiAP'  name=\"commitWiFiAP\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitWiFiAP\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br />");
		/************************ WiFi Client **************************/
		strcat(html, "<br />\n");
		strcat(html, "<form id='formWiFiClient' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		strcat(html, "<table>\n");
		strcat(html, "<th colspan=\"2\"><span><b>WiFi Multi Station</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>WiFi STA Enable:</b></td>\n");
		String wifiClientEnFlag = "";
		if (config.wifi_mode & WIFI_STA_FIX)
			wifiClientEnFlag = "checked";
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"wificlient\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", wifiClientEnFlag);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>WiFi RF Power:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"wifi_pwr\" id=\"wifi_pwr\">\n");
		for (int i = 0; i < 12; i++)
		{
			if (config.wifi_power == (int8_t)wifiPwr[i][0])
				snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" selected>%.1f dBm</option>\n", (int8_t)wifiPwr[i][0], wifiPwr[i][1]);
			else
				snprintf(tempHtml, sizeof(tempHtml), "<option value=\"%d\" >%.1f dBm</option>\n", (int8_t)wifiPwr[i][0], wifiPwr[i][1]);
			strcat(html, tempHtml);
		}
		strcat(html, "</select>\n");
		strcat(html, "</td>\n");
		strcat(html, "</tr>\n");
		for (int n = 0; n < 5; n++)
		{
			strcat(html, "<tr>\n");
			snprintf(tempHtml, sizeof(tempHtml), "<td align=\"right\"><b>Station #%d:</b></td>\n", n + 1);
			strcat(html, tempHtml);
			strcat(html, "<td align=\"center\">\n");
			snprintf(tempHtml, sizeof(tempHtml), "<fieldset id=\"filterDispGrp%d\">\n", n + 1);
			strcat(html, tempHtml);
			snprintf(tempHtml, sizeof(tempHtml), "<legend>WiFi Station #%d</legend>\n<table style=\"text-align:unset;border-width:0px;background:unset\">", n + 1);
			strcat(html, tempHtml);
			strcat(html, "<tr style=\"background:unset;\">");
			// strcat(html, "<tr>\n";
			strcat(html, "<td align=\"right\" width=\"120\"><b>Enable:</b></td>\n");
			char wifiClientEnFlag[10];
			if (config.wifi_sta[n].enable)
				strcpy(wifiClientEnFlag, "checked");
			else
				strcpy(wifiClientEnFlag, "");

			snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"wifiStation%d\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", n, wifiClientEnFlag);
			strcat(html, tempHtml);
			strcat(html, "</tr>\n");
			strcat(html, "<tr>\n");
			strcat(html, "<td align=\"right\"><b>WiFi SSID:</b></td>\n");
			snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input size=\"32\" maxlength=\"32\" name=\"wifi_ssid%d\" type=\"text\" value=\"%s\" /></td>\n", n, config.wifi_sta[n].wifi_ssid);
			strcat(html, tempHtml);
			strcat(html, "</tr>\n");
			strcat(html, "<tr>\n");
			strcat(html, "<td align=\"right\"><b>WiFi PASSWORD:</b></td>\n");
			snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input size=\"63\" maxlength=\"63\" name=\"wifi_pass%d\" type=\"password\" value=\"%s\" /></td>\n", n, config.wifi_sta[n].wifi_pass);
			strcat(html, tempHtml);
			strcat(html, "</tr>\n");
			strcat(html, "</tr></table></fieldset>\n");
			strcat(html, "</td></tr>\n");
		}
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitWiFiClient'  name=\"commitWiFiClient\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitWiFiClient\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form><br />");
		/************************ Bluetooth **************************/
#ifdef BLUETOOTH
		strcat(html, "<br />\n");
		strcat(html, "<form id='formBluetooth' method=\"POST\" action='#' enctype='multipart/form-data'>\n");
		// strcat(html, "<h2>Bluetooth Master (BLE)</h2>\n";
		strcat(html, "<table>\n");
		// strcat(html, "<tr>\n";
		// strcat(html, "<th width=\"200\"><span><b>Setting</b></span></th>\n";
		// strcat(html, "<th><span><b>Value</b></span></th>\n";
		// strcat(html, "</tr>\n";
		strcat(html, "<th colspan=\"2\"><span><b>Bluetooth Master (BLE)</b></span></th>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>Enable:</b></td>\n");

		char btFlag[10];
		if (config.bt_master)
			strcpy(btFlag, "checked");
		else
			strcpy(btFlag, "");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><label class=\"switch\"><input type=\"checkbox\" name=\"btMaster\" value=\"OK\" %s><span class=\"slider round\"></span></label></td>\n", btFlag);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>NAME:</b></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"20\" id=\"bt_name\" name=\"bt_name\" type=\"text\" value=\"%s\" /></td>\n", config.bt_name);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>PIN:</b></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input min=\"0\" max=\"999999\" id=\"bt_pin\" name=\"bt_pin\" type=\"number\" value=\"%d\" /></td> <i>*Value 0 is no auth.</i>\n", config.bt_pin);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		#if !defined(CONFIG_IDF_TARGET_ESP32)
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>UUID:</b></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"37\" size=\"38\" id=\"bt_uuid\" name=\"bt_uuid\" type=\"text\" value=\"%s\" /></td>\n", config.bt_uuid);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>UUID RX:</b></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"37\" size=\"38\" id=\"bt_uuid_rx\" name=\"bt_uuid_rx\" type=\"text\" value=\"%s\" /></td>\n", config.bt_uuid_rx);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		strcat(html, "<tr>\n");
		strcat(html, "<td align=\"right\"><b>UUID TX:</b></td>\n");
		snprintf(tempHtml, sizeof(tempHtml), "<td style=\"text-align: left;\"><input maxlength=\"37\" size=\"38\" id=\"bt_uuid_tx\" name=\"bt_uuid_tx\" type=\"text\" value=\"%s\" /></td>\n", config.bt_uuid_tx);
		strcat(html, tempHtml);
		strcat(html, "</tr>\n");
		#endif
		strcat(html, "<td align=\"right\"><b>MODE:</b></td>\n");
		strcat(html, "<td style=\"text-align: left;\">\n");
		strcat(html, "<select name=\"bt_mode\" id=\"bt_mode\">\n");
		const char *btModeOff = (config.bt_mode == 0) ? "selected" : "";
		const char *btModeTNC2 = (config.bt_mode == 1) ? "selected" : "";
		const char *btModeKISS = (config.bt_mode == 2) ? "selected" : "";
		snprintf(tempHtml, sizeof(tempHtml), "<option value=\"0\" %s>NONE</option>\n<option value=\"1\" %s>TNC2</option>\n<option value=\"2\" %s>KISS</option>\n", btModeOff, btModeTNC2, btModeKISS);
		strcat(html, tempHtml);
		strcat(html, "</select>\n");

		strcat(html, "<label style=\"font-size: 8pt;text-align: right;\">*See the following for generating UUIDs: <a href=\"https://www.uuidgenerator.net\" target=\"_blank\">https://www.uuidgenerator.net</a></label></td>\n");
		strcat(html, "</tr>\n");
		strcat(html, "<tr><td colspan=\"2\" align=\"right\">\n");
		strcat(html, "<div><button class=\"button\" type='submit' id='submitBluetooth'  name=\"commitBluetooth\"> Apply Change </button></div>\n");
		strcat(html, "<input type=\"hidden\" name=\"commitBluetooth\"/>\n");
		strcat(html, "</td></tr></table><br />\n");
		strcat(html, "</form>");
#endif

	AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, html);
	response->addHeader("wifi", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);

	}
}

// extern String lastPkgRaw;
// extern float dBV;
// extern int mVrms;
//  void handle_realtime(AsyncWebServerRequest *request)
//  {
//  	// char jsonMsg[1000];
//  	char *jsonMsg;
//  	time_t timeStamp;
//  	time(&timeStamp);

// 	if (afskSync && (lastPkgRaw.length() > 5))
// 	{
// 		int input_length = lastPkgRaw.length();
// 		jsonMsg = (char *)malloc((input_length * 2) + 200);
// 		char *input_buffer = (char *)malloc(input_length + 2);
// 		char *output_buffer = (char *)malloc(input_length * 2);
// 		if (output_buffer)
// 		{
// 			// lastPkgRaw.toCharArray(input_buffer, lastPkgRaw.length(), 0);
// 			memcpy(input_buffer, lastPkgRaw.c_str(), lastPkgRaw.length());
// 			lastPkgRaw.clear();
// 			encode_base64((unsigned char *)input_buffer, input_length, (unsigned char *)output_buffer);
// 			// Serial.println(output_buffer);
// 			sprintf(jsonMsg, "{\"Active\":\"1\",\"mVrms\":\"%d\",\"RAW\":\"%s\",\"timeStamp\":\"%li\"}", mVrms, output_buffer, timeStamp);
// 			// Serial.println(jsonMsg);
// 			free(input_buffer);
// 			free(output_buffer);
// 		}
// 	}
// 	else
// 	{
// 		jsonMsg = (char *)malloc(100);
// 		if (afskSync)
// 			sprintf(jsonMsg, "{\"Active\":\"1\",\"mVrms\":\"%d\",\"RAW\":\"REVDT0RFIEZBSUwh\",\"timeStamp\":\"%li\"}", mVrms, timeStamp);
// 		else
// 			sprintf(jsonMsg, "{\"Active\":\"0\",\"mVrms\":\"0\",\"RAW\":\"\",\"timeStamp\":\"%li\"}", timeStamp);
// 	}
// 	afskSync = false;
// 	request->send(200, "text/html", String(jsonMsg));

// 	delay(100);
// 	free(jsonMsg);
// }

// void handle_ws(String Raw,uint16_t mVrms)
void handle_ws(char *Raw, size_t len, uint16_t mVrms)
{
	if (ws.count() < 1)
		return;

	char *jsonMsg;
	time_t timeStamp;
	time(&timeStamp);

	if (len > 5)
	{
		int input_length = len;
		jsonMsg = (char *)calloc((input_length * 2) + 200, sizeof(char));
		if (jsonMsg)
		{
			char *input_buffer = (char *)calloc(input_length + 2, sizeof(char));
			char *output_buffer = (char *)calloc(input_length * 2, sizeof(char));
			if (output_buffer)
			{
				memset(input_buffer, 0, (input_length + 2));
				memset(output_buffer, 0, (input_length * 2));
				// lastPkgRaw.toCharArray(input_buffer, input_length, 0);
				memcpy(input_buffer, Raw, len);
				encode_base64((unsigned char *)input_buffer, input_length, (unsigned char *)output_buffer);
				// Serial.println(output_buffer);
				sprintf(jsonMsg, "{\"Active\":\"1\",\"mVrms\":\"%d\",\"RAW\":\"%s\",\"timeStamp\":\"%li\"}", mVrms, output_buffer, timeStamp);
				// Serial.println(jsonMsg);
				free(input_buffer);
				free(output_buffer);
			}
			ws.textAll(jsonMsg);
			free(jsonMsg);
		}
	}
	else
	{
		jsonMsg = (char *)calloc(300, sizeof(char));
		if (jsonMsg)
		{
			if (mVrms > 0)
				sprintf(jsonMsg, "{\"Active\":\"1\",\"mVrms\":\"%d\",\"RAW\":\"REVDT0RFIEZBSUwh\",\"timeStamp\":\"%li\"}", mVrms, timeStamp);
			else
				sprintf(jsonMsg, "{\"Active\":\"0\",\"mVrms\":\"0\",\"RAW\":\"\",\"timeStamp\":\"%li\"}", timeStamp);
			ws.textAll(jsonMsg);
			free(jsonMsg);
		}
	}
}

void handle_ws_audio_samples(const float *samples, size_t len, uint16_t sampleRate)
{
	serviceWebPttTimeout();

	if (samples == nullptr || len == 0 || sampleRate == 0)
	{
		return;
	}

	if (audioTuneActive)
	{
		const uint32_t now = millis();
		if ((uint32_t)(now - audioTuneLastUserMs) > AUDIO_TUNE_IDLE_TIMEOUT_MS)
		{
			audioTuneResetToAprs("idle timeout");
		}
	}

	if (ws_audio.count() < 1)
	{
		audioMonitorBufferLen = 0;
		audioMonitorAccumulator = 0;
		return;
	}

	for (size_t i = 0; i < len; i++)
	{
		audioMonitorAccumulator += AUDIO_MONITOR_RATE;
		if (audioMonitorAccumulator < sampleRate)
		{
			continue;
		}
		audioMonitorAccumulator -= sampleRate;

		float s = samples[i];
		if (s > 1.0f)
		{
			s = 1.0f;
		}
		else if (s < -1.0f)
		{
			s = -1.0f;
		}

		const int16_t pcm = clampToInt16((int32_t)(s * 32767.0f));
		audioMonitorBuffer[audioMonitorBufferLen++] = linearToMuLaw(pcm);
		if (audioMonitorBufferLen >= AUDIO_MONITOR_CHUNK)
		{
			ws_audio.binaryAll(audioMonitorBuffer, audioMonitorBufferLen);
			audioMonitorBufferLen = 0;
		}
	}
}

void handle_ws_gnss(char *nmea, size_t size)
{
	if (ws_gnss.count() < 1)
		return;

	time_t timeStamp;
	time(&timeStamp);
	// unsigned int output_length = encode_base64_length(size);
	// unsigned char nmea_enc[output_length];
	// char jsonMsg[output_length + 100];
	// encode_base64((unsigned char *)nmea, size, (unsigned char *)nmea_enc);
	// sprintf(jsonMsg, "{\"en\":\"%d\",\"lat\":\"%.5f\",\"lng\":\"%.5f\",\"alt\":\"%.2f\",\"spd\":\"%.2f\",\"csd\":\"%.1f\",\"hdop\":\"%.2f\",\"sat\":\"%d\",\"time\":\"%d\",\"timeStamp\":\"%li\",\"RAW\":\"", (int)config.gnss_enable, gps.location.lat(), gps.location.lng(), gps.altitude.meters(), gps.speed.kmph(), gps.course.deg(), gps.hdop.hdop(), gps.satellites.value(), gps.time.value(), timeStamp);
	// strncat(jsonMsg, (const char *)nmea_enc, output_length);
	// strcat(jsonMsg, "\"}");
	unsigned int output_length = encode_base64_length(size);
	unsigned char *nmea_enc = (unsigned char *)calloc(output_length + 2, sizeof(unsigned char));
	char *jsonMsg = (char *)calloc(output_length + 200, sizeof(char));
	if (nmea_enc && jsonMsg)
	{
		encode_base64((unsigned char *)nmea, size, (unsigned char *)nmea_enc);
		sprintf(jsonMsg, "{\"en\":\"%d\",\"lat\":\"%.5f\",\"lng\":\"%.5f\",\"alt\":\"%.2f\",\"spd\":\"%.2f\",\"csd\":\"%.1f\",\"hdop\":\"%.2f\",\"sat\":\"%d\",\"time\":\"%d\",\"timeStamp\":\"%li\",\"RAW\":\"", (int)config.gnss_enable, gps.location.lat(), gps.location.lng(), gps.altitude.meters(), gps.speed.kmph(), gps.course.deg(), gps.hdop.hdop(), gps.satellites.value(), gps.time.value(), timeStamp);
		strncat(jsonMsg, (const char *)nmea_enc, output_length);
		strcat(jsonMsg, "\"}");
		ws_gnss.textAll(jsonMsg);
		free(nmea_enc);
		free(jsonMsg);
	}
	
}

void handle_test(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	request->redirect("/app/");
}

void handle_audio(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	request->redirect("/app/");
}

void handle_audio_tune(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}

	float freqMin = 0.0F;
	float freqMax = 0.0F;
	getRfRangeForType(config.rf_type, freqMin, freqMax);

	auto sendStatus = [&](int statusCode, bool ok, const char *message)
	{
		char json[320];
		const uint32_t timeoutSec = AUDIO_TUNE_IDLE_TIMEOUT_MS / 1000UL;
		const uint32_t idleSec = (audioTuneLastUserMs == 0) ? timeoutSec : ((uint32_t)(millis() - audioTuneLastUserMs) / 1000UL);
		const float aprsFreq = audioTuneActive ? audioTuneAprsFreqRx : config.freq_rx;
		snprintf(json, sizeof(json),
				 "{\"ok\":%s,\"active\":%s,\"rx\":%.4f,\"aprs\":%.4f,\"min\":%.4f,\"max\":%.4f,\"idleSec\":%lu,\"timeoutSec\":%lu,\"message\":\"%s\"}",
				 ok ? "true" : "false",
				 audioTuneActive ? "true" : "false",
				 config.freq_rx,
				 aprsFreq,
				 freqMin,
				 freqMax,
				 (unsigned long)idleSec,
				 (unsigned long)timeoutSec,
				 message ? message : "");
		request->send(statusCode, "application/json", json);
	};

	String action = request->arg("action");
	action.toLowerCase();

	if (action.length() == 0 || action == "status")
	{
		sendStatus(200, true, "status");
		return;
	}

	if (action == "touch")
	{
		audioTuneTouch();
		sendStatus(200, true, "touch");
		return;
	}

	if (action == "reset")
	{
		audioTuneResetToAprs("manual reset");
		sendStatus(200, true, "reset");
		return;
	}

	if (action == "set")
	{
		if (!request->hasArg("freq"))
		{
			sendStatus(400, false, "missing freq");
			return;
		}
		if (!config.rf_en)
		{
			sendStatus(409, false, "radio disabled");
			return;
		}

		const float newFreq = request->arg("freq").toFloat();
		if (newFreq < freqMin || newFreq > freqMax)
		{
			sendStatus(422, false, "freq out of range");
			return;
		}

		if (!audioTuneActive)
		{
			audioTuneAprsFreqRx = config.freq_rx;
		}
		audioTuneActive = true;
		audioTuneTouch();

		if (absFloat(config.freq_rx - newFreq) > 0.00005F)
		{
			config.freq_rx = newFreq;
			RF_MODULE(false);
		}
		sendStatus(200, true, "tuned");
		return;
	}

	sendStatus(400, false, "invalid action");
}

void handle_about(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		return request->requestAuthentication();
	}
	char strCID[50];
	uint64_t chipid = ESP.getEfuseMac();
	sprintf(strCID, "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);

	// Allocate memory for HTML content
	char *webString = allocateStringMemory(8192); // Start with 8KB buffer
	if (!webString)
	{
		request->send(500, "text/html", "Memory allocation failed");
		return;
	}
	strcpy(webString, "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;\"><td width=\"49%\" style=\"border:unset;\">");

	strcat(webString, "<table>");
	strcat(webString, "<th colspan=\"2\"><span><b>System Information</b></span></th>\n");
	// strcat(webString, "<tr><th width=\"200\"><span><b>Name</b></span></th><th><span><b>Information</b></span></th></tr>";
	strcat(webString, "<tr><td align=\"right\"><b>Hardware Version: </b></td><td align=\"left\">");
#if defined(CONFIG_IDF_TARGET_ESP32)
	strcat(webString, "ESP32-WROOM,ESP32 DoIt DevKit");
#elif defined(ESP32C3_MINI)
	strcat(webString, "ESP32C3-Mini,ESP32-C3 DIY");
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
	strcat(webString, "ESP32C3,ESP32-C3 DIY");
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
	strcat(webString, "ESP32C6,ESP32-C6 DIY");
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
	strcat(webString, "ESP32-S3-DevKit,ESP32-S3-WROOM");
#else
	strcat(webString, "UNKNOWN,ESP32 DIY");
#endif
	strcat(webString, "</td></tr>");

	char *temp_str = allocateStringMemory(512);
	if (temp_str)
	{
		snprintf(temp_str, 512, "<tr><td align=\"right\"><b>Firmware Version: </b></td><td align=\"left\"> V%s%c</td></tr>\n", VERSION, VERSION_BUILD);
		strcat(webString, temp_str);

		snprintf(temp_str, 512, "<tr><td align=\"right\"><b>RF Module: </b></td><td align=\"left\"> %s</td></tr>\n", RF_TYPE[config.rf_type]);
		strcat(webString, temp_str);

		snprintf(temp_str, 512, "<tr><td align=\"right\"><b>ESP32 Model: </b></td><td align=\"left\"> %s</td></tr>", ESP.getChipModel());
		strcat(webString, temp_str);

		snprintf(temp_str, 512, "<tr><td align=\"right\"><b>Revision: </b></td><td align=\"left\"> %d</td></tr>", ESP.getChipRevision());
		strcat(webString, temp_str);

		snprintf(temp_str, 512, "<tr><td align=\"right\"><b>Chip ID: </b></td><td align=\"left\"> %s</td></tr>", strCID);
		strcat(webString, temp_str);

		snprintf(temp_str, 512, "<tr><td align=\"right\"><b>Flash: </b></td><td align=\"left\">%d KByte</td></tr>", ESP.getFlashChipSize() / 1024);
		strcat(webString, temp_str);

		snprintf(temp_str, 512, "<tr><td align=\"right\"><b>PSRAM: </b></td><td align=\"left\">%.1f/%.1f KByte</td></tr>",
				 (float)ESP.getFreePsram() / 1024, (float)ESP.getPsramSize() / 1024);
		strcat(webString, temp_str);

		snprintf(temp_str, 512, "<tr><td align=\"right\"><b>FILE SYSTEM: </b></td><td align=\"left\">%.1f/%.1f KByte</td></tr>",
				 (float)LITTLEFS.usedBytes() / 1024, (float)LITTLEFS.totalBytes() / 1024);
		strcat(webString, temp_str);

		free(temp_str);
	}

	strcat(webString, "</table>");
	strcat(webString, "</td><td width=\"2%\" style=\"border:unset;\"></td>");
	strcat(webString, "<td width=\"49%\" style=\"border:unset;\">");

	strcat(webString, "<table>");
	strcat(webString, "<th colspan=\"2\"><span><b>Developer/Support Information</b></span></th>\n");
	strcat(webString, "<tr><td align=\"right\"><b>Author: </b></td><td align=\"left\">Mr.Somkiat Nakhonthai </td></tr>");
	strcat(webString, "<tr><td align=\"right\"><b>Callsign: </b></td><td align=\"left\">HS5TQA,Atten,Nakhonthai</td></tr>\n");
	strcat(webString, "<tr><td align=\"right\"><b>Country: </b></td><td align=\"left\">Bangkok,Thailand</td></tr>\n");
	strcat(webString, "<tr><td align=\"right\"><b>Github: </b></td><td align=\"left\"><a href=\"https://github.com/nakhonthai\" target=\"_github\">https://github.com/nakhonthai</a></td></tr>");
	strcat(webString, "<tr><td align=\"right\"><b>Youtube: </b></td><td align=\"left\"><a href=\"https://www.youtube.com/@HS5TQA\" target=\"_youtube\">https://www.youtube.com/@HS5TQA</a></td></tr>");
	strcat(webString, "<tr><td align=\"right\"><b>Facebook: </b></td><td align=\"left\"><a href=\"https://www.facebook.com/atten\" target=\"_facebook\">https://www.facebook.com/atten</a></td></tr>");
	strcat(webString, "<tr><td align=\"right\"><b>Chat: </b></td><td align=\"left\">Telegram:<a href=\"https://t.me/HS5TQA\" target=\"_line\">@HS5TQA</a> , WeChat:HS5TQA</td></tr>");
	strcat(webString, "<tr><td align=\"right\"><b>Sponsors: </b></td><td align=\"left\"><a href=\"https://github.com/sponsors/nakhonthai\" target=\"_sponsor\">https://github.com/sponsors/nakhonthai</a></td></tr>");
	strcat(webString, "<tr><td align=\"right\"><b>Donate: </b></td><td align=\"left\"><a href=\"https://www.paypal.me/0hs5tqa0\" target=\"_sponsor\">https://www.paypal.me/0hs5tqa0</a></td></tr>");

	strcat(webString, "</table>");
	strcat(webString, "</td></tr></table><br />");

	strcat(webString, "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;\"><td width=\"49%\" style=\"border:unset;\">");

	strcat(webString, "<table>\n");
	strcat(webString, "<th colspan=\"2\"><span><b>WiFi Status</b></span></th>\n");
	strcat(webString, "<tr><td align=\"right\"><b>Mode:</b></td>\n");
	strcat(webString, "<td align=\"left\">");
	if (config.wifi_mode == WIFI_AP_FIX)
	{
		strcat(webString, "AP");
	}
	else if (config.wifi_mode == WIFI_STA_FIX)
	{
		strcat(webString, "STA");
	}
	else if (config.wifi_mode == WIFI_AP_STA_FIX)
	{
		strcat(webString, "AP+STA");
	}
	else
	{
		strcat(webString, "OFF");
	}
	uint8_t proto = 0;
	esp_wifi_get_protocol(WIFI_IF_STA, &proto);
	strcat(webString, " (802.11");
	if (proto & WIFI_PROTOCOL_11B)
		strcat(webString, "b");
	if (proto & WIFI_PROTOCOL_11G)
		strcat(webString, "g");
	if (proto & WIFI_PROTOCOL_11N)
		strcat(webString, "n");
	if (proto & WIFI_PROTOCOL_LR)
		strcat(webString, "lr");
	strcat(webString, ")");

	wifi_power_t wpr = WiFi.getTxPower();
	char wifipower[20] = "";
	if (wpr < 8)
	{
		strcpy(wifipower, "-1 dBm");
	}
	else if (wpr < 21)
	{
		strcpy(wifipower, "2 dBm");
	}
	else if (wpr < 29)
	{
		strcpy(wifipower, "5 dBm");
	}
	else if (wpr < 35)
	{
		strcpy(wifipower, "8.5 dBm");
	}
	else if (wpr < 45)
	{
		strcpy(wifipower, "11 dBm");
	}
	else if (wpr < 53)
	{
		strcpy(wifipower, "13 dBm");
	}
	else if (wpr < 61)
	{
		strcpy(wifipower, "15 dBm");
	}
	else if (wpr < 69)
	{
		strcpy(wifipower, "17 dBm");
	}
	else if (wpr < 75)
	{
		strcpy(wifipower, "18.5 dBm");
	}
	else if (wpr < 77)
	{
		strcpy(wifipower, "19 dBm");
	}
	else if (wpr < 80)
	{
		strcpy(wifipower, "19.5 dBm");
	}
	else
	{
		strcpy(wifipower, "20 dBm");
	}

	strcat(webString, "</td></tr>\n");

	char *temp_str1 = allocateStringMemory(512);
	if (temp_str1)
	{
		snprintf(temp_str1, 512, "<tr><td align=\"right\" width=\"30%%\"><b>MAC:</b></td>\n<td align=\"left\">%s</td></tr>\n", WiFi.macAddress().c_str());
		strcat(webString, temp_str1);
		snprintf(temp_str1, 512, "<tr><td align=\"right\"><b>Channel:</b></td>\n<td align=\"left\">%d</td></tr>\n", WiFi.channel());
		strcat(webString, temp_str1);

		snprintf(temp_str1, 512, "<tr><td align=\"right\"><b>TX Power:</b></td>\n<td align=\"left\">%s</td></tr>\n", wifipower);
		strcat(webString, temp_str1);

		snprintf(temp_str1, 512, "<tr><td align=\"right\"><b>SSID:</b></td>\n<td align=\"left\">%s</td></tr>\n", WiFi.SSID().c_str());
		strcat(webString, temp_str1);

		snprintf(temp_str1, 512, "<tr><td align=\"right\"><b>Local IP:</b></td>\n<td align=\"left\">%s</td></tr>\n", WiFi.localIP().toString().c_str());
		strcat(webString, temp_str1);

		snprintf(temp_str1, 512, "<tr><td align=\"right\"><b>Gateway IP:</b></td>\n<td align=\"left\">%s</td></tr>\n", WiFi.gatewayIP().toString().c_str());
		strcat(webString, temp_str1);
		snprintf(temp_str1, 512, "<tr><td align=\"right\"><b>DNS:</b></td>\n<td align=\"left\">%s</td></tr>\n", WiFi.dnsIP().toString().c_str());
		strcat(webString, temp_str1);

		free(temp_str1);
	}

	strcat(webString, "</table>\n");

	strcat(webString, "</td><td width=\"2%\" style=\"border:unset;\"></td>");
	strcat(webString, "<td width=\"49%\" style=\"border:unset;\">");
	strcat(webString, "<table>\n");
#ifdef PPPOS
	strcat(webString, "<th colspan=\"2\"><span><b>PPPoS Status</b></span></th>\n");

	char *temp_str2 = allocateStringMemory(512);
	if (temp_str2)
	{
		snprintf(temp_str2, 512, "<tr><td align=\"right\" width=\"30%%\"><b>Manufacturer:</b></td>\n<td align=\"left\">%s</td></tr>\n", pppStatus.manufacturer);
		strcat(webString, temp_str2);

		snprintf(temp_str2, 512, "<tr><td align=\"right\"><b>Model:</b></td>\n<td align=\"left\">%s</td></tr>\n", pppStatus.model);
		strcat(webString, temp_str2);

		snprintf(temp_str2, 512, "<tr><td align=\"right\"><b>IMEI:</b></td>\n<td align=\"left\">%s</td></tr>\n", pppStatus.imei);
		strcat(webString, temp_str2);
		snprintf(temp_str2, 512, "<tr><td align=\"right\"><b>IMSI:</b></td>\n<td align=\"left\">%s</td></tr>\n", pppStatus.imsi);
		strcat(webString, temp_str2);

		snprintf(temp_str2, 512, "<tr><td align=\"right\"><b>Operator:</b></td>\n<td align=\"left\">%s</td></tr>\n", pppStatus.oper);
		strcat(webString, temp_str2);

		snprintf(temp_str2, 512, "<tr><td align=\"right\"><b>RSSI:</b></td>\n<td align=\"left\">%d dBm</td></tr>\n", pppStatus.rssi);
		strcat(webString, temp_str2);

		snprintf(temp_str2, 512, "<tr><td align=\"right\"><b>IP:</b></td>\n<td align=\"left\">%s</td></tr>\n", IPAddress(pppStatus.ip).toString().c_str());
		strcat(webString, temp_str2);

		snprintf(temp_str2, 512, "<tr><td align=\"right\"><b>Gateway:</b></td>\n<td align=\"left\">%s</td></tr>\n", IPAddress(pppStatus.gateway).toString().c_str());
		strcat(webString, temp_str2);
		free(temp_str2);
	}

// strcat(webString, "<tr><td align=\"right\"><b>DNS:</b></td>\n");
// strcat(webString, "<td align=\"left\">%s</td></tr>\n", IPAddress(pppStatus.dns).toString().c_str());
#endif
	strcat(webString, "</table>\n");
	strcat(webString, "</td></tr></table><br />");

	// strcat(webString, "<table style=\"text-align:unset;border-width:0px;background:unset\"><tr style=\"background:unset;\"><td width=\"96%\" style=\"border:unset;\">");
	#ifndef NO_OTA
	strcat(webString, "<form method='POST' action='#' enctype='multipart/form-data' id='upload_form' class=\"form-horizontal\">\n");
	strcat(webString, "<table>");
	strcat(webString, "<th colspan=\"2\"><span><b>Firmware Update</b></span></th>\n");
	strcat(webString, "<tr><td align=\"right\"><b>File:</b></td><td align=\"left\"><input id=\"file\" name=\"update\" type=\"file\" onchange='sub(this)' /></td></tr>\n");
	strcat(webString, "<tr><td align=\"right\"><b>Progress:</b></td><td><div id='prgbar'><div id='bar' style=\"width: 0px;\"><label id='prg'></label></div></div></td></tr>\n");
	strcat(webString, "<tr><td align=\"right\"><b>Support Firmware:</b></td><td align=\"left\"><a target=\"_download\" href=\"https://github.com/nakhonthai/ESP32APRS_Audio/releases\">https://github.com/nakhonthai/ESP32APRS_Audio/releases</a></td></tr>\n");
	strcat(webString, "</table><br />\n");
	strcat(webString, "<div class=\"col-sm-3 col-xs-4\"><input type='submit' class=\"btn btn-danger\" id=\"update_sumbit\" value='Firmware Update'></div>\n");

	strcat(webString, "</form>\n");
	// strcat(webString, "</td></tr></table><br />");

	strcat(webString, "<script>"
					  "function sub(obj){"
					  "var fileName = obj.value.split('\\\\');"
					  "document.getElementById('file-input').innerHTML = '   '+ fileName[fileName.length-1];"
					  "};"
					  "$('form').submit(function(e){"
					  "e.preventDefault();"
					  "var form = $('#upload_form')[0];"
					  "var data = new FormData(form);"
					  "document.getElementById('update_sumbit').disabled = true;"
					  "$.ajax({"
					  "url: '/update',"
					  "type: 'POST',"
					  "data: data,"
					  "contentType: false,"
					  "processData:false,"
					  "xhr: function() {"
					  "var xhr = new window.XMLHttpRequest();"
					  "xhr.upload.addEventListener('progress', function(evt) {"
					  "if (evt.lengthComputable) {"
					  "var per = evt.loaded / evt.total;"
					  "$('#prg').html(Math.round(per*100) + '%');"
					  "$('#bar').css('width',Math.round(per*100) + '%');"
					  "}"
					  "}, false);"
					  "return xhr;"
					  "},"
					  "success:function(d, s) {"
					  "alert('Wait for system reboot 10sec');"
					  "},"
					  "error: function (a, b, c) {"
					  "}"
					  "});"
					  "});"
					  "</script>");
	#endif
	strcat(webString, "</body></html>\n");

	AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, webString);
	response->addHeader("About", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
}

void handle_gnss(AsyncWebServerRequest *request)
{
	// Allocate initial memory for HTML content
	char *webString = allocateStringMemory(8192); // Start with 8KB buffer
	if (!webString)
	{
		request->send(500, "text/html", "Memory allocation failed");
		return;
	}
	strcpy(webString, "<html>\n<head>\n");
	strcat(webString, "<script src=\"https://apps.bdimg.com/libs/jquery/2.1.4/jquery.min.js\"></script>\n");
	strcat(webString, "<script src=\"https://code.highcharts.com/highcharts.js\"></script>\n");
	strcat(webString, "<script src=\"https://code.highcharts.com/highcharts-more.js\"></script>\n");
	strcat(webString, "<script language=\"JavaScript\">");

	// Add some life
	strcat(webString, "function gnss() { \n"); // the chart may be destroyed
	strcat(webString, "var raw=\"\";var timeStamp;\n");
	strcat(webString, "var host='ws://'+location.hostname+':81/ws_gnss'\n");
	strcat(webString, "const ws = new WebSocket(host);\n");
	strcat(webString, "ws.onopen = function() { console.log('Connection opened');};\n ws.onclose = function() { console.log('Connection closed');};\n");
	strcat(webString, "ws.onmessage = function(event) {\n  console.log(event.data);\n");
	strcat(webString, "const jsonR=JSON.parse(event.data);\n");
	strcat(webString, "document.getElementById(\"en\").innerHTML=parseInt(jsonR.en);\n");
	strcat(webString, "document.getElementById(\"lat\").innerHTML=parseFloat(jsonR.lat);\n");
	strcat(webString, "document.getElementById(\"lng\").innerHTML=parseFloat(jsonR.lng);\n");
	strcat(webString, "document.getElementById(\"alt\").innerHTML=parseFloat(jsonR.alt);\n");
	strcat(webString, "document.getElementById(\"spd\").innerHTML=parseFloat(jsonR.spd);\n");
	strcat(webString, "document.getElementById(\"csd\").innerHTML=parseFloat(jsonR.csd);\n");
	strcat(webString, "document.getElementById(\"hdop\").innerHTML=parseFloat(jsonR.hdop);\n");
	strcat(webString, "document.getElementById(\"sat\").innerHTML=parseInt(jsonR.sat);\n");
	strcat(webString, "document.getElementById(\"time\").innerHTML=parseInt(jsonR.time);\n");
	strcat(webString, "raw=jsonR.RAW;\n");
	strcat(webString, "timeStamp=Number(jsonR.timeStamp);\n");
	strcat(webString, "var textArea=document.getElementById(\"raw_txt\");\n");
	strcat(webString, "textArea.value+=atob(raw)+\"\\n\";\n");
	strcat(webString, "textArea.scrollTop = textArea.scrollHeight;\n");
	strcat(webString, "}\n");
	strcat(webString, "};\n</script>\n");
	strcat(webString, "</head><body onload=\"gnss()\">\n");

	strcat(webString, "<table width=\"200\" border=\"1\">");
	strcat(webString, "<th colspan=\"2\" style=\"background-color: #00BCD4;\"><span><b>GNSS Information</b></span></th>\n");
	// strcat(webString, "<tr><th width=\"200\"><span><b>Name</b></span></th><th><span><b>Information</b></span></th></tr>");

	{
		char *temp_en = allocateStringMemory(512);
		if (temp_en)
		{
			snprintf(temp_en, 512, "<tr><td align=\"right\"><b>Enable: </b></td><td align=\"left\"> <label id=\"en\">%d</label></td></tr>", config.gnss_enable);
			strcat(webString, temp_en);
			free(temp_en);
		}
	}

	{
		char *temp_lat = allocateStringMemory(512);
		if (temp_lat)
		{
			snprintf(temp_lat, 512, "<tr><td align=\"right\"><b>Latitude: </b></td><td align=\"left\"> <label id=\"lat\">%.5f</label></td></tr>", gps.location.lat());
			strcat(webString, temp_lat);
			free(temp_lat);
		}
	}

	{
		char *temp_lng = allocateStringMemory(512);
		if (temp_lng)
		{
			snprintf(temp_lng, 512, "<tr><td align=\"right\"><b>Longitude: </b></td><td align=\"left\"> <label id=\"lng\">%.5f</label></td></tr>", gps.location.lng());
			strcat(webString, temp_lng);
			free(temp_lng);
		}
	}

	{
		char *temp_alt = allocateStringMemory(512);
		if (temp_alt)
		{
			snprintf(temp_alt, 512, "<tr><td align=\"right\"><b>Altitude: </b></td><td align=\"left\"> <label id=\"alt\">%.2f</label> m.</td></tr>", gps.altitude.meters());
			strcat(webString, temp_alt);
			free(temp_alt);
		}
	}

	{
		char *temp_spd = allocateStringMemory(512);
		if (temp_spd)
		{
			snprintf(temp_spd, 512, "<tr><td align=\"right\"><b>Speed: </b></td><td align=\"left\"> <label id=\"spd\">%.2f</label> km/h</td></tr>", gps.speed.kmph());
			strcat(webString, temp_spd);
			free(temp_spd);
		}
	}

	{
		char *temp_csd = allocateStringMemory(512);
		if (temp_csd)
		{
			snprintf(temp_csd, 512, "<tr><td align=\"right\"><b>Course: </b></td><td align=\"left\"> <label id=\"csd\">%.1f</label></td></tr>", gps.course.deg());
			strcat(webString, temp_csd);
			free(temp_csd);
		}
	}

	{
		char *temp_hdop = allocateStringMemory(512);
		if (temp_hdop)
		{
			snprintf(temp_hdop, 512, "<tr><td align=\"right\"><b>HDOP: </b></td><td align=\"left\"> <label id=\"hdop\">%.2f</label> </td></tr>", gps.hdop.hdop());
			strcat(webString, temp_hdop);
			free(temp_hdop);
		}
	}

	{
		char *temp_sat = allocateStringMemory(512);
		if (temp_sat)
		{
			snprintf(temp_sat, 512, "<tr><td align=\"right\"><b>SAT: </b></td><td align=\"left\"> <label id=\"sat\">%d</label> </td></tr>", gps.satellites.value());
			strcat(webString, temp_sat);
			free(temp_sat);
		}
	}

	{
		char *temp_time = allocateStringMemory(512);
		if (temp_time)
		{
			snprintf(temp_time, 512, "<tr><td align=\"right\"><b>Time: </b></td><td align=\"left\"> <label id=\"time\">%d</label> </td></tr>", gps.time.value());
			strcat(webString, temp_time);
			free(temp_time);
		}
	}

	strcat(webString, "</table><table>");
	strcat(webString, "<tr><td><b>Terminal:</b><br /><textarea id=\"raw_txt\" name=\"raw_txt\" rows=\"30\" cols=\"80\" /></textarea></td></tr>\n");
	strcat(webString, "</table>\n");

	strcat(webString, "</body></html>\n");

	AsyncWebServerResponse *response = beginOwnedHtmlResponse(request, webString);
	response->addHeader("GNSS", "content");
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
}

void handle_default()
{
	defaultSetting = true;
	defaultConfig();
	defaultSetting = false;
}

static bool ensureWebAuth(AsyncWebServerRequest *request)
{
	if (!request->authenticate(config.http_username, config.http_password))
	{
		request->requestAuthentication();
		return false;
	}
	return true;
}

static bool isTruthyValue(const String &v)
{
	return (v == "1" || v.equalsIgnoreCase("true") || v.equalsIgnoreCase("on") || v.equalsIgnoreCase("ok"));
}

static void sendJsonDoc(AsyncWebServerRequest *request, int code, JsonDocument &doc)
{
	String body;
	serializeJson(doc, body);
	request->send(code, "application/json", body);
}

static int getSqlState()
{
	if (config.rf_sql_gpio < 0)
	{
		return -1;
	}
	const int raw = digitalRead(config.rf_sql_gpio);
	return ((raw ^ (config.rf_sql_active ? 1 : 0)) == 0) ? 1 : 0;
}

static void fillRadioStatus(JsonDocument &doc)
{
	doc["ok"] = true;
	doc["rf_en"] = config.rf_en;
	doc["rf_type"] = config.rf_type;
	doc["freq_rx"] = config.freq_rx;
	doc["freq_tx"] = config.freq_tx;
	doc["tone_rx"] = config.tone_rx;
	doc["tone_tx"] = config.tone_tx;
	doc["sql_level"] = config.sql_level;
	doc["volume"] = config.volume;
	doc["band"] = config.band;
	doc["rf_power"] = config.rf_power ? 1 : 0;
	doc["ptt"] = getTransmit() ? 1 : 0;
	doc["sq"] = getSqlState();
	doc["dcss_supported"] = false;
	doc["ctcss_count"] = (sizeof(ctcss) / sizeof(ctcss[0]));
}

void handle_api_status(AsyncWebServerRequest *request)
{
	if (!ensureWebAuth(request))
		return;
	serviceWebPttTimeout();

	JsonDocument doc;
	doc["ok"] = true;
	doc["host_name"] = config.host_name;
	doc["fw_version"] = VERSION;
	doc["schema_version"] = config.cfg_version;
	doc["wifi_mode"] = config.wifi_mode;
	doc["ap_ssid"] = config.wifi_ap_ssid;
	doc["ap_ip"] = WiFi.softAPIP().toString();
	doc["sta_connected"] = (WiFi.status() == WL_CONNECTED);
	doc["sta_ip"] = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "";
	doc["mac"] = WiFi.macAddress();
	doc["rf_en"] = config.rf_en;
	doc["audio_tune_active"] = audioTuneActive;
	doc["audio_rx_freq"] = config.freq_rx;
	doc["audio_aprs_freq"] = audioTuneActive ? audioTuneAprsFreqRx : config.freq_rx;
	doc["ws_audio_clients"] = ws_audio.count();
	sendJsonDoc(request, 200, doc);
}

typedef struct
{
	char call[11];
	time_t lastHeard;
	time_t lastMsg;
	time_t lastActive;
} ContactRecord;

static int findContact(ContactRecord *contacts, int count, const char *call)
{
	for (int i = 0; i < count; i++)
	{
		if (strcasecmp(contacts[i].call, call) == 0)
		{
			return i;
		}
	}
	return -1;
}

void handle_api_contacts(AsyncWebServerRequest *request)
{
	if (!ensureWebAuth(request))
		return;

	ContactRecord contacts[PKGLISTSIZE * 2];
	memset(contacts, 0, sizeof(contacts));
	int contactCount = 0;

	for (int i = 0; i < PKGLISTSIZE; i++)
	{
		pkgListType pkg = getPkgList(i);
		if (pkg.time <= 0 || pkg.calsign[0] == 0)
		{
			continue;
		}
		char call[11];
		memset(call, 0, sizeof(call));
		strlcpy(call, pkg.calsign, sizeof(call));
		int idx = findContact(contacts, contactCount, call);
		if (idx < 0)
		{
			if (contactCount < (PKGLISTSIZE * 2))
			{
				strlcpy(contacts[contactCount].call, call, sizeof(contacts[contactCount].call));
				contacts[contactCount].lastHeard = pkg.time;
				contacts[contactCount].lastMsg = 0;
				contacts[contactCount].lastActive = pkg.time;
				contactCount++;
			}
		}
		else
		{
			if (pkg.time > contacts[idx].lastHeard)
				contacts[idx].lastHeard = pkg.time;
			if (pkg.time > contacts[idx].lastActive)
				contacts[idx].lastActive = pkg.time;
		}
	}

	for (int i = 0; i < PKGLISTSIZE; i++)
	{
		msgType msg = getMsgList(i);
		if (msg.time <= 0 || msg.callsign[0] == 0)
		{
			continue;
		}
		char call[11];
		memset(call, 0, sizeof(call));
		strlcpy(call, msg.callsign, sizeof(call));
		int idx = findContact(contacts, contactCount, call);
		if (idx < 0)
		{
			if (contactCount < (PKGLISTSIZE * 2))
			{
				strlcpy(contacts[contactCount].call, call, sizeof(contacts[contactCount].call));
				contacts[contactCount].lastHeard = 0;
				contacts[contactCount].lastMsg = msg.time;
				contacts[contactCount].lastActive = msg.time;
				contactCount++;
			}
		}
		else
		{
			if (msg.time > contacts[idx].lastMsg)
				contacts[idx].lastMsg = msg.time;
			if (msg.time > contacts[idx].lastActive)
				contacts[idx].lastActive = msg.time;
		}
	}

	for (int i = 0; i < contactCount - 1; i++)
	{
		for (int j = i + 1; j < contactCount; j++)
		{
			if (contacts[j].lastActive > contacts[i].lastActive)
			{
				ContactRecord tmp = contacts[i];
				contacts[i] = contacts[j];
				contacts[j] = tmp;
			}
		}
	}

	JsonDocument doc;
	doc["ok"] = true;
	JsonArray arr = doc["contacts"].to<JsonArray>();
	for (int i = 0; i < contactCount; i++)
	{
		JsonObject row = arr.add<JsonObject>();
		row["call"] = contacts[i].call;
		row["last_active"] = (uint32_t)contacts[i].lastActive;
		row["last_heard"] = (uint32_t)contacts[i].lastHeard;
		row["last_msg"] = (uint32_t)contacts[i].lastMsg;
	}
	sendJsonDoc(request, 200, doc);
}

void handle_api_messages(AsyncWebServerRequest *request)
{
	if (!ensureWebAuth(request))
		return;

	String contact = request->arg("contact");
	contact.trim();
	contact.toUpperCase();

	typedef struct
	{
		time_t time;
		int8_t ack;
		bool rxtx;
		uint16_t msgID;
		char call[11];
		char text[241];
	} MsgRow;

	MsgRow rows[PKGLISTSIZE];
	memset(rows, 0, sizeof(rows));
	int rowCount = 0;

	for (int i = 0; i < PKGLISTSIZE; i++)
	{
		msgType m = getMsgList(i);
		if (m.time <= 0 || m.callsign[0] == 0 || m.text == nullptr)
		{
			continue;
		}
		char call[11];
		memset(call, 0, sizeof(call));
		strlcpy(call, m.callsign, sizeof(call));
		String callUpper = String(call);
		callUpper.toUpperCase();
		if (contact.length() > 0 && callUpper != contact)
		{
			continue;
		}
		if (rowCount >= PKGLISTSIZE)
		{
			continue;
		}
		rows[rowCount].time = m.time;
		rows[rowCount].ack = m.ack;
		rows[rowCount].rxtx = m.rxtx;
		rows[rowCount].msgID = m.msgID;
		strlcpy(rows[rowCount].call, call, sizeof(rows[rowCount].call));
		strlcpy(rows[rowCount].text, m.text, sizeof(rows[rowCount].text));
		rowCount++;
	}

	for (int i = 0; i < rowCount - 1; i++)
	{
		for (int j = i + 1; j < rowCount; j++)
		{
			if (rows[j].time < rows[i].time)
			{
				MsgRow tmp = rows[i];
				rows[i] = rows[j];
				rows[j] = tmp;
			}
		}
	}

	JsonDocument doc;
	doc["ok"] = true;
	JsonArray arr = doc["messages"].to<JsonArray>();
	for (int i = 0; i < rowCount; i++)
	{
		JsonObject row = arr.add<JsonObject>();
		row["ts"] = (uint32_t)rows[i].time;
		row["call"] = rows[i].call;
		row["dir"] = rows[i].rxtx ? "rx" : "tx";
		row["ack"] = rows[i].ack;
		row["msg_id"] = rows[i].msgID;
		row["text"] = rows[i].text;
	}
	sendJsonDoc(request, 200, doc);
}

void handle_api_send_message(AsyncWebServerRequest *request)
{
	if (!ensureWebAuth(request))
		return;

	String to = request->arg("to");
	String text = request->arg("text");
	to.trim();
	text.trim();
	to.toUpperCase();
	if (to.length() == 0 || text.length() == 0)
	{
		JsonDocument doc;
		doc["ok"] = false;
		doc["message"] = "to/text required";
		sendJsonDoc(request, 400, doc);
		return;
	}

	bool encrypt = config.msg_encrypt;
	if (request->hasArg("encrypt"))
	{
		encrypt = isTruthyValue(request->arg("encrypt"));
	}
	sendAPRSMessage(to, text, encrypt);

	JsonDocument doc;
	doc["ok"] = true;
	doc["message"] = "queued";
	sendJsonDoc(request, 200, doc);
}

void handle_api_radio_status(AsyncWebServerRequest *request)
{
	if (!ensureWebAuth(request))
		return;
	serviceWebPttTimeout();
	JsonDocument doc;
	fillRadioStatus(doc);
	sendJsonDoc(request, 200, doc);
}

void handle_api_radio_set(AsyncWebServerRequest *request)
{
	if (!ensureWebAuth(request))
		return;
	serviceWebPttTimeout();

	bool changed = false;
	float freqMin = 0.0F;
	float freqMax = 0.0F;
	getRfRangeForType(config.rf_type, freqMin, freqMax);

	auto clampInt = [](int v, int minV, int maxV)
	{
		if (v < minV)
			return minV;
		if (v > maxV)
			return maxV;
		return v;
	};

	if (request->hasArg("freq_rx"))
	{
		const float v = request->arg("freq_rx").toFloat();
		if (v >= freqMin && v <= freqMax && absFloat(config.freq_rx - v) > 0.00005F)
		{
			config.freq_rx = v;
			changed = true;
		}
	}
	if (request->hasArg("freq_tx"))
	{
		const float v = request->arg("freq_tx").toFloat();
		if (v >= freqMin && v <= freqMax && absFloat(config.freq_tx - v) > 0.00005F)
		{
			config.freq_tx = v;
			changed = true;
		}
	}
	if (request->hasArg("tone_rx"))
	{
		const int v = clampInt(request->arg("tone_rx").toInt(), 0, (int)(sizeof(ctcss) / sizeof(ctcss[0])) - 1);
		if (v != config.tone_rx)
		{
			config.tone_rx = v;
			changed = true;
		}
	}
	if (request->hasArg("tone_tx"))
	{
		const int v = clampInt(request->arg("tone_tx").toInt(), 0, (int)(sizeof(ctcss) / sizeof(ctcss[0])) - 1);
		if (v != config.tone_tx)
		{
			config.tone_tx = v;
			changed = true;
		}
	}
	if (request->hasArg("sql_level"))
	{
		const int v = clampInt(request->arg("sql_level").toInt(), 0, 8);
		if (v != config.sql_level)
		{
			config.sql_level = v;
			changed = true;
		}
	}
	if (request->hasArg("volume"))
	{
		const int v = clampInt(request->arg("volume").toInt(), 1, 8);
		if (v != config.volume)
		{
			config.volume = v;
			changed = true;
		}
	}
	if (request->hasArg("band"))
	{
		const int v = clampInt(request->arg("band").toInt(), 0, 1);
		if (v != config.band)
		{
			config.band = v;
			changed = true;
		}
	}
	if (request->hasArg("rf_power"))
	{
		const bool v = isTruthyValue(request->arg("rf_power"));
		if (v != config.rf_power)
		{
			config.rf_power = v;
			changed = true;
		}
	}
	if (request->hasArg("rf_en"))
	{
		const bool v = isTruthyValue(request->arg("rf_en"));
		if (v != config.rf_en)
		{
			config.rf_en = v;
			changed = true;
		}
	}

	if (changed)
	{
		RF_MODULE(false);
	}
	if (request->hasArg("save") && isTruthyValue(request->arg("save")))
	{
		saveConfiguration("/default.cfg", config);
	}

	JsonDocument doc;
	fillRadioStatus(doc);
	doc["changed"] = changed;
	sendJsonDoc(request, 200, doc);
}

void handle_api_radio_ptt(AsyncWebServerRequest *request)
{
	if (!ensureWebAuth(request))
		return;
	serviceWebPttTimeout();

	if (!request->hasArg("state"))
	{
		JsonDocument doc;
		doc["ok"] = false;
		doc["message"] = "state required";
		sendJsonDoc(request, 400, doc);
		return;
	}

	const bool state = isTruthyValue(request->arg("state"));
	const uint32_t holdMs = request->hasArg("hold_ms") ? (uint32_t)request->arg("hold_ms").toInt() : 1500UL;

	if (state)
	{
		if (!config.rf_en)
		{
			JsonDocument doc;
			doc["ok"] = false;
			doc["message"] = "radio disabled";
			sendJsonDoc(request, 409, doc);
			return;
		}
		webPttActive = true;
		webPttExpireMs = millis() + holdMs;
		setPtt(true);
	}
	else if (webPttActive)
	{
		setPtt(false);
		webPttActive = false;
		webPttExpireMs = 0;
	}

	JsonDocument doc;
	doc["ok"] = true;
	doc["ptt"] = getTransmit() ? 1 : 0;
	doc["web_ptt"] = webPttActive ? 1 : 0;
	sendJsonDoc(request, 200, doc);
}

void handle_api_diag_rf(AsyncWebServerRequest *request)
{
	if (!ensureWebAuth(request))
		return;
	serviceWebPttTimeout();

	JsonDocument doc;
	doc["ok"] = true;
	doc["ptt"] = getTransmit() ? 1 : 0;
	doc["sq"] = getSqlState();
	doc["tx_audio_activity"] = webPttActive ? 1 : 0;
	doc["rx_audio_mv"] = mVrms;
	doc["sta_connected"] = (WiFi.status() == WL_CONNECTED);
	doc["sta_ip"] = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "";
	doc["ap_ip"] = WiFi.softAPIP().toString();
	doc["rf_en"] = config.rf_en;
	sendJsonDoc(request, 200, doc);
}

void handle_api_selftest(AsyncWebServerRequest *request)
{
	if (!ensureWebAuth(request))
		return;
	serviceWebPttTimeout();

	JsonDocument doc;
	doc["ok"] = true;

#if defined(KV4P_HT)
	bool allOk = true;
	JsonArray checks = doc["checks"].to<JsonArray>();
	auto addCheck = [&](const char *name, int current, int expected)
	{
		JsonObject item = checks.add<JsonObject>();
		item["name"] = name;
		item["current"] = current;
		item["expected"] = expected;
		const bool ok = (current == expected);
		item["ok"] = ok;
		if (!ok)
			allOk = false;
	};

	addCheck("rf_tx_gpio", config.rf_tx_gpio, 17);
	addCheck("rf_rx_gpio", config.rf_rx_gpio, 16);
	addCheck("rf_sql_gpio", config.rf_sql_gpio, 4);
	addCheck("rf_pd_gpio", config.rf_pd_gpio, 19);
	addCheck("rf_ptt_gpio", config.rf_ptt_gpio, 18);
	addCheck("adc_gpio", config.adc_gpio, 34);
	addCheck("dac_gpio", config.dac_gpio, 25);
	doc["kv4p_2_0d_pass"] = allOk;
#else
	doc["kv4p_2_0d_pass"] = false;
	doc["message"] = "Self-test profile currently implemented for KV4P_HT build.";
#endif

	doc["ptt_state"] = getTransmit() ? 1 : 0;
	doc["sql_state"] = getSqlState();
	sendJsonDoc(request, 200, doc);
}

void handle_api_config(AsyncWebServerRequest *request)
{
	if (!ensureWebAuth(request))
		return;

	if (request->method() == HTTP_POST)
	{
		if (request->hasArg("msg_webhook_enable"))
		{
			config.msg_webhook_enable = isTruthyValue(request->arg("msg_webhook_enable"));
		}
		if (request->hasArg("msg_webhook_timeout_ms"))
		{
			uint16_t timeout = request->arg("msg_webhook_timeout_ms").toInt();
			if (timeout < 200)
				timeout = 200;
			if (timeout > 10000)
				timeout = 10000;
			config.msg_webhook_timeout_ms = timeout;
		}
		if (request->hasArg("msg_webhook_url"))
		{
			strlcpy(config.msg_webhook_url, request->arg("msg_webhook_url").c_str(), sizeof(config.msg_webhook_url));
		}
		config.cfg_version = 2;
		saveConfiguration("/default.cfg", config);
	}

	JsonDocument doc;
	doc["ok"] = true;
	doc["cfg_version"] = config.cfg_version;
	doc["msg_webhook_enable"] = config.msg_webhook_enable;
	doc["msg_webhook_url"] = config.msg_webhook_url;
	doc["msg_webhook_timeout_ms"] = config.msg_webhook_timeout_ms;
	sendJsonDoc(request, 200, doc);
}

void handle_api_config_backup(AsyncWebServerRequest *request)
{
	if (!ensureWebAuth(request))
		return;
	if (!LITTLEFS.exists("/default.cfg"))
	{
		request->send(404, "application/json", "{\"ok\":false,\"message\":\"default.cfg missing\"}");
		return;
	}
	AsyncWebServerResponse *response = request->beginResponse(LITTLEFS, "/default.cfg", "application/json", true);
	response->addHeader("Content-Disposition", "attachment; filename=\"default.cfg\"");
	request->send(response);
}

static bool configRestoreUploadOk = true;
void handleConfigRestoreUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
{
	(void)filename;
	if (index == 0)
	{
		if (!request->authenticate(config.http_username, config.http_password))
		{
			configRestoreUploadOk = false;
			return;
		}
		configRestoreUploadOk = true;
		if (LITTLEFS.exists("/default.cfg.upload"))
		{
			LITTLEFS.remove("/default.cfg.upload");
		}
		request->_tempFile = LITTLEFS.open("/default.cfg.upload", "w");
		if (!request->_tempFile)
		{
			configRestoreUploadOk = false;
			return;
		}
	}

	if (!configRestoreUploadOk)
	{
		return;
	}

	if (request->_tempFile && len > 0)
	{
		if (request->_tempFile.write(data, len) != len)
		{
			configRestoreUploadOk = false;
		}
	}

	if (final)
	{
		if (request->_tempFile)
		{
			request->_tempFile.close();
		}
		if (!configRestoreUploadOk)
		{
			LITTLEFS.remove("/default.cfg.upload");
			return;
		}
		if (LITTLEFS.exists("/default.cfg"))
		{
			LITTLEFS.remove("/default.cfg");
		}
		if (!LITTLEFS.rename("/default.cfg.upload", "/default.cfg"))
		{
			configRestoreUploadOk = false;
			LITTLEFS.remove("/default.cfg.upload");
		}
	}
}

void handle_api_config_restore(AsyncWebServerRequest *request)
{
	if (!ensureWebAuth(request))
		return;

	JsonDocument doc;
	if (!configRestoreUploadOk)
	{
		doc["ok"] = false;
		doc["message"] = "upload failed";
		sendJsonDoc(request, 500, doc);
		return;
	}
	if (!loadConfiguration("/default.cfg", config))
	{
		doc["ok"] = false;
		doc["message"] = "invalid config";
		sendJsonDoc(request, 422, doc);
		return;
	}

	saveConfiguration("/default.cfg", config);
	RF_MODULE(false);
	doc["ok"] = true;
	doc["message"] = "config restored";
	sendJsonDoc(request, 200, doc);
}

static void serveAppAsset(AsyncWebServerRequest *request, const char *path, const char *contentType)
{
	if (!ensureWebAuth(request))
		return;

	String gzPath = String(path) + ".gz";
	AsyncWebServerResponse *response = nullptr;
	if (LITTLEFS.exists(gzPath))
	{
		response = request->beginResponse(LITTLEFS, gzPath, contentType);
		response->addHeader("Content-Encoding", "gzip");
	}
	else if (LITTLEFS.exists(path))
	{
		response = request->beginResponse(LITTLEFS, path, contentType);
	}
	else
	{
		request->send(404, "text/plain", "Static asset not found");
		return;
	}
	response->addHeader("Cache-Control", "no-cache");
	request->send(response);
}

void handle_app_index(AsyncWebServerRequest *request);

void handle_app_root(AsyncWebServerRequest *request)
{
	const String url = request->url();
	if (!(url == "/" || url == "/app"))
	{
		request->send(404, "text/plain", "Not found");
		return;
	}
	handle_app_index(request);
}

void handle_app_index(AsyncWebServerRequest *request)
{
	serveAppAsset(request, "/app/index.html", "text/html");
}

void handle_app_js(AsyncWebServerRequest *request)
{
	serveAppAsset(request, "/app/app.js", "application/javascript");
}

void handle_app_css(AsyncWebServerRequest *request)
{
	serveAppAsset(request, "/app/app.css", "text/css");
}

void handle_app_commit(AsyncWebServerRequest *request)
{
	serveAppAsset(request, "/app/commit.txt", "text/plain");
}

void handle_legacy_route(AsyncWebServerRequest *request)
{
	if (!ensureWebAuth(request))
		return;
	AsyncWebServerResponse *response = request->beginResponse(302);
	response->addHeader("Location", "/app/");
	response->addHeader("Cache-Control", "no-store");
	request->send(response);
}

void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
    if (!index) {
        // Open the file in write mode on the first chunk
        request->_tempFile = LITTLEFS.open("/" + filename, "w");
    }
    if (len < (LITTLEFS.totalBytes()-LITTLEFS.usedBytes())) {
        // Write the data chunk to the file
        request->_tempFile.write(data, len);
    }else{
		// Not enough space to write the file
		request->_tempFile.close();
		LITTLEFS.remove("/" + filename); // Remove the incomplete file
		request->send(500, "text/plain", "Not enough space to upload the file");
		return;
	}
    if (final) {
        // Close the file on the last chunk and redirect
        request->_tempFile.close();
        request->redirect("/"); // Redirect back to the main page
    }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
	serviceWebPttTimeout();

	if (type == WS_EVT_CONNECT)
	{
		log_d("Websocket client connection received");
		if (server == &ws_audio)
		{
			audioTuneTouch();
			client->text("{\"type\":\"cfg\",\"codec\":\"mulaw\",\"rate\":8000}");
		}
	}
	else if (type == WS_EVT_DISCONNECT)
	{

		log_d("Client disconnected");
	}
	else if (type == WS_EVT_DATA && server == &ws_audio)
	{
		AwsFrameInfo *info = (AwsFrameInfo *)arg;
		if (info && info->opcode == WS_TEXT && info->final && info->index == 0)
		{
			if (len == 4 && strncmp((const char *)data, "ping", 4) == 0)
			{
				audioTuneTouch();
				client->text("pong");
			}
		}
	}
}

// void handle_vpn_request(AsyncWebServerRequest *request) {
//     HTTPClient http;

//     String url = "http://vpn.nakhonthai.net:82/wg/create";

//     String mac = WiFi.macAddress();
//     mac.replace(":", "");

//     String payload = "{\"name\":\"" + mac + "\"}";

//     http.begin(url);
//     http.addHeader("Content-Type", "application/json");

//     int httpCode = http.POST(payload);

//     if (httpCode > 0) {
//         String res = http.getString();
//         request->send(200, "application/json", res);
//     } else {
//         request->send(500, "text/plain", "Error contacting VPN server");
//     }

//     http.end();
// }

bool webServiceBegin = true;
void webService()
{
	if (webServiceBegin)
	{
		webServiceBegin = false;
	}
	else
	{
		return;
	}
	ws_audio.onEvent(onWsEvent);

	// web client handlers
	async_server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_app_root(request); });
	async_server.on("/app/", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_app_index(request); });
	async_server.on("/app/index.html", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_app_index(request); });
	async_server.on("/app/app.js", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_app_js(request); });
	async_server.on("/app/app.css", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_app_css(request); });
	async_server.on("/app/commit.txt", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_app_commit(request); });
	// Compatibility routes for stale cached HTML that references root-level assets.
	async_server.on("/app.js", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_app_js(request); });
	async_server.on("/app.css", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_app_css(request); });
	async_server.on("/commit.txt", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_app_commit(request); });
	async_server.on("/app", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_app_root(request); });
	async_server.on("/legacy", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_legacy_route(request); });
	async_server.on("/legacy/", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_legacy_route(request); });
	async_server.on("/logout", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_logout(request); });
	async_server.on("/audio_tune", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_audio_tune(request); });
	async_server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_api_status(request); });
	async_server.on("/api/contacts", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_api_contacts(request); });
	async_server.on("/api/messages", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_api_messages(request); });
	async_server.on("/api/message/send", HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_api_send_message(request); });
	async_server.on("/api/radio/status", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_api_radio_status(request); });
	async_server.on("/api/radio/set", HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_api_radio_set(request); });
	async_server.on("/api/radio/ptt", HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_api_radio_ptt(request); });
	async_server.on("/api/diag/rf", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_api_diag_rf(request); });
	async_server.on("/api/selftest", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_api_selftest(request); });
	async_server.on("/api/config", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_api_config(request); });
	async_server.on("/api/config/backup", HTTP_GET, [](AsyncWebServerRequest *request)
					{ handle_api_config_backup(request); });
	async_server.on("/api/config/restore", HTTP_POST, [](AsyncWebServerRequest *request)
					{ handle_api_config_restore(request); },
					handleConfigRestoreUpload);

	// async_server.on("/api/vpnreq", HTTP_GET, handle_vpn_request);
	async_server.on(
		"/update", HTTP_POST, [](AsyncWebServerRequest *request)
		{
		if (!request->authenticate(config.http_username, config.http_password))
		{
			return request->requestAuthentication();
		}
  		bool espShouldReboot = !Update.hasError();
  		AsyncWebServerResponse *response = request->beginResponse(200, "text/html", espShouldReboot ? "<h1><strong>Update DONE</strong></h1><br><a href='/app/'>Return Home</a>" : "<h1><strong>Update FAILED</strong></h1><br><a href='/app/'>Retry?</a>");
  		response->addHeader("Connection", "close");
  		request->send(response); },
		[](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
		{
			if (!request->authenticate(config.http_username, config.http_password))
			{
				return;
			}
			if (!index)
			{
				log_d("Update Start: %s\n", filename.c_str());
				if (!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000))
				{
					Update.printError(Serial);
				}
				else
				{
					adcEn = -1;
					dacEn = -1;
					delay(500);
					// disableLoopWDT();
					// disableCore0WDT();
					// disableCore1WDT();
					//  vTaskSuspend(taskAPRSPollHandle);
					//  vTaskSuspend(taskAPRSHandle);
					//  vTaskSuspend(taskSensorHandle);
					//  vTaskSuspend(taskSerialHandle);
					//  vTaskSuspend(taskGPSHandle);
					//  vTaskSuspend(taskSensorHandle);
				}
			}
			if (!Update.hasError())
			{
				if (Update.write(data, len) != len)
				{
					Update.printError(Serial);
				}
			}
			if (final)
			{
				if (Update.end(true))
				{
					log_d("Update Success: %uByte\n", index + len);
					delay(1000);
					esp_restart();
				}
				else
				{
					Update.printError(Serial);
				}
			}
		});

	lastheard_events.onConnect([](AsyncEventSourceClient *client)
							   {
    if(client->lastId()){
      log_d("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    //String html = event_lastHeard(true);
    //client->send(html.c_str(), "lastHeard", time(NULL), 5000); 
});
	async_server.addHandler(&lastheard_events);

	message_events.onConnect([](AsyncEventSourceClient *client)
							 {
    if(client->lastId()){
      log_d("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
	String html = event_chatMessage(true);
    client->send(html.c_str(), "chatMsg", time(NULL), 5000); });
	async_server.addHandler(&message_events);

	async_server.onNotFound(notFound);
	async_server.begin();
	async_server.addHandler(&ws_audio);
}
