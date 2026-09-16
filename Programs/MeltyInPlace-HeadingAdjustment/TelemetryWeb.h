#pragma once

// ESP32 Arduino, header-only, read-only Wi-Fi telemetry console.
// Include ONLY from your main .ino file. No additional Arduino libraries needed.
// Call telemetryBegin() once from setup(), while motor outputs are safe.
// Call telemetryPrintf()/telemetryPrintln() from normal task code, NOT an ISR.
// These functions copy to a bounded queue with zero wait for free space.
// They do not wait for a browser or transmit network data from the caller.
// Wi-Fi and formatting still consume resources; this is NOT a real-time guarantee.

#ifndef ARDUINO_ARCH_ESP32
#error "TelemetryWeb.h requires an ESP32 Arduino board."
#endif

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace telemetry_detail {

constexpr size_t MESSAGE_CHARS = 192;  // Includes the terminating zero.
constexpr UBaseType_t QUEUE_DEPTH = 16;
constexpr size_t HISTORY_LINES = 32;
constexpr size_t LINE_CHARS = MESSAGE_CHARS + 32;

struct Message {
    uint32_t timestampMs;
    char text[MESSAGE_CHARS];
};

static QueueHandle_t messages = nullptr;
static TaskHandle_t httpTaskHandle = nullptr;
static WebServer server(80);
static std::atomic<uint32_t> dropped{0};
static bool attempted = false;
static bool started = false;

// Only the HTTP task reads or modifies the history and response buffers.
static char history[HISTORY_LINES][LINE_CHARS] = {};
static char response[HISTORY_LINES * LINE_CHARS + 128] = {};
static size_t nextLine = 0;
static size_t lineCount = 0;

static const char PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<link rel="icon" href="data:,">
<title>ESP32 Telemetry</title>
<style>
:root { color-scheme: dark; }
body { margin: 0; background: #10151c; color: #e7edf5; font: 16px system-ui,sans-serif; }
main { max-width: 1100px; margin: auto; padding: 24px; }
h1 { font-size: 26px; margin-bottom: 8px; }
p { color: #aebdd0; line-height: 1.5; }
.controls { display: flex; align-items: center; gap: 12px; flex-wrap: wrap; margin: 20px 0; }
button { padding: 9px 15px; border-radius: 6px; border: 1px solid #53657c; background: #243247; color: inherit; cursor: pointer; }
#status { color: #a9d8bd; }
pre { margin: 0; padding: 18px; border: 1px solid #38475b; border-radius: 8px; background: #080d13; height: 62vh; overflow: auto; white-space: pre-wrap; overflow-wrap: anywhere; font: 14px/1.55 ui-monospace,monospace; }
small { color: #aebdd0; }
</style>
</head>
<body>
<main>
<h1>ESP32 Telemetry</h1>
<p>Read-only telemetry. The latest 32 messages are kept in RAM.
Timestamps are milliseconds since the ESP32 started.</p>
<div class="controls">
<button id="pause" type="button">Pause display</button>
<button id="save" type="button">Save visible log</button>
<span id="status" role="status">Connecting...</span>
</div>
<pre id="log">Waiting for telemetry...</pre>
<p><small>The page checks for updates about five times per second.
Pausing this display does not pause your ESP32 program.</small></p>
</main>
<script>
const output = document.getElementById('log');
const statusText = document.getElementById('status');
const pauseButton = document.getElementById('pause');
let paused = false;
let lastText = '';

pauseButton.addEventListener('click', () => {
  paused = !paused;
  pauseButton.textContent = paused ? 'Resume display' : 'Pause display';
  statusText.textContent = paused ? 'Display paused' : 'Connecting...';
});

document.getElementById('save').addEventListener('click', () => {
  const blob = new Blob([lastText], {type: 'text/plain;charset=utf-8'});
  const url = URL.createObjectURL(blob);
  const link = document.createElement('a');
  link.href = url;
  link.download = 'esp32-telemetry.txt';
  document.body.appendChild(link);
  link.click();
  link.remove();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});

async function poll() {
  if (paused || document.hidden) {
    setTimeout(poll, 200);
    return;
  }
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 2000);
  try {
    const result = await fetch('/log', {
      cache: 'no-store', signal: controller.signal
    });
    if (!result.ok) throw new Error('HTTP ' + result.status);
    const text = await result.text();
    if (!paused) {
      const follow = output.scrollHeight - output.scrollTop - output.clientHeight < 50;
      if (text !== lastText) {
        lastText = text;
        output.textContent = text; // Never interpret telemetry as HTML.
        if (follow) output.scrollTop = output.scrollHeight;
      }
      statusText.textContent = 'Connected - ' + new Date().toLocaleTimeString();
    }
  } catch (error) {
    if (!paused) statusText.textContent = 'No response - reconnecting...';
  } finally {
    clearTimeout(timeout);
    setTimeout(poll, 200); // Never overlap requests from this page.
  }
}
poll();
</script>
</body>
</html>
)HTML";

static void appendHistory(const Message& message) {
    std::snprintf(history[nextLine], LINE_CHARS, "[%10lu ms] %s\n",
                  static_cast<unsigned long>(message.timestampMs), message.text);
    nextLine = (nextLine + 1) % HISTORY_LINES;
    if (lineCount < HISTORY_LINES) ++lineCount;
}

static void drainQueue() {
    Message message{};
    // Limit work per pass even if another task logs continuously.
    for (UBaseType_t i = 0; i < QUEUE_DEPTH; ++i) {
        if (xQueueReceive(messages, &message, 0) != pdTRUE) break;
        appendHistory(message);
    }
}

static const char* snapshot() {
    const int headerLength = std::snprintf(
        response, sizeof(response), "Queue-overflow drops since startup: %lu\n\n",
        static_cast<unsigned long>(dropped.load(std::memory_order_relaxed)));
    size_t used = headerLength > 0 ? static_cast<size_t>(headerLength) : 0;
    const size_t first = (nextLine + HISTORY_LINES - lineCount) % HISTORY_LINES;
    for (size_t i = 0; i < lineCount; ++i) {
        const char* line = history[(first + i) % HISTORY_LINES];
        const size_t length = std::strlen(line);
        if (used + length >= sizeof(response)) break;
        std::memcpy(response + used, line, length);
        used += length;
    }
    response[used] = '\0';
    return response;
}

static void runHttp(void*) {
    const TickType_t pauseTicks = pdMS_TO_TICKS(2) > 0 ? pdMS_TO_TICKS(2) : 1;
    for (;;) {
        drainQueue();
        server.handleClient(); // Only this task handles HTTP, never Arduino loop().
        vTaskDelay(pauseTicks);
    }
}

static void cleanupFailedStart() {
    server.stop();
    WiFi.softAPdisconnect(true);
    if (messages != nullptr) {
        vQueueDelete(messages);
        messages = nullptr;
    }
}

} // namespace telemetry_detail

// Start once from setup(). Returns false if validation or startup fails.
// The access point permits ONE connected Wi-Fi device. IP: 192.168.4.1.
// This switches Wi-Fi to AP-only mode; it does not join an existing router.
static inline bool telemetryBegin(const char* ssid, const char* password) {
    using namespace telemetry_detail;
    if (started) return true;
    if (attempted) return false;
    attempted = true;

    if (ssid == nullptr || password == nullptr ||
        std::strlen(ssid) == 0 || std::strlen(ssid) > 32 ||
        std::strlen(password) < 8 || std::strlen(password) > 63) {
        return false;
    }

    messages = xQueueCreate(QUEUE_DEPTH, sizeof(Message));
    if (messages == nullptr) return false;

    const IPAddress ip(192, 168, 4, 1);
    const IPAddress subnet(255, 255, 255, 0);
    if (!WiFi.mode(WIFI_AP) || !WiFi.softAPConfig(ip, ip, subnet) ||
        !WiFi.softAP(ssid, password, 1, 0, 1)) {
        cleanupFailedStart();
        return false;
    }

    server.on("/", HTTP_GET, []() {
        server.sendHeader("Cache-Control", "no-store");
        server.send_P(200, "text/html; charset=utf-8", PAGE);
    });
    server.on("/log", HTTP_GET, []() {
        drainQueue();
        server.sendHeader("Cache-Control", "no-store");
        server.send(200, "text/plain; charset=utf-8", snapshot());
    });
    server.onNotFound([]() {
        server.send(404, "text/plain", "Not found");
    });
    server.begin();

    // On a normal dual-core ESP32, keep HTTP on the opposite core from setup/loop.
    // Single-core builds still get a separate task, but not a separate CPU core.
#if defined(CONFIG_FREERTOS_UNICORE) && CONFIG_FREERTOS_UNICORE
    const BaseType_t httpCore = 0;
#else
    const BaseType_t httpCore = (xPortGetCoreID() == 0) ? 1 : 0;
#endif
    const BaseType_t result = xTaskCreatePinnedToCore(
        runHttp, "telemetry-http", 8192, nullptr, 1, &httpTaskHandle, httpCore);
    if (result != pdPASS) {
        cleanupFailedStart();
        return false;
    }
    started = true;
    return true;
}

static inline bool telemetryPrintf(const char* format, ...)
    __attribute__((format(printf, 1, 2)));

// One call creates one timestamped record. A newline is added automatically.
// Text longer than 191 bytes is truncated. Use constant printf format strings.
// Returns false if not started, formatting fails, or the queue is full.
static inline bool telemetryPrintf(const char* format, ...) {
    using namespace telemetry_detail;
    if (messages == nullptr || format == nullptr) return false;
    Message message{};
    message.timestampMs = static_cast<uint32_t>(millis());
    va_list arguments;
    va_start(arguments, format);
    const int length = std::vsnprintf(message.text, sizeof(message.text), format, arguments);
    va_end(arguments);
    if (length < 0) return false;
    if (static_cast<size_t>(length) >= sizeof(message.text)) {
        std::memcpy(message.text + sizeof(message.text) - 4, "...", 4);
    }
    if (xQueueSend(messages, &message, 0) == pdTRUE) return true;
    dropped.fetch_add(1, std::memory_order_relaxed);
    return false;
}

static inline bool telemetryPrintln(const char* text) {
    return text != nullptr && telemetryPrintf("%s", text);
}
