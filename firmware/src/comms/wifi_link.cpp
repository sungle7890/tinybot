#include "comms/wifi_link.h"

#if defined(TINYBOT_PLATFORM_R4)

#include <Arduino.h>
#include <OTAUpdate.h>
#include <WiFiS3.h>
#include <string.h>

#include "comms/console.h"
#include "comms/fmt.h"
#include "comms/telemetry.h"
#include "config.h"
#include "control/control_loop.h"
#include "hal/hal.h"

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef TINYBOT_WIFI_SSID
#define TINYBOT_WIFI_SSID ""
#define TINYBOT_WIFI_PASS ""
#endif
#ifndef TINYBOT_AP_SSID
#define TINYBOT_AP_SSID "tinybot"
#endif
// No default password, on purpose. The command API has no authentication, so
// whoever joins the robot's own network can drive it - and a default published
// with the source is a password everyone already has. Without one set in
// secrets.h the robot simply does not open a network of its own.
#ifndef TINYBOT_AP_PASS
#define TINYBOT_AP_PASS ""
#endif

namespace wifi_link {
namespace {

// Network work only starts when at least this much of the control period is
// still ahead. A request is never waited for: bytes are taken as they arrive,
// across as many loop passes as it takes, so a 300 ms round trip costs the
// control loop nothing.
constexpr uint32_t kHeadroomUs = 12000;
constexpr uint32_t kClientTimeoutMs = 15000;

// Asking the modem whether a connection is waiting costs up to 7 ms, and the
// loop runs thousands of times a second, so polling it freely lands on a tick
// now and then. Once per this interval is plenty when a round trip is 300 ms.
constexpr uint32_t kAcceptIntervalMs = 100;

// Round trips over this link cost 100-400 ms, so a client polls slowly and
// takes a large batch each time rather than asking often.
constexpr uint8_t kTelemetryBatch = 40;

// A response is handed out in small pieces, each sent only when the tick is
// far enough away. One 1.6 kB write to the modem blocks for tens of
// milliseconds, which on this single-core board is a missed control tick.
constexpr size_t kChunkBytes = 128;
constexpr uint32_t kWriteHeadroomUs = 9000;

WiFiServer g_server(80);
WiFiClient g_client;
String g_request;
String g_out;
size_t g_outPos = 0;
uint32_t g_lastTelemetryFetchMs = 0;
// Telemetry only queues while switched on (`v`), and `v` is a toggle a remote
// client cannot see the state of. A fetch is an unambiguous request for data,
// so it switches the stream on; the stream goes back off once fetching stops,
// but only if it was a fetch that turned it on - never a `v` typed over USB.
bool g_telemetryOnForWifi = false;

// The join used to happen once, at boot. When the link dropped - the robot
// roaming out of range, or the battery sagging under the motors - the board
// stayed up with its lights on and never came back until a power cycle, and
// in the meantime nothing could reach it to say stop. Now the link is
// watched: a drop is reported to the control loop, which ends any session,
// and the robot rejoins once it is standing still.
constexpr uint32_t kLinkCheckMs = 1000;
constexpr uint32_t kRejoinIntervalMs = 15000;
bool g_linkLost = false;
uint32_t g_lastLinkCheckMs = 0;
uint32_t g_lastRejoinMs = 0;
uint32_t g_drops = 0;
uint32_t g_rejoins = 0;

// Firmware update over the air. The Wi-Fi module downloads the image and
// writes it into the main MCU, so the whole thing blocks for as long as the
// transfer takes - which is why it only runs with the robot idle, and only
// after the reply to the `ota` request has gone out.
char g_otaUrl[96] = "";
bool g_otaPending = false;
uint32_t g_otaRequestedMs = 0;
const char* g_otaStatus = "none this boot";
int g_otaCode = 0;
uint32_t g_clientStartedMs = 0;
uint32_t g_lastAcceptMs = 0;
bool g_up = false;
bool g_ap = false;
bool g_hasClient = false;
uint32_t g_accepted = 0, g_handled = 0, g_dropped = 0, g_closed = 0;

// Worst-case cost of each modem call, to find what stalls the control loop.
uint32_t g_maxAcceptUs = 0, g_maxPollUs = 0, g_maxReadUs = 0, g_maxWriteUs = 0;
#define TIME_CALL(slot, expr)                     \
  ({                                              \
    const uint32_t t0__ = micros();               \
    auto result__ = (expr);                       \
    const uint32_t dt__ = micros() - t0__;        \
    if (dt__ > slot) slot = dt__;                 \
    result__;                                     \
  })
char g_ip[20] = "0.0.0.0";

// Collects a console reply so it can be sent as an HTTP body.
// Capped so a runaway command cannot exhaust the heap, but high enough for the
// full `q` dump (~2 KB). Hitting the cap is said out loud: a reply that just
// stops mid-line reads as a firmware fault.
class StringSink : public Print {
 public:
  static constexpr size_t kMaxReply = 3000;
  size_t write(uint8_t c) override {
    if (text.length() < kMaxReply) {
      text += static_cast<char>(c);
    } else if (!truncated) {
      truncated = true;
      text += "\n...(reply truncated)\n";
    }
    return 1;
  }
  String text;
  bool truncated = false;
};

void storeIp(IPAddress ip) {
  snprintf(g_ip, sizeof(g_ip), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}

void urlDecode(String& s) {
  s.replace('+', ' ');
  int i;
  while ((i = s.indexOf('%')) >= 0 && i + 2 < static_cast<int>(s.length())) {
    const String hex = s.substring(i + 1, i + 3);
    s = s.substring(0, i) + static_cast<char>(strtol(hex.c_str(), nullptr, 16)) +
        s.substring(i + 3);
  }
}

// Sends Content-Length so the connection can stay open for the next request.
// Opening a fresh socket per request wedges this stack after the first reply.
void sendResponse(WiFiClient& c, const char* type, const String& body) {
  (void)c;
  char header[170];
  snprintf(header, sizeof(header),
           "HTTP/1.1 200 OK\r\nContent-Type: %s\r\nContent-Length: %u\r\n"
           // the dashboard is opened from a local file, so its origin is not
           // this board
           "Access-Control-Allow-Origin: *\r\nConnection: keep-alive\r\n\r\n",
           type, static_cast<unsigned>(body.length()));
  g_out = header;
  g_out += body;
  g_outPos = 0;
}

// Returns true while a response is still going out.
bool pumpResponse() {
  while (g_outPos < g_out.length()) {
    if (hal::microsUntilNextTick() < kWriteHeadroomUs) return true;
    const size_t n = min(kChunkBytes, g_out.length() - g_outPos);
    TIME_CALL(g_maxWriteUs, g_client.write(reinterpret_cast<const uint8_t*>(g_out.c_str()) + g_outPos, n));
    g_outPos += n;
  }
  g_out = "";
  g_outPos = 0;
  return false;
}

void handle(WiFiClient& c, const String& path) {
  if (path.startsWith("/api/telemetry")) {
    g_lastTelemetryFetchMs = millis();
    if (!telemetry::enabled()) {
      telemetry::setEnabled(true);
      g_telemetryOnForWifi = true;
    }
    String body;
    telemetry::Sample s;
    for (uint8_t n = 0; n < kTelemetryBatch && telemetry::pop(s); ++n) {
      char line[cfg::kTelemetryLineMax];
      telemetry::formatSample(s, line, sizeof(line));
      body += line;
    }
    sendResponse(c, "text/plain", body);
    return;
  }
  if (path.startsWith("/api/cmd")) {
    String cmd;
    const int q = path.indexOf("c=");
    if (q >= 0) {
      cmd = path.substring(q + 2);
      const int amp = cmd.indexOf('&');
      if (amp >= 0) cmd = cmd.substring(0, amp);
      urlDecode(cmd);
    }
    StringSink out;
    if (cmd.length() > 0) console::execute(cmd.c_str(), out);
    sendResponse(c, "text/plain", out.text);
    return;
  }
  sendResponse(c, "text/html",
               "<!doctype html><meta charset=utf-8><title>tinybot</title>"
               "<body style=\"font-family:system-ui;padding:24px\">"
               "<h1>tinybot</h1><p>Telemetry: <code>/api/telemetry</code><br>"
               "Command: <code>/api/cmd?c=st</code></p>");
}

// stop() must be called even when the peer hung up: without it the socket is
// never returned on the modem side, and after a couple of requests the board
// runs out and stops answering.
void closeClient() {
  g_client.stop();
  g_request = "";
  g_hasClient = false;
  ++g_closed;
}

bool idleForBlockingWork() {
  const control::Mode m = control::mode();
  return m == control::Mode::kIdle || m == control::Mode::kSafetyStop;
}

// Rejoining blocks while the module negotiates, and a blocked loop leaves the
// motors on their last duty - so it only happens while nothing is driving.
void checkLink() {
  if (g_ap) return;
  const uint32_t now = millis();
  if (now - g_lastLinkCheckMs < kLinkCheckMs) return;
  g_lastLinkCheckMs = now;

  if (WiFi.status() == WL_CONNECTED) {
    g_linkLost = false;
    return;
  }
  if (!g_linkLost) {
    g_linkLost = true;
    ++g_drops;
    if (g_hasClient) closeClient();
  }
  if (!idleForBlockingWork()) return;
  if (now - g_lastRejoinMs < kRejoinIntervalMs) return;
  g_lastRejoinMs = now;
  ++g_rejoins;

  WiFi.disconnect();
  WiFi.begin(TINYBOT_WIFI_SSID, TINYBOT_WIFI_PASS);
  for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; ++i) delay(300);
  if (WiFi.status() != WL_CONNECTED) return;
  for (int i = 0; i < 20 && WiFi.localIP() == IPAddress(0, 0, 0, 0); ++i) delay(250);
  storeIp(WiFi.localIP());
  g_server.begin();
  g_linkLost = false;
}

void runOta() {
  g_otaPending = false;
  if (g_hasClient) closeClient();
  OTAUpdate ota;
  // Each step reports its own failure; a success ends in a reset, so the only
  // sign of it is the new build time in `st` after the robot comes back.
  static const char kFile[] = "/update.bin";
  g_otaStatus = "failed at begin";
  g_otaCode = ota.begin(kFile);
  if (g_otaCode != OTAUpdate::OTA_ERROR_NONE) return;
  g_otaStatus = "failed at download";
  g_otaCode = ota.download(g_otaUrl, kFile);
  if (g_otaCode <= 0) return;
  g_otaStatus = "failed at verify";
  g_otaCode = ota.verify();
  if (g_otaCode != OTAUpdate::OTA_ERROR_NONE) return;
  g_otaStatus = "failed at update";
  g_otaCode = ota.update(kFile);
}

}  // namespace

bool begin() {
  if (WiFi.status() == WL_NO_MODULE) return false;

  const char* ssid = TINYBOT_WIFI_SSID;
  if (ssid[0] != '\0') {
    WiFi.begin(ssid, TINYBOT_WIFI_PASS);
    for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; ++i) delay(300);
  }
  if (WiFi.status() == WL_CONNECTED) {
    // Association can report success a moment before DHCP hands out the
    // address, which would otherwise be recorded as 0.0.0.0.
    for (int i = 0; i < 40 && WiFi.localIP() == IPAddress(0, 0, 0, 0); ++i) {
      delay(250);
    }
    storeIp(WiFi.localIP());
  } else {
    // No network configured, or joining failed: make our own so the robot is
    // still reachable without a cable - but only with a password the owner
    // chose. WPA2 needs at least 8 characters; anything shorter would either
    // fail here or, worse, fall back to an open network.
    if (strlen(TINYBOT_AP_PASS) < 8) return false;
    if (WiFi.beginAP(TINYBOT_AP_SSID, TINYBOT_AP_PASS) != WL_AP_LISTENING) {
      return false;
    }
    delay(2000);
    g_ap = true;
    storeIp(WiFi.localIP());
  }
  g_server.begin();
  g_up = true;
  return true;
}

void service() {
  if (!g_up) return;
  // Wait for the reply to the `ota` request to leave first, or the client that
  // asked never hears that the update started.
  if (g_otaPending && g_out.length() == 0 && millis() - g_otaRequestedMs > 500 &&
      idleForBlockingWork()) {
    runOta();
    return;
  }
  checkLink();
  if (g_linkLost) return;
  if (g_telemetryOnForWifi && millis() - g_lastTelemetryFetchMs >= 3000) {
    g_telemetryOnForWifi = false;
    if (telemetry::enabled()) telemetry::setEnabled(false);
  }
  if (hal::microsUntilNextTick() < kHeadroomUs) return;

  // Finish sending before looking at anything else.
  if (g_hasClient && pumpResponse()) return;

  if (g_hasClient && !TIME_CALL(g_maxPollUs, g_client.connected())) closeClient();

  if (!g_hasClient) {
    if (millis() - g_lastAcceptMs < kAcceptIntervalMs) return;
    g_lastAcceptMs = millis();
    // accept(), not available(): WiFiS3's available() caches the previous
    // client and keeps handing back its closed socket, so the second request
    // of a session never arrives.
    WiFiClient incoming = TIME_CALL(g_maxAcceptUs, g_server.accept());
    if (!incoming || !incoming.connected()) return;
    g_client = incoming;
    g_request = "";
    g_clientStartedMs = millis();
    g_hasClient = true;
    ++g_accepted;
  }

  // Read in bulk: every single-byte read() is a round trip to the Wi-Fi
  // module, so reading a request byte by byte costs a hundred of them and
  // stalls the control loop.
  int pending = TIME_CALL(g_maxPollUs, g_client.available());
  while (pending > 0) {
    uint8_t buf[192];
    const int n = TIME_CALL(g_maxReadUs, g_client.read(buf, min(pending, static_cast<int>(sizeof(buf)))));
    if (n <= 0) break;
    pending -= n;
    for (int i = 0; i < n; ++i) {
      const char ch = static_cast<char>(buf[i]);
      if (ch == '\r') continue;
      if (ch != '\n') {
        if (g_request.length() < 200) g_request += ch;
        continue;
      }
      if (g_request.length() == 0) continue;  // blank line ends the headers
      if (g_request.startsWith("GET ")) {
        const int sp = g_request.indexOf(' ', 4);
        handle(g_client, sp > 4 ? g_request.substring(4, sp) : String("/"));
        g_clientStartedMs = millis();
        ++g_handled;
      }
      g_request = "";
    }
    if (g_out.length() > 0) break;  // a reply is queued; send it first
  }

  // Drop a connection that has gone quiet, so a dead client cannot hold the
  // single server slot forever.
  if (millis() - g_clientStartedMs > kClientTimeoutMs) {
    ++g_dropped;
    closeClient();
  }
}

bool connected() { return g_up && !g_linkLost; }
bool linkLost() { return g_up && g_linkLost; }
uint32_t drops() { return g_drops; }
uint32_t rejoins() { return g_rejoins; }

bool requestOta(const char* url) {
  if (strncmp(url, "http://", 7) != 0) return false;
  if (strlen(url) >= sizeof(g_otaUrl)) return false;
  strcpy(g_otaUrl, url);
  g_otaRequestedMs = millis();
  g_otaPending = true;
  g_otaStatus = "pending";
  return true;
}

const char* otaStatus() { return g_otaStatus; }
int otaCode() { return g_otaCode; }

const char* ipAddress() {
  // Refresh in case the lease arrived after begin() gave up waiting.
  if (g_up && !g_ap && WiFi.status() == WL_CONNECTED &&
      WiFi.localIP() != IPAddress(0, 0, 0, 0)) {
    storeIp(WiFi.localIP());
  }
  return g_ip;
}

bool isAccessPoint() { return g_ap; }

// True while a client is actively pulling telemetry, so the USB console does
// not also drain the queue and leave the network one sample per request.
bool streamingTelemetry() {
  return g_up && millis() - g_lastTelemetryFetchMs < 3000;
}

void report(Print& out) {
  out.print("wifi status   : ");
  out.println(WiFi.status());
  out.print("client held   : ");
  out.println(g_hasClient ? "yes" : "no");
  out.print("accepted      : ");
  out.println(g_accepted);
  out.print("handled       : ");
  out.println(g_handled);
  out.print("timed out     : ");
  out.println(g_dropped);
  out.print("closed        : ");
  out.println(g_closed);
  out.print("max accept us : ");
  out.println(g_maxAcceptUs);
  out.print("max poll us   : ");
  out.println(g_maxPollUs);
  out.print("max read us   : ");
  out.println(g_maxReadUs);
  out.print("max write us  : ");
  out.println(g_maxWriteUs);
  g_maxAcceptUs = g_maxPollUs = g_maxReadUs = g_maxWriteUs = 0;
}

}  // namespace wifi_link

#else  // the ESP32 build has no link yet

namespace wifi_link {
bool begin() { return false; }
void service() {}
bool connected() { return false; }
bool linkLost() { return false; }
uint32_t drops() { return 0; }
uint32_t rejoins() { return 0; }
bool requestOta(const char*) { return false; }
const char* otaStatus() { return "not built"; }
int otaCode() { return 0; }
const char* ipAddress() { return "0.0.0.0"; }
bool isAccessPoint() { return false; }
bool streamingTelemetry() { return false; }
void report(Print& out) { out.println("wifi          : not built"); }
}  // namespace wifi_link

#endif
