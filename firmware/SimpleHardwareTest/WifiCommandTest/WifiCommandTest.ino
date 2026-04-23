/*
 * WiFi command test — UNO R4 WiFi (WiFiS3 / ESP32-S3 coprocessor)
 *
 * What it does
 *   Connects to your Wi‑Fi, prints the board IP on Serial, and listens on TCP_PORT
 *   for plain-text commands (one line per command, newline-terminated).
 *
 * Quick setup (Arduino IDE 2.x)
 *   1. Board manager: install "Arduino UNO R4 Boards" (Renesas UNO R4).
 *   2. Tools → Board → Arduino UNO R4 WiFi.
 *   3. Edit WIFI_SSID and WIFI_PASS below (2.4 GHz WPA2/Personal networks work;
 *      many home routers use the same SSID for 2.4 and 5 GHz — that is fine).
 *   4. Upload this sketch; open Serial Monitor at SERIAL_BAUD.
 *   5. Wait for "WiFi station MAC:" (for router allow‑lists) and "IP address:" after connect;
 *      then connect from your PC/phone on the same LAN.
 *
 * MAC address (WiFi interface)
 *   On UNO R4 WiFi, WiFiS3 only returns a valid station MAC after WiFi.begin / beginAP /
 *   scanNetworks. This sketch runs a short scan at boot so you can copy the MAC before
 *   association. You can also look in your router’s DHCP/connected‑clients list once online.
 *
 * How to send test commands
 *   Replace BOARD_IP with the address printed on Serial.
 *
 *   Linux / macOS (netcat):
 *     printf 'HELP\n' | nc BOARD_IP TCP_PORT
 *     printf 'PING\nSTATUS\nLED\n' | nc BOARD_IP TCP_PORT
 *
 *   Windows PowerShell (Test-NetConnection only checks reachability):
 *     Test-NetConnection BOARD_IP -Port TCP_PORT
 *   For an interactive session, install nmap and use: ncat BOARD_IP TCP_PORT
 *   Or use PuTTY: Connection type "Raw", host BOARD_IP, port TCP_PORT.
 *
 * Commands (case-insensitive, trim whitespace)
 *   HELP   — list commands
 *   PING   — reply PONG
 *   STATUS — SSID, IP, RSSI
 *   LED    — toggle the built-in LED (pin 13 on UNO R4 WiFi)
 */

#include "WiFiS3.h"

// ---------------------------------------------------------------------------
// Edit for your network
// ---------------------------------------------------------------------------
static const char *WIFI_SSID = "NU-IoT";
static const char *WIFI_PASS = "lvzddqvs";

// TCP port for plain-text commands (avoid 80 unless you want HTTP tooling)
static const uint16_t TCP_PORT = 8888;

// Serial speed for USB CDC on UNO R4 WiFi
static const long SERIAL_BAUD = 115200;

// ---------------------------------------------------------------------------

static int wifi_status = WL_IDLE_STATUS;
static WiFiServer server(TCP_PORT);

static void print_mac_to_print(Print &out, const uint8_t mac[6]) {
  for (unsigned i = 0; i < 6; i++) {
    if (i) {
      out.print(':');
    }
    if (mac[i] < 16) {
      out.print('0');
    }
    out.print(mac[i], HEX);
  }
}

static void print_station_mac_serial(const char *label) {
  uint8_t mac[6] = {0};
  WiFi.macAddress(mac);
  Serial.print(label);
  print_mac_to_print(Serial, mac);
  Serial.println();
}

static void print_wifi_status(void) {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  print_station_mac_serial("WiFi station MAC: ");

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.print("Signal (RSSI): ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
}

static String read_line(WiFiClient &client) {
  String line;
  line.reserve(64);
  const unsigned long deadline = millis() + 3000;
  while (client.connected() && millis() < deadline) {
    while (client.available()) {
      char c = static_cast<char>(client.read());
      if (c == '\n') {
        return line;
      }
      if (c != '\r') {
        line += c;
        if (line.length() > 128) {
          return String();
        }
      }
    }
    delay(1);
  }
  return line;
}

static String normalize_cmd(String s) {
  s.trim();
  s.toUpperCase();
  return s;
}

static void handle_client(WiFiClient client) {
  Serial.println("[TCP] Client connected");
  client.print("UNO R4 WiFi command test. Type HELP.\r\n");

  while (client.connected()) {
    if (!client.available()) {
      delay(1);
      continue;
    }

    String line = read_line(client);
    String cmd = normalize_cmd(line);

    if (cmd.length() == 0) {
      continue;
    }

    Serial.print("[TCP] cmd: ");
    Serial.println(cmd);

    if (cmd == "HELP") {
      client.print("Commands: HELP PING STATUS LED\r\n");
    } else if (cmd == "PING") {
      client.print("PONG\r\n");
    } else if (cmd == "STATUS") {
      uint8_t mac[6] = {0};
      WiFi.macAddress(mac);
      client.print("SSID: ");
      client.print(WiFi.SSID());
      client.print("  MAC: ");
      print_mac_to_print(client, mac);
      client.print("  IP: ");
      client.print(WiFi.localIP());
      client.print("  RSSI: ");
      client.print(WiFi.RSSI());
      client.print(" dBm\r\n");
    } else if (cmd == "LED") {
      static bool led_on = false;
      led_on = !led_on;
      digitalWrite(LED_BUILTIN, led_on ? HIGH : LOW);
      client.print(led_on ? "LED ON\r\n" : "LED OFF\r\n");
    } else {
      client.print("ERR unknown command (HELP for list)\r\n");
    }
  }

  client.stop();
  Serial.println("[TCP] Client disconnected");
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(SERIAL_BAUD);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 5000) {
    delay(10);
  }

  Serial.println();
  Serial.println("WiFi command test starting...");

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("ERROR: WiFi module not responding (WL_NO_MODULE). Halt.");
    while (true) {
      delay(1000);
    }
  }

  String fv = WiFi.firmwareVersion();
  Serial.print("WiFi firmware: ");
  Serial.println(fv);
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Note: WiFi firmware may be older than latest; consider updating.");
  }

  // WiFiS3: macAddress() is valid only after begin / beginAP / scanNetworks.
  Serial.println("Reading station MAC (short network scan)...");
  {
    const int n = WiFi.scanNetworks();
    Serial.print("Scan done; networks seen: ");
    Serial.println(n);
  }
  print_station_mac_serial("WiFi station MAC: ");
  Serial.println("(Use this MAC for router allow‑lists / DHCP reservations.)");

  while (wifi_status != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.print(WIFI_SSID);
    Serial.println(" ...");
    wifi_status = WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(8000);
  }

  Serial.println("Connected.");
  print_wifi_status();

  server.begin();
  Serial.print("TCP server on port ");
  Serial.println(TCP_PORT);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost; reconnecting...");
    wifi_status = WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(5000);
    return;
  }

  WiFiClient client = server.available();
  if (client) {
    handle_client(client);
  }
}
