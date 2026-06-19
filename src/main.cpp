#include <Arduino.h>
bool shouldRestart = false;
unsigned long restartRequestTime = 0;
#include "./blled/logSerial.h"
#include "./blled/leds.h"
#include "./blled/filesystem.h"
#include "./blled/types.h"
#include "./blled/bblPrinterDiscovery.h"
#include "./blled/web-server.h"
#include "./blled/mqttmanager.h"
#include "./blled/hamanager.h"
#include "./blled/serialmanager.h"
#include "./blled/wifi-manager.h"
#include "./blled/ssdp.h"


int wifi_reconnect_count = 0;

// WiFi + reconnect diagnostics task — prints every 5 s, summarises per minute.
// Accesses g_mqttConnectAttempts (mqttmanager.h) and g_haConnectAttempts (hamanager.h).
static void wifiMonitorTask(void *pv)
{
    static int prev_mqttAttempts    = 0;
    static int prev_haAttempts      = 0;
    static int prev_wifiDrops       = 0;
    static unsigned long prev_rxMsgs  = 0;
    static unsigned long prev_rxBytes = 0;
    unsigned long minuteStartMs     = millis();

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));

        Serial.printf("[WiFi] T=%lus status=%d RSSI=%ddBm sleep=%d heap=%u\n",
                      millis() / 1000,
                      (int)WiFi.status(),
                      (int)WiFi.RSSI(),
                      (int)WiFi.getSleep(),
                      (unsigned)ESP.getFreeHeap());

        if ((millis() - minuteStartMs) >= 60000)
        {
            int mqttDelta = g_mqttConnectAttempts - prev_mqttAttempts;
            int haDelta   = g_haConnectAttempts   - prev_haAttempts;
            int wifiDelta = wifi_reconnect_count   - prev_wifiDrops;

            unsigned long rxMsgsDelta  = g_mqttRxMsgs  - prev_rxMsgs;
            unsigned long rxBytesDelta = g_mqttRxBytes - prev_rxBytes;
            g_mqttRxMsgsPerMin  = rxMsgsDelta;
            g_mqttRxBytesPerMin = rxBytesDelta;

            Serial.printf("[DIAG] 1-min: mqttConnects=%d haConnects=%d wifiDrops=%d\n",
                          mqttDelta, haDelta, wifiDelta);
            Serial.printf("[MQTT] rxMsgs=%lu/min rxBytes=%lu/min\n", rxMsgsDelta, rxBytesDelta);

            if (mqttDelta > 20)
                Serial.println(F("[WARN] EXCESSIVE printer MQTT connect rate (>20/min) — check accessCode and printerIP"));
            if (haDelta > 20)
                Serial.println(F("[WARN] EXCESSIVE HA MQTT connect rate (>20/min) — check HA broker host/port"));
            if (wifiDelta > 20)
                Serial.println(F("[WARN] EXCESSIVE WiFi drop rate (>20/min)"));

            prev_mqttAttempts = g_mqttConnectAttempts;
            prev_haAttempts   = g_haConnectAttempts;
            prev_wifiDrops    = wifi_reconnect_count;
            prev_rxMsgs       = g_mqttRxMsgs;
            prev_rxBytes      = g_mqttRxBytes;
            minuteStartMs     = millis();
        }
    }
}

void defaultcolors()
{
    LogSerial.println(F("Setting default customisable colors"));
    printerConfig.runningColor = hex2rgb("#000000", 255, 255); // WHITE Running
    printerConfig.testColor = hex2rgb("#3F3CFB");              // Violet Test
    printerConfig.finishColor = hex2rgb("#00FF00");            // Green Finish

    printerConfig.stage14Color = hex2rgb("#000000"); // OFF Cleaning Nozzle
    printerConfig.stage1Color = hex2rgb("#000055");  // OFF Bed Leveling
    printerConfig.stage8Color = hex2rgb("#000000");  // OFF Calibrating Extrusion
    printerConfig.stage9Color = hex2rgb("#000000");  // OFF Scanning Bed Surface
    printerConfig.stage10Color = hex2rgb("#000000"); // OFF First Layer Inspection

    printerConfig.wifiRGB = hex2rgb("#FFA500"); // Orange Wifi Scan

    printerConfig.pauseRGB = hex2rgb("#0000FF");          // Blue Pause
    printerConfig.firstlayerRGB = hex2rgb("#0000FF");     // Blue
    printerConfig.nozzleclogRGB = hex2rgb("#0000FF");     // Blue
    printerConfig.hmsSeriousRGB = hex2rgb("#FF0000");     // Red
    printerConfig.hmsFatalRGB = hex2rgb("#FF0000");       // Red
    printerConfig.filamentRunoutRGB = hex2rgb("#FF0000"); // Red
    printerConfig.frontCoverRGB = hex2rgb("#FF0000");     // Red
    printerConfig.nozzleTempRGB = hex2rgb("#FF0000");     // Red
    printerConfig.bedTempRGB = hex2rgb("#FF0000");        // Red
}

void setup()
{
    Serial.begin(115200);
    delay(100);
    Serial.println(F("Initializing"));
    Serial.println(ESP.getFreeHeap());
    Serial.println("");
    Serial.print(F("** Using firmware version: "));
    Serial.print(globalVariables.FWVersion);
    Serial.println(F(" **"));
    Serial.println("");
    defaultcolors();
    setupLeds();
    tweenToColor(100, 100, 100, 100, 100); // ALL LEDS ON
    Serial.println(F(""));

    tweenToColor(255, 0, 0, 0, 0); // RED
    setupFileSystem();
    loadFileSystem();
    Serial.println(F(""));

    tweenToColor(printerConfig.wifiRGB); // Customisable - Default is ORANGE

    if (!printerConfig.wifiRadioEnabled) {
        Serial.println(F("[DIAG] WiFi radio DISABLED by config — radio off, serial recovery only"));
        Serial.println(F("[DIAG] To re-enable: send {\"wifiRadio\":true} over USB serial"));
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        tweenToColor(50, 0, 50, 0, 0); // dim purple = radio off
        for (;;) {
            serialLoop();
            if (shouldRestart) ESP.restart();
        }
    }

#ifdef DEBUG_WIFI_OFF
    Serial.println(F("[DEBUG] WiFi OFF mode — radio disabled for noise diagnosis"));
    WiFi.mode(WIFI_OFF);
    tweenToColor(100, 0, 100, 0, 0); // Purple = WiFi completely off
    for (;;) {
        ledsloop();
        serialLoop();
        if (shouldRestart) ESP.restart();
    }
#endif

    setupSerial();

    if (strlen(globalVariables.SSID) == 0 || strlen(globalVariables.APPW) == 0)
    {
        Serial.println(F("SSID or password is missing. Please configure both by going to: https://dutchdevelop.com/blled-configuration-setup/"));
        tweenToColor(100, 0, 100, 0, 0); // PINK
        startAPMode();
        setupWebserver();
        return;
    }
    else

        if (!connectToWifi())
    {
        Serial.println(F("[WiFiManager] Not connected → AP Mode"));
        startAPMode();
        setupWebserver();
        return;
    }
    else
    {
        Serial.println(F("[WiFiManager] connected. Starting webUI."));
        tweenToColor(0, 0, 255, 0, 0); // BLUE
        setupWebserver();
    }

    // Start WiFi diagnostics monitor on Core 0 (main loop / led work runs on Core 1)
    xTaskCreatePinnedToCore(wifiMonitorTask, "wifiMon", 4096, NULL, 1, NULL, 0);

    // SSDP responder — passive, responds to UPnP queries
    if (printerConfig.diagEnableSsdp) {
        start_ssdp();
    } else {
        Serial.println(F("[DIAG] SSDP DISABLED"));
    }

    // Printer MQTT — gated by diagEnableMqtt runtime switch
    if (printerConfig.diagEnableMqtt) {
        tweenToColor(34, 224, 238, 0, 0); // CYAN
        setupMqtt();
        printerVariables.disconnectMQTTms = millis();
    } else {
        Serial.println(F("[DIAG] Printer MQTT DISABLED"));
    }

    // Home Assistant MQTT — gated by diagEnableHa runtime switch
    if (printerConfig.diagEnableHa) {
        setupHa();
        startHaMqttTask();
    } else {
        Serial.println(F("[DIAG] HA MQTT DISABLED"));
    }
    Serial.println();
    Serial.print(F("** BLLED Controller started "));
    Serial.print(F("using firmware version: "));
    Serial.print(globalVariables.FWVersion);
    Serial.println(F(" **"));
    Serial.println();
    globalVariables.started = true;

    // Tell the user how to reach the web UI.
    LogSerial.println();
    LogSerial.print(F("** Web UI: http://"));
    LogSerial.print(globalVariables.Host);
    LogSerial.print(F(".local   (or http://"));
    LogSerial.print(WiFi.localIP().toString());
    LogSerial.println(F(") **"));
    LogSerial.println();

    Serial.println(F("Updating LEDs from Setup"));
    updateleds();
}

void loop()
{
    serialLoop();
    if (globalVariables.started)
    {
        websocketLoop();
        ledsloop();
        // haLoop() now runs in its own FreeRTOS task (see startHaMqttTask) for responsiveness

        if (WiFi.status() != WL_CONNECTED)
        {
            LogSerial.print(F("Wifi connection dropped.  "));
            LogSerial.print(F("Wifi Status: "));
            LogSerial.println(wl_status_to_string(WiFi.status()));
            LogSerial.println(F("Attempting to reconnect to WiFi..."));
            wifi_reconnect_count += 1;
            if (wifi_reconnect_count <= 2)
            {
                WiFi.disconnect();
                delay(100);
                WiFi.reconnect();
            }
            else
            {
                // Not connecting after 10 simple disconnect / reconnects
                // Do something more drastic in case needing to switch to new AP
                scanNetwork();
                connectToWifi();
                wifi_reconnect_count = 0;
            }
        }
        if (WiFi.getMode() == WIFI_AP)
        {
            dnsServer.processNextRequest();
        }
        if(WiFi.status() == WL_CONNECTED && WiFi.getMode() != WIFI_AP
           && printerConfig.diagEnableBblScan)
        {
            bblSearchPrinters();
        }
    }
    if (printerConfig.rescanWiFiNetwork)
    {
        LogSerial.println(F("Web submitted refresh of Wifi Scan (assigning Strongest AP)"));
        tweenToColor(printerConfig.wifiRGB); // Customisable - Default is ORANGE
        scanNetwork();                       // Sets the MAC address for following connection attempt
        printerConfig.rescanWiFiNetwork = false;
        updateleds();
    }
    if (shouldRestart && millis() - restartRequestTime > 1500)
    {
        LogSerial.println(F("[WiFiSetup] Restarting now..."));
        ESP.restart();
    }
}