#include "SerialModule.h"
#include "GeoCoord.h"
#include "MeshService.h"
#include "NMEAWPL.h"
#include "NodeDB.h"
#include "RTC.h"
#include "Router.h"
#include "configuration.h"
#include <Arduino.h>
#include <Throttle.h>

// Forward declarations for binary weather station functionss
struct WeatherData;
bool processBinaryWeatherDataFromBuffer(const uint8_t *buffer, size_t bufferSize, char *outBuffer, size_t outSize);
WeatherData parseBinaryFrame(const uint8_t *frame, size_t frameSize);
void addWeatherReading(const WeatherData &data);
WeatherData calculateWeatherAverage();
bool formatAveragedWeatherData(const WeatherData &avg, const WeatherData &minVals, const WeatherData &maxVals, char *outBuffer, size_t outSize);

/*
    SerialModule
        A simple interface to send messages over the mesh network by sending strings
        over a serial port.

        There are no PIN defaults, you have to enable the second serial port yourself.

    Need help with this module? Post your question on the Meshtastic Discourse:
       https://meshtastic.discourse.group

    Basic Usage:

        1) Enable the module by setting enabled to 1.
        2) Set the pins (rxd / rxd) for your preferred RX and TX GPIO pins.
           On tbeam, recommend to use:
                RXD 35
                TXD 15
        3) Set timeout to the amount of time to wait before we consider
           your packet as "done".
        4) not applicable any more
        5) Connect to your device over the serial interface at 38400 8N1.
        6) Send a packet up to 240 bytes in length. This will get relayed over the mesh network.
        7) (Optional) Set echo to 1 and any message you send out will be echoed back
           to your device.

    TODO (in this order):
        * Define a verbose RX mode to report on mesh and packet information.
            - This won't happen any time soon.

    KNOWN PROBLEMS
        * Until the module is initialized by the startup sequence, the TX pin is in a floating
          state. Device connected to that pin may see this as "noise".
        * Will not work on Linux device targets.


*/
#ifdef HELTEC_MESH_SOLAR
#include "meshSolarApp.h"
#endif

#if (defined(ARCH_ESP32) || defined(ARCH_NRF52) || defined(ARCH_RP2040) || defined(ARCH_STM32WL)) &&                             \
    !defined(CONFIG_IDF_TARGET_ESP32S2) && !defined(CONFIG_IDF_TARGET_ESP32C3)

#define RX_BUFFER 256
#define TIMEOUT 250
#define BAUD 38400
#define ACK 1

// API: Defaulting to the formerly removed phone_timeout_secs value of 15 minutes
#define SERIAL_CONNECTION_TIMEOUT (15 * 60) * 1000UL

SerialModule *serialModule;
SerialModuleRadio *serialModuleRadio;

#ifndef SERIAL_PRINT_PORT
#define SERIAL_PRINT_PORT 2
#endif

#if SERIAL_PRINT_PORT == 0
#define SERIAL_PRINT_OBJECT Serial
#elif SERIAL_PRINT_PORT == 1
#define SERIAL_PRINT_OBJECT Serial1
#elif SERIAL_PRINT_PORT == 2
#define SERIAL_PRINT_OBJECT Serial2
#else
#error "Unsupported SERIAL_PRINT_PORT value. Allowed values are 0, 1, or 2."
#endif

SerialModule::SerialModule() : StreamAPI(&SERIAL_PRINT_OBJECT), concurrency::OSThread("Serial")
{
    api_type = TYPE_SERIAL;
}
static Print *serialPrint = &SERIAL_PRINT_OBJECT;

char serialBytes[512];
size_t serialPayloadSize;

bool SerialModule::isValidConfig(const meshtastic_ModuleConfig_SerialConfig &config)
{
    if (config.override_console_serial_port && !IS_ONE_OF(config.mode, meshtastic_ModuleConfig_SerialConfig_Serial_Mode_NMEA,
                                                          meshtastic_ModuleConfig_SerialConfig_Serial_Mode_CALTOPO,
                                                          meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MS_CONFIG)) {
        const char *warning =
            "Invalid Serial config: override console serial port is only supported in NMEA and CalTopo output-only modes.";
        LOG_ERROR(warning);
#if !IS_RUNNING_TESTS
        meshtastic_ClientNotification *cn = clientNotificationPool.allocZeroed();
        cn->level = meshtastic_LogRecord_Level_ERROR;
        cn->time = getValidTime(RTCQualityFromNet);
        snprintf(cn->message, sizeof(cn->message), "%s", warning);
        service->sendClientNotification(cn);
#endif
        return false;
    }

    return true;
}

SerialModuleRadio::SerialModuleRadio() : MeshModule("SerialModuleRadio")
{
    switch (moduleConfig.serial.mode) {
    case meshtastic_ModuleConfig_SerialConfig_Serial_Mode_TEXTMSG:
        ourPortNum = meshtastic_PortNum_TEXT_MESSAGE_APP;
        break;
    case meshtastic_ModuleConfig_SerialConfig_Serial_Mode_NMEA:
    case meshtastic_ModuleConfig_SerialConfig_Serial_Mode_CALTOPO:
        ourPortNum = meshtastic_PortNum_POSITION_APP;
        break;
    default:
        ourPortNum = meshtastic_PortNum_SERIAL_APP;
        // restrict to the serial channel for rx
        boundChannel = Channels::serialChannel;
        break;
    }
}

/**
 * @brief Checks if the serial connection is established.
 *
 * @return true if the serial connection is established, false otherwise.
 *
 * For the serial2 port we can't really detect if any client is on the other side, so instead just look for recent messages
 */
bool SerialModule::checkIsConnected()
{
    return Throttle::isWithinTimespanMs(lastContactMsec, SERIAL_CONNECTION_TIMEOUT);
}

int32_t SerialModule::runOnce()
{
    /*
        Uncomment the preferences below if you want to use the module
        without having to configure it from the PythonAPI or WebUI.
    */

    // moduleConfig.serial.enabled = true;
    // moduleConfig.serial.rxd = 35;
    // moduleConfig.serial.txd = 15;
    // moduleConfig.serial.override_console_serial_port = true;
    // moduleConfig.serial.mode = meshtastic_ModuleConfig_SerialConfig_Serial_Mode_CALTOPO;
    // moduleConfig.serial.timeout = 1000;
    // moduleConfig.serial.echo = 1;

    if (!moduleConfig.serial.enabled)
        return disable();

    if (moduleConfig.serial.override_console_serial_port || (moduleConfig.serial.rxd && moduleConfig.serial.txd)) {
        if (firstTime) {
            // Interface with the serial peripheral from in here.
            LOG_INFO("Init serial peripheral interface");

            uint32_t baud = getBaudRate();

            if (moduleConfig.serial.override_console_serial_port) {
#ifdef RP2040_SLOW_CLOCK
                Serial2.flush();
                serialPrint = &Serial2;
#else
                Serial.flush();
                serialPrint = &Serial;
#endif
                // Give it a chance to flush out 💩
                delay(10);
            }
#if defined(CONFIG_IDF_TARGET_ESP32C6)
            if (moduleConfig.serial.rxd && moduleConfig.serial.txd) {
                Serial1.setRxBufferSize(RX_BUFFER);
                Serial1.begin(baud, SERIAL_8N1, moduleConfig.serial.rxd, moduleConfig.serial.txd);
            } else {
                Serial.begin(baud);
                Serial.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
            }
#elif defined(ARCH_STM32WL)
#ifndef RAK3172
            HardwareSerial *serialInstance = &Serial2;
#else
            HardwareSerial *serialInstance = &Serial1;
#endif
            if (moduleConfig.serial.rxd && moduleConfig.serial.txd) {
                serialInstance->setTx(moduleConfig.serial.txd);
                serialInstance->setRx(moduleConfig.serial.rxd);
            }
            serialInstance->begin(baud);
            serialInstance->setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
#elif defined(ARCH_ESP32)

            if (moduleConfig.serial.rxd && moduleConfig.serial.txd) {
                Serial2.setRxBufferSize(RX_BUFFER);
                Serial2.begin(baud, SERIAL_8N1, moduleConfig.serial.rxd, moduleConfig.serial.txd);
            } else {
                Serial.begin(baud);
                Serial.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
            }
#elif SERIAL_PRINT_PORT != 0

            if (moduleConfig.serial.rxd && moduleConfig.serial.txd) {
#ifdef ARCH_RP2040
                Serial2.setFIFOSize(RX_BUFFER);
                Serial2.setPinout(moduleConfig.serial.txd, moduleConfig.serial.rxd);
#else
                Serial2.setPins(moduleConfig.serial.rxd, moduleConfig.serial.txd);
#endif
                Serial2.begin(baud, SERIAL_8N1);
                Serial2.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
            } else {
#ifdef RP2040_SLOW_CLOCK
                Serial2.begin(baud, SERIAL_8N1);
                Serial2.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
#else
                Serial.begin(baud, SERIAL_8N1);
                Serial.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
#endif
            }
#else
            Serial.begin(baud, SERIAL_8N1);
            Serial.setTimeout(moduleConfig.serial.timeout > 0 ? moduleConfig.serial.timeout : TIMEOUT);
#endif
            serialModuleRadio = new SerialModuleRadio();

            firstTime = 0;

            // in API mode send rebooted sequence
            if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_PROTO) {
                emitRebooted();
            }
        } else {
            if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_PROTO) {
                return runOncePart();
            } else if ((moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_NMEA) && HAS_GPS) {
                // in NMEA mode send out GGA every 2 seconds, Don't read from Port
                if (!Throttle::isWithinTimespanMs(lastNmeaTime, 2000)) {
                    lastNmeaTime = millis();
                    printGGA(outbuf, sizeof(outbuf), localPosition);
                    serialPrint->printf("%s", outbuf);
                }
            } else if ((moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_CALTOPO) && HAS_GPS) {
                if (!Throttle::isWithinTimespanMs(lastNmeaTime, 10000)) {
                    lastNmeaTime = millis();
                    uint32_t readIndex = 0;
                    const meshtastic_NodeInfoLite *tempNodeInfo = nodeDB->readNextMeshNode(readIndex);
                    while (tempNodeInfo != NULL) {
                        if (tempNodeInfo->has_user && nodeDB->hasValidPosition(tempNodeInfo)) {
                            printWPL(outbuf, sizeof(outbuf), tempNodeInfo->position, tempNodeInfo->user.long_name, true);
                            serialPrint->printf("%s", outbuf);
                        }
                        tempNodeInfo = nodeDB->readNextMeshNode(readIndex);
                    }
                }
            }

#if SERIAL_PRINT_PORT != 0
            else if ((moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_WS85)) {
                processWXSerial();

            }
#if defined(HELTEC_MESH_SOLAR)
            else if ((moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_MS_CONFIG)) {
                serialPayloadSize = Serial.readBytes(serialBytes, sizeof(serialBytes) - 1);
                // If the parsing fails, the following parsing will be performed.
                if ((serialPayloadSize > 0) && (meshSolarCmdHandle(serialBytes) != 0)) {
                    return runOncePart(serialBytes, serialPayloadSize);
                }
            }
#endif
            else {
#if defined(CONFIG_IDF_TARGET_ESP32C6)
                while (Serial1.available()) {
                    serialPayloadSize = Serial1.readBytes(serialBytes, meshtastic_Constants_DATA_PAYLOAD_LEN);
#else
#ifndef RAK3172
                HardwareSerial *serialInstance = &Serial2;
#else
                HardwareSerial *serialInstance = &Serial1;
#endif
                while (serialInstance->available()) {
                    serialPayloadSize = serialInstance->readBytes(serialBytes, meshtastic_Constants_DATA_PAYLOAD_LEN);
#endif
                    
                    // In SIMPLE mode, also check for binary weather station data
                    if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_SIMPLE) {
                        // Try to process binary weather station data from the received bytes
                        char weatherOutput[300];
                        bool hasWeatherData = processBinaryWeatherDataFromBuffer((const uint8_t*)serialBytes, serialPayloadSize, weatherOutput, sizeof(weatherOutput));
                        if (hasWeatherData) {
                            // Send parsed binary data as separate payload (averaged data ready)
                            size_t dataLen = strlen(weatherOutput);
                            if (dataLen < meshtastic_Constants_DATA_PAYLOAD_LEN) {
                                memcpy(serialBytes, weatherOutput, dataLen);
                serialBytes[dataLen] = '\0';
                                serialPayloadSize = dataLen;
                                serialModuleRadio->sendPayload();
                                continue; // Skip normal processing for this iteration
                            }
                        } else {
                            // Binary weather data processed but not enough readings yet
                            // Don't send anything, just skip this iteration
                            continue;
                        }
                    }
                    
                    // Original SIMPLE mode behavior: send raw data
                    serialModuleRadio->sendPayload();
                }
            }
#endif
        }
        return (10);
    } else {
        return disable();
    }
}

/**
 * Sends telemetry packet over the mesh network.
 *
 * @param m The telemetry data to be sent
 *
 * @return void
 *
 * @throws None
 */
void SerialModule::sendTelemetry(meshtastic_Telemetry m)
{
    meshtastic_MeshPacket *p = router->allocForSending();
    p->decoded.portnum = meshtastic_PortNum_TELEMETRY_APP;
    p->decoded.payload.size =
        pb_encode_to_bytes(p->decoded.payload.bytes, sizeof(p->decoded.payload.bytes), &meshtastic_Telemetry_msg, &m);
    p->to = NODENUM_BROADCAST;
    p->decoded.want_response = false;
    if (config.device.role == meshtastic_Config_DeviceConfig_Role_SENSOR) {
        p->want_ack = true;
        p->priority = meshtastic_MeshPacket_Priority_HIGH;
    } else {
        p->priority = meshtastic_MeshPacket_Priority_RELIABLE;
    }
    service->sendToMesh(p, RX_SRC_LOCAL, true);
}

/**
 * Allocates a new mesh packet for use as a reply to a received packet.
 *
 * @return A pointer to the newly allocated mesh packet.
 */
meshtastic_MeshPacket *SerialModuleRadio::allocReply()
{
    auto reply = allocDataPacket(); // Allocate a packet for sending

    return reply;
}

/**
 * Sends a payload to a specified destination node.
 *
 * @param dest The destination node number.
 * @param wantReplies Whether or not to request replies from the destination node.
 */
void SerialModuleRadio::sendPayload(NodeNum dest, bool wantReplies)
{
    const meshtastic_Channel *ch = (boundChannel != NULL) ? &channels.getByName(boundChannel) : NULL;
    meshtastic_MeshPacket *p = allocReply();
    p->to = dest;
    if (ch != NULL) {
        p->channel = ch->index;
    }
    p->decoded.want_response = wantReplies;

    p->want_ack = ACK;

    p->decoded.payload.size = serialPayloadSize; // You must specify how many bytes are in the reply
    memcpy(p->decoded.payload.bytes, serialBytes, p->decoded.payload.size);

    service->sendToMesh(p);
}

/**
 * Handle a received mesh packet.
 *
 * @param mp The received mesh packet.
 * @return The processed message.
 */
ProcessMessage SerialModuleRadio::handleReceived(const meshtastic_MeshPacket &mp)
{
    if (moduleConfig.serial.enabled) {
        if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_PROTO) {
            // in API mode we don't care about stuff from radio.
            return ProcessMessage::CONTINUE;
        }

        auto &p = mp.decoded;
        // LOG_DEBUG("Received text msg self=0x%0x, from=0x%0x, to=0x%0x, id=%d, msg=%.*s",
        //          nodeDB->getNodeNum(), mp.from, mp.to, mp.id, p.payload.size, p.payload.bytes);

        if (isFromUs(&mp)) {

            /*
             * If moduleConfig.serial.echo is true, then echo the packets that are sent out
             * back to the TX of the serial interface.
             */
            if (moduleConfig.serial.echo) {

                // For some reason, we get the packet back twice when we send out of the radio.
                //   TODO: need to find out why.
                if (lastRxID != mp.id) {
                    lastRxID = mp.id;
                    // LOG_DEBUG("* * Message came this device");
                    // serialPrint->println("* * Message came this device");
                    serialPrint->printf("%s", p.payload.bytes);
                }
            }
        } else {

            if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_DEFAULT ||
                moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_SIMPLE) {
                serialPrint->write(p.payload.bytes, p.payload.size);
            } else if (moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_TEXTMSG) {
                meshtastic_NodeInfoLite *node = nodeDB->getMeshNode(getFrom(&mp));
                const char *sender = (node && node->has_user) ? node->user.short_name : "???";
                serialPrint->println();
                serialPrint->printf("%s: %s", sender, p.payload.bytes);
                serialPrint->println();
            } else if ((moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_NMEA ||
                        moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_CALTOPO) &&
                       HAS_GPS) {
                // Decode the Payload some more
                meshtastic_Position scratch;
                meshtastic_Position *decoded = NULL;
                if (mp.which_payload_variant == meshtastic_MeshPacket_decoded_tag && mp.decoded.portnum == ourPortNum) {
                    memset(&scratch, 0, sizeof(scratch));
                    if (pb_decode_from_bytes(p.payload.bytes, p.payload.size, &meshtastic_Position_msg, &scratch)) {
                        decoded = &scratch;
                    }
                    // send position packet as WPL to the serial port
                    printWPL(outbuf, sizeof(outbuf), *decoded, nodeDB->getMeshNode(getFrom(&mp))->user.long_name,
                             moduleConfig.serial.mode == meshtastic_ModuleConfig_SerialConfig_Serial_Mode_CALTOPO);
                    serialPrint->printf("%s", outbuf);
                }
            }
        }
    }
    return ProcessMessage::CONTINUE; // Let others look at this message also if they want
}

/**
 * @brief Returns the baud rate of the serial module from the module configuration.
 *
 * @return uint32_t The baud rate of the serial module.
 */
uint32_t SerialModule::getBaudRate()
{
    if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_110) {
        return 110;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_300) {
        return 300;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_600) {
        return 600;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_1200) {
        return 1200;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_2400) {
        return 2400;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_4800) {
        return 4800;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_9600) {
        return 9600;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_19200) {
        return 19200;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_38400) {
        return 38400;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_57600) {
        return 57600;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_115200) {
        return 115200;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_230400) {
        return 230400;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_460800) {
        return 460800;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_576000) {
        return 576000;
    } else if (moduleConfig.serial.baud == meshtastic_ModuleConfig_SerialConfig_Serial_Baud_BAUD_921600) {
        return 921600;
    }
    return BAUD;
}

// Add this structure to help with parsing WindGust =       24.4 serial lines.
struct ParsedLine {
    char name[64];
    char value[128];
};

// Structure for binary weather station data
struct WeatherData {
    float wind_dir_deg;
    float wind_speed_m_s;
    float gust_speed_m_s;
    float temperature_C;
    float humidity_percent;
    float rain_mm;
    uint16_t uv_index;
    float light_lux;
    float pressure_hPa;
    bool low_battery;
    bool valid;
};

// Weather data averaging system (40 readings)
#define WEATHER_READINGS_COUNT 40
static WeatherData weatherReadings[WEATHER_READINGS_COUNT];
static size_t weatherReadingsIndex = 0;
static bool weatherReadingsFull = false;

// Counter to control transmission frequency
static size_t sendCounter = 0;

// Min/Max tracking
static WeatherData minValues, maxValues;

/**
 * CRC8 calculation for weather station data validation
 */
uint8_t crc8(const uint8_t *data, uint8_t len, uint8_t polynomial, uint8_t init)
{
    uint8_t crc = init;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ polynomial) : (crc << 1);
        }
    }
    return crc;
}

/**
 * Parse binary weather station frame using byte-level operations
 * Protocol: WS69/WH65LP (17, 21 or 25 bytes)
 * 
 * Byte layout (0-indexed):
 * [0]  FC (Family Code, always 0x24)
 * [1]  ID_LSB (8-bit ID)
 * [2]  DIR[7:0] (wind direction bits 0-7)
 * [3]  DIR[8], WSP_FLAG, -, -, lowBat, TMP[10:8]
 * [4]  TMP[7:0]
 * [5]  HUM[6:0], TMP[0]
 * [6]  WSP[7:0] (wind speed)
 * [7]  GUST[7:0]
 * [8]  RAIN[15:8]
 * [9]  RAIN[7:0]
 * [10] UVI[15:8]
 * [11] UVI[7:0]
 * [12] LIGHT[23:16]
 * [13] LIGHT[15:8]
 * [14] LIGHT[7:0]
 * [15] CRC8 (of bytes 0-14, poly=0x31, init=0x00)
 * [16] CHECKSUM (sum of bytes 0-15)
 * [17] PRE[16] or WSP[10] in 25-byte mode, then unused[6:0]
 * [18] PRE[15:8]
 * [19] PRE[7:0], CHECKSUM_P (bits 7:0 of pressure checksum)
 * [20] PRESSURE_CHECKSUM (checksum of bytes 17-19)
 * [21] ID_MSB (bits 23:16)
 * [22] ID_HSB (bits 15:8)
 * [23] CRC8_2 (of bytes 0-22)
 * [24] CHECKSUM2 (sum of bytes 0-23)
 */
WeatherData parseBinaryFrame(const uint8_t *frame, size_t frameSize)
{
    WeatherData data = {0};
    data.valid = false;
    
    // Minimum frame size is 17 bytes
    if (frameSize < 17) {
        return data;
    }
    
    // Validate frame start byte (Family Code = 0x24)
    if (frame[0] != 0x24) {
        return data;
    }
    
    // Calculate CRC8 for bytes 0-14 (15 bytes)
    uint8_t calculated_crc = crc8(frame, 15, 0x31, 0x00);
    if (calculated_crc != frame[15]) {
        return data;
    }
    
    // Calculate checksum for bytes 0-15
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 16; i++) {
        checksum += frame[i];
    }
    if (checksum != frame[16]) {
        return data;
    }
    
    // --- WIND DIRECTION (Byte 2 + Byte 3 bit 7) ---
    // Byte 2: DIR[7:0]
    // Byte 3 bit 7: DIR[8]
    uint16_t dir_raw = ((uint16_t)frame[2]) | (((uint16_t)(frame[3] & 0x80)));
    data.wind_dir_deg = (dir_raw == 0x1FF) ? -1.0f : (float)(dir_raw % 360);
    
    // --- LOW BATTERY (Byte 3 bit 3) ---
    data.low_battery = ((frame[3] >> 3) & 0x01) != 0;
    
    // --- WIND SPEED (Byte 7) ---
    // Formula: WSP * 0.51 / 8 = m/s (average of 8 readings over 16 seconds)
    data.wind_speed_m_s = (frame[6] * 0.51f) / 8.0f;
    
    // --- TEMPERATURE (Byte 3 bits 2-0 + Byte 4 + Byte 5 bit 7) ---
    // TMP[10:8] from byte 3 bits 2-0
    // TMP[7:1] from byte 4
    // TMP[0] from byte 5 bit 7
    uint16_t tmp_raw = (((uint16_t)(frame[3] & 0x07)) << 8) | frame[4] | (((frame[5] >> 7) & 0x01));
    data.temperature_C = (tmp_raw == 0x7FF) ? -999.0f : (float)(tmp_raw - 400) / 10.0f;
    
    // --- HUMIDITY (Byte 5 bits 0-6) ---
    uint8_t hum_raw = frame[5] & 0x7F;
    data.humidity_percent = (hum_raw == 0xFF) ? -1.0f : (float)hum_raw;
    
    // --- GUST SPEED (Byte 8) ---
    // Formula: GUST * 0.51 = m/s
    data.gust_speed_m_s = frame[7] * 0.51f;
    
    // --- RAIN (Bytes 8-9, 14 bits) ---
    uint16_t rain_raw = (((uint16_t)frame[8]) << 8) | frame[9];
    data.rain_mm = (float)rain_raw * 0.254f;
    
    // --- UV INDEX (Bytes 10-11, 16 bits) ---
    uint16_t uv_raw = (((uint16_t)frame[10]) << 8) | frame[11];
    data.uv_index = uv_raw;
    
    // --- LIGHT (Bytes 12-14, 24 bits) ---
    uint32_t light_raw = (((uint32_t)frame[12]) << 16) | (((uint32_t)frame[13]) << 8) | ((uint32_t)frame[14]);
    if (light_raw >= 0xFFFFFE) {
        data.light_lux = 300000.0f;
    } else {
        data.light_lux = (float)light_raw / 10.0f;
    }
    
    // --- PRESSURE (Bytes 17-19, only if frameSize >= 21) ---
    // Format: 0x1XXXX (bits 16-0), pressure = value / 100 hPa
    uint32_t pre_raw = 0x1FFFF; // Invalid by default
    if (frameSize >= 21) {
        // Validate pressure checksum (byte 20)
        uint8_t pre_checksum = frame[17] + frame[18] + frame[19];
        if ((pre_checksum & 0xFF) == frame[20]) {
            pre_raw = (((uint32_t)(frame[17] & 0x01)) << 16) | 
                      (((uint32_t)frame[18]) << 8) | 
                      ((uint32_t)frame[19]);
        }
    }
    data.pressure_hPa = (pre_raw == 0x1FFFF) ? 0.0f : (float)pre_raw / 100.0f;
    
    data.valid = true;
    return data;
}

/**
 * Parse a line of format "Name = Value" into name/value pair
 * @param line Input line to parse
 * @return ParsedLine containing name and value, or empty strings if parse failed
 */
ParsedLine parseLine(const char *line)
{
    ParsedLine result = {"", ""};

    // Find equals sign
    const char *equals = strchr(line, '=');
    if (!equals) {
        return result;
    }

    // Extract name by copying substring
    char nameBuf[64]; // Temporary buffer
    size_t nameLen = equals - line;
    if (nameLen >= sizeof(nameBuf)) {
        nameLen = sizeof(nameBuf) - 1;
    }
    strncpy(nameBuf, line, nameLen);
    nameBuf[nameLen] = '\0';

    // Trim whitespace from name
    char *nameStart = nameBuf;
    while (*nameStart && isspace(*nameStart))
        nameStart++;
    char *nameEnd = nameStart + strlen(nameStart) - 1;
    while (nameEnd > nameStart && isspace(*nameEnd))
        *nameEnd-- = '\0';

    // Copy trimmed name
    strncpy(result.name, nameStart, sizeof(result.name) - 1);
    result.name[sizeof(result.name) - 1] = '\0';

    // Extract value part (after equals)
    const char *valueStart = equals + 1;
    while (*valueStart && isspace(*valueStart))
        valueStart++;
    strncpy(result.value, valueStart, sizeof(result.value) - 1);
    result.value[sizeof(result.value) - 1] = '\0';

    // Trim trailing whitespace from value
    char *valueEnd = result.value + strlen(result.value) - 1;
    while (valueEnd > result.value && isspace(*valueEnd))
        *valueEnd-- = '\0';

    return result;
}


/**
 * Calculate min/max from circular buffer when it's full
 */
static void calculateMinMaxFromBuffer()
{
    // Buscar primer valor válido
    size_t firstValid = 0;
    while (firstValid < WEATHER_READINGS_COUNT && !weatherReadings[firstValid].valid) firstValid++;
    if (firstValid >= WEATHER_READINGS_COUNT) return; // No hay datos válidos
    
    minValues = weatherReadings[firstValid];
    maxValues = weatherReadings[firstValid];
    
    for (size_t i = 1; i < WEATHER_READINGS_COUNT; i++) {
        const WeatherData &r = weatherReadings[i];
        if (!r.valid) continue;
        
        if (r.wind_speed_m_s >= 0 && r.wind_speed_m_s < minValues.wind_speed_m_s)
            minValues.wind_speed_m_s = r.wind_speed_m_s;
        if (r.wind_speed_m_s >= 0 && r.wind_speed_m_s > maxValues.wind_speed_m_s)
            maxValues.wind_speed_m_s = r.wind_speed_m_s;
            
        if (r.gust_speed_m_s >= 0 && r.gust_speed_m_s < minValues.gust_speed_m_s)
            minValues.gust_speed_m_s = r.gust_speed_m_s;
        if (r.gust_speed_m_s >= 0 && r.gust_speed_m_s > maxValues.gust_speed_m_s)
            maxValues.gust_speed_m_s = r.gust_speed_m_s;
            
        if (r.temperature_C > -500 && r.temperature_C < minValues.temperature_C)
            minValues.temperature_C = r.temperature_C;
        if (r.temperature_C > -500 && r.temperature_C > maxValues.temperature_C)
            maxValues.temperature_C = r.temperature_C;
            
        if (r.humidity_percent >= 0 && r.humidity_percent < minValues.humidity_percent)
            minValues.humidity_percent = r.humidity_percent;
        if (r.humidity_percent >= 0 && r.humidity_percent > maxValues.humidity_percent)
            maxValues.humidity_percent = r.humidity_percent;
            
        if (r.rain_mm >= 0 && r.rain_mm < minValues.rain_mm)
            minValues.rain_mm = r.rain_mm;
        if (r.rain_mm >= 0 && r.rain_mm > maxValues.rain_mm)
            maxValues.rain_mm = r.rain_mm;
            
        if (r.uv_index > 0 && r.uv_index < minValues.uv_index)
            minValues.uv_index = r.uv_index;
        if (r.uv_index > 0 && r.uv_index > maxValues.uv_index)
            maxValues.uv_index = r.uv_index;
            
        if (r.light_lux > 0 && r.light_lux < minValues.light_lux)
            minValues.light_lux = r.light_lux;
        if (r.light_lux > 0 && r.light_lux > maxValues.light_lux)
            maxValues.light_lux = r.light_lux;
            
        if (r.pressure_hPa > 0 && r.pressure_hPa < minValues.pressure_hPa)
            minValues.pressure_hPa = r.pressure_hPa;
        if (r.pressure_hPa > 0 && r.pressure_hPa > maxValues.pressure_hPa)
            maxValues.pressure_hPa = r.pressure_hPa;
    }
}

/**
 * Add weather reading to circular buffer and update min/max values
 * @param data WeatherData reading to add
 */
void addWeatherReading(const WeatherData &data)
{
    if (!data.valid) return;
    
    // Add to circular buffer
    weatherReadings[weatherReadingsIndex] = data;
    weatherReadingsIndex = (weatherReadingsIndex + 1) % WEATHER_READINGS_COUNT;
    
    // Mark as full after first complete cycle
    if (weatherReadingsIndex == 0) {
        weatherReadingsFull = true;
        // Calculate min/max from all readings in buffer
        calculateMinMaxFromBuffer();
        return;
    }
    
    // Initialize min/max on first reading
    if (weatherReadingsIndex == 1 && !weatherReadingsFull) {
        minValues = data;
        maxValues = data;
        return;
    }
    
    // Update min/max incrementally with new reading
    if (data.wind_speed_m_s >= 0 && data.wind_speed_m_s < minValues.wind_speed_m_s)
        minValues.wind_speed_m_s = data.wind_speed_m_s;
    if (data.wind_speed_m_s >= 0 && data.wind_speed_m_s > maxValues.wind_speed_m_s)
        maxValues.wind_speed_m_s = data.wind_speed_m_s;
        
    if (data.gust_speed_m_s >= 0 && data.gust_speed_m_s < minValues.gust_speed_m_s)
        minValues.gust_speed_m_s = data.gust_speed_m_s;
    if (data.gust_speed_m_s >= 0 && data.gust_speed_m_s > maxValues.gust_speed_m_s)
        maxValues.gust_speed_m_s = data.gust_speed_m_s;
        
    if (data.temperature_C > -500 && data.temperature_C < minValues.temperature_C)
        minValues.temperature_C = data.temperature_C;
    if (data.temperature_C > -500 && data.temperature_C > maxValues.temperature_C)
        maxValues.temperature_C = data.temperature_C;
        
    if (data.humidity_percent >= 0 && data.humidity_percent < minValues.humidity_percent)
        minValues.humidity_percent = data.humidity_percent;
    if (data.humidity_percent >= 0 && data.humidity_percent > maxValues.humidity_percent)
        maxValues.humidity_percent = data.humidity_percent;
        
    if (data.rain_mm >= 0 && data.rain_mm < minValues.rain_mm)
        minValues.rain_mm = data.rain_mm;
    if (data.rain_mm >= 0 && data.rain_mm > maxValues.rain_mm)
        maxValues.rain_mm = data.rain_mm;
        
    if (data.uv_index > 0 && data.uv_index < minValues.uv_index)
        minValues.uv_index = data.uv_index;
    if (data.uv_index > 0 && data.uv_index > maxValues.uv_index)
        maxValues.uv_index = data.uv_index;
        
    if (data.light_lux > 0 && data.light_lux < minValues.light_lux)
        minValues.light_lux = data.light_lux;
    if (data.light_lux > 0 && data.light_lux > maxValues.light_lux)
        maxValues.light_lux = data.light_lux;
        
    if (data.pressure_hPa > 0 && data.pressure_hPa < minValues.pressure_hPa)
        minValues.pressure_hPa = data.pressure_hPa;
    if (data.pressure_hPa > 0 && data.pressure_hPa > maxValues.pressure_hPa)
        maxValues.pressure_hPa = data.pressure_hPa;
}

/**
 * Calculate average of weather readings
 * @return WeatherData with averaged values
 */
WeatherData calculateWeatherAverage()
{
    WeatherData avg = {0};
    avg.valid = false;
    
    size_t count = weatherReadingsFull ? WEATHER_READINGS_COUNT : weatherReadingsIndex;
    if (count == 0) return avg;
    
    float wind_dir_sum_x = 0, wind_dir_sum_y = 0;
    float wind_speed_sum = 0, gust_sum = 0, temp_sum = 0, humidity_sum = 0;
    float rain_max = 0, uv_sum = 0, light_sum = 0, pressure_sum = 0;
    int valid_wind_dir = 0, valid_wind_speed = 0, valid_gust = 0, valid_temp = 0;
    int valid_humidity = 0, valid_uv = 0, valid_light = 0, valid_pressure = 0;
    bool lowBat = false;  // Track if any reading has low battery
    
    for (size_t i = 0; i < count; i++) {
        const WeatherData &reading = weatherReadings[i];
        if (!reading.valid) continue;
        
        // Wind direction - circular average
        if (reading.wind_dir_deg >= 0) {
            float rad = reading.wind_dir_deg * M_PI / 180.0;
            wind_dir_sum_x += cos(rad);
            wind_dir_sum_y += sin(rad);
            valid_wind_dir++;
        }
        
        // Wind speed
        if (reading.wind_speed_m_s >= 0) {
            wind_speed_sum += reading.wind_speed_m_s;
            valid_wind_speed++;
        }
        
        // Gust
        if (reading.gust_speed_m_s >= 0) {
            gust_sum += reading.gust_speed_m_s;
            valid_gust++;
        }
        
        // Temperature
        if (reading.temperature_C > -500) {
            temp_sum += reading.temperature_C;
            valid_temp++;
        }
        
        // Humidity
        if (reading.humidity_percent >= 0) {
            humidity_sum += reading.humidity_percent;
            valid_humidity++;
        }
        
        // Rain - use maximum (accumulated value)
        if (reading.rain_mm >= 0 && reading.rain_mm > rain_max) {
            rain_max = reading.rain_mm;
        }
        
        // UV
        if (reading.uv_index > 0) {
            uv_sum += reading.uv_index;
            valid_uv++;
        }
        
        // Light
        if (reading.light_lux > 0) {
            light_sum += reading.light_lux;
            valid_light++;
        }
        
        // Pressure
        if (reading.pressure_hPa > 0) {
            pressure_sum += reading.pressure_hPa;
            valid_pressure++;
        }
        
        // Low battery - track if any reading has low battery
        if (reading.low_battery) {
            lowBat = true;
        }
    }
    
    // Calculate averages
    if (valid_wind_dir > 0) {
        float avg_rad = atan2(wind_dir_sum_y, wind_dir_sum_x);
        avg.wind_dir_deg = fmod(avg_rad * 180.0 / M_PI + 360.0, 360.0);
    } else {
        avg.wind_dir_deg = -1;
    }
    
    avg.wind_speed_m_s = valid_wind_speed > 0 ? wind_speed_sum / valid_wind_speed : -1;
    avg.gust_speed_m_s = valid_gust > 0 ? gust_sum / valid_gust : -1;
    avg.temperature_C = valid_temp > 0 ? temp_sum / valid_temp : -999;
    avg.humidity_percent = valid_humidity > 0 ? humidity_sum / valid_humidity : -1;
    avg.rain_mm = rain_max; // Use maximum for rain
    avg.uv_index = valid_uv > 0 ? uv_sum / valid_uv : 0;
    avg.light_lux = valid_light > 0 ? light_sum / valid_light : 0;
    avg.pressure_hPa = valid_pressure > 0 ? pressure_sum / valid_pressure : 0;
    avg.low_battery = lowBat;  // true if any reading had low battery
    avg.valid = true;
    
    return avg;
}

/**
 * Format averaged weather data with min/max values
 * @param avg Average values
 * @param minVals Minimum values
 * @param maxVals Maximum values
 * @return Formatted string for transmission
 */
bool formatAveragedWeatherData(const WeatherData &avg, const WeatherData &minVals, const WeatherData &maxVals, char *outBuffer, size_t outSize)
{
    if (!avg.valid || outBuffer == nullptr || outSize == 0) {
        return false;
    }
    
    // Write directly to output buffer - no heap allocations
    int pos = 0;
    
    // Header
    pos += snprintf(outBuffer + pos, outSize - pos, "WH65LP,");
    
    // Wind direction (only average, no min/max - doesn't make sense for circular degrees)
    if (avg.wind_dir_deg >= 0) {
        pos += snprintf(outBuffer + pos, outSize - pos, "WD%d,", (int)avg.wind_dir_deg);
    }
    
    // Wind speed
    if (avg.wind_speed_m_s >= 0) {
        pos += snprintf(outBuffer + pos, outSize - pos, "WS%.1f,", avg.wind_speed_m_s);
        pos += snprintf(outBuffer + pos, outSize - pos, "WSM%.1f,", maxVals.wind_speed_m_s);
        pos += snprintf(outBuffer + pos, outSize - pos, "WSm%.1f,", minVals.wind_speed_m_s);
    }
    
    // Gust
    if (avg.gust_speed_m_s >= 0) {
        pos += snprintf(outBuffer + pos, outSize - pos, "WG%.1f,", avg.gust_speed_m_s);
        pos += snprintf(outBuffer + pos, outSize - pos, "WGM%.1f,", maxVals.gust_speed_m_s);
        pos += snprintf(outBuffer + pos, outSize - pos, "WGm%.1f,", minVals.gust_speed_m_s);
    }
    
    // Temperature
    if (avg.temperature_C > -500) {
        pos += snprintf(outBuffer + pos, outSize - pos, "T%.1f,", avg.temperature_C);
        pos += snprintf(outBuffer + pos, outSize - pos, "TM%.1f,", maxVals.temperature_C);
        pos += snprintf(outBuffer + pos, outSize - pos, "Tm%.1f,", minVals.temperature_C);
    }
    
    // Humidity
    if (avg.humidity_percent >= 0) {
        pos += snprintf(outBuffer + pos, outSize - pos, "H%.0f,", avg.humidity_percent);
        pos += snprintf(outBuffer + pos, outSize - pos, "HM%.0f,", maxVals.humidity_percent);
        pos += snprintf(outBuffer + pos, outSize - pos, "Hm%.0f,", minVals.humidity_percent);
    }
    
    // Rain (accumulated, use max as the value)
    if (maxVals.rain_mm >= 0) {
        pos += snprintf(outBuffer + pos, outSize - pos, "R%.1f,", maxVals.rain_mm);
    }
    
    // UV
    if (avg.uv_index > 0) {
        pos += snprintf(outBuffer + pos, outSize - pos, "UV%u,", avg.uv_index);
        pos += snprintf(outBuffer + pos, outSize - pos, "UVM%u,", maxVals.uv_index);
        pos += snprintf(outBuffer + pos, outSize - pos, "UVm%u,", minVals.uv_index);
    }
    
    // Light
    if (avg.light_lux > 0) {
        pos += snprintf(outBuffer + pos, outSize - pos, "LX%.1f,", avg.light_lux);
        pos += snprintf(outBuffer + pos, outSize - pos, "LXM%.1f,", maxVals.light_lux);
        pos += snprintf(outBuffer + pos, outSize - pos, "LXm%.1f,", minVals.light_lux);
    }
    
    // Pressure
    if (avg.pressure_hPa > 0) {
        pos += snprintf(outBuffer + pos, outSize - pos, "P%.1f,", avg.pressure_hPa);
        pos += snprintf(outBuffer + pos, outSize - pos, "PM%.1f,", maxVals.pressure_hPa);
        pos += snprintf(outBuffer + pos, outSize - pos, "Pm%.1f,", minVals.pressure_hPa);
    }
    
    // Low battery
    if (avg.low_battery) {
        pos += snprintf(outBuffer + pos, outSize - pos, "LB=1,");
    }
    
    // Success - return true if we wrote something
    return pos > 0;
}

/**
 * Process binary weather station frames from a buffer
 * @param buffer Buffer containing received data
 * @param bufferSize Size of the buffer
 * @param outBuffer Output buffer for formatted data
 * @param outSize Size of output buffer
 * @return true if data was written to outBuffer, false otherwise
 */
bool processBinaryWeatherDataFromBuffer(const uint8_t *buffer, size_t bufferSize, char *outBuffer, size_t outSize)
{
    // Protocol versions:
    // - 17 bytes: Original (no pressure)
    // - 21 bytes: With pressure
    // - 25 bytes: Full (with ID, CRC2, Checksum2)
    // Minimum needed: 17 bytes for basic CRC/checksum validation
    if (bufferSize < 17) {
        return false;
    }
    
    // Look for frames in the buffer (supports 17-25 byte variants)
    for (size_t i = 0; i <= bufferSize - 21; i++) {
        if (buffer[i] == 0x24) {
            // Try to parse frame starting at this position
            WeatherData data = parseBinaryFrame(&buffer[i], bufferSize - i);
            if (data.valid) {
                // Add reading to buffer
                addWeatherReading(data);
                sendCounter++;
                
                // Only send if buffer is full (guarantees 40 real samples)
                if (weatherReadingsFull) {
                    // Check if we have enough new readings to send averaged data
                    if (sendCounter >= WEATHER_READINGS_COUNT) {
                        // Calculate averages and format
                        WeatherData avg = calculateWeatherAverage();
                        if (avg.valid) {
                            // Reset counter for next transmission cycle
                            sendCounter = 0;
                            return formatAveragedWeatherData(avg, minValues, maxValues, outBuffer, outSize);
                        }
                        sendCounter = 0;
                    }
                }
                
                // Not enough readings yet, return false
                return false;
            }
        }
    }
    
    return false;
}

/**
 * Process the received weather station serial data, extract wind, voltage, and temperature information,
 * calculate averages and send telemetry data over the mesh network.
 *
 * @return void
 */
void SerialModule::processWXSerial()
{
#if SERIAL_PRINT_PORT != 0 && !defined(ARCH_STM32WL) && !defined(CONFIG_IDF_TARGET_ESP32C6)

    static unsigned int lastAveraged = 0;
    static unsigned int averageIntervalMillis = 300000; // 5 minutes hard coded.
    static double dir_sum_sin = 0;
    static double dir_sum_cos = 0;
    static float velSum = 0;
    static float gust = 0;
    static float lull = -1;
    static int velCount = 0;
    static int dirCount = 0;
    static char windDir[4] = "xxx";   // Assuming windDir is 3 characters long + null terminator
    static char windVel[5] = "xx.x";  // Assuming windVel is 4 characters long + null terminator
    static char windGust[5] = "xx.x"; // Assuming windGust is 4 characters long + null terminator
    static char batVoltage[5] = "0.0V";
    static char capVoltage[5] = "0.0V";
    static char temperature[5] = "00.0";
    static float batVoltageF = 0;
    static float capVoltageF = 0;
    static float temperatureF = 0;

    static char rainStr[] = "5780860000";
    static int rainSum = 0;
    static float rain = 0;
    bool gotwind = false;

    while (Serial2.available()) {
        // clear serialBytes buffer
        memset(serialBytes, '\0', sizeof(serialBytes));
        // memset(formattedString, '\0', sizeof(formattedString));
        serialPayloadSize = Serial2.readBytes(serialBytes, 512);
        // check for a strings we care about
        // example output of serial data fields from the WS85
        // WindDir      = 79
        // WindSpeed    = 0.5
        // WindGust     = 0.6
        // GXTS04Temp   = 24.4
        // Temperature = 23.4 // WS80

        // RainIntSum     = 0
        // Rain           = 0.0
        if (serialPayloadSize > 0) {
            // Define variables for line processing
            int lineStart = 0;
            int lineEnd = -1;

            // Process each byte in the received data
            for (size_t i = 0; i < serialPayloadSize; i++) {
                // go until we hit the end of line and then process the line
                if (serialBytes[i] == '\n') {
                    lineEnd = i;
                    // Extract the current line
                    char line[meshtastic_Constants_DATA_PAYLOAD_LEN];
                    memset(line, '\0', sizeof(line));
                    if ((size_t)(lineEnd - lineStart) < sizeof(line) - 1) {
                        memcpy(line, &serialBytes[lineStart], lineEnd - lineStart);

                        ParsedLine parsed = parseLine(line);
                        if (strlen(parsed.name) > 0) {
                            if (strcmp(parsed.name, "WindDir") == 0) {
                                strlcpy(windDir, parsed.value, sizeof(windDir));
                                double radians = GeoCoord::toRadians(strtof(windDir, nullptr));
                                dir_sum_sin += sin(radians);
                                dir_sum_cos += cos(radians);
                                dirCount++;
                                gotwind = true;
                            } else if (strcmp(parsed.name, "WindSpeed") == 0) {
                                strlcpy(windVel, parsed.value, sizeof(windVel));
                                float newv = strtof(windVel, nullptr);
                                velSum += newv;
                                velCount++;
                                if (newv < lull || lull == -1) {
                                    lull = newv;
                                }
                                gotwind = true;
                            } else if (strcmp(parsed.name, "WindGust") == 0) {
                                strlcpy(windGust, parsed.value, sizeof(windGust));
                                float newg = strtof(windGust, nullptr);
                                if (newg > gust) {
                                    gust = newg;
                                }
                                gotwind = true;
                            } else if (strcmp(parsed.name, "BatVoltage") == 0) {
                                strlcpy(batVoltage, parsed.value, sizeof(batVoltage));
                                batVoltageF = strtof(batVoltage, nullptr);
                                break; // last possible data we want so break
                            } else if (strcmp(parsed.name, "CapVoltage") == 0) {
                                strlcpy(capVoltage, parsed.value, sizeof(capVoltage));
                                capVoltageF = strtof(capVoltage, nullptr);
                            } else if (strcmp(parsed.name, "GXTS04Temp") == 0 || strcmp(parsed.name, "Temperature") == 0) {
                                strlcpy(temperature, parsed.value, sizeof(temperature));
                                temperatureF = strtof(temperature, nullptr);
                            } else if (strcmp(parsed.name, "RainIntSum") == 0) {
                                strlcpy(rainStr, parsed.value, sizeof(rainStr));
                                rainSum = int(strtof(rainStr, nullptr));
                            } else if (strcmp(parsed.name, "Rain") == 0) {
                                strlcpy(rainStr, parsed.value, sizeof(rainStr));
                                rain = strtof(rainStr, nullptr);
                            }
                        }

                        // Update lineStart for the next line
                        lineStart = lineEnd + 1;
                    }
                }
            }
            break;
            // clear the input buffer
            while (Serial2.available() > 0) {
                Serial2.read(); // Read and discard the bytes in the input buffer
            }
        }
    }
    if (gotwind) {

        LOG_INFO("WS8X : %i %.1fg%.1f %.1fv %.1fv %.1fC rain: %.1f, %i sum", atoi(windDir), strtof(windVel, nullptr),
                 strtof(windGust, nullptr), batVoltageF, capVoltageF, temperatureF, rain, rainSum);
    }
    if (gotwind && !Throttle::isWithinTimespanMs(lastAveraged, averageIntervalMillis) && velCount > 0 && dirCount > 0) {
        // calculate averages and send to the mesh
        float velAvg = 1.0 * velSum / velCount;

        double avgSin = dir_sum_sin / dirCount;
        double avgCos = dir_sum_cos / dirCount;

        double avgRadians = atan2(avgSin, avgCos);
        float dirAvg = GeoCoord::toDegrees(avgRadians);

        if (dirAvg < 0) {
            dirAvg += 360.0;
        }
        lastAveraged = millis();

        // make a telemetry packet with the data
        meshtastic_Telemetry m = meshtastic_Telemetry_init_zero;
        m.which_variant = meshtastic_Telemetry_environment_metrics_tag;

        m.variant.environment_metrics.wind_speed = velAvg;
        m.variant.environment_metrics.has_wind_speed = true;

        m.variant.environment_metrics.wind_direction = dirAvg;
        m.variant.environment_metrics.has_wind_direction = true;

        m.variant.environment_metrics.temperature = temperatureF;
        m.variant.environment_metrics.has_temperature = true;

        m.variant.environment_metrics.voltage =
            capVoltageF > batVoltageF ? capVoltageF : batVoltageF; // send the larger of the two voltage values.
        m.variant.environment_metrics.has_voltage = true;

        m.variant.environment_metrics.wind_gust = gust;
        m.variant.environment_metrics.has_wind_gust = true;

        m.variant.environment_metrics.rainfall_24h = rainSum;
        m.variant.environment_metrics.has_rainfall_24h = true;

        // not sure if this value is actually the 1hr sum so needs to do some testing
        m.variant.environment_metrics.rainfall_1h = rain;
        m.variant.environment_metrics.has_rainfall_1h = true;

        if (lull == -1)
            lull = 0;
        m.variant.environment_metrics.wind_lull = lull;
        m.variant.environment_metrics.has_wind_lull = true;

        LOG_INFO("WS8X Transmit speed=%fm/s, direction=%d , lull=%f, gust=%f, voltage=%f temperature=%f",
                 m.variant.environment_metrics.wind_speed, m.variant.environment_metrics.wind_direction,
                 m.variant.environment_metrics.wind_lull, m.variant.environment_metrics.wind_gust,
                 m.variant.environment_metrics.voltage, m.variant.environment_metrics.temperature);

        sendTelemetry(m);

        // reset counters and gust/lull
        velSum = velCount = dirCount = 0;
        dir_sum_sin = dir_sum_cos = 0;
        gust = 0;
        lull = -1;
    }
#endif
    return;
}
#endif