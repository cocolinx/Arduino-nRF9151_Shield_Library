#ifndef CATLINK_H
#define CATLINK_H

#include <Arduino.h>
#include <SoftwareSerial.h>

class CocoLinx {
public:
    typedef enum {
        SERIAL_HARDWARE,
        SERIAL_SOFTWARE
    } SerialType;

    typedef enum {
        CLOSED = 0,
        OPENED = 1,
        DISCONNECTED = 0,
        CONNECTED = 1,
    } NetStatus;

    typedef enum {
        ACK_OKAY = 0,
        ACK_ERR_UNKNOWN,    
    
        // # error return negative sign
        ACK_ERR_ARG, // args
        ACK_ERR_NO_RESPONSE, 
        ACK_ERR_CRC_ACK, 
        ACK_ERR_SERIAL,
        ACK_ERR_PARSE,

        // ack-return
        ACK_ERR_CRC_REQ = 16, // crc error
        ACK_ERR_CATEGORY, // category not supported
        ACK_ERR_CMD, // cmd not supported
        ACK_ERR_FAILED, // operation failed
        ACK_ERR_NOT_SUPPORT, // function not supported
        ACK_ERR_PARAMETER, // parameter check error
        ACK_ERR_NOT_FOUND, // ddns ip not found
        ACK_ERR_LTE_DISCONNECTED,
        ACK_ERR_NO_SOCK, // udp, tcp, mqtt not opened(connected)
        ACK_ERR_SENT, // 
        ACK_ERR_NOT_READY,
        ACK_ERR_HOST_TIMEOUT, // ping
        ACK_ERR_GNSS_NOT_FIXED,
        ACK_ERR_NULL = 255,
    } AckCode; 

    typedef enum {
        PLMN_NOT_SET = -1,
        PLMN_AUTO = 0,
        PLMN_SKT = 45005,
        PLMN_KT = 45008,
        PLMN_LGU = 45006,
    } LtePlmn;

    typedef struct
    {
        char imei[16]; // 15digits, string
        char iccid[24]; // 20digits, string
        char imsi[16]; // 
        uint32_t plmn;
        uint32_t cell_id;
        uint16_t area_code;
        uint16_t band;
        int32_t rsrp_dbm;
        char ip_address[16]; // "xxx.xxx.xxx.xxx"
    } LteInfo;
        
    typedef struct
    {
        int64_t elapsed_ms; // elapsed time from last fix.

        uint16_t fix_counter; // over flow, used to check if data is updated
        uint8_t satellites_visible; // 
        uint8_t satellites_used; // 
        uint16_t year;	/** 4-digit representation (Gregorian calendar). */
        uint8_t month; /** 1...12 */	
        uint8_t day; /** 1...31 */

        uint8_t hour; /** 0...23 */	
        uint8_t minute; /** 0...59 */

        uint8_t seconds; /** 0...59 */		        
        uint8_t rsv_internal[5];

        double latitude;
        double longitude;
        float accuracy; // meter, position accuracy
        float altitude;
        float speed; // m/s
        float heading; // degrees
    } GnssData;

    CocoLinx();
    bool begin(SerialType type);
    void loop();

    // sys
    int32_t sysCheck();
    bool sysReset();
    int32_t sysGetVersion(uint8_t *ver4);
    int32_t sysSetLed(bool onoff);
    int32_t sysGetBtnCount(uint32_t *pCount);
    bool sysFactoryReset();
    int32_t sysGetRtc(int64_t *pRtc);

    // rs-485
    int32_t rs485Enable(uint32_t baudrate);
    int32_t rs485Disable();
    int32_t rs485Send(const uint8_t *data, uint32_t size);
    int32_t rs485Send(const char *data, uint32_t size);
    int32_t rs485Recv(uint8_t *buffer, uint32_t bufferSize);
    int32_t rs485Recv(char *buffer, uint32_t bufferSize);
    int32_t rs485ClearBuffer();

    // gnss
    int32_t gnssStart();
    int32_t gnssStop();
    int32_t gnssIsRunning();
    int32_t gnssRead(GnssData *pGnssData);

    // lte
    int32_t lteConnect(uint32_t timeout_ms, int32_t plmn);
    int32_t lteDisconnect();
    int32_t lteIsConnected();
    int32_t lteReadInfo(LteInfo *pInfo);    

    // ddns
    int32_t ddnsResolve(const char* hostname, uint8_t* hostIp);

    // ping
    int32_t ping(const uint8_t* dstIp, uint32_t *sent, uint32_t *recv, uint32_t *rttAvg, uint32_t *rttMin, uint32_t *rttMax);
    
    // datetime unix
    int32_t datetimeUnixMillis(int64_t *unixMillis);

    // modem direct at command
    int32_t modemAtCmd(char *cmd, char *respBuf, uint32_t respBufSize, int32_t *pRetCode, uint32_t timeoutMs);

    // udp
    int32_t udpOpen(const uint8_t* dstIp, uint16_t dstPort);
    int32_t udpClose();
    int32_t udpIsOpen();
    int32_t udpSend(const uint8_t* data, uint32_t size);
    int32_t udpSend(const char* data, uint32_t size);
    int32_t udpRecv(uint8_t* buffer, uint32_t bufferSize); // packet
    int32_t udpRecv(char* buffer, uint32_t bufferSize);
    int32_t udpClearBuffer();

    // tcp
    int32_t tcpConnect(const uint8_t* dstIp, uint16_t dstPort);
    int32_t tcpDisconnect();
    int32_t tcpIsConnected();
    int32_t tcpSend(const uint8_t* data, uint32_t size);
    int32_t tcpSend(const char *data, uint32_t size);
    int32_t tcpRecv(uint8_t* buffer, uint32_t bufferSize); // stream
    int32_t tcpRecv(char *buffer, uint32_t bufferSize);
    int32_t tcpClearBuffer();

    // mqtt
    // void mqttSetPrivateKey(const uint8_t *key, uint32_t keySize);    
    int32_t mqttConnect(const uint8_t* dstIp, uint16_t dstPort, 
        uint16_t keepAliveSecs, bool cleanSeesion, 
        const char *clientid, const char *username, const char *password);
        
    int32_t mqttDisconnect();
    int32_t mqttIsConnected();
    int32_t mqttPublish(const char *topic, const uint8_t *payload, uint32_t payloadSize, uint8_t qosLevel, bool retain);
    int32_t mqttPublish(const char *topic, const char *payload, uint32_t payloadSize, uint8_t qosLevel, bool retain);
    int32_t mqttSubscribe(const char *topic, uint8_t qosLevel);
    int32_t mqttUnSubscribe(const char *topic);
    
    // return payload size
    int32_t mqttRecvMsg(char *topicBuffer, uint32_t topicBufferSize, uint8_t *payloadBuffer, uint32_t payloadBufferSize, int32_t *pRecvPayloadSize); //
    int32_t mqttRecvMsg(char *topicBuffer, uint32_t topicBufferSize, char *payloadBuffer, uint32_t payloadBufferSize, int32_t *pRecvPayloadSize); //
    int32_t mqttClearBuffer();

private:
    #define CATLINK_DATA_SIZE_MAX     1400
    #define CATLINK_PACKET_SIZE_MAX   (CATLINK_DATA_SIZE_MAX + 16)
    #define CATLINK_HOSTNAME_LEN_MAX    128

    HardwareSerial* _hwSerial;
    SoftwareSerial* _swSerial;
    SerialType _serialType;
    uint8_t _pktbuf[CATLINK_PACKET_SIZE_MAX + 8];
    uint8_t _pktidx = 0;
    
    const uint8_t HW_RX_PIN = 0;
    const uint8_t HW_TX_PIN = 1;
    const uint8_t SW_RX_PIN = 7;
    const uint8_t SW_TX_PIN = 8;

    const uint32_t BAUDRATE = 115200;
    
    typedef enum
    {
        CATEGORY_SYS,
        CATEGORY_RS485,
        CATEGORY_GNSS,
        CATEGORY_LTE,
        CATEGORY_UDP,
        CATEGORY_TCP,
        CATEGORY_MQTT,
    } cocolinx_category_e;

    typedef enum
    {
        CMD_SYS_HELLO, // req>null, ack>null
        CMD_SYS_STATE, // data[0](0:booting, 1:ready)
        CMD_SYS_VERSION, // 
        CMD_SYS_RESET, // null
        CMD_SYS_SLEEP, // null
        CMD_SYS_WAKEUP, // null
        CMD_SYS_LED_USER_ON, // null
        CMD_SYS_LED_USER_OFF, // null
        CMD_SYS_BTN_COUNT,
        CMD_SYS_FACTORY_RESET, // modem factory reset + system reset
        CMD_SYS_RTC, // int64_t(ms)
    } cocolinx_cmd_sys_e;
    
    typedef enum
    {
        CMD_RS485_ENABLE, // baudrate
        CMD_RS485_DISABLE,
        CMD_RS485_SEND, // 
        CMD_RS485_RECV, // 
        CMD_RS485_CLEAR_BUF,
    } cocolinx_cmd_rs485_e;    

    typedef enum
    {
        CMD_GNSS_START, // req>interval, timeout
        CMD_GNSS_STOP,
        CMD_GNSS_STATUS,
        CMD_GNSS_READ,
    } cocolinx_cmd_gnss_e;

    typedef enum
    {
        CMD_LTE_CONNECT_SYNC, // req>data[0:3]=timeout_sec, data[4:7]=plmn, ans>data[0]=0:no connection, 1:connected
        CMD_LTE_DISCONNECT, // req>null
        CMD_LTE_STATUS, // req>null, ack>data[0]='x_lte_connection_status_e'    
        CMD_LTE_READ_INFO, // req>null, ack>'x_lte_info_ack_t'
        CMD_LTE_READ_IMEI, // req>null, ack>imei[16]
        CMD_LTE_READ_ICCID, // req>null, ack>string[20]
        CMD_LTE_DDNS, // req>hostname[128], ack>'x_ipaddr_t'
        CMD_LTE_PING_IPV4, // req>'x_lte_ping_req_t', ack>'x_lte_ping_ack_t'    
        CMD_LTE_DATETIME_UTC, // unix ms
        CMD_LTE_MODEM_ATCMD, // direct at command to modem(shell)
    } cocolinx_cmd_lte_e;

    typedef enum
    {
        CMD_UDP_OPEN, // req>'x_ipaddr_t'
        CMD_UDP_CLOSE,
        CMD_UDP_STATUS, // req>null, ack>[0]=0:closed, 1:opened
        CMD_UDP_SEND, // req>data, ack>null
        CMD_UDP_RECV, // req>null, ack>recv_data[recv_size]
        CMD_UDP_CLEAR, // req>null, ack>null, *clear rx buffer
    } cocolinx_cmd_udp_e;    

    typedef enum
    {
        CMD_TCP_CONNECT, // req>'x_ipaddr_t'
        CMD_TCP_DISCONNECT,
        CMD_TCP_STATUS,
        CMD_TCP_SEND,
        CMD_TCP_RECV,
        CMD_TCP_CLEAR,
    } cocolinx_cmd_tcp_e;

    typedef enum
    {        
        CMD_MQTT_CONNECT,
        CMD_MQTT_DISCONNECT,
        CMD_MQTT_STATUS,
        CMD_MQTT_PUBLISH,
        CMD_MQTT_SUBSCRIBE, // req>topic[]
        CMD_MQTT_UNSUBSCRIBE,
        CMD_MQTT_RECV_MSG, // req>buffersize, ack>topicLen
        CMD_MQTT_CLEAR,
    } cocolinx_cmd_mqtt_e;

    bool initSerial();
    uint16_t crc16Calculate(uint8_t *data, uint32_t size);
    int32_t transferPkt(uint8_t category, uint8_t cmd, uint16_t datasize, uint32_t timeout_ms);
    bool waitBootup(uint32_t timeout_ms);
    uint16_t getPktDataSize();
};

#endif // CATLINK_H
