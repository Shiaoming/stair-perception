#ifndef __PACKET_H__
#define __PACKET_H__

#include <stdint.h>
#include <stdbool.h>

#define MAX_PACKET_LEN          (128)

typedef enum
{
  kItemKeyStatus = 0x80, /* key status           size: 4 */
  kItemID = 0x90, /* user programed ID    size: 1 */
  kItemUID = 0x91, /* Unique ID            size: 4 */
  kItemIPAdress = 0x92, /* ip address           size: 4 */
  kItemAccRaw = 0xA0, /* raw acc              size: 3x2 */
  kItemAccCalibrated = 0xA1,
  kItemAccFiltered = 0xA2,
  kItemAccLinear = 0xA5,
  kItemGyoRaw = 0xB0, /* raw gyro             size: 3x2 */
  kItemGyoCalibrated = 0xB1,
  kItemGyoFiltered = 0xB2,
  kItemMagRaw = 0xC0, /* raw mag              size: 3x2 */
  kItemMagCalibrated = 0xC1,
  kItemMagFiltered = 0xC2,
  kItemRotationEular = 0xD0, /* eular angle          size:3x2 */
  kItemRotationEular2 = 0xD9, /* new eular angle      size:3x4 */
  kItemRotationQuat = 0xD1, /* att q,               size:4x4 */
  kItemTemperature = 0xE0,
  kItemPressure = 0xF0, /* pressure             size:1x4 */
  kItemEnd = 0x00,
} ItemID_t;

typedef struct
{
  uint32_t ofs;
  uint8_t buf[MAX_PACKET_LEN]; /* total frame buffer */
  uint16_t payload_len;
  uint16_t len; /* total frame len */
  uint8_t type;
} Packet_t;

/* packet Tx API */
uint32_t
Packet_AddData (Packet_t *pkt, uint8_t *buf, uint16_t len);
uint32_t
Packet_Begin (Packet_t *pkt);
uint32_t
Packet_Final (Packet_t *pkt);
uint32_t
Packet_CreatePing (Packet_t *pkt);
uint32_t
Packet_CreatePingAck (Packet_t *pkt, uint8_t major, uint8_t minor,
		      uint8_t bugfix, uint16_t option);

/* packet Rx API */
typedef void
(*OnDataReceivedEvent) (Packet_t *pkt);
void
Packet_DecodeInit (Packet_t *pkt, OnDataReceivedEvent rx_handler);
uint32_t
Packet_Decode (uint8_t c);

#endif

