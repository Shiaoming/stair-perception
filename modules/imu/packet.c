#include <string.h>
#include <stdio.h>

#include "packet.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x)	(sizeof(x) / sizeof((x)[0]))
#endif

#ifndef CH_OK
#define CH_OK   (0)
#endif

#ifndef CH_ERR
#define CH_ERR  (1)
#endif

static void
crc16_update (uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
  uint32_t crc = *currectCrc;
  uint32_t j;
  for (j = 0; j < lengthInBytes; ++j)
    {
      uint32_t i;
      uint32_t byte = src[j];
      crc ^= byte << 8;
      for (i = 0; i < 8; ++i)
	{
	  uint32_t temp = crc << 1;
	  if (crc & 0x8000)
	    {
	      temp ^= 0x1021;
	    }
	  crc = temp;
	}
    }
  *currectCrc = crc;
}

uint32_t
Packet_CreatePing (Packet_t *pkt)
{
  pkt->buf[0] = 0x5A;
  pkt->buf[1] = 0xA6;
  pkt->payload_len = 0;
  pkt->len = 2;
  return CH_OK;
}

uint32_t
Packet_CreatePingAck (Packet_t *pkt, uint8_t major, uint8_t minor,
		      uint8_t bugfix, uint16_t option)
{
  pkt->buf[0] = 0x5A;
  pkt->buf[1] = 0xA7;

  /* protocol bug fix */
  pkt->buf[2] = bugfix;
  /* protocol minor */
  pkt->buf[3] = minor;
  /* protocol major */
  pkt->buf[4] = major;
  pkt->buf[5] = 'P';

  /* option low: sender's address low */
  pkt->buf[6] = (option & 0x00FF) >> 0;

  /* option high: sender's address high */
  pkt->buf[7] = (option & 0xFF00) >> 8;

  /* crc */
  uint16_t crc;
  crc = 0;
  crc16_update (&crc, &pkt->buf[0], 8);
  pkt->buf[8] = (crc & 0x00FF) >> 0;
  pkt->buf[9] = (crc & 0xFF00) >> 8;

  pkt->payload_len = 0;
  pkt->type = 0xA7;
  pkt->len = 10;
  return CH_OK;
}

uint32_t
Packet_Begin (Packet_t *pkt)
{
  pkt->ofs = 6; /* sof(2) len(2) + crc(2) */
  memset (&pkt->buf[0], 0, sizeof(pkt->buf));
  pkt->buf[0] = 0x5A; /* header */
  pkt->buf[1] = 0xA5; /* data packet */
  return CH_OK;
}

uint32_t
Packet_AddData (Packet_t *pkt, uint8_t *buf, uint16_t len)
{
  /* add item content into buffer */
  memcpy ((pkt->buf + pkt->ofs), buf, len);
  pkt->ofs += len;
  return CH_OK;
}

uint32_t
Packet_Final (Packet_t *pkt)
{

  pkt->payload_len = pkt->ofs - 6;
  pkt->len = pkt->ofs;

  pkt->buf[2] = (pkt->payload_len & 0x00FF) >> 0;
  pkt->buf[3] = (pkt->payload_len & 0xFF00) >> 8;

  /* crc */
  uint16_t crc;
  crc = 0;
  crc16_update (&crc, &pkt->buf[0], 4);
  crc16_update (&crc, &pkt->buf[6], pkt->payload_len);
  pkt->buf[4] = (crc & 0x00FF) >> 0;
  pkt->buf[5] = (crc & 0xFF00) >> 8;

  return CH_OK;
}

enum status
{
  kStatus_Idle,
  kStatus_Cmd,
  kStatus_LenLow,
  kStatus_LenHigh,
  kStatus_CRCLow,
  kStatus_CRCHigh,
  kStatus_Data,
};

/* function pointer */
static OnDataReceivedEvent EventHandler;
static Packet_t *RxPkt;

/**
 * @brief  ��ʼ����̬����ģ��
 * @note   ��ɳ�ʼ��һ����������
 * @param  pkt ���հ�ָ��
 * @param  ���ճɹ��ص�����
 * @code

 *      void OnDataReceived(Packet_t *pkt)
 *      {
 *          pkt->buf Ϊ���� pkt->payload_len Ϊ���յ����ֽڳ��� 
 *      }
 *
 *      Packet_t pkt;
 *      Packet_DecodeInit(&pkt, OnDataReceived);
 * @endcode
 * @retval None
 */
void
Packet_DecodeInit (Packet_t *pkt, OnDataReceivedEvent Func)
{
  EventHandler = Func;
  memset (pkt, 0, sizeof(Packet_t));
  RxPkt = pkt;
}

/**
 * @brief  ����IMU����
 * @note   �ڴ��ڽ����ж��е��ô˺���
 * @param  c ��������
 * @retval CH_OK
 */
uint32_t
Packet_Decode (uint8_t c)
{
  static uint16_t CRCReceived = 0; /* CRC value received from a frame */
  static uint16_t CRCCalculated = 0; /* CRC value caluated from a frame */
  static uint8_t status = kStatus_Idle; /* state machine */
  static uint8_t crc_header[4] =
    { 0x5A, 0xA5, 0x00, 0x00 };

  switch (status)
    {
    case kStatus_Idle:
      if (c == 0x5A)
	status = kStatus_Cmd;
      break;
    case kStatus_Cmd:
      RxPkt->type = c;
      switch (RxPkt->type)
	{
	case 0xA5: /* Data */
	  status = kStatus_LenLow;
	  break;
	case 0xA6: /* Ping */
	  if (EventHandler != NULL)
	    {
	      EventHandler (RxPkt);
	    }
	  status = kStatus_Idle;
	  break;
	case 0xA7: /* Ping Respond */
	  RxPkt->ofs = 0;
	  status = kStatus_Data;
	  break;
	}
      break;
    case kStatus_LenLow:
      RxPkt->payload_len = c;
      crc_header[2] = c;
      status = kStatus_LenHigh;
      break;
    case kStatus_LenHigh:
      RxPkt->payload_len |= (c << 8);
      crc_header[3] = c;
      status = kStatus_CRCLow;
      break;
    case kStatus_CRCLow:
      CRCReceived = c;
      status = kStatus_CRCHigh;
      break;
    case kStatus_CRCHigh:
      CRCReceived |= (c << 8);
      RxPkt->ofs = 0;
      CRCCalculated = 0;
      status = kStatus_Data;
      break;
    case kStatus_Data:
      RxPkt->buf[RxPkt->ofs++] = c;
      if (RxPkt->type == 0xA7 && RxPkt->ofs >= 8)
	{
	  RxPkt->payload_len = 8;
	  EventHandler (RxPkt);
	  status = kStatus_Idle;
	}
      if (RxPkt->ofs >= MAX_PACKET_LEN)
	{
	  status = kStatus_Idle;
	  return CH_ERR;
	}

      if (RxPkt->ofs >= RxPkt->payload_len && RxPkt->type == 0xA5)
	{
	  /* calculate CRC */
	  crc16_update (&CRCCalculated, crc_header, 4);
	  crc16_update (&CRCCalculated, RxPkt->buf, RxPkt->ofs);

	  /* CRC match */
	  if (CRCCalculated == CRCReceived)
	    {
	      EventHandler (RxPkt);
	    }
	  status = kStatus_Idle;
	}
      break;
    default:
      status = kStatus_Idle;
      break;
    }
  return CH_OK;
}

