#include <string.h>
#include <stdio.h>

#include "packet.h"
#include "imu_data_decode.h"

static Packet_t RxPkt; /* used for data receive */
static int16_t acc[3];
static int16_t gyo[3];
static int16_t mag[3];
static float eular[3];
static float quat[4];
static uint8_t id;

int
get_raw_acc(int16_t *a)
{
    memcpy(a, acc, sizeof(acc));
    return 0;
}

int
get_raw_gyo(int16_t *g)
{
    memcpy(g, gyo, sizeof(gyo));
    return 0;
}

int
get_raw_mag(int16_t *m)
{
    memcpy(m, mag, sizeof(mag));
    return 0;
}

int
get_eular(float *e)
{
    memcpy(e, eular, sizeof(eular));
    return 0;
}

int
get_quat(float *q)
{
    memcpy(q, quat, sizeof(quat));
    return 0;
}

int
get_id(uint8_t *user_id)
{
    *user_id = id;
    return 0;
}

/*  callback function of  when recv a data frame successfully */
static void
OnDataReceived(Packet_t *pkt)
{

    if (pkt->type != 0xA5)
    {
        return;
    }

    int offset = 0;
    uint8_t *p = pkt->buf;
    while (offset < pkt->payload_len)
    {
        switch (p[offset])
        {
            case kItemID:
                id = p[1];
                offset += 2;
                break;
            case kItemAccRaw:
            case kItemAccCalibrated:
            case kItemAccFiltered:
            case kItemAccLinear:
                memcpy(acc, p + offset + 1, sizeof(acc));
                offset += 7;
                break;
            case kItemGyoRaw:
            case kItemGyoCalibrated:
            case kItemGyoFiltered:
                memcpy(gyo, p + offset + 1, sizeof(gyo));
                offset += 7;
                break;
            case kItemMagRaw:
            case kItemMagCalibrated:
            case kItemMagFiltered:
                memcpy(mag, p + offset + 1, sizeof(mag));
                offset += 7;
                break;
            case kItemRotationEular:
                eular[0] = ((float) (int16_t) (p[offset + 1] + (p[offset + 2] << 8)))
                           / 100;
                eular[1] = ((float) (int16_t) (p[offset + 3] + (p[offset + 4] << 8)))
                           / 100;
                eular[2] = ((float) (int16_t) (p[offset + 5] + (p[offset + 6] << 8)))
                           / 10;
                offset += 7;
                break;
            case kItemRotationEular2:
                memcpy(eular, p + offset + 1, sizeof(eular));
                offset += 13;
                break;
            case kItemRotationQuat:
                memcpy(quat, p + offset + 1, sizeof(quat));
                offset += 17;
                break;
            case kItemPressure:
                offset += 5;
                break;
            case kItemTemperature:
                offset += 5;
                break;
            default:
                printf("data decode wrong\r\n");
                return;
                break;
        }
    }
}

/* imu data decode init

 Example for usage:
 1. add packet.c, imu_data_decode.c into your project, and add their include path 
 2. main function:


 uint8_t ID;
 int16_t Acc[3];
 int16_t Gyo[3];
 int16_t Mag[3];
 float Eular[3];
 float Quat[4];
 int32_t Pressure;

 main()
 {
 // init UART .etc...

 // imu_data_decode init 
 imu_data_decode_init();

 while(1)
 {
 get_raw_acc(Acc);
 get_raw_gyo(Gyo);
 get_raw_mag(Mag);
 get_eular(Eular);
 get_quat(Quat);
 get_id(&ID);
 
 //printf("id:%d\r\n", ID);
 
 printf("Acc: %d %d %d\r\n", Acc[0], Acc[1], Acc[2]);
 printf("Gyo: %d %d %d\r\n", Gyo[0], Gyo[1], Gyo[2]);
 printf("Mag: %d %d %d\r\n", Mag[0], Mag[1], Mag[2]);
 printf("Eular(P R Y):    %0.2f %0.2f %0.2f\r\n", Eular[0], Eular[1], Eular[2]);
 printf("Quat(W X Y Z):   %0.3f %0.3f %0.3f %0.3f\r\n", Quat[0], Quat[1], Quat[2], Quat[3]);
 
 //delay a while 
 delay();
 }
 }

 //your UART interrupt function when received a char
 void UART_ISR(void)
 {
 uint8_t ch;

 // get a UART char 
 ch = uart_read();

 Packet_Decode(ch);
 }

 */
int
imu_data_decode_init(void)
{
    Packet_DecodeInit(&RxPkt, OnDataReceived);
    return 0;
}

