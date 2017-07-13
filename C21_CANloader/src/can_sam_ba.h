/*
 * can_sam_ba.h
 *
 * Created: 7/12/2017 8:50:04 AM
 *  Author: avasquez
 */ 


#ifndef CAN_SAM_BA_H_
#define CAN_SAM_BA_H_

/* can buffer size (must be a power of two) */
#define CAN_BUFFER_SIZE        128

/* Define the default time-out value for can. */
#define CAN_DEFAULT_TIMEOUT    1000

/* Xmodem related defines */
/* CRC16  polynomial */
#define CRC16POLY                0x1021

#define SHARP_CHARACTER          '#'

/* X/Ymodem protocol: */
#define SOH                      0x01
//#define STX                    0x02
#define EOT                      0x04
#define ACK                      0x06
#define NAK                      0x15
#define CAN                      0x18
#define ESC                      0x1b

#define PKTLEN_128               128
//#define PKTLEN_1K   1024

/**
 * \brief Open the given can
 */
void can_open(void);

/**
 * \brief Stops the can
 */
void can_close(void);

/**
 * \brief Puts a byte on can line
 *
 * \param value      Value to put
 *
 * \return \c 1 if function was successfully done, otherwise \c 0.
 */
int can_putc(int value);

/**
 * \brief Waits and gets a value on can line
 *
 * \return value read on can line
 */
int can_getc(void);

/**
 * \brief Returns true if the SAM-BA CAN received the sharp char
 *
 * \return Returns true if the SAM-BA CAN received the sharp char
 */
int can_sharp_received(void);

/**
 * \brief This function checks if a character has been received on the can line
 *
 * \return \c 1 if a byte is ready to be read.
 */
bool can_is_rx_ready(void);

/**
 * \brief Gets a value on can line
 *
 * \return value read on can line
 */
int can_readc(void);

/**
 * \brief Send buffer on can line
 *
 * \param data pointer
 * \param number of data to send
 * \return number of data sent
 */
uint32_t can_putdata(void const* data, uint32_t length); //Send given data (polling)

/**
 * \brief Gets data from can line
 *
 * \param data pointer
 * \param number of data to get
 * \return value read on can line
 */
uint32_t can_getdata(void* data, uint32_t length); //Get data from comm. device

/**
 * \brief Send buffer on can line using Xmodem protocol
 *
 * \param data pointer
 * \param number of data to send
 * \return number of data sent
 */
uint32_t can_putdata_xmd(void const* data, uint32_t length); //Send given data (polling) using xmodem (if necessary)

/**
 * \brief Gets data from can line using Xmodem protocol
 *
 * \param data pointer
 * \param number of data to get
 * \return value read on can line
 */
uint32_t can_getdata_xmd(void* data, uint32_t length); //Get data from comm. device using xmodem (if necessary)

/**
 * \brief Gets data from can line using Xmodem protocol
 *
 * \param data pointer
 * \param number of data to get
 * \return value read on can line
 */
unsigned short add_crc(char ptr, unsigned short crc);

uint8_t getPacket(uint8_t *pData, uint8_t sno);

#endif /* CAN_SAM_BA_H_ */