#pragma once

/* USE_CRC16_CCITT_FALSE
* Computes CRC
* CRC-16/CCITT-FALSE	16	0x1021	0xFFFF	false	false	0x0	0x29B1
* https://ru.wikipedia.org/wiki/%D0%A6%D0%B8%D0%BA%D0%BB%D0%B8%D1%87%D0%B5%D1%81%D0%BA%D0%B8%D0%B9_%D0%B8%D0%B7%D0%B1%D1%8B%D1%82%D0%BE%D1%87%D0%BD%D1%8B%D0%B9_%D0%BA%D0%BE%D0%B4
*/
/* USE_CRC16_X25 <- Use in SPODES
* CRC-16/X-25	16	0x1021	0xFFFF	true	true	0xFFFF	0x906E
* https://ru.wikipedia.org/wiki/%D0%A6%D0%B8%D0%BA%D0%BB%D0%B8%D1%87%D0%B5%D1%81%D0%BA%D0%B8%D0%B9_%D0%B8%D0%B7%D0%B1%D1%8B%D1%82%D0%BE%D1%87%D0%BD%D1%8B%D0%B9_%D0%BA%D0%BE%D0%B4
*/

///< Select default CRC-16
#if !USE_CRC16_X25 && !USE_CRC16_CCITT_FALSE
    #define USE_CRC16_X25               1
#endif

#define FCS_CONST                   0xF0B8

#if USE_CRC16_X25
    #define CRC_LSB                     1
    #define CRC_HSB                     0
    #define USE_CRC16_CCITT_FALSE       0
#elif USE_CRC16_CCITT_FALSE
    #define CRC_LSB                     0
    #define CRC_HSB                     1
    #define USE_CRC16_X25               0
#else
    #error "Must choose the calculation algorithm CRC-16."
#endif

void compute_crc16(uint8_t data[], int size, uint8_t crc[2]);
uint16_t compute_crc16(uint16_t fcs, uint8_t data);