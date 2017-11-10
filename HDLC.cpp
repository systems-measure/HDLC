#define HDLC_C
/****************************************************************************/
/*                              MODULES USED                                */
/****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "HDLC.h"
#include "hdlc_crc.h"

/****************************************************************************/
/*                          DEFINITIONS AND MACROS                          */
/****************************************************************************/

#define MAX_HDLC_FR_BITS        (MAX_HDLC_FR_LEN_WITH_STUF*8)
#define MIN_HDLC_FR_BITS        (MIN_HDLC_FR_LEN*8)

#define FLAG                    (0x7E)        /* HDLC frame start/end flag */
#define HDLC_TERM_FLAG          (0x7F)        /* HDLC terminate flag */

/****************************************************************************/
/*                          TYPEDEFS AND STRUCTURES                         */
/****************************************************************************/

/****************************************************************************/
/*                            EXTERNAL VARIABLES                            */
/****************************************************************************/

/****************************************************************************/
/*                            EXTERNAL FUNCTIONS                            */
/****************************************************************************/

/****************************************************************************/
/*                       PROTOTYPES AND LOCAL FUNCTIONS                     */
/****************************************************************************/

void clear_hdlc_ctxt(hdlc_ch_ctxt_t *hdlc_ch_ctxt);
void detect_hdlc_frame(hdlc_ch_ctxt_t *hdlc_ch_ctxt, uint8_t rec_byte, int offset);
void remove_stuffing(uint8_t dest_fr[], uint8_t src_frame[], uint16_t bit_cnt, uint16_t *byte_cnt, hdlc_fr_detect_errors_e *err, int offset);

int hdlc_CRC_match(uint8_t hdlc_frame[MAX_HDLC_FR_LEN],int frame_size);

void handle_hdlc_frame(hdlc_ch_ctxt_t *ctxt, int offset);

/****************************************************************************/
/*                             EXPORTED VARIABLES                           */
/****************************************************************************/

/****************************************************************************/
/*                              GLOBAL VARIABLES                            */
/****************************************************************************/

const uint8_t bitmask[] = {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE};

/* FLAG cyclic rotation */
const uint8_t flag_lookup[] = {0xFC, 0xF9, 0xF3, 0xE7, 0xCF, 0x9F, 0x3F, 0x7E};

/****************************************************************************/
/*                             EXPORTED FUNCTIONS                           */
/****************************************************************************/

void HDLC_init(hdlc_ch_ctxt_t *hdlc_ch_ctxt, void* instance_cb, Fun_CallBack_HDLC cb)
{
    clear_hdlc_ctxt(hdlc_ch_ctxt);

    if(cb != NULL)
    {
        hdlc_ch_ctxt->instance_cb = instance_cb;
        hdlc_ch_ctxt->callback = cb;
    }
    else
    {
        hdlc_LogOut("\nHDLC FRAME: ERROR HDLC context callback is not initialized");
        exit(1);
    }
}

void HDLC(uint8_t inp8, int offset, hdlc_ch_ctxt_t *hdlc_ch_ctxt)
{
    detect_hdlc_frame(hdlc_ch_ctxt, inp8, offset);
    if(TRUE==hdlc_ch_ctxt->frame_ready)
        handle_hdlc_frame(hdlc_ch_ctxt, offset);
}

/****************************************************************************/
/*                              LOCAL FUNCTIONS                             */
/****************************************************************************/

/* 
* Removes stuffing, handles frame length errors, Checks CRC 
*/
void handle_hdlc_frame(hdlc_ch_ctxt_t *ctxt, int offset)
{
    uint8_t    fr_minus_stuffing[MAX_HDLC_FR_LEN_WITH_STUF]={0};
    uint16_t    valid_bytes = 0;

    ctxt->frame_ready = FALSE;

    remove_stuffing(fr_minus_stuffing, ctxt->ready_fr, ctxt->ready_fr_bit_cnt, &valid_bytes, &ctxt->err_type, offset);
    memset(&ctxt->ready_fr, 0, MAX_HDLC_FR_LEN_WITH_STUF);

    if(NO_ERR == ctxt->err_type)
    {
        if (hdlc_CRC_match(fr_minus_stuffing, valid_bytes))
        {
            if(ctxt->callback != NULL)
                ctxt->callback(ctxt->instance_cb, fr_minus_stuffing, valid_bytes); /* Call Application */
        }
        else
        {    
            ctxt->err_type = CRC_MISMATCH;
            hdlc_LogOut("\n%d\tHDLC FRAME: ERROR %d", offset, ctxt->err_type);
            ctxt->err_type = NO_ERR;    /* Handle Error & Reset */
        }
    }
    else
    {
        hdlc_LogOut("\n%d\tHDLC FRAME: ERROR %d", offset, ctxt->err_type);
        ctxt->err_type = NO_ERR;    /* Handle Error & Reset */
    }
}

/*
* clears sinagling context.
*/
void clear_hdlc_ctxt(hdlc_ch_ctxt_t *hdlc_ch_ctxt)
{
    hdlc_ch_ctxt->flag_pos_ctr = 0;
    hdlc_ch_ctxt->rec_bits=0;
    hdlc_ch_ctxt->fr_bit_cnt=0;
    hdlc_ch_ctxt->ready_fr_bit_cnt=0;
    hdlc_ch_ctxt->frame_ready = FALSE;
    hdlc_ch_ctxt->state=FLAG_SEARCH;
    hdlc_ch_ctxt->err_type=NO_ERR;
    memset(&hdlc_ch_ctxt->frame, 0, MAX_HDLC_FR_LEN_WITH_STUF);
    memset(&hdlc_ch_ctxt->ready_fr, 0, MAX_HDLC_FR_LEN_WITH_STUF);
    hdlc_ch_ctxt->callback = NULL;
}

/*
* detects hdlc frame.
*/
void detect_hdlc_frame(hdlc_ch_ctxt_t *ctxt, uint8_t rec_byte, int offset)
{
    uint8_t    i;
    uint16_t    byte_index;

    for(i=0;i<=7;++i)
    {
        /* get next bit */
        ctxt->rec_bits = (ctxt->rec_bits << 1);
        ctxt->rec_bits = (ctxt->rec_bits | (rec_byte>>(7-i) & 0x01));

        switch(ctxt->state)
        {
        case    FLAG_SEARCH: 
            /* look for 7E */
            if(FLAG == ctxt->rec_bits)
            {
                ctxt->state        = FLAG_SYNC_EST;    /* move to flag sync established state */
                ctxt->flag_pos_ctr    = 0;
            }
            break;

        case    FLAG_SYNC_EST:
            /* look for non 7E rec bits */
            if(flag_lookup[ctxt->flag_pos_ctr] == ctxt->rec_bits)
            {
                ctxt->flag_pos_ctr++;
                if(8 == ctxt->flag_pos_ctr)
                {
                    ctxt->flag_pos_ctr = 0; /* reset counter */
                }
            }
            else
            {
                /* move to frame rx state and copy last (ctxt->flag_pos_ctr+1) bits update frame bit counter */
                ctxt->frame[0]        = (ctxt->rec_bits&(~bitmask[(8-(ctxt->flag_pos_ctr+1))]))<<(8-(ctxt->flag_pos_ctr+1));
                ctxt->fr_bit_cnt    = (ctxt->flag_pos_ctr+1);
                ctxt->state        = FRAME_RX;
            }
            break;

        case    FRAME_RX:
            /* keep copying frame bits until closing flag is found */
            byte_index                = (ctxt->fr_bit_cnt/8);
            ctxt->frame[byte_index] = (ctxt->frame[byte_index] | (ctxt->rec_bits&0x01)<<(7-(ctxt->fr_bit_cnt%8)));
            ctxt->fr_bit_cnt++;

            if(MAX_HDLC_FR_BITS < ctxt->fr_bit_cnt) /* error: frame length exceeded */
            {
                memset(&ctxt->frame, 0, MAX_HDLC_FR_LEN_WITH_STUF);
                ctxt->fr_bit_cnt    = 0;
                ctxt->state        = FLAG_SEARCH;
                hdlc_LogOut("\n%d\tHDLC FRAME: ERROR frame length exceeded during RX", offset);
                break;
            }

            if(FLAG == ctxt->rec_bits)
            {
                /* closing flag found */
                ctxt->state        = FLAG_SYNC_EST;
                ctxt->flag_pos_ctr    = 0;

                if(MIN_HDLC_FR_BITS > ctxt->fr_bit_cnt) /* error: invalid frame length */
                {
                    memset(&ctxt->frame, 0, MAX_HDLC_FR_LEN_WITH_STUF);
                    ctxt->fr_bit_cnt    = 0;
                    ctxt->state        = FLAG_SEARCH;
                    hdlc_LogOut("\n%d\tHDLC FRAME: ERROR frame length less than %d bits", offset, MIN_HDLC_FR_BITS*8);
                    break;
                }

                /* remove last 8 bits from frame and update frame bit counter */
                ctxt->frame[byte_index]    = 0;
                ctxt->fr_bit_cnt    = (ctxt->fr_bit_cnt-8);
                byte_index        = (ctxt->fr_bit_cnt/8);
                ctxt->frame[byte_index]    = ((ctxt->frame[byte_index])&(bitmask[ctxt->fr_bit_cnt%8]));

                /* copy frame to caller's copy */
                memcpy(&ctxt->ready_fr,&ctxt->frame,MAX_HDLC_FR_LEN_WITH_STUF);
                ctxt->ready_fr_bit_cnt    = ctxt->fr_bit_cnt;
                ctxt->frame_ready     = TRUE;

                /* reset working copy */
                memset(&ctxt->frame, 0, MAX_HDLC_FR_LEN_WITH_STUF);
                ctxt->fr_bit_cnt    = 0;
            }
            else if(HDLC_TERM_FLAG == ctxt->rec_bits)
            {
                memset(&ctxt->frame, 0, MAX_HDLC_FR_LEN_WITH_STUF);
                ctxt->fr_bit_cnt    = 0;
                ctxt->state        = FLAG_SEARCH;
                break;
            }

            break;

        default:
            /* not possible */
            assert(0);
        }
    }
}

/*
* removes stuffed bits. assumption: any bit after 5 1s is ignored
*/
void remove_stuffing(uint8_t dst_fr[], uint8_t src_fr[], uint16_t bit_cnt, uint16_t *byte_cnt, hdlc_fr_detect_errors_e *err, int offset)
{
    uint16_t    i;
    uint16_t    src_byte;
    uint16_t    dst_byte;
    uint8_t    curr_bit;
    uint16_t    pos=0;    
    uint8_t    ones=0;

    for(i=0;i<bit_cnt;++i,++pos)
    {
        src_byte=i/8;
        curr_bit=(src_fr[src_byte]>>(7-(i%8)))&0x01;
        if(curr_bit)
        {
            dst_byte=pos/8;
            dst_fr[dst_byte]= dst_fr[dst_byte] | curr_bit<<(7-(pos%8));
            ++ones;        /* inc. 1s count */
            if(5==ones)
            {
                ++i;    /* skip next bit */
                ones=0; /* reset 1s count */
            }
        }
        else
        {
            dst_byte=pos/8;
            dst_fr[dst_byte]= dst_fr[dst_byte] | curr_bit<<(7-(pos%8));
            ones=0;        /* reset 1s count */
        }
    }

    *byte_cnt=pos/8;

    /* look for invalid frame length error */
    if(((pos%8)!=0) || (*byte_cnt<5) || (*byte_cnt>MAX_HDLC_FR_LEN))
    {
        if((pos%8)!=0)
        {
            hdlc_LogOut("\n%d\tHDLC FRAME: ERROR (frame_length_bits%%8)!=0", offset);
        }

        *err=INVALID_FR_LEN;
    }
}



/*
* runs CRC check
*/
int hdlc_CRC_match(uint8_t hdlc_frame[MAX_HDLC_FR_LEN], int frame_size)
{
    uint8_t hdlc_crc[2] = {0};

    if ((frame_size<5)||(frame_size>=MAX_HDLC_FR_LEN)) return 0;

    compute_crc16(hdlc_frame, frame_size, hdlc_crc);

    hdlc_crc[0]    =    (~(hdlc_crc[0]));
    hdlc_crc[1]    =    (~(hdlc_crc[1]));

    if ((hdlc_crc[0]==hdlc_frame[frame_size-2])&&(hdlc_crc[1]==hdlc_frame[frame_size-1])) return 1;
    else return 0;
}

/****************************************************************************/
/*                                   EOF                                    */
/****************************************************************************/


