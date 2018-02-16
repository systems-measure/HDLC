#define HDLC_C
#define HDLC_PRIVATE
/****************************************************************************/
/*                              MODULES USED                                */
/****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <algorithm>

#include "HDLC.h"
#include "hdlc_crc.h"

/* [Help HDLC](http://www.erg.abdn.ac.uk/users/gorry/course/dl-pages/hdlc-framing.html) */

/****************************************************************************/
/*                          DEFINITIONS AND MACROS                          */
/****************************************************************************/

#define MAX_HDLC_FR_BITS        (MAX_HDLC_FR_LEN_WITH_STUF*8)
#define MIN_HDLC_FR_BITS        (MIN_HDLC_FR_LEN*8)

#define FLAG                    (0x7E)        /* HDLC frame start/end flag */
#define HDLC_TERM_FLAG          (0x7F)        /* HDLC terminate flag "abort sequence"*/

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

void HDLC_init(hdlc_ch_ctxt_t *hdlc_ch_ctxt, hdlc_callback_t *callback, uint16_t FrameTimeOut)
{
    clear_hdlc_ctxt(hdlc_ch_ctxt);
    // -- Check Valid callback context
    if(callback != NULL){
        if (callback->cb_RecieverFrame) {
            memcpy(&hdlc_ch_ctxt->callback, callback, sizeof(hdlc_ch_ctxt->callback));
        }else hdlc_LogOut("HDLC FRAME: ERROR HDLC context callback is not initialized.\r\n");
        // -- Default set if need
        HDLC_UpdateFrameTimeOut(hdlc_ch_ctxt, FrameTimeOut);
        // --
        if (!callback->cb_ResetFrameTimeOut) 
            callback->cb_ResetFrameTimeOut = funCb_HDLC_FrameTimeOut_DEF;
    }else hdlc_LogOut("HDLC FRAME: ERROR HDLC context callback is not initialized.\r\n");
}

void HDLC_reset(hdlc_ch_ctxt_t *ctxt)
{
    // --
    ctxt->state = FLAG_SEARCH;
    ctxt->err_type = NO_ERR;
    ctxt->offset = 0;
    ctxt->frame_ready = FALSE;
#   if USE_BIT_STAFFING
        memset(ctxt->frame, 0, sizeof(ctxt->frame));
#   endif
}
void HDLC_UpdateFrameTimeOut(hdlc_ch_ctxt_t *ctxt, uint16_t newFrameTimeOut) {
    ctxt->chkFrameTimeOut = (newFrameTimeOut) ? newFrameTimeOut : LSB_HDLC_TIMER;
    ctxt->chkFrameTimeOut += LSB_HDLC_TIMER;    // for jitter timer
}

bool HDLC(uint8_t inp8, int offset, hdlc_ch_ctxt_t *hdlc_ch_ctxt)
{
    bool r = false;

    int *pOffset = &offset;
    if (*pOffset == -1) pOffset = &(++hdlc_ch_ctxt->offset);

    if (!hdlc_ch_ctxt->frame_ready) {
        detect_hdlc_frame(hdlc_ch_ctxt, inp8, *pOffset);
        if (hdlc_ch_ctxt->frame_ready) {
            handle_hdlc_frame(hdlc_ch_ctxt, *pOffset);
            // --
            r = hdlc_ch_ctxt->frame_ready;
        }
    }

__exit:
    return r;
}
bool HDLC_GetFrameReady(hdlc_ch_ctxt_t *ctxt){
    return ctxt->frame_ready;
}
void HDLC_timer_20ms(hdlc_ch_ctxt_t *ctxt) {
    if (ctxt->cntFrameTimeOut) {
        if (ctxt->cntFrameTimeOut == LSB_HDLC_TIMER) {
            // -- Message for extern application
            ctxt->callback.cb_ResetFrameTimeOut(ctxt->callback.instance_cb);
            HDLC_reset(ctxt);
        }
        ctxt->cntFrameTimeOut -= LSB_HDLC_TIMER;
    }
}

/****************************************************************************/
/*                              LOCAL FUNCTIONS                             */
/****************************************************************************/

/* 
* Removes stuffing, handles frame length errors, Checks CRC 
*/
void handle_hdlc_frame(hdlc_ch_ctxt_t *ctxt, int offset)
{
    uint16_t    valid_bytes = 0;

    ctxt->frame_ready = FALSE;

    remove_bit_stuffing(ctxt->frame, ctxt->frame, ctxt->fr_bit_cnt, &valid_bytes, &ctxt->err_type, offset);
    remove_byte_stuffing(ctxt->frame, ctxt->fr_byte_cnt, &valid_bytes, &ctxt->err_type, offset);

    if(NO_ERR == ctxt->err_type)
    {
        if (hdlc_CRC_match(ctxt->frame, valid_bytes))
        {
            ctxt->frame_ready = TRUE;
            ctxt->cntFrameTimeOut = 0;  ///< Stop Frame TimeOut
            ctxt->callback.cb_RecieverFrame(ctxt->callback.instance_cb, ctxt->frame, valid_bytes); /* Call Application */
        }
        else
        {    
            ctxt->err_type = CRC_MISMATCH;
            hdlc_LogOut("%d\tHDLC FRAME: ERROR %d (CRC_MISMATCH)\r\n", offset, ctxt->err_type);
            WorkaroundLackBitStaff(ctxt, offset);
        }
    }
    else
    {
        hdlc_LogOut("%d\tHDLC FRAME: ERROR %d\r\n", offset, ctxt->err_type);
        ctxt->err_type = NO_ERR;    /* Handle Error & Reset */
        ctxt->offset = 0;
    }
}

/*
* clears sinagling context.
*/
void clear_hdlc_ctxt(hdlc_ch_ctxt_t *hdlc_ch_ctxt)
{
    memset(hdlc_ch_ctxt, 0, sizeof(*hdlc_ch_ctxt));
    // --
    hdlc_ch_ctxt->state=FLAG_SEARCH;
    hdlc_ch_ctxt->err_type=NO_ERR;
}
/*
*  
*/
void WorkaroundLackBitStaff(hdlc_ch_ctxt_t *ctxt, int offset) {
    ctxt->err_type = NO_ERR;    /* Handle Error & Reset */
    ctxt->offset = 0;
    // -- Because no bitstafàing - then this crutch closes the hole.
    ctxt->fcs = compute_crc16(ctxt->fcs, FLAG);
    ctxt->frame[ctxt->fr_byte_cnt++] = FLAG;
    if (MAX_HDLC_FR_LEN_WITH_STUF < ctxt->fr_byte_cnt) {
        ctxt->state = FLAG_SEARCH;
        ctxt->fr_byte_cnt = 0;
        hdlc_LogOut("%d\tHDLC FRAME: ERROR frame length exceeded during RX\r\n", offset);
    }
}

/*
* detects hdlc frame. use bit staffing
*/
#if USE_BIT_STAFFING
void detect_hdlc_frame(hdlc_ch_ctxt_t *ctxt, uint8_t rec_byte, int offset)
{
    uint8_t    i;
    uint16_t    byte_index;

    // -- Reload frame timeout
    ctxt->cntFrameTimeOut = chkFrameTimeOut;
    // -- 
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
                ctxt->fr_bit_cnt    = 0;
                ctxt->state        = FLAG_SEARCH;
                hdlc_LogOut("%d\tHDLC FRAME: ERROR frame length exceeded during RX\r\n", offset);
                ctxt->offset = 0;
                break;
            }

            if(FLAG == ctxt->rec_bits)
            {
                /* closing flag found */
                ctxt->state        = FLAG_SYNC_EST;
                ctxt->flag_pos_ctr    = 0;

                if(MIN_HDLC_FR_BITS > ctxt->fr_bit_cnt) /* error: invalid frame length */
                {
                    ctxt->fr_bit_cnt    = 0;
                    ctxt->state        = FLAG_SEARCH;
                    hdlc_LogOut("%d\tHDLC FRAME: ERROR frame length less than %d bits\r\n", offset, MIN_HDLC_FR_BITS*8);
                    ctxt->offset = 0;
                    break;
                }

                /* remove last 8 bits from frame and update frame bit counter */
                ctxt->frame[byte_index]    = 0;
                ctxt->fr_bit_cnt    = (ctxt->fr_bit_cnt-8);
                byte_index        = (ctxt->fr_bit_cnt/8);
                ctxt->frame[byte_index]    = ((ctxt->frame[byte_index])&(bitmask[ctxt->fr_bit_cnt%8]));

                /* Ready HDLC frame */
                ctxt->frame_ready     = TRUE;
            }
            /* Abort HDLC frame */
            else if(HDLC_TERM_FLAG == ctxt->rec_bits)
            {
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
* detects hdlc frame. use byte staffing
*/
#else /*!USE_BIT_STAFFING*/
void detect_hdlc_frame(hdlc_ch_ctxt_t *ctxt, uint8_t rec_byte, int offset){
    // -- Reload frame timeout
    ctxt->cntFrameTimeOut = ctxt->chkFrameTimeOut;
    // -- 
    switch (ctxt->state){
    case    FLAG_SEARCH:
        /* look for 7E */
        if (FLAG == rec_byte){
            ctxt->state = FRAME_RX;
            ctxt->fr_byte_cnt = 0;
            ctxt->fcs = 0xFFFF;
        }
        break;
    case    FRAME_RX:
        if (FLAG == rec_byte){
            if (MIN_HDLC_FR_LEN > ctxt->fr_byte_cnt) /* error: invalid frame length */
            {
                ctxt->fr_byte_cnt = 0;
                ctxt->state = FRAME_RX;
                hdlc_LogOut("%d\tHDLC FRAME: ERROR frame length less than %d bits\r\n", offset, MIN_HDLC_FR_LEN);
                break;
            }

            /* Ready HDLC frame */
            if (ctxt->fcs == FCS_CONST) {
                ctxt->frame_ready = TRUE;
            }else WorkaroundLackBitStaff(ctxt, offset);
        }
        else {
            ctxt->fcs = compute_crc16(ctxt->fcs, rec_byte);
            ctxt->frame[ctxt->fr_byte_cnt++] = rec_byte;
            if (MAX_HDLC_FR_LEN_WITH_STUF < ctxt->fr_byte_cnt) {
                ctxt->state = FLAG_SEARCH;
                hdlc_LogOut("%d\tHDLC FRAME: ERROR frame length exceeded during RX\r\n", offset);
                ctxt->offset = 0;
                break;
            }
        }
        break;

    default:
        /* not possible */
        assert(0);
    }
}
#endif /*USE_BIT_STAFFING*/
/*
* removes stuffed bits. assumption: any bit after 5 1s is ignored
*/
#if USE_BIT_STAFFING
void remove_bit_stuffing(uint8_t dst_fr[], uint8_t src_fr[], uint16_t bit_cnt, uint16_t *byte_cnt, hdlc_fr_detect_errors_e *err, int offset)
{
    uint16_t    i;
    uint8_t     src_frame_byte;
    uint16_t    src_byte, read_src_byte = UINT16_MAX;
    uint16_t    dst_byte;
    uint8_t     curr_bit;
    uint16_t    pos=0;    
    uint8_t     ones=0;

    for(i = 0; i < bit_cnt; ++i, ++pos)
    {
        // -- Get src frame byte
        src_byte = i / 8;
        if (read_src_byte != src_byte) {
            src_frame_byte = src_fr[src_byte];
            read_src_byte = src_byte;
        }
        // -- Get next src frame bit
        curr_bit = (src_frame_byte >> (7 - (i % 8))) & 0x01;
        if(curr_bit){
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
            hdlc_LogOut("%d\tHDLC FRAME: ERROR (frame_length_bits%%8)!=0\r\n", offset);
            ///< @TODO: ctxt->offset = 0;
        }

        *err=INVALID_FR_LEN;
    }
}
#else /*!USE_BIT_STAFFING*/
void remove_byte_stuffing(uint8_t frame[], uint16_t byte_cnt, uint16_t *valid_bytes, hdlc_fr_detect_errors_e *err, int offset) {
    *valid_bytes = byte_cnt;
    ///< @TODO: Need realize .
}
#endif /*USE_BIT_STAFFING*/

/*
* runs CRC check
*/
int hdlc_CRC_match(uint8_t hdlc_frame[MAX_HDLC_FR_LEN], int frame_size)
{
    uint8_t hdlc_crc[2] = {0};

    if ((frame_size<5)||(frame_size>=MAX_HDLC_FR_LEN)) return 0;

    compute_crc16(hdlc_frame, frame_size - sizeof(hdlc_crc), hdlc_crc);

    hdlc_crc[0]    =    (~(hdlc_crc[0]));
    hdlc_crc[1]    =    (~(hdlc_crc[1]));

    if ((hdlc_crc[0]==hdlc_frame[frame_size-2])&&(hdlc_crc[1]==hdlc_frame[frame_size-1])) return 1;
    else return 0;
}

void funCb_HDLC_FrameTimeOut_DEF(void *instance_cb) {
    instance_cb = instance_cb;
}

/****************************************************************************/
/*                                   EOF                                    */
/****************************************************************************/


