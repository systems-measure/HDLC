#ifndef HDLC_H
#define HDLC_H

/*
* HDLC Frame Detection Refer ITU-T Rec Q.921 $2
*/

/****************************************************************************/
/*                              MODULES USED                                */
/****************************************************************************/

/****************************************************************************/
/*                          DEFINITIONS AND MACROS                          */
/****************************************************************************/

#define TRUE                        1
#define FALSE                       0

#define MIN_HDLC_FR_LEN             5            /* Minimum HDLC frame length Allowed */
#define MAX_HDLC_FR_LEN             900            /* Maximum HDLC frame length Allowed */
#define MAX_HDLC_FR_LEN_WITH_STUF   (MAX_HDLC_FR_LEN*(4/3))    /* HDLC frame length + Stuffing bits */
#define USE_CRC16_X25               1

/****************************************************************************/
/*                          TYPEDEFS AND STRUCTURES                         */
/****************************************************************************/
typedef void(*FunCb_HDLC_RecieverFrame)(void *instance_cb, uint8_t *fr, uint16_t len);
typedef void(*FunCb_HDLC_FrameTimeOut)(void *instance_cb);

/* HDLC FSM States */
typedef enum hdlc_fr_detect_states
{
    FLAG_SEARCH = 6543,            /* looking for flag state */
    FLAG_SYNC_EST,                /* flag sync established state */
    FRAME_RX,                /* receiving frame state */
    MAX_HDLC_FR_DETECT_STATE
}hdlc_fr_detect_states_e;

/* HDLC Error Codes */
typedef enum hdlc_fr_detect_errors
{
    NO_ERR = 55555,
    INVALID_FR_LEN,
    CRC_MISMATCH,
    MAX_HDLC_FR_DETECT_ERRORS
}hdlc_fr_detect_errors_e;

/* context for hdlc CallBack function */
typedef struct hdlc_callback {
    void*                       instance_cb;
    FunCb_HDLC_RecieverFrame    cb_RecieverFrame;
    FunCb_HDLC_FrameTimeOut     cb_ResetFrameTimeOut;
    FunCb_HDLC_FrameTimeOut     cb_StopFrameTimeOut;
}hdlc_callback_t;
/* context for hdlc link */
typedef struct hdlc_ch_ctxt
{
    hdlc_fr_detect_states_e    state;                       /* HDLC frame detection state */
    hdlc_fr_detect_errors_e    err_type;                    /* error type received */        
    uint16_t        fr_bit_cnt;                             /* number of bits received between opening and closing flags (working copy)*/
    uint8_t         flag_pos_ctr;                           /* counter for flag matching index in flag_lookup table */
    uint8_t         rec_bits;                               /* stores last 8 bits received */
    uint8_t         frame_ready;                            /* flag to signal a ready frame */
    uint8_t         frame[MAX_HDLC_FR_LEN_WITH_STUF];       /* stores frame received (working copy)*/
    hdlc_callback_t callback;
}hdlc_ch_ctxt_t;

/****************************************************************************/
/*                             EXPORTED VARIABLES                           */
/****************************************************************************/

#ifndef HDLC_C
#endif //HDLC_C
/****************************************************************************/
/*                             PRIVATE VARIABLES                           */
/****************************************************************************/
#ifdef HDLC_PRIVATE
    void funCb_HDLC_FrameTimeOut_DEF(void *instance_cb);
#endif //HDLC_PRIVATE
/*
* HDLC frame detection algorithm.
*
* inp8        - Byte from HDLC input Stream.
* offset    - Stream Offset (Application Specific)
* hdlc_ch_ctxt    - HDLC Link Context
*/
void HDLC(uint8_t inp8, int offset, hdlc_ch_ctxt_t *hdlc_ch_ctxt);

/*
* Intializes HDLC frame detection algorithm context.
*
* @param hdlc_ch_ctxt   - HDLC Link Context
* @param callback       - Application Callback 
* @return               - Function for resetting FRAME detection
*/
void HDLC_init(hdlc_ch_ctxt_t *hdlc_ch_ctxt, hdlc_callback_t *callback);

/*
* Reset HDLC frame detection.
*
* @param hdlc_ch_ctxt   - HDLC Link Context
*/
void HDLC_reset(hdlc_ch_ctxt_t *ctxt);


/****************************************************************************/
/*                             VIRTUAL FUNCTIONS                            */
/****************************************************************************/
#ifndef hdlc_LogOut
    #define hdlc_LogOut(...)   { printf(__VA_ARGS__); }
#endif // !hdlc_LogOut

#endif //HDLC_H
/****************************************************************************/
/*                                   EOF                                    */
/****************************************************************************/


