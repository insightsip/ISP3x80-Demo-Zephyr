/******************************************************************************
 * @file    twr.c
 * @author  Insight SiP
 *
 * @attention
 *	THIS SOFTWARE IS PROVIDED BY INSIGHT SIP "AS IS" AND ANY EXPRESS
 *	OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *	OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *	DISCLAIMED. IN NO EVENT SHALL INSIGHT SIP OR CONTRIBUTORS BE
 *	LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *	CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 *	GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 *	OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#define MODULE twr
#include <app_event_manager.h>
#include <caf/events/module_state_event.h>
#include "twr_event.h"
#include "config_event.h"
#include "deca_device_api.h"
#include "dw_hw.h"
#include "dw_probe_interface.h"
#include <zephyr/drivers/regulator.h>

LOG_MODULE_REGISTER(MODULE);

#define THREAD_STACK_SIZE 1024
#define THREAD_PRIORITY 0
#define MSGQ_MAX_MSG 32

// Frame indexes
#define FC_IDX 0	  // Frame control index
#define SN_IDX 2	  // Sequence number index
#define PANID_IDX 3	  // PANID index
#define DEST_IDX 5	  // Destination address index
#define SRC_IDX 13	  // Source address index
#define MESGID_IDX 21 // Message ID index

// Simple 2WR function codes
#define SIMPLE_MSG_TAG_POLL (0x51)	// ISP Tag poll message
#define SIMPLE_MSG_ANCH_RESP (0x52) // ISP Anchor response to poll

// Function code byte offset (valid for all message types).
#define FCODE_POS 0		   // Function code is 1st byte of messageData
#define POLL_MSG_TOF_POS 1 // ToF is 2nd to 6th byte of messageData

// Simple anchor response byte offsets.
#define POLL_RX_TS 1 // Poll message reception timestamp(2)
#define RESP_TX_TS 5 // Response message transmission timestamp(2)

// Lengths including the ranging Message Function Code byte
#define SIMPLE_MSG_TAG_POLL_LEN 1		  // FunctionCode(1),
#define SIMPLE_MSG_TAG_POLL_WITH_RG_LEN 9 // FunctionCode(1), Range(8),
#define SIMPLE_MSG_ANCH_RESP_LEN 9		  // FunctionCode(1), poll message reception timestamp(4), response message transmission timestamp(4)

#define ADDR_BYTE_SIZE_L (8)
#define ADDR_BYTE_SIZE_S (2)

#define STANDARD_FRAME_SIZE 127
#define ADDR_BYTE_SIZE_L (8)
#define ADDR_BYTE_SIZE_S (2)
#define FRAME_CONTROL_BYTES 2
#define FRAME_SEQ_NUM_BYTES 1
#define FRAME_PANID 2
#define FRAME_CRC 2
#define FRAME_SOURCE_ADDRESS_S (ADDR_BYTE_SIZE_S)
#define FRAME_DEST_ADDRESS_S (ADDR_BYTE_SIZE_S)
#define FRAME_SOURCE_ADDRESS_L (ADDR_BYTE_SIZE_L)
#define FRAME_DEST_ADDRESS_L (ADDR_BYTE_SIZE_L)
#define FRAME_CTRLP (FRAME_CONTROL_BYTES + FRAME_SEQ_NUM_BYTES + FRAME_PANID)										 // 5
#define FRAME_CRTL_AND_ADDRESS_L (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_L + FRAME_CTRLP)						 // 21 bytes for 64-bit addresses
#define FRAME_CRTL_AND_ADDRESS_S (FRAME_DEST_ADDRESS_S + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP)						 // 9 bytes for 16-bit addresses
#define FRAME_CRTL_AND_ADDRESS_LS (FRAME_DEST_ADDRESS_L + FRAME_SOURCE_ADDRESS_S + FRAME_CTRLP)						 // 15 bytes for 1 16-bit address and 1 64-bit address)
#define MAX_USER_PAYLOAD_STRING_LL (STANDARD_FRAME_SIZE - FRAME_CRTL_AND_ADDRESS_L - TAG_FINAL_MSG_LEN - FRAME_CRC)	 // 127 - 21 - 16 - 2 = 88
#define MAX_USER_PAYLOAD_STRING_SS (STANDARD_FRAME_SIZE - FRAME_CRTL_AND_ADDRESS_S - TAG_FINAL_MSG_LEN - FRAME_CRC)	 // 127 - 9 - 16 - 2 = 100
#define MAX_USER_PAYLOAD_STRING_LS (STANDARD_FRAME_SIZE - FRAME_CRTL_AND_ADDRESS_LS - TAG_FINAL_MSG_LEN - FRAME_CRC) // 127 - 15 - 16 - 2 = 94

/* Delay between frames, in UWB microseconds. Should be fine tuned to optimize consumption */
#define POLL_RX_TO_RESP_TX_DLY_UUS 850 //650 /**< delay from Frame RX timestamp to TX reply timestamp. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 350 /**< Delay from the end of the frame transmission to the enable of the receiver. */
#define RESP_RX_TIMEOUT_UUS 600	 //400  /**< Receive response timeout. */

#define SPEED_OF_LIGHT 299702547 /**< Speed of light in air, in metres per second. */

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor. 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Default TX powers */
#define DEFAULT_CH5_PWR 0xf2f2f0f2 /**< Default TX power value for channel 5 */
#define DEFAULT_CH9_PWR 0xa2a2a0a2 /**< Default TX power value for channel 9 */
#define DEFAULT_CH5_PGDLY 0x34	   /**< Default PG Delay value for channel 5 */
#define DEFAULT_CH9_PGDLY 0x27	   /**< Default TX power value for channel 9 */

/* Default Antenna delays */
#define DEFAULT_TX_ANT_DLY 14445
#define DEFAULT_RX_ANT_DLY 18385

// OTP memory address list
#define OTP_EUID_ADDR_L 0x00	 /**< First 4 Bytes of 64 bit EUID OTP address */
#define OTP_EUID_ADDR_H 0x01	 /**< Last 4 Bytes of 64 bit EUID OTP address */
#define OTP_ALT_EUID_ADDR_L 0x02 /**< First 4 Bytes of alternative 64 bit EUID OTP address */
#define OTP_ALT_EUID_ADDR_H 0x03 /**< Last 4 Bytes of alternative 64 bit EUID OTP address */
// Reserved
#define OTP_CH5_ANT_DLY_ADDR 0x10 /**< Channel 5 antenna delay OTP address */
#define OTP_CH9_ANT_DLY_ADDR 0x11 /**< Channel 9 antenna delay OTP address */
#define OTP_CUST12_ADDR 0x12	  /**< Customer OTP address */
#define OTP_CUST13_ADDR 0x13	  /**< Customer OTP address */
#define OTP_CH5_PWR_ADDR 0x14	  /**< Channel 5 TX Power OTP address */
#define OTP_CH9_PWR_ADDR 0x15	  /**< Channel 9 TX Powe OTP address */
#define OTP_CUST16_ADDR 0x16	  /**< Customer OTP address */
#define OTP_CUST17_ADDR 0x17	  /**< Customer OTP address */
#define OTP_CUST18_ADDR 0x18	  /**< Customer OTP address */
#define OTP_CUST19_ADDR 0x19	  /**< Customer OTP address */
#define OTP_CUST1A_ADDR 0x1A	  /**< Customer OTP address */
#define OTP_CUST1B_ADDR 0x1B	  /**< Customer OTP address */
#define OTP_CUST1C_ADDR 0x1C	  /**< Customer OTP address */
#define OTP_CUST1D_ADDR 0x1D	  /**< Customer OTP address */
#define OTP_XTAL_ADDR 0x1E		  /**< XTAL trim OTP address */
#define OTP_REV_ADDR 0x1E		  /**< Revision OTP address */
#define OTP_MEMORY_MAX_ADDR 0x1F
#define EMPTY_OTP_VAL 0

/// @brief List of different actions the TWR Thread can execute
enum twr_action
{
	TWR_START,
	TWR_STOP,
	TWR_SEND_POLL_MSG,
	TWR_RCV_RESP_MSG,
	TWR_SEND_RESP_MSG
};

// simple 802.15.4 frame structure - using long addresses
typedef struct
{
	uint8_t frameCtrl[2];	 /**<  frame control bytes 00-01 */
	uint8_t seqNum;			 /**<  sequence_number 02 */
	uint8_t panID[2];		 /**<  PAN ID 03-04 */
	uint8_t destAddr[8];	 /**<  05-12 using 64 bit addresses */
	uint8_t sourceAddr[8];	 /**<  13-20 using 64 bit addresses */
	uint8_t messageData[88]; /**<  22-124 (application data and any user payload) */
	uint8_t fcs[2];			 /**<  125-126  we allow space for the CRC as it is logically part of the message. However ScenSor TX calculates and adds these bytes. */
} srd_msg_dlsl;

/// @brief Data passed to the msg queue
typedef struct
{
	uint8_t type;

} msgq_data_t;

static void twr_run(void *, void *, void *);

K_THREAD_DEFINE(twr_thread, THREAD_STACK_SIZE, twr_run, NULL, NULL, NULL, THREAD_PRIORITY, 0, 0);
K_SEM_DEFINE(twr_init_sem, 0, 1);
K_SEM_DEFINE(wakeup_sem, 0, 1);
char __aligned(4) twr_msgq_buffer[MSGQ_MAX_MSG * sizeof(msgq_data_t)];
static struct k_msgq twr_msgq;
static struct k_work_delayable delayed_uwb_tx_work;

static bool initialized = false;
static bool enabled = false;
#if IS_ENABLED(CONFIG_ISP_TWR_INITIATOR)
static twr_role_t twr_role = TWR_ROLE_INITIATOR;
static bool tx_rx_leds_enabled = false;
static bool sleep_enabled = true;
#elif IS_ENABLED(CONFIG_ISP_TWR_RESPONDER)
static twr_role_t twr_role = TWR_ROLE_RESPONDER;
static bool tx_rx_leds_enabled = true;
static bool sleep_enabled = false;
#else
#error "Unknown TWR role"
#endif
#if IS_ENABLED(CONFIG_ISP_TWR_FRAME_FILTERING)
static bool filter_enabled = true;
#else
static bool filter_enabled = false;
#endif
static bool is_uwb_sleeping = false;
static double last_range = -1.0;
static srd_msg_dlsl tx_poll_msg;
static srd_msg_dlsl tx_resp_msg;
static srd_msg_dlsl rx_msg;
static uint16_t rx_msg_len;
static uint8_t src_address[ADDR_BYTE_SIZE_L];  /**< Address of the device */
static uint8_t dest_address[ADDR_BYTE_SIZE_L]; /**< Address of the device to range with */
static uint8_t frame_sn;					   /**< Frame counter */
static uint32_t poll_tx_timestamp_u32, poll_rx_timestamp_u32, resp_tx_timestamp_u32, resp_rx_timestamp_u32;
static uint64_t poll_rx_timestamp_u64, resp_tx_timestamp_u64;
static uint32_t range_interval = 300; /**< Interval between ranging operations */
static uint16_t tx_ant_dly;
static uint16_t rx_ant_dly;

static dwt_config_t uwb_config = {
	5,				  /* Channel number. */
	DWT_PLEN_128,	  /* Preamble length. Used in TX only. */
	DWT_PAC8,		  /* Preamble acquisition chunk size. Used in RX only. */
	9,				  /* TX preamble code. Used in TX only. */
	9,				  /* RX preamble code. Used in RX only. */
	1,				  /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
	DWT_BR_6M8,		  /* Data rate. */
	DWT_PHRMODE_STD,  /* PHY header mode. */
	DWT_PHRRATE_STD,  /* PHY header rate. */
	(129 + 8 - 8),	  /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
	DWT_STS_MODE_OFF, /* STS enabled */
	DWT_STS_LEN_256,  /* Cipher length see allowed values in Enum dwt_sts_lengths_e */
	DWT_PDOA_M0		  /* PDOA mode 3 */
};

static dwt_txconfig_t tx_config = {
	DEFAULT_CH5_PGDLY, /* PG delay. */
	DEFAULT_CH5_PWR,   /* TX power. */
	0x0				   /* PG count. */
};

static uint64_t get_rx_timestamp_u64(void)
{
	uint8_t ts_tab[5];
	uint64_t ts = 0;
	int i;

	dwt_readrxtimestamp(ts_tab, DWT_COMPAT_NONE);
	for (i = 4; i > -1; i--)
	{
		ts <<= 8;
		ts |= ts_tab[i];
	}

	return ts;
}

static void resp_msg_get_ts(uint8_t *ts_field, uint32_t *ts)
{
	int i;

	*ts = 0;
	for (i = 0; i < 4; i++)
	{
		*ts += ts_field[i] << (i * 8);
	}
}

static void resp_msg_set_ts(uint8_t *ts_field, const uint64_t ts)
{
	int i;

	for (i = 0; i < 4; i++)
	{
		ts_field[i] = (ts >> (i * 8)) & 0xFF;
	}
}

/**
 * @brief Put a new item in the wifi thread msgq
 *
 * @param evt Structure with the information about the wifi evt
 */
static void msgq_put_item(msgq_data_t evt)
{
	while (k_msgq_put(&twr_msgq, &evt, K_NO_WAIT) != 0)
	{
		/* message queue is full: purge old data & try again */
		LOG_DBG("msgq full, purging...");
		k_msgq_purge(&twr_msgq);
	}
}

static uint32_t uwb_wake_up(void)
{
	if (sleep_enabled)
	{
		uint16_t wake_up_cpt = 0;

		dwt_wakeup_ic();
	
		k_sem_take(&wakeup_sem, K_MSEC(10));
	}

	return 0;
}

static void uwb_sleep(void)
{
	if (sleep_enabled)
	{
		dwt_entersleep(DWT_DW_IDLE_RC);
		is_uwb_sleeping = true;
	}
}

static void cb_tx_done(const dwt_cb_data_t *txd)
{
	LOG_DBG("cb_tx_done");
}

static void cb_rx_ok(const dwt_cb_data_t *rxd)
{
	LOG_DBG("cb_rx_ok");

	msgq_data_t evt;

	// Read Data Frame
	rx_msg_len = rxd->datalength;
	dwt_readrxdata((uint8_t *)&rx_msg, rx_msg_len, 0);

	// Check frame control bytes - must be 0x41dc
	if (rx_msg.frameCtrl[0] != 0x41 || rx_msg.frameCtrl[1] != 0xdc)
	{
		LOG_WRN("Unexpected frame control received");
		// unexpected frame control
		if (twr_role == TWR_ROLE_RESPONDER)
		{
			dwt_rxenable(DWT_START_RX_IMMEDIATE);
		}
		else
		{
			uwb_sleep();
		}
		return;
	}

	if (rx_msg.messageData[FCODE_POS] == SIMPLE_MSG_ANCH_RESP)
	{
		evt.type = TWR_RCV_RESP_MSG;
		msgq_put_item(evt);
	}
	else if (rx_msg.messageData[FCODE_POS] == SIMPLE_MSG_TAG_POLL)
	{
		evt.type = TWR_SEND_RESP_MSG;
		msgq_put_item(evt);
	}
}

static void cb_rx_to(const dwt_cb_data_t *rxd)
{
	LOG_DBG("cb_rx_to");

	if (twr_role == TWR_ROLE_INITIATOR)
	{
		uwb_sleep();
		// Submit error event
		struct twr_error_event *twr_error_event = new_twr_error_event();
		twr_error_event->code = TWR_RX_RESP_TIMEOUT;
		APP_EVENT_SUBMIT(twr_error_event);
	}
	else if (twr_role == TWR_ROLE_RESPONDER)
	{
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		// Submit error event
		struct twr_error_event *twr_error_event = new_twr_error_event();
		twr_error_event->code = TWR_RX_POLL_TIMEOUT;
		APP_EVENT_SUBMIT(twr_error_event);
	}
}

static void cb_rx_err(const dwt_cb_data_t *rxd)
{
	LOG_DBG("cb_rx_err");

	if (twr_role == TWR_ROLE_INITIATOR)
	{
		uwb_sleep();
		// Submit error event
		struct twr_error_event *twr_error_event = new_twr_error_event();
		twr_error_event->code = TWR_RX_RESP_FAILED;
		APP_EVENT_SUBMIT(twr_error_event);
	}
	else if (twr_role == TWR_ROLE_RESPONDER)
	{
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		// Submit error event
		struct twr_error_event *twr_error_event = new_twr_error_event();
		twr_error_event->code = TWR_RX_POLL_FAILED;
		APP_EVENT_SUBMIT(twr_error_event);
	}
}

static void cb_spi_ready(const dwt_cb_data_t *cb_data)
{
	LOG_DBG("cb_spi_ready");
	(void)cb_data;
	// Need to make sure DW IC is in IDLE_RC before proceeding
	while (!dwt_checkidlerc())
	{
	};

	// Restore the required configurations on wake
	dwt_restoreconfig(1);

	// Set EUI as it will not be preserved unless the EUI is programmed and loaded from NVM
	dwt_seteui(src_address);

	is_uwb_sleeping = false; // device is awake
	k_sem_give(&wakeup_sem);
}

static dwt_callbacks_s dwt_callbacks = {
	.cbRxOk = cb_rx_ok,
	.cbRxTo = cb_rx_to,
	.cbRxErr = cb_rx_err,
	.cbTxDone = cb_tx_done,
	.cbSPIRdy = cb_spi_ready};

static void delayed_tx_handler(struct k_work *item)
{
	msgq_data_t evt;
	evt.type = TWR_SEND_POLL_MSG;
	msgq_put_item(evt);

	k_work_reschedule(&delayed_uwb_tx_work, K_MSEC(range_interval));
}

/**
 * @brief Thread which handles most TWR operations
 */
void twr_run(void *p1, void *p2, void *p3)
{
	msgq_data_t evt;

	k_sem_take(&twr_init_sem, K_FOREVER);

	while (1)
	{
		k_msgq_get(&twr_msgq, &evt, K_FOREVER);

		switch (evt.type)
		{
		case TWR_START:
		{
			dw_hw_interrupt_enable();

			if (twr_role == TWR_ROLE_INITIATOR)
			{
				k_work_schedule(&delayed_uwb_tx_work, K_MSEC(range_interval));
			}
			else if (twr_role == TWR_ROLE_RESPONDER)
			{
				uwb_wake_up();
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
			}
		}
		break;

		case TWR_STOP:
		{
			dw_hw_interrupt_disable();

			if (twr_role == TWR_ROLE_INITIATOR)
			{
				k_work_cancel_delayable(&delayed_uwb_tx_work);
			}
			else if (twr_role == TWR_ROLE_RESPONDER)
			{
				dwt_forcetrxoff();
			}
			uwb_sleep();
		}
		break;

		case TWR_SEND_POLL_MSG:
		{
			uint16_t length;

			// Add last range result
			if (last_range != -1)
			{
				memcpy(&tx_poll_msg.messageData[POLL_MSG_TOF_POS], &last_range, 8);
				length = SIMPLE_MSG_TAG_POLL_WITH_RG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;
			}
			else
			{
				length = SIMPLE_MSG_TAG_POLL_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;
			}
			// wake up
			uwb_wake_up();

			// Write the frame data
			dwt_writetxdata(length, (uint8_t *)&tx_poll_msg, 0); /* Zero offset in TX buffer. */
			dwt_writetxfctrl(length, 0, 1);						 /* Zero offset in TX buffer, ranging. */

			// Set the delayed rx on time (the response message will be sent after this delay)
			dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
			dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);

			if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED))
			{
				LOG_ERR("dwt_starttx failed");
			}
			// Increment frame counter
			tx_poll_msg.seqNum = frame_sn++;
		}
		break;

		case TWR_RCV_RESP_MSG:
		{
			int32_t rtd_init, rtd_resp;
			float clock_offset_ratio;
			double tof;

			// Retrieve poll transmission and response reception timestamps.
			poll_rx_timestamp_u32 = dwt_readrxtimestamplo32(DWT_COMPAT_NONE);
			poll_tx_timestamp_u32 = dwt_readtxtimestamplo32();

			/* Read carrier integrator value and calculate clock offset ratio */
			clock_offset_ratio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);

			// Put uwb chip in sleep mode
			uwb_sleep();

			// Get timestamps embedded in response message
			resp_msg_get_ts(&rx_msg.messageData[POLL_RX_TS], &resp_rx_timestamp_u32);
			resp_msg_get_ts(&rx_msg.messageData[RESP_TX_TS], &resp_tx_timestamp_u32);

			/* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
			rtd_init = poll_rx_timestamp_u32 - poll_tx_timestamp_u32;
			rtd_resp = resp_tx_timestamp_u32 - resp_rx_timestamp_u32;
			tof = (((rtd_init - rtd_resp * (1.0F - clock_offset_ratio)) / 2.0F) * DWT_TIME_UNITS);
			last_range = tof * SPEED_OF_LIGHT;
			LOG_DBG("Range = %.02f m", last_range);

			// Submit event
			struct range_event *range_event = new_range_event();
			range_event->range = last_range;
			APP_EVENT_SUBMIT(range_event);
		}
		break;

		case TWR_SEND_RESP_MSG:
		{
			uint32_t resp_tx_time;
			int ret;

			/* Retrieve poll reception timestamp. */
			poll_rx_timestamp_u64 = get_rx_timestamp_u64();

			/* Compute response message transmission time. */
			resp_tx_time = (poll_rx_timestamp_u64 + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
			dwt_setdelayedtrxtime(resp_tx_time);

			/* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
			resp_tx_timestamp_u64 = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + tx_ant_dly;

			// Prepare and send ANCHOR RESP msg to TWR_INITIATOR
			tx_resp_msg.seqNum = frame_sn++;
			resp_msg_set_ts(&tx_resp_msg.messageData[POLL_RX_TS], poll_rx_timestamp_u64); // Poll message reception timestamp
			resp_msg_set_ts(&tx_resp_msg.messageData[RESP_TX_TS], resp_tx_timestamp_u64); // Response message transmission timestamp

			uint16_t length = SIMPLE_MSG_ANCH_RESP_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC;

			// Write the frame data
			dwt_writetxdata(length, (uint8_t *)&tx_resp_msg, 0); /* Zero offset in TX buffer. */
			dwt_writetxfctrl(length, 0, 1);						 /* Zero offset in TX buffer, ranging. */
			// Send frame
			ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
			if (ret == DWT_ERROR)
			{
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
				LOG_ERR("dwt_starttx failed");
			}

			// Now we can check if there is a valid Range in the payload
			if (rx_msg_len == (SIMPLE_MSG_TAG_POLL_WITH_RG_LEN + FRAME_CRTL_AND_ADDRESS_L + FRAME_CRC))
			{
				memcpy(&last_range, &rx_msg.messageData[POLL_MSG_TOF_POS], 8);

				// Generate data event
				if (last_range != -1)
				{
					LOG_INF("Range = %.02f m", last_range);

					// Submit event
					struct range_event *range_event = new_range_event();
					range_event->range = last_range;
					APP_EVENT_SUBMIT(range_event);
				}
			}
		}
		break;

		default:
			break;
		}
	}
}

static void twr_start(void)
{
	msgq_data_t evt;
	evt.type = TWR_START;
	msgq_put_item(evt);

}

static void twr_stop(void)
{
	msgq_data_t evt;
	evt.type = TWR_STOP;
	msgq_put_item(evt);
}

static int twr_init(void)
{
	dw_hw_init();
	dw_hw_reset();
	k_msleep(2);

	if (dw_probe_interface_init())
	{
		return -1;
	}

	while (!dwt_checkidlerc())
	{
	};

	if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
	{
		LOG_ERR("dwt_initialise failed");
		return -1;
	}

	dw_spi_fastrate_set();

	// Configure uwb interrupts
	dwt_setinterrupt(DWT_INT_ARFE_BIT_MASK | DWT_INT_TXFRS_BIT_MASK | DWT_INT_RXFCG_BIT_MASK | DWT_INT_RXFTO_BIT_MASK |
						 DWT_INT_RXPTO_BIT_MASK | DWT_INT_RXPHE_BIT_MASK | DWT_INT_RXFCE_BIT_MASK | DWT_INT_RXFSL_BIT_MASK |
						 DWT_INT_RXSTO_BIT_MASK | DWT_INT_SPIRDY_BIT_MASK,
					 0,
					 DWT_ENABLE_INT);
	dwt_setcallbacks(&dwt_callbacks);

	// Configure uwb signal
	if (dwt_configure(&uwb_config))
	{
		LOG_ERR("dwt_configure failed");
		return -1;
	}

	// Read OTP memory
	uint32_t otp_memory[OTP_MEMORY_MAX_ADDR];
	dwt_otpread(OTP_EUID_ADDR_L, otp_memory, OTP_MEMORY_MAX_ADDR);

	// Configure uwb tx power & pulse shape
	if (uwb_config.chan == 5)
	{
		tx_config.power = otp_memory[OTP_CH5_PWR_ADDR] != EMPTY_OTP_VAL ? otp_memory[OTP_CH5_PWR_ADDR] : DEFAULT_CH5_PWR;
		tx_config.PGdly = DEFAULT_CH5_PGDLY;
		dwt_configuretxrf(&tx_config);
		dwt_set_alternative_pulse_shape(0);
	}
	else if (uwb_config.chan == 9)
	{
		tx_config.power = otp_memory[OTP_CH9_PWR_ADDR] != EMPTY_OTP_VAL ? otp_memory[OTP_CH9_PWR_ADDR] : DEFAULT_CH9_PWR;
		tx_config.PGdly = DEFAULT_CH9_PGDLY;
		dwt_configuretxrf(&tx_config);
		dwt_set_alternative_pulse_shape(1);
	}
	else
	{
		LOG_ERR("dwt_configuretxrf failed");
		return -1;
	}
	LOG_DBG("UWB Tx power: %x", tx_config.power);

	// Configure antenna delays
	if (uwb_config.chan == 5)
	{
		rx_ant_dly = otp_memory[OTP_CH5_ANT_DLY_ADDR] != EMPTY_OTP_VAL ? (otp_memory[OTP_CH5_ANT_DLY_ADDR] >> 16) & 0xFFFF : DEFAULT_RX_ANT_DLY;
		tx_ant_dly = otp_memory[OTP_CH5_ANT_DLY_ADDR] != EMPTY_OTP_VAL ? otp_memory[OTP_CH5_ANT_DLY_ADDR] & 0xFFFF : DEFAULT_TX_ANT_DLY;
		dwt_setrxantennadelay(rx_ant_dly);
		dwt_settxantennadelay(tx_ant_dly);
	}
	else if (uwb_config.chan == 9)
	{
		rx_ant_dly = otp_memory[OTP_CH9_ANT_DLY_ADDR] != EMPTY_OTP_VAL ? (otp_memory[OTP_CH9_ANT_DLY_ADDR] >> 16) & 0xFFFF : DEFAULT_RX_ANT_DLY;
		tx_ant_dly = otp_memory[OTP_CH9_ANT_DLY_ADDR] != EMPTY_OTP_VAL ? otp_memory[OTP_CH9_ANT_DLY_ADDR] & 0xFFFF : DEFAULT_TX_ANT_DLY;
		dwt_setrxantennadelay(rx_ant_dly);
		dwt_settxantennadelay(tx_ant_dly);
	}
	else
	{
		LOG_ERR("dwt_setrxantennadelay/dwt_settxantennadelay failed");
		return -1;
	}
	LOG_DBG("UWB Rx Ant delay: 0x%08x, Tx Ant delay 0x%08x", rx_ant_dly, tx_ant_dly);

	// Configure filter
	if (filter_enabled)
	{
		dwt_setpanid(0xdeca);
		dwt_seteui(src_address);
		dwt_configureframefilter(DWT_FF_ENABLE_802_15_4, DWT_FF_DATA_EN);
		LOG_DBG("UWB Filter enabled");
	}

	// Configure LEDs
	if (tx_rx_leds_enabled)
	{
		dwt_setleds(0x3);
		LOG_DBG("UWB TX/RX LEDs enabled");
	}

	// Configure sleep mode
	if (sleep_enabled)
	{
		dwt_configuresleep(DWT_CONFIG | DWT_PGFCAL, DWT_SLP_EN | DWT_WAKE_WUP | DWT_PRES_SLEEP);
		dwt_entersleep(DWT_DW_IDLE_RC);
		LOG_DBG("UWB Sleep enabled");
	}

	// Pre fill initiator message
	tx_poll_msg.frameCtrl[0] = 0x41;
	tx_poll_msg.frameCtrl[1] = 0xdc;
	tx_poll_msg.seqNum = 0;
	tx_poll_msg.panID[0] = (0xdeca) & 0xff;
	tx_poll_msg.panID[1] = (0xdeca) >> 8;
	memcpy(&tx_poll_msg.sourceAddr[0], src_address, ADDR_BYTE_SIZE_L);
	memcpy(&tx_poll_msg.destAddr[0], dest_address, ADDR_BYTE_SIZE_L);
	tx_poll_msg.messageData[FCODE_POS] = SIMPLE_MSG_TAG_POLL;

	// Pre fill responder message
	tx_resp_msg.frameCtrl[0] = 0x41 /*frame type 0x1 == data*/ /*PID comp*/;
	tx_resp_msg.frameCtrl[1] = 0xdc /*dest extended address (64bits)*/ /*src extended address (64bits)*/;
	tx_resp_msg.seqNum = 0;
	tx_resp_msg.panID[0] = (0xdeca) & 0xff;
	tx_resp_msg.panID[1] = (0xdeca) >> 8;
	memcpy(&tx_resp_msg.sourceAddr[0], src_address, ADDR_BYTE_SIZE_L);
	memcpy(&tx_resp_msg.destAddr[0], dest_address, ADDR_BYTE_SIZE_L);
	tx_resp_msg.messageData[FCODE_POS] = SIMPLE_MSG_ANCH_RESP;

	// Create delayed tx timer
	k_work_init_delayable(&delayed_uwb_tx_work, delayed_tx_handler);

	// Create message queue
	k_msgq_init(&twr_msgq, twr_msgq_buffer, sizeof(msgq_data_t), MSGQ_MAX_MSG);

	// Tag boards have dedicated supply circuit
#if DT_NODE_EXISTS(DT_NODELABEL(curr_mode_pin))
	const struct device *curr_mode_dev = DEVICE_DT_GET(DT_NODELABEL(curr_mode_pin));
	if (!device_is_ready(curr_mode_dev))
	{
		return -ENODEV;
	}
	regulator_enable(curr_mode_dev);
#endif

	// Start TWR operations
	k_sem_give(&twr_init_sem);
	twr_start();

	return 0;
}

static bool app_event_handler(const struct app_event_header *aeh)
{
	if (is_module_state_event(aeh))
	{
		const struct module_state_event *event = cast_module_state_event(aeh);

		if (check_state(event, MODULE_ID(main), MODULE_STATE_READY))
		{
			__ASSERT_NO_MSG(!initialized);
			initialized = true;

			if (twr_init())
			{
				LOG_ERR("twr_init failed");
				module_set_state(MODULE_STATE_ERROR);
			}
			else
			{
				module_set_state(MODULE_STATE_READY);
			}
		}

		return false;
	}

	if (is_config_twr_event(aeh))
	{
		struct config_twr_event *event = cast_config_twr_event(aeh);
		if (event->enable)
		{
			twr_role = event->role;
			twr_start();
			enabled = true;
		}
		else
		{
			// ignore event->role in this case
			twr_stop();
			enabled = false;
		}
		return false;
	}

	if (is_config_twr_interval_event(aeh))
	{
		struct config_twr_interval_event *event = cast_config_twr_interval_event(aeh);

		range_interval = event->interval;

		if (enabled)
		{
			k_work_reschedule(&delayed_uwb_tx_work, K_MSEC(range_interval));
		}

		return false;
	}

	/* If event is unhandled, unsubscribe. */
	__ASSERT_NO_MSG(false);

	return false;
}

APP_EVENT_LISTENER(MODULE, app_event_handler);
APP_EVENT_SUBSCRIBE_FINAL(MODULE, module_state_event);
APP_EVENT_SUBSCRIBE(MODULE, twr_config_event);