/******************************************************************************
 * @file    dfu_module.c
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

#include "dfu_module.h"


LOG_MODULE_REGISTER(dfu);

struct mgmt_callback callback_dfu_started;
struct mgmt_callback callback_dfu_stopped;
struct mgmt_callback callback_dfu_pending;

static void dfu_started_cb(uint32_t event, enum mgmt_cb_return prev_status,
                                int32_t *rc, uint16_t *group, bool *abort_more,
                                void *data, size_t data_size)
{
	LOG_INF("DFU Started");

	struct dfu_ctrl_event *dfu_ctrl_event = new_dfu_ctrl_event();
	dfu_ctrl_event->op_enable = false;
	APP_EVENT_SUBMIT(dfu_ctrl_event);

	// wait a bit for uwb activities to stop
	k_msleep(100);
}

static void dfu_stopped_cb(uint32_t event, enum mgmt_cb_return prev_status,
                                int32_t *rc, uint16_t *group, bool *abort_more,
                                void *data, size_t data_size)
{
	LOG_INF("DFU Stopped");

	// Now that current control pin is set to 1, we can resume uwb activities
	struct dfu_ctrl_event *dfu_ctrl_event = new_dfu_ctrl_event();
	dfu_ctrl_event->op_enable = true;
	APP_EVENT_SUBMIT(dfu_ctrl_event);
}

static void dfu_pending_cb(uint32_t event, enum mgmt_cb_return prev_status,
                                int32_t *rc, uint16_t *group, bool *abort_more,
                                void *data, size_t data_size)
{
	LOG_INF("DFU Pending");
}

int dfu_module_init(void)
{
	int err = 0;

    callback_dfu_started.callback = dfu_started_cb;
    callback_dfu_started.event_id = MGMT_EVT_OP_IMG_MGMT_DFU_STARTED;
    mgmt_callback_register(&callback_dfu_started);

    callback_dfu_stopped.callback = dfu_stopped_cb;
    callback_dfu_stopped.event_id = MGMT_EVT_OP_IMG_MGMT_DFU_STOPPED;
    mgmt_callback_register(&callback_dfu_stopped);

	callback_dfu_pending.callback = dfu_pending_cb;
    callback_dfu_pending.event_id = MGMT_EVT_OP_IMG_MGMT_DFU_PENDING;
    mgmt_callback_register(&callback_dfu_pending);

	return err;
}
