/******************************************************************************
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * @file    dfu_module_event.c
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
#include <stdio.h>

#include "dfu_module_event.h"


static void log_dfu_ctrl_event(const struct app_event_header *aeh)
{
	const struct dfu_ctrl_event *event = cast_dfu_ctrl_event(aeh);
	if (event->op_enable) 
	{
		APP_EVENT_MANAGER_LOG(aeh, "enable UWB op");
	}
	else 
	{
		APP_EVENT_MANAGER_LOG(aeh, "disable UWB op");
	}
}

APP_EVENT_TYPE_DEFINE(dfu_ctrl_event, log_dfu_ctrl_event, NULL, APP_EVENT_FLAGS_CREATE());

