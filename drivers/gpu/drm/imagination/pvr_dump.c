// SPDX-License-Identifier: GPL-2.0 OR MIT
/* Copyright (c) 2026 Imagination Technologies Ltd. */

#include "pvr_context.h"
#include "pvr_device.h"
#include "pvr_dump.h"
#include "pvr_rogue_cr_defs.h"
#include "pvr_rogue_fwif.h"
#include "pvr_vm.h"

#include <drm/drm_print.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/types.h>

static const char *
get_reset_reason_desc(enum rogue_context_reset_reason reason)
{
	switch (reason) {
	case ROGUE_CONTEXT_RESET_REASON_NONE:
		return "None";
	case ROGUE_CONTEXT_RESET_REASON_GUILTY_LOCKUP:
		return "Guilty lockup";
	case ROGUE_CONTEXT_RESET_REASON_INNOCENT_LOCKUP:
		return "Innocent lockup";
	case ROGUE_CONTEXT_RESET_REASON_GUILTY_OVERRUNING:
		return "Guilty overrunning";
	case ROGUE_CONTEXT_RESET_REASON_INNOCENT_OVERRUNING:
		return "Innocent overrunning";
	case ROGUE_CONTEXT_RESET_REASON_HARD_CONTEXT_SWITCH:
		return "Hard context switch";
	case ROGUE_CONTEXT_RESET_REASON_WGP_CHECKSUM:
		return "CDM Mission/safety checksum mismatch";
	case ROGUE_CONTEXT_RESET_REASON_TRP_CHECKSUM:
		return "TRP checksum mismatch";
	case ROGUE_CONTEXT_RESET_REASON_GPU_ECC_OK:
		return "GPU ECC error (corrected, OK)";
	case ROGUE_CONTEXT_RESET_REASON_GPU_ECC_HWR:
		return "GPU ECC error (uncorrected, HWR)";
	case ROGUE_CONTEXT_RESET_REASON_FW_ECC_OK:
		return "Firmware ECC error (corrected, OK)";
	case ROGUE_CONTEXT_RESET_REASON_FW_ECC_ERR:
		return "Firmware ECC error (uncorrected, ERR)";
	case ROGUE_CONTEXT_RESET_REASON_FW_WATCHDOG:
		return "Firmware watchdog";
	case ROGUE_CONTEXT_RESET_REASON_FW_PAGEFAULT:
		return "Firmware pagefault";
	case ROGUE_CONTEXT_RESET_REASON_FW_EXEC_ERR:
		return "Firmware execution error";
	case ROGUE_CONTEXT_RESET_REASON_HOST_WDG_FW_ERR:
		return "Host watchdog";
	case ROGUE_CONTEXT_GEOM_OOM_DISABLED:
		return "Geometry OOM disabled";

	default:
		return "Unknown";
	}
}

static const char *
get_dm_name(u32 dm)
{
	switch (dm) {
	case PVR_FWIF_DM_GP:
		return "General purpose";
	/* PVR_FWIF_DM_TDM has the same index, but is discriminated by a device feature */
	case PVR_FWIF_DM_2D:
		return "2D or TDM";
	case PVR_FWIF_DM_GEOM:
		return "Geometry";
	case PVR_FWIF_DM_FRAG:
		return "Fragment";
	case PVR_FWIF_DM_CDM:
		return "Compute";
	case PVR_FWIF_DM_RAY:
		return "Raytracing";
	case PVR_FWIF_DM_GEOM2:
		return "Geometry 2";
	case PVR_FWIF_DM_GEOM3:
		return "Geometry 3";
	case PVR_FWIF_DM_GEOM4:
		return "Geometry 4";

	default:
		return "Unknown";
	}
}

/**
 * pvr_dump_context_reset_notification() - Handle context reset notification from FW
 * @pvr_dev: Device pointer.
 * @data: Data provided by FW.
 *
 * This will decode the data structure provided by FW and print the results via drm_info().
 */
void
pvr_dump_context_reset_notification(struct pvr_device *pvr_dev,
				    struct rogue_fwif_fwccb_cmd_context_reset_data *data)
{
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);

	if (data->flags & ROGUE_FWIF_FWCCB_CMD_CONTEXT_RESET_FLAG_ALL_CTXS) {
		drm_info(drm_dev, "Received context reset notification for all contexts\n");
	} else {
		drm_info(drm_dev, "Received context reset notification on context %u\n",
			 data->server_common_context_id);
	}

	if (!(data->flags & ROGUE_FWIF_FWCCB_CMD_CONTEXT_RESET_FLAG_ALL_CTXS)) {
		struct pvr_context *ctx;
		pid_t pid = -1;
		char comm[TASK_COMM_LEN] = "?";

		/* Contexts leave ctx_ids before they are freed (LOCAL debug). */
		xa_lock(&pvr_dev->ctx_ids);
		ctx = xa_load(&pvr_dev->ctx_ids, data->server_common_context_id);
		if (ctx) {
			pid = ctx->owner_pid;
			strscpy(comm, ctx->owner_comm);
		}
		xa_unlock(&pvr_dev->ctx_ids);
		drm_info(drm_dev, "  Context owner=%s[%d]%s\n", comm, pid,
			 ctx ? "" : " (context already gone)");
	}

	drm_info(drm_dev, "  Reset reason=%u (%s)\n", data->reset_reason,
		 get_reset_reason_desc((enum rogue_context_reset_reason)data->reset_reason));
	drm_info(drm_dev, "  Data Master=%u (%s)\n", data->dm, get_dm_name(data->dm));
	drm_info(drm_dev, "  Job ref=%u\n", data->reset_job_ref);

	if (data->flags & ROGUE_FWIF_FWCCB_CMD_CONTEXT_RESET_FLAG_PF) {
		drm_info(drm_dev, "  Page fault occurred, fault address=%llx\n",
			 data->fault_address);
	}
}

/* LOCAL debug: the firmware's HWR log (struct rogue_fwif_hwrinfobuf). */

static DEFINE_MUTEX(pvr_hwrinfo_log_lock);

static const char *
get_hwr_type_name(u32 type)
{
	static const char * const names[] = {
		"unknown", "overrun", "poll failure", "BIF0 fault", "BIF1 fault",
		"TEXAS BIF0 fault", "MMU fault", "MMU META fault", "MIPS TLB fault",
		"ECC fault", "MMU RISC-V fault",
	};

	return type < ARRAY_SIZE(names) ? names[type] : "?";
}

static void
pvr_hwrinfo_format(const struct rogue_hwrinfo *h, char *buf, size_t len)
{
	int n;

	n = scnprintf(buf, len,
		      "HWR #%u: %s, DM %u (%s), pid %u, frame %u, hwrt 0x%x, event 0x%x, recovery 0x%x, cr_timer %llu",
		      h->hwr_number, get_hwr_type_name(h->hwr_type), h->dm,
		      get_dm_name(h->dm), h->pid, h->frame_num, h->active_hwrt_data,
		      h->event_status, h->hwr_recovery_flags, h->cr_timer);

	switch (h->hwr_type) {
	case ROGUE_HWRTYPE_BIF0FAULT:
	case ROGUE_HWRTYPE_BIF1FAULT:
	case ROGUE_HWRTYPE_TEXASBIF0FAULT: {
		u64 req = h->hwr_data.bif_info.bif_req_status;
		u64 mmu = h->hwr_data.bif_info.bif_mmu_status;

		scnprintf(buf + n, len - n,
			  "; req 0x%llx (addr 0x%llx %s tag_sb %llu tag_id %llu) mmu 0x%llx (cat_base %llu data_type %llu%s%s) pc 0x%llx",
			  req, req & ~ROGUE_CR_BIF_FAULT_BANK0_REQ_STATUS_ADDRESS_CLRMSK,
			  (req & ROGUE_CR_BIF_FAULT_BANK0_REQ_STATUS_RNW_EN) ? "read" : "write",
			  (req >> ROGUE_CR_BIF_FAULT_BANK0_REQ_STATUS_TAG_SB_SHIFT) & 0x1f,
			  (req >> ROGUE_CR_BIF_FAULT_BANK0_REQ_STATUS_TAG_ID_SHIFT) & 0xf,
			  mmu, (mmu >> ROGUE_CR_BIF_FAULT_BANK0_MMU_STATUS_CAT_BASE_SHIFT) & 0xf,
			  (mmu >> ROGUE_CR_BIF_FAULT_BANK0_MMU_STATUS_DATA_TYPE_SHIFT) & 0x3,
			  (mmu & ROGUE_CR_BIF_FAULT_BANK0_MMU_STATUS_FAULT_RO_EN) ? " ro" : "",
			  (mmu & ROGUE_CR_BIF_FAULT_BANK0_MMU_STATUS_FAULT_PM_META_RO_EN) ? " pm_meta_ro" : "",
			  h->hwr_data.bif_info.pc_address);
		break;
	}
	case ROGUE_HWRTYPE_MMUFAULT:
	case ROGUE_HWRTYPE_MMUMETAFAULT:
		scnprintf(buf + n, len - n, "; mmu_status 0x%llx 0x%llx pc 0x%llx",
			  h->hwr_data.mmu_info.mmu_status[0],
			  h->hwr_data.mmu_info.mmu_status[1],
			  h->hwr_data.mmu_info.pc_address);
		break;
	case ROGUE_HWRTYPE_POLLFAILURE:
		scnprintf(buf + n, len - n, "; poll thread %u addr 0x%x mask 0x%x last 0x%x",
			  h->hwr_data.poll_info.thread_num, h->hwr_data.poll_info.cr_poll_addr,
			  h->hwr_data.poll_info.cr_poll_mask,
			  h->hwr_data.poll_info.cr_poll_last_value);
		break;
	default:
		break;
	}
}

static bool
pvr_hwrinfo_used(const struct rogue_hwrinfo *h)
{
	return h->cr_timer || h->os_timer || h->hwr_number;
}

/* The process whose VM has page catalogue @pc, if any (LOCAL debug). */
static void
pvr_pc_owner(struct pvr_device *pvr_dev, u64 pc, char *comm, pid_t *pid)
{
	struct pvr_context *ctx;
	unsigned long id;

	strscpy(comm, "?", TASK_COMM_LEN);
	*pid = -1;
	xa_lock(&pvr_dev->ctx_ids);
	xa_for_each(&pvr_dev->ctx_ids, id, ctx) {
		if (ctx->vm_ctx && pvr_vm_get_page_table_root_addr(ctx->vm_ctx) == pc) {
			strscpy(comm, ctx->owner_comm, TASK_COMM_LEN);
			*pid = ctx->owner_pid;
			break;
		}
	}
	xa_unlock(&pvr_dev->ctx_ids);
}

/**
 * pvr_dump_hwrinfo() - Log the firmware HWR records not logged yet (LOCAL debug)
 * @pvr_dev: Device pointer.
 * @why: What made us look.
 */
void
pvr_dump_hwrinfo(struct pvr_device *pvr_dev, const char *why)
{
	struct pvr_fw_device *fw_dev = &pvr_dev->fw_dev;
	struct rogue_fwif_hwrinfobuf *buf = fw_dev->hwrinfobuf;
	struct drm_device *drm_dev = from_pvr_device(pvr_dev);
	char *line;

	BUILD_BUG_ON(ARRAY_SIZE(fw_dev->hwr_logged_timer) != ROGUE_FWIF_HWINFO_MAX);
	if (!buf)
		return;

	line = kmalloc(512, GFP_KERNEL);
	if (!line)
		return;

	mutex_lock(&pvr_hwrinfo_log_lock);
	for (u32 i = 0; i < ROGUE_FWIF_HWINFO_MAX; i++) {
		struct rogue_hwrinfo h;
		char comm[TASK_COMM_LEN];
		pid_t pid = -1;

		memcpy(&h, &buf->hwr_info[i], sizeof(h));
		if (!pvr_hwrinfo_used(&h) || h.cr_timer == fw_dev->hwr_logged_timer[i])
			continue;
		fw_dev->hwr_logged_timer[i] = h.cr_timer;
		pvr_hwrinfo_format(&h, line, 512);
		comm[0] = 0;
		if (ROGUE_FWIF_HWRTYPE_PAGE_FAULT_GET(h.hwr_type) &&
		    (h.hwr_type == ROGUE_HWRTYPE_BIF0FAULT ||
		     h.hwr_type == ROGUE_HWRTYPE_BIF1FAULT ||
		     h.hwr_type == ROGUE_HWRTYPE_TEXASBIF0FAULT))
			pvr_pc_owner(pvr_dev, h.hwr_data.bif_info.pc_address, comm, &pid);
		if (comm[0])
			drm_info(drm_dev, "  [%s] slot %u %s, pc owner %s[%d]\n", why, i, line,
				 comm, pid);
		else
			drm_info(drm_dev, "  [%s] slot %u %s\n", why, i, line);
	}
	mutex_unlock(&pvr_hwrinfo_log_lock);
	kfree(line);
}

static int
pvr_hwrinfo_show(struct seq_file *m, void *unused)
{
	struct pvr_device *pvr_dev = m->private;
	struct rogue_fwif_hwrinfobuf *buf = pvr_dev->fw_dev.hwrinfobuf;
	char *line;

	if (!buf)
		return -ENODEV;

	line = kmalloc(512, GFP_KERNEL);
	if (!line)
		return -ENOMEM;

	seq_printf(m, "hwr_counter %u write_index %u dd_req_count %u flags 0x%x\n",
		   buf->hwr_counter, buf->write_index, buf->dd_req_count,
		   buf->hwr_info_buf_flags);
	for (u32 dm = 0; dm < PVR_FWIF_DM_MAX; dm++)
		seq_printf(m, "DM %u (%s): locked_up %u overran %u recovered %u false_detect %u\n",
			   dm, get_dm_name(dm), buf->hwr_dm_locked_up_count[dm],
			   buf->hwr_dm_overran_count[dm], buf->hwr_dm_recovered_count[dm],
			   buf->hwr_dm_false_detect_count[dm]);
	for (u32 i = 0; i < ROGUE_FWIF_HWINFO_MAX; i++) {
		struct rogue_hwrinfo h;

		memcpy(&h, &buf->hwr_info[i], sizeof(h));
		if (!pvr_hwrinfo_used(&h))
			continue;
		pvr_hwrinfo_format(&h, line, 512);
		seq_printf(m, "[%u] %s\n", i, line);
	}
	kfree(line);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(pvr_hwrinfo);

void
pvr_hwrinfo_debugfs_init(struct pvr_device *pvr_dev, struct dentry *dir)
{
	debugfs_create_file("hwrinfo", 0400, dir, pvr_dev, &pvr_hwrinfo_fops);
}
