/*
 * Data mover design example
 *
 * Copyright Altera Corporation (C) 2014. All rights reserved
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/mailbox.h>
#include <linux/mailbox_client.h>
#include <linux/kernel.h>
#include <linux/kthread.h>

/* Define physically address*/
#define	OCRAM_ADDR			0xFFFF0000
#define	PERF_COUNTER_HPS_ADDR		0xFF200300
#define	PERF_COUNTER_NII_ADDR		0xFF200000
#define SGDMA_NII_CSR_ADDR		0xFF200080
#define SGDMA_NII_DES_ADDR		0xFF2000a0
#define SGDMA_HPS_CSR_ADDR		0xFF2000c0
#define SGDMA_HPS_DES_ADDR		0xFF2000e0

/* Define IRQ number */
#define	SGDMA_NII_IRQ			0x48

/* Define register offset and bit mask */
#define	SGDMA_DES_CONTROL_OFFSET	0x2c
#define	INGRESS_STOP_OFFSET		0x10
#define	REGRESS_START_OFFSET		0x24
#define	CMD_SIZE_MSK			0x3FFFFFFF
#define	CMD_MODE_MSK			0xC0000000

/* Define parameter */
#define	SGDMA_GO_CMD			0x80004000
#define MAGIC_WORD			0xF0F0F0F0
#define	OCRAM_SIZE			0x4000
#define SGDMA_SIZE			0x40
#define PERF_SIZE			0x80
#define CPU_ID				0x1

/* Define macros */
#define GET_CMD_SIZE(_value)		(_value & CMD_SIZE_MSK)
#define GET_CMD_MODE(_value)		((_value & CMD_MODE_MSK) >> 30)
#define PERF_BEGIN(p, n)		writel(0, (p) + ((((n)*4)+1)*4))
#define PERF_END(p, n)			writel(0, (p) + ((((n)*4))*4))
#define PERF_RESET(p)			writel(1, (p))
#define PERF_START_MEASURING(p)		PERF_BEGIN((p), 0)
#define GET_PERF_RESULT(p, n)		readl((p) + ((((n)*4))*4))

struct dm_struct {
	void __iomem *perfcounter_nii;
	void __iomem *perfcounter_hps;
	void __iomem *msgdma_nii;
	void __iomem *msgdma_hps;
	void __iomem *acp_sdram;
	void __iomem *perfcounter_node;
	void __iomem *ocram;
	int regress_address;
	int ingress_address;
	int target_address;
	int size;
	int state;
	int pending;
	int data_write_period;
	struct delayed_work create_thread;
};

static struct dm_struct datamover_device;
static struct task_struct *task;
static void *tx_ch;
static void *rx_ch;


int datamover_poll(void *data)
{
	int run = 1;
	
	u32 msg[2] = {datamover_device.data_write_period, 0};

	while (run) {
		while ((datamover_device.pending == 0)
			&& (ioread32(datamover_device.ocram
				+ datamover_device.size - 4) != MAGIC_WORD))
			;

		if (datamover_device.pending == 0) {
			/* Stop ingress performance counter */
			writel(0, datamover_device.perfcounter_nii +
				INGRESS_STOP_OFFSET);

			/* Send regress data transfer command */
			writel(SGDMA_GO_CMD, datamover_device.msgdma_hps +
				SGDMA_DES_CONTROL_OFFSET);

			/* Start regress performance counter */
			writel(0, datamover_device.perfcounter_nii +
				REGRESS_START_OFFSET);

			/* Clear magic word for next iteration */
			writel(0, (datamover_device.ocram
				+ datamover_device.size - 4));

		} else {
			if (datamover_device.state == 0) {
				/* Handshaking mailbox for polling mode here */
				ipc_send_message(tx_ch, (void *)msg);
				datamover_device.pending = 0;
			} else {
				datamover_device.pending = 0;
				run = 0;
			}
		}
	}

	do_exit(0);

	return 0;
}

static void create_thread(struct work_struct *work)
{
	task = kthread_create(&datamover_poll, &datamover_device,
		"datamover_poll");

	kthread_bind(task, CPU_ID);

	if (task) {
		/* Waking up task after thread creation */
		wake_up_process(task);
	} else {
		pr_err("Failed to initiate thread affinity\n");
	}

}

irqreturn_t msgdma_isr(int this_irq, void *dev_id)
{
	/* Clear msgdma interrupt */
	writel(readl(datamover_device.msgdma_nii),
		datamover_device.msgdma_nii);

	/* Stop ingress performance counter */
	writel(0, datamover_device.perfcounter_nii +
		INGRESS_STOP_OFFSET);

	/* Send regress data transfer command */
	writel(SGDMA_GO_CMD, datamover_device.msgdma_hps +
		SGDMA_DES_CONTROL_OFFSET);

	/* Start regress performance counter */
	writel(0, datamover_device.perfcounter_nii +
		REGRESS_START_OFFSET);

	return IRQ_HANDLED;
}


void datamover_rx_mbox_cb(void *cl_id, void *mssg)
{
	u32 *data = (u32 *)mssg;
	u32 temp_state;

	/* Process mailbox commands */
	datamover_device.size = data[0] & CMD_SIZE_MSK;
	temp_state = (data[0] & CMD_MODE_MSK) >> 30;

	/* Notification on change of mode */
	if (temp_state == 0) {
		pr_info("Datamover CMD: Mode: poll, Data Size: %d\n",
			datamover_device.size);
	} else {
		pr_info("Datamover CMD: Mode: interrupt, Data Size: %d\n",
			datamover_device.size);
	}

	/* Clearing magic number from memory */
	writel(0, (datamover_device.ocram + datamover_device.size - 4));

	/* Start delayed work to create new thread if necessary */
	if (temp_state != datamover_device.state) {
		datamover_device.state = temp_state;
		if (datamover_device.state == 0)
			schedule_delayed_work(&datamover_device.create_thread
				, 0);
	}

	/* handshaking mailbox for interrupt mode here */
	if (temp_state == 1) {
		data[0] = datamover_device.data_write_period;
		ipc_send_message(tx_ch, (void *)data);
	}

	/* Setting the pending flag */
	datamover_device.pending = 1;
}

static int __init datamover_init(void)
{
	struct ipc_client mbox_rx;
	struct ipc_client mbox_tx;

	int i;
	int ret;

	mbox_rx.txcb = NULL;
	mbox_rx.rxcb = datamover_rx_mbox_cb;
	mbox_rx.cl_id = NULL;
	mbox_rx.tx_block = false;
	mbox_rx.tx_tout = 0;
	mbox_rx.link_data = NULL;
	mbox_rx.knows_txdone = false;

	mbox_tx.txcb = 0;
	mbox_tx.rxcb = 0;
	mbox_tx.cl_id = 0;
	mbox_tx.tx_block = false;
	mbox_tx.tx_tout = 0;
	mbox_tx.link_data = 0;
	mbox_tx.knows_txdone = false;


	datamover_device.pending = 0;
	datamover_device.state = -1;

	/* Allocate the region of On Chip Ram 0xFFFF0000 */
	if (!request_mem_region(OCRAM_ADDR, OCRAM_SIZE, "ocram")) {
		pr_err("request mem region for ocram failed\n");
		goto err_nodev;
	}
	datamover_device.ocram = ioremap(OCRAM_ADDR, OCRAM_SIZE);
	if (!datamover_device.ocram) {
		pr_err("Failed to map on chip ram");
		goto err_nodev;
	}

	/* Performance counter for Nios II*/
	if (!request_mem_region(PERF_COUNTER_NII_ADDR, PERF_SIZE,
		"nios ii perfcounter")) {
		pr_err("Failed requesting performance counter region\n");
		goto err_nodev;
	}

	datamover_device.perfcounter_nii = ioremap(PERF_COUNTER_NII_ADDR,
		PERF_SIZE);
	if (!datamover_device.perfcounter_nii) {
		pr_err("Failed to map perfcounter\n");
		goto err_nodev;
	}

	/* Performance counter for HPS */
	if (!request_mem_region(PERF_COUNTER_HPS_ADDR, PERF_SIZE,
		"hps perfcounter")) {
		pr_err("Failed requesting performance counter region\n");
		goto err_nodev;
	}

	datamover_device.perfcounter_hps = ioremap(PERF_COUNTER_HPS_ADDR,
		PERF_SIZE);
	if (!datamover_device.perfcounter_hps) {
		pr_err("failed to map perfcounter\n");
		goto err_nodev;
	}

	/* Nios II to HPS SGDMA*/
	if (!request_mem_region(SGDMA_NII_CSR_ADDR, SGDMA_SIZE,
		"nios ii sgdma")) {
		pr_err("Failed requesting nios ii sgdma region\n");
		goto err_nodev;
	}

	datamover_device.msgdma_nii = ioremap(SGDMA_NII_CSR_ADDR,
		SGDMA_SIZE);

	if (!datamover_device.msgdma_nii) {
		pr_err("Failed to map nios ii sdgma\n");
		goto err_nodev;
	}

	ret = request_irq(SGDMA_NII_IRQ, msgdma_isr, 0, "sgdma_isr", 0);
	if (ret) {
		pr_err("Failed to register sgdma interrupt\n");
		goto err_nodev;
	}

	/* HPS to Nios II SGDMA*/
	if (!request_mem_region(SGDMA_HPS_CSR_ADDR, SGDMA_SIZE, "hps sgdma")) {
		pr_err("Failed requesting hps sgdma region\n");
		goto err_nodev;
	}

	datamover_device.msgdma_hps = ioremap(SGDMA_HPS_CSR_ADDR,
		SGDMA_SIZE);

	if (!datamover_device.msgdma_hps) {
		pr_err("Failed to map hps sdgma\n");
		goto err_nodev;
	}

	mbox_rx.chan_name = "mailbox_nios2ar:0";
	rx_ch = ipc_request_channel(&mbox_rx);

	mbox_tx.chan_name = "mailbox_arm2nio:0";
	tx_ch = ipc_request_channel(&mbox_tx);

	/* Initialize workqueue for thread affinity creation*/
	INIT_DELAYED_WORK(&datamover_device.create_thread, create_thread);

	PERF_RESET(datamover_device.perfcounter_hps);
	PERF_START_MEASURING(datamover_device.perfcounter_hps);
	for (i = 0; i < 100; i++) {
		PERF_BEGIN(datamover_device.perfcounter_hps, 1);
		PERF_END(datamover_device.perfcounter_hps, 1);
	}

	datamover_device.data_write_period = GET_PERF_RESULT
	(datamover_device.perfcounter_hps, 1);

	PERF_RESET(datamover_device.perfcounter_hps);

	return 0;

err_nodev:
	return -1;
}

static void __exit datamover_exit(void)
{
	 /* Release OCRAM */
	 release_mem_region(OCRAM_ADDR, OCRAM_SIZE);
	 iounmap(datamover_device.ocram);

	 /* Release Nios II performance counter */
	 release_mem_region(PERF_COUNTER_NII_ADDR, PERF_SIZE);
	 iounmap(datamover_device.perfcounter_nii);

	 /* Release HPS performance counter */
	 release_mem_region(PERF_COUNTER_HPS_ADDR, PERF_SIZE);
	 iounmap(datamover_device.perfcounter_hps);

	 /* Release Nios II SGDMA */
	 release_mem_region(SGDMA_NII_CSR_ADDR, SGDMA_SIZE);
	 iounmap(datamover_device.msgdma_nii);

	 /* Release HPS SGDMA */
	 release_mem_region(SGDMA_HPS_CSR_ADDR, SGDMA_SIZE);
	 iounmap(datamover_device.msgdma_hps);

	 /* Free up SGDMA IRQ */
	 free_irq(SGDMA_NII_IRQ, 0);

	 /* Release mailboxes */
	 ipc_free_channel(tx_ch);
	 ipc_free_channel(rx_ch);

	 return;
}

MODULE_LICENSE("GPL");

module_init(datamover_init);
module_exit(datamover_exit);

