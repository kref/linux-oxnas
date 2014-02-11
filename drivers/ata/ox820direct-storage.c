/*
 * drivers/ata/ox820direct-storage.c
 *
 * Copyright (C) 2010 PLX Technology 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/autoconf.h>
#include <linux/blkdev.h>
#include <linux/fs.h>
#include <linux/genhd.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/ata.h>
#include <linux/proc_fs.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/libata.h>
#include <mach/ox820sata.h>
#include <mach/direct-storage.h>

/* Need these for some useful definitions, but they pull in alot of the 'hacky'
 * SATA/DMA driver's helper functions */
#include <mach/dma.h>

//#define OX820_SATA_DEBUG
//#define X_CRAXY_DUMP_DEBUG
#define ERROR_INJECTION

#ifdef OX820_SATA_DEBUG
#define EPRINTK(fmt, args...) \
    printk(KERN_WARNING "%d %s: " fmt, raw_smp_processor_id(), __FUNCTION__, ## args)
#else
#define EPRINTK(fmt, args...) {while(0);}
#endif

#if 0
    #if 0
        typedef struct {
            u32 a;
            u32 d;
            u32 w;
        } regaccess;
        static u32 regindex = 0;
        static regaccess regarray[1024];
        
        #define newcommand {regarray[regindex].w |= 2;}
    #endif

    #ifdef writel
    #undef writel
    #endif
    static inline void writel(u32 v,u32 a) {printk("[%08x]<=%08x\n",a,v);*((volatile u32*)(a)) = v;} 
    //#define writel(vv,aa) {regarray[regindex].a=(aa); regarray[regindex].d=(vv); regarray[regindex].w=1; ++regindex; regindex &= 1023;*((volatile u32*)(aa)) = (vv);} 

    #ifdef readl
    #undef readl
    #endif
    static inline u32 myreadl(u32 a) {u32 v =(*((volatile u32*)(a))); printk("[%08x]=>%08x\n",a,v);return v;}
    //static inline u32 myreadl(u32 a) {u32 v =(*((volatile u32*)(a)));regarray[regindex].a=a; regarray[regindex].d=v; regarray[regindex].w=0; ++regindex; regindex &= 1023;return v;}
    #define readl(a) (myreadl(a))
#endif

MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

/* How long we allow for a spun-down disk to complete a command before we
   exhaust our command retry attempts */
#define SPIN_UP_DELAY (30*HZ)

/* The penalty applied to an unsuccessful command on a port. Also is how many
   successful commands to a port are needed to cancel out one unsuccessful
   command */
#define COMMAND_FAILURE_PENALTY	5

/* How long to allow for a single SATA command to complete successfully */
#define COMMAND_TIMEOUT (HZ)

/* The amount of 'unreliability' before a port is considered to have failed */
#define RELIABILITY_THRESHOLD (SPIN_UP_DELAY * COMMAND_FAILURE_PENALTY / COMMAND_TIMEOUT)

enum done_errors {
	no_error = 0,
	command_timeout,
	command_failed,
};

typedef struct direct_command {
	struct direct_access_context *context;
    unsigned long                 flags;
    unsigned long                 error_flags;
    u8                            ata_cmd;
    sector_t                      sector;
    sector_t                      nsects;
    unsigned int                  write;
    dma_addr_t                    dma_list_pa;	// Pointer to the memory-side prd/sg
    done_fn_t                    *done;		// Reports the result of the command to the bio layer or direct access client
    void                         *data;		// Data for the done() callback
    struct timer_list             command_timeout;  
    struct work_struct            eh_work;
    int error;
} direct_command_t;

enum cmd_active_states {
	CMD_INACTIVE,
	CMD_TIMEOUT,
	CMD_ACTIVE
};

/* hardware (almost a device) */
typedef struct direct_host {
    direct_command_t cmd;
	int cmd_active;

    /* dma control */
    unsigned int* sgdma_controller;
    oxnas_dma_simple_sg_info_t* sgdma_request_va;
    dma_addr_t sgdma_request_pa;

    /* storage */
    unsigned int faults;

#ifdef ERROR_INJECTION
    unsigned int error_period;
    unsigned long next_error;
#endif
} direct_host_t;

typedef struct single_disk_context {
    struct direct_access_context base_context;
    
    /** partition's start sector */ 
    sector_t start;
    
    /* sata port */
    int port;
} single_disk_context_t;


static irqreturn_t direct_isr(int irq_status, unsigned long arg);
static void direct_analyse(int timed_out);

#ifdef X_CRAXY_DUMP_DEBUG
static void xCrazyDumpDebug(void);
#endif // X_CRAXY_DUMP_DEBUG
#ifdef ERROR_INJECTION
static int ox820direct_error_inject_show(char *page, char **start, off_t off, int count, int *eof, void *data);
static int ox820direct_error_inject_store(struct file *file,const char __user *buffer,unsigned long count,void *data);
#endif // ERROR_INJECTION

static direct_host_t host = {
	.cmd_active = CMD_INACTIVE,

    /* dma is on channel 0 */
    .sgdma_controller = (u32* )(SATASGDMA_REGS_BASE),
    .sgdma_request_va = (oxnas_dma_simple_sg_info_t*)(OX820SATA_SGDMA_REQ + (1 * sizeof(oxnas_dma_simple_sg_info_t))),
    .sgdma_request_pa = (dma_addr_t)(OX820SATA_SGDMA_REQ_PA + (1 * sizeof(oxnas_dma_simple_sg_info_t))),

#ifdef ERROR_INJECTION
    .error_period = 0,
#endif
};

/* work queue for error handling */
static struct workqueue_struct* error_handler_wq;

static spinlock_t cmd_active_lock = SPIN_LOCK_UNLOCKED;

/**
 * returns a pointer to the host structure.
 */ 
static inline direct_host_t* get_host(void) 
{
    return &host;
}

/**
 * Determine if the port is still reliable
 */
static inline int is_reliable(void) 
{
    return (get_host()->faults < RELIABILITY_THRESHOLD);
}

/* 
 * Should be called after every command for every physical port involved
 * in the command. 
 */
static inline void update_faults(int error)
{
    direct_host_t* host = get_host();

    if (unlikely(error)) {
        host->faults += COMMAND_FAILURE_PENALTY;
        printk(KERN_INFO"Direct SATA fault-score %d/%d\n",
            host->faults, RELIABILITY_THRESHOLD);
    }
}

/* 
 * Should be called when preparing a command
 */
static inline void reset_faults(void)
{
    get_host()->faults = 0;
}

static int direct_acquire(
	direct_access_context_t *context,
	int                      timeout_jiffies,
	void                    *uid,
	int                      is_reader)
{
	int retval = 0;

	if (!acquire_sata_core_direct(direct_isr, 0, timeout_jiffies, uid, is_reader ? SATA_READER : SATA_WRITER)) {
		printk(KERN_WARNING "direct_acquire() Failed to acquire SATA core\n");
		retval = -EBUSY;
	}

	EPRINTK("context %p %s %s\n", context, retval ? "failure" : "success",
		context->read_only ? "reader" : "writer");

	return retval;
}

static void direct_release(int is_reader)
{
	EPRINTK("\n");
    release_sata_core(is_reader ? SATA_READER : SATA_WRITER);
}

static int direct_prepare(
	direct_access_context_t *context,
	int                      write,
	sector_t                 start,
	sector_t                 nsect,
	dma_addr_t               dma_list_pa,
	void (*done)(int error, void *data),
	void *data)
{
	direct_host_t    *host = get_host();
    direct_command_t *cmd = &host->cmd;
	struct single_disk_context* single_context = 
	    container_of( context, struct single_disk_context, base_context);

	/* Must not try to re-use the one-and-only command */
	smp_rmb();
	BUG_ON(host->cmd_active != CMD_INACTIVE);

	cmd->context = context;
	cmd->write = write;

	if (write) {
		cmd->ata_cmd = ATA_CMD_WRITE_EXT;
	} else {
		cmd->ata_cmd = ATA_CMD_READ_EXT;
	}

	cmd->sector = start + single_context->start;
	cmd->nsects = nsect;
	cmd->done = done;
	cmd->data = data;
	cmd->dma_list_pa = dma_list_pa;

	EPRINTK("cmd 0x%x, loc 0x%llx, num 0x%llx, mem_prd_pa %p,\n", 
		cmd->ata_cmd, cmd->sector, cmd->nsects, (void*)cmd->dma_list_pa);

	return 0;
}

/**
 * At any time we could be interrupted by a completing command.
 */
static void ox820direct_timeout(unsigned long unused)
{
	direct_host_t           *host;
	direct_command_t        *cmd;
	unsigned long            flags;

	host = get_host();
	spin_lock_irqsave(&cmd_active_lock, flags);
	switch (host->cmd_active) {
		case CMD_ACTIVE:
			host->cmd_active = CMD_TIMEOUT;
			spin_unlock_irqrestore(&cmd_active_lock, flags);

			cmd = &host->cmd;
			printk(KERN_WARNING "ox820direct_timeout timeout() cmd %p, context %p %s\n",
				cmd, cmd->context, cmd->context->read_only ? "reader" : "writer");
			direct_analyse(1);
			break;
		default:
			// The command normal completion interrupt has raced with the
			// timeout timer and the normal isr has won and is servicing the
			// interrupt
			spin_unlock_irqrestore(&cmd_active_lock, flags);
			printk(KERN_WARNING "ox820direct_timeout() Command not active\n");
	}
}

static int direct_execute(void)
{
    direct_host_t           *host = get_host();
	direct_command_t        *cmd = &host->cmd;
	direct_access_context_t *context = cmd->context;
    u32                     *base;
    u32                      orb[6];
	u32                      reg;
	struct single_disk_context* single_context = 
	    container_of( context, struct single_disk_context, base_context);
	
    base = (u32*)(single_context->port ? SATA1_REGS_BASE : SATA0_REGS_BASE);
	EPRINTK("cmd %p, context %p %s\n", cmd, context, context->read_only ? "reader" : "writer");

	/* Remember that the one-and-only command is in-use */
	WARN(host->cmd_active != CMD_INACTIVE, "direct_execute() cmd_active is %d\n", host->cmd_active);
	host->cmd_active = CMD_ACTIVE;
	smp_wmb();

	/* Prepare the DMA controller for a PRD driven transfer */
	odrb_dma_sata_prd(cmd->write ? OXNAS_DMA_TO_DEVICE : OXNAS_DMA_FROM_DEVICE,
		cmd->nsects, cmd->dma_list_pa, 0);

	/* Form the SATA command - only 48 bit disks supported */
    orb[1] = (ATA_LBA << 24);
    orb[2] = (cmd->ata_cmd << 24) | (cmd->nsects & 0xFFFF);
    orb[3] = (u32)cmd->sector;
    orb[4] = (cmd->sector & 0xFFFF00000000ULL) >> 32;           
    orb[5] = (cmd->nsects & 0xFFFF0000) >> 16;

    /* enable passing of error signals to DMA sub-core by clearing the 
    appropriate bit (all transfers are on dma channel 0)*/
    reg = readl(OX820SATA_DATA_PLANE_CTRL);
    reg &= ~(OX820SATA_DPC_ERROR_MASK_BIT);
    writel(reg, OX820SATA_DATA_PLANE_CTRL);

	/* Disable all interrupts for ports and RAID controller and core */
	writel(~0, (u32*)SATA0_REGS_BASE + OX820SATA_INT_DISABLE);
	writel(~0, (u32*)SATA1_REGS_BASE + OX820SATA_INT_DISABLE);
	writel(~0, (u32*)SATARAID_REGS_BASE + OX820SATA_INT_DISABLE);
	writel(~0, OX820SATA_CORE_INT_DISABLE);

    /* clear any pending interrupts */
	writel(~0, (u32*)SATA0_REGS_BASE + OX820SATA_INT_CLEAR);
	writel(~0, (u32*)SATA1_REGS_BASE + OX820SATA_INT_CLEAR);
	writel(~0, (u32*)SATARAID_REGS_BASE + OX820SATA_INT_CLEAR);
	writel(~0, OX820SATA_CORE_INT_CLEAR);

	/* write values to registers */
    writel(orb[1], base + OX820SATA_ORB1);
    writel(orb[2], base + OX820SATA_ORB2);
    writel(orb[3], base + OX820SATA_ORB3);
    writel(orb[4], base + OX820SATA_ORB4);
    writel(orb[5], base + OX820SATA_ORB5);

    /* enable End of command interrupt */
    writel(OX820SATA_INT_WANT, base + OX820SATA_INT_ENABLE);
    writel(OX820SATA_COREINT_END, OX820SATA_CORE_INT_ENABLE);

#ifdef ERROR_INJECTION
	if (host->error_period) {
	    u32 val;
		/* if the request error rate is bigger than prand then instigate
		 * an error */
		if (time_after(jiffies, host->next_error))
		{
            host->next_error = jiffies + host->error_period;
			printk(".");
            val = (cmd->write) ? 0x4 : 0x8; 
            val |= 1;
		} else {
			val = 1;
		}
		ox820sata_link_write(base, 0x14, val );
	}
#endif

	/* Ensure all setup has reached the h/w before starting the transfer */
    wmb();

    /* (Re)start timer */
    cmd->command_timeout.data = (unsigned long)cmd;
    cmd->command_timeout.function = ox820direct_timeout;
    mod_timer(&cmd->command_timeout, jiffies + COMMAND_TIMEOUT);

    /* start command */
    reg = readl(base + OX820SATA_SATA_COMMAND);
    reg &= ~SATA_OPCODE_MASK;
    reg |= CMD_WRITE_TO_ORB_REGS;
    writel(reg , base + OX820SATA_SATA_COMMAND);

    return 0;
}

/**
 * Called from the irq handler if the normal driver doesn't own up to the 
 * interrupt. 
 */
static irqreturn_t direct_isr(int irq_status, unsigned long arg)
{
	irqreturn_t retval = IRQ_NONE;

    /* If a command hasn't finished we don't want to know */
    if (!(irq_status & OX820SATA_COREINT_END)) {
		printk(KERN_WARNING "direct_isr() Command is NOT finished\n");
    } else {
    	unsigned long     flags;
    	direct_command_t *cmd;
    	direct_host_t    *host = get_host();

		spin_lock_irqsave(&cmd_active_lock, flags);
		switch (host->cmd_active) {
			case CMD_ACTIVE:
				host->cmd_active = CMD_INACTIVE;
				spin_unlock_irqrestore(&cmd_active_lock, flags);

				cmd = &host->cmd;
				if (!del_timer(&cmd->command_timeout))
					printk(KERN_WARNING "direct_isr() Timer already inactive\n");

				EPRINTK("cmd %p, context %p %s\n", cmd, cmd->context, cmd->context->read_only ? "reader" : "writer");
				direct_analyse(0);
				retval = IRQ_HANDLED;
				break;
			case CMD_TIMEOUT:
				// The timeout time has raced with the normal interrupt and won
				// and is handling the command. We return IRQ_HANDLED because
				// the direct SATA code was responsible for the interrupt
				spin_unlock_irqrestore(&cmd_active_lock, flags);
				printk(KERN_WARNING "direct_isr() Timeout processing in progress\n");
				retval = IRQ_HANDLED;
				break;
			case CMD_INACTIVE:
				// Appears that the direct SATA driver was not responsible for
				// the interrupt so don't claim to have handled it
				spin_unlock_irqrestore(&cmd_active_lock, flags);
				printk(KERN_WARNING "direct_isr() Command not active\n");
				break;
			default:
				spin_unlock_irqrestore(&cmd_active_lock, flags);
				printk(KERN_ERR "direct_isr() Unknown command state %d\n", host->cmd_active);
				break;
		}
	}

	return retval;
}

static void log_error(direct_command_t* cmd, int timed_out)
{
    u32 error;
    u32 serror;
    direct_host_t* host = get_host();
    u32 *base;
	struct single_disk_context* single_context = 
	    container_of( cmd->context, struct single_disk_context, base_context);
    
    base = (u32*)(single_context->port ? SATA1_REGS_BASE : SATA0_REGS_BASE);

    EPRINTK("\n");
    if (timed_out) {
        printk(KERN_ERR"Timed out waiting for direct SATA command to finish\n");
    } else {
        printk(KERN_ERR"Error found during direct SATA command\n");
    }

    printk(KERN_ERR"Port=%d Cmd=%02x Sector=%012llx NSect=%04x\n",
        0, cmd->ata_cmd, cmd->sector, (int)cmd->nsects );

    /* which error bits are set in the port */
    error = readl(base + OX820SATA_INT_STATUS);
    if (error & OX820SATA_RAW_ERROR) {
        int orb_err;

        /* examine the orb error byte */
        orb_err = readl(base + OX820SATA_ORB2);
        orb_err >>= 16;
        orb_err &= 0xff;

        if (orb_err & ATA_ICRC) {
            printk(KERN_ERR"CRC error\n");
        }
        if (orb_err & ATA_UNC) {
            printk(KERN_ERR"Uncorrectable media error\n");
        }
        if (orb_err & ATA_MC) {
            printk(KERN_ERR"Media changed\n");
        }
        if (orb_err & ATA_IDNF) {
            printk(KERN_ERR"ID not found\n");
        }
        if (orb_err & ATA_MCR) {
            printk(KERN_ERR"Media change requested\n");
        }
        if (orb_err & ATA_ABORTED) {
            printk(KERN_ERR"Command aborted\n");
        }
        if (orb_err & ATA_TRK0NF) {
            printk(KERN_ERR"Track zero not found\n");
        }
        if (orb_err & ATA_AMNF) {
            printk(KERN_ERR"Address mark not found\n");
        }
    }

    /* which error bits are set in the link */
    serror = ox820sata_link_read(base, 0x24);
    if (serror) {
		printk(KERN_ERR"SError: {%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s}\n",
            serror & SERR_DATA_RECOVERED ? "RecovData " : "",
            serror & SERR_COMM_RECOVERED ? "RecovComm " : "",
            serror & SERR_DATA ? "UnrecovData " : "",
            serror & SERR_PERSISTENT ? "Persist " : "",
            serror & SERR_PROTOCOL ? "Proto " : "",
            serror & SERR_INTERNAL ? "HostInt " : "",
            serror & SERR_PHYRDY_CHG ? "PHYRdyChg " : "",
            serror & SERR_PHY_INT_ERR ? "PHYInt " : "",
            serror & SERR_COMM_WAKE ? "CommWake " : "",
            serror & SERR_10B_8B_ERR ? "10B8B " : "",
            serror & SERR_DISPARITY ? "Dispar " : "",
            serror & SERR_CRC ? "BadCRC " : "",
            serror & SERR_HANDSHAKE ? "Handshk " : "",
            serror & SERR_LINK_SEQ_ERR ? "LinkSeq " : "",
            serror & SERR_TRANS_ST_ERROR ? "TrStaTrns " : "",
            serror & SERR_UNRECOG_FIS ? "UnrecFIS " : "",
            serror & SERR_DEV_XCHG ? "DevExch " : "");
    }

    /* which error bits are set in the dma */
    error = readl(host->sgdma_controller + OX820SATA_SGDMA_STATUS);
    if (error & OX820SATA_SGDMA_BUSY) {
        printk(KERN_ERR"SATA DMA waiting to complete\n");
    }
    if (error & OX820SATA_SGDMA_ERRORMASK) {
        printk(KERN_ERR"SATA DMA error.\n");
    }
}

static void cleanup(direct_command_t* cmd)
{
    cleanup_recovery_t action;
    EPRINTK("\n");
    
    action = ox820sata_cleanup();
#if 0
    while (action) {
    {
        /* send reset */
        sata_std_hardreset( &ox820sata_get_ap(0)->link, NULL, jiffies + HZ);
        sata_std_hardreset( &ox820sata_get_ap(1)->link, NULL, jiffies + HZ);
        action = ox820sata_cleanup();
    }
#else
#warning incremental cleanup disabled due to problems with unaligned transfers
#endif        

    /* all errors gone */
    cmd->error_flags = 0;
	ox820sata_thaw_host(0);
}


static void direct_eh_work(struct work_struct *work) 
{
    direct_command_t* cmd = container_of(work, direct_command_t, eh_work);
    int timed_out = test_bit(command_timeout, &cmd->error_flags);

    /* report and cleanup the error */
    log_error(cmd, timed_out );
    cleanup(cmd);

    /* Having reached here means there was an error of some sort. If this
     * included the command having timed out then now that the command has been
     * aborted and thus we should be assured there is no further chance of the
     * interrupt for the command firing due to the command having taken a very
     * long time we should update the command state to inactive
     */
    if (timed_out) {
		get_host()->cmd_active = CMD_INACTIVE;
		smp_wmb();
	}

    if (is_reliable()) {
        /* can retry on the same disk */
        printk(KERN_WARNING"retrying cmd\n");
        cmd->context->execute_command();
    } else {
		EPRINTK("Invoke done callback: done_fn %p, done_data %p\n", cmd->done, cmd->data);
		/* Invoke the fast reader/writer SATA completion callback */
        cmd->done( (timed_out ? command_timeout : command_failed), cmd->data);
        reset_faults();
    }
}


/**
 * Called for reads or writes that were sent to one physical disk.
 * If writes are using this, then there is only one disk left in the array
 */
static void direct_analyse(int timed_out)
{
    direct_host_t           *host = get_host();
	direct_command_t        *cmd = &host->cmd;
	direct_access_context_t *context = cmd->context;
	u32* base;
	u32 sata_error = 0;
	u32 dma_error = 0;
	u32 error = 0;
	struct single_disk_context* single_context = 
	    container_of( context, struct single_disk_context, base_context);

    base = (u32*)(single_context->port ? SATA1_REGS_BASE : SATA0_REGS_BASE);
    EPRINTK("cmd %p, context %p, port %d, timed_out %d\n", cmd, context, single_context->port, timed_out);

    /* check for an error */
    sata_error = readl(base + OX820SATA_INT_STATUS) & OX820SATA_RAW_ERROR;

    /* check dma completed ok */
    dma_error = readl(host->sgdma_controller + OX820SATA_SGDMA_STATUS);

	error = sata_error | dma_error | timed_out;
	EPRINTK("sata_error %p, dma_error %p, error %p\n", (void*)sata_error, (void*)dma_error, (void*)error);

    update_faults(error);
    if (unlikely(error)) {
        /* freeze the host so to stop kblockd from polling us to death */
        ox820sata_freeze_host(0);
        if (timed_out) {
            set_bit(command_timeout, &cmd->error_flags);
        }
        INIT_WORK(&cmd->eh_work, direct_eh_work);
        queue_work(error_handler_wq, &cmd->eh_work);
    } else {
		/* Invoke the fast reader/writer SATA completion callback */
		EPRINTK("Invoke done callback: done_fn %p, done_data %p\n", cmd->done, cmd->data);
        cmd->done(no_error, cmd->data);
        reset_faults();
    }
}

/*
 * Looks at the partition on which the passed inode resides and associates
 * appropriate callbacks for dealing with that partition, e.g. single disk,
 * RAID
 */
int alloc_direct_sata_context(
	struct inode             *inode,
	int                       read_only,
	direct_access_context_t **context)
{
	int                  retval = 0;
	struct block_device	*bd;
	struct hd_struct	*partition;
	struct single_disk_context* single_context;

	/* Find out where on disk the file associated with the inode is */
	bd = bdget(MKDEV(MAJOR(inode->i_sb->s_dev), MINOR(inode->i_sb->s_dev)));
	if (!bd) {
		retval = -ENODEV;
		goto out;
	}
	partition = bd->bd_part;

	if (!partition) {
		retval = -ENODEV;
		goto out;
	}

	/* abort before trampling over the OS */
	if (unlikely(partition->start_sect == 0)) {
	    printk(KERN_CRIT
	        "This partition the fast code is using appears to starts at "
	        "sector zero\nThis is probably because you're using single "
	        "disk code to access a RAID partition.\nNow crashing to protect "
	        "the OS partition.\n");
	    BUG();
	}

	/* Need to track direct access context for this inode */
	single_context = kzalloc(sizeof(single_disk_context_t), GFP_KERNEL);
	if (!single_context) {
		retval = -ENOMEM;
		goto out;
	}

    /* Setup function pointers etc. for single disk access */
    single_context->base_context.read_only = read_only;
    single_context->base_context.acquire         = direct_acquire;
    single_context->base_context.prepare_command = direct_prepare;
    single_context->base_context.execute_command = direct_execute;
    single_context->base_context.release         = direct_release;
    single_context->base_context.has_waiters     = sata_core_has_waiters;

    single_context->start     = partition->start_sect;
	single_context->port      = CONFIG_OXNAS_FAST_SATA_PORT;

	/* set the context pointer to point to the base class in our superclass */
	*context = &( single_context->base_context );
out:
	EPRINTK("single_context %p (containing context %p) %s for %s\n",
	    single_context,
	    *context,
	    retval ? "failure" : "success",
	    read_only ? "reader" : "writer");
	return retval;
}

void free_direct_sata_context(direct_access_context_t *context)
{
	struct single_disk_context* single_context = 
	    container_of( context, struct single_disk_context, base_context);

	EPRINTK("single_context %p (containing context %p)\n", single_context, context);
	kfree(single_context);
}

static int __init ox820direct_init(void) 
{
	direct_command_t *cmd = &get_host()->cmd;

    init_timer(&cmd->command_timeout);

    /* create a workqueue for error handling */
    error_handler_wq = create_singlethread_workqueue("ox820direct_eh");

#ifdef ERROR_INJECTION
    {
        struct proc_dir_entry *res = create_proc_entry("ox820direct_errorinject",0,NULL);
        if (res) {
            res->read_proc  = ox820direct_error_inject_show;
            res->write_proc = ox820direct_error_inject_store;
            res->data = NULL;
        }
    }
#endif
    return 0;
}

module_init(ox820direct_init);

#ifdef X_CRAXY_DUMP_DEBUG
/**
 * Outputs all the registers in the SATA core for diagnosis of faults.
 *
 * It will quickly capture the registers to memory, then "play" them back 
 * with printk's
 *
 * @param ap Hardware with the registers in
 */
static void xCrazyDumpDebug(void)
{
#define NO_PORT_REG     48
#define NO_LINK_REG     16
#define NO_DMA_REG      16
#define NO_SGDMA_REG    8
#define NO_RAID_REG     48
#define NO_CORE_REG     80
#define NO_CAPTURED_REG ( (NO_PORT_REG + NO_LINK_REG) * 2 + \
                          NO_DMA_REG + NO_SGDMA_REG + NO_RAID_REG + NO_CORE_REG)

    static u32 registers[NO_CAPTURED_REG];
    u32 offset;
    volatile u32* ioaddr;
    u32 i=0 ;
    
#if 0
    {
        for(i = 0;i < 1024;++i) {
            if (regarray[regindex].w & 2) printk("new sata command\n");
            printk("[%08x]%s%08x\n",
                regarray[regindex].a,
                (regarray[regindex].w & 1) ? "<=" : "=>",
                regarray[regindex].d
                );
            ++regindex;
            regindex &= 1023;
        }
    }
#endif
    /* capture */

    ioaddr = (u32* )SATADMA_REGS_BASE;
    for(offset = 0; offset < NO_DMA_REG;offset++)
    {
        registers[i++] = *(ioaddr + offset);
    }

    ioaddr = (u32* )SATASGDMA_REGS_BASE;
    for(offset = 0; offset < NO_SGDMA_REG;offset++)
    {
        registers[i++] = *(ioaddr + offset);
    }

    /* port 0 */
    ioaddr = (u32* )SATA0_REGS_BASE;
    for(offset = 0; offset < NO_PORT_REG;offset++)
    {
        registers[i++] = *(ioaddr + offset);
    }

    for(offset = 0; offset < NO_LINK_REG;++offset)
    {
        u32 patience;
        *(ioaddr + OX820SATA_LINK_RD_ADDR ) = (offset*4);
        wmb();
    
        for (patience = 0x100000;patience > 0;--patience)
        {
            if (*(ioaddr + OX820SATA_LINK_CONTROL) & 0x00000001)
                break;
        }
    
        registers[i++] = *(ioaddr + OX820SATA_LINK_DATA);
    }

    /* port 1 */
    ioaddr = (u32* )SATA1_REGS_BASE;
    for(offset = 0; offset < NO_PORT_REG;offset++)
    {
        registers[i++] = *(ioaddr + offset);
    }

    for(offset = 0; offset < NO_LINK_REG;++offset)
    {
        u32 patience;
        *(ioaddr + OX820SATA_LINK_RD_ADDR ) = (offset*4);
        wmb();
    
        for (patience = 0x100000;patience > 0;--patience)
        {
            if (*(ioaddr + OX820SATA_LINK_CONTROL) & 0x00000001)
                break;
        }
    
        registers[i++] = *(ioaddr + OX820SATA_LINK_DATA);
    }
    
    /* port 15 */
    ioaddr = (u32* )SATARAID_REGS_BASE;
    for(offset = 0; offset < NO_RAID_REG;offset++)
    {
        registers[i++] = *(ioaddr + offset);
    }
    
    /* port 14 */
    ioaddr = (u32* )SATACORE_REGS_BASE;
    for(offset = 0; offset < NO_CORE_REG;offset++)
    {
        registers[i++] = *(ioaddr + offset);
    }
    
    /* display the registers */
    i = 0;
    printk("Direct raid crazy dump debug\n");

    printk("DMA registers\n");
    for(offset = 0; offset < NO_DMA_REG;offset++)
    {
        printk("[%02x] %08x\n", offset * 4, registers[i++]);
    }

    printk("SGDMA registers\n");
    for(offset = 0; offset < NO_SGDMA_REG;offset++)
    {
        printk("[%02x] %08x\n", offset * 4, registers[i++]);
    }

    /* port 0 */
    printk("Port 0 High level registers\n");
    for(offset = 0; offset < NO_PORT_REG;offset++)
    {
        printk("[%02x] %08x\n", offset * 4, registers[i++]);
    }

    printk("Port 0 link layer registers\n");
    for(offset = 0; offset < NO_LINK_REG;++offset)
    {
        printk("[%02x] %08x\n", offset*4, registers[i++]);
    }

    /* port 1 */
    printk("Port 1 High level registers\n");
    for(offset = 0; offset < NO_PORT_REG;offset++)
    {
        printk("[%02x] %08x\n", offset * 4, registers[i++]);
    }

    printk("Port 1 link layer registers\n");
    for(offset = 0; offset < NO_LINK_REG;++offset)
    {
        printk("[%02x] %08x\n", offset*4, registers[i++]);
    }
    
    /* port 15 */
    printk("RAID registers (port 15)\n");
    for(offset = 0; offset < NO_RAID_REG;offset++)
    {
        printk("[%02x] %08x\n", offset * 4, registers[i++]);
    }
    
    /* port 14 */
    printk("CORE registers (port 14)\n");
    for(offset = 0; offset < NO_CORE_REG;offset++)
    {
        printk("[%02x] %08x\n", offset * 4, registers[i++]);
    }
    
    printk("micro-code program counter poll\n");
    for(offset = 0; offset < 80;offset++)
    {
        printk("%d\n", *((volatile u32*)OX820SATA_PROC_PC) );
    }
}
#endif // X_CRAXY_DUMP_DEBUG

#ifdef ERROR_INJECTION

/**
 * procfs read-file function, displays the error period in jiffies or "off"
 * if no error injection is enabled
 */
static int ox820direct_error_inject_show(
    char  *page,
	char **start,
	off_t  off,
	int    count,
	int   *eof,
	void  *data)
{
    direct_host_t* host = get_host();

    if (page) {
        if ( host->error_period ) {
            int ret;
            ret = sprintf(page,"%d\n",host->error_period);
            return ret;
        } else {
            return sprintf(page, "off\n" );
        }
    }

    /* if we get here, there's been an error */
    return -EIO;
}

/**
 * Procfs write-file function, accepts an error period in jiffies, 0 turns error
 * injection off.
 */
static int ox820direct_error_inject_store(
	struct file       *file,
	const char __user *buffer,
	unsigned long      count,
	void              *data) 
{
    direct_host_t* host = get_host();
    if (count) {
        sscanf(buffer, "%d", &(host->error_period));
        host->next_error = jiffies + host->error_period;
        return count;
    }

    /* if we get here, there's been an error */
    return -EIO;
}
#endif /* ERROR_INJECTION */
