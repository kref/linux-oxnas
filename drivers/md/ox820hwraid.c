/*
 * drivers/md/ox820hwraid.c
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
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/genhd.h>
#include <linux/hdreg.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/ata.h>
#include <linux/libata.h>
#include <linux/proc_fs.h>
#include <linux/workqueue.h>
#include <linux/raid/md_p.h>
#include <linux/raid/md_u.h>
#include "md.h"
#include "raid0.h"
#include "raid1.h"
#include "bitmap.h"
#include "hwraid.h"

#include <mach/desc_alloc.h>
#include <mach/ox820sata.h>
#include <mach/direct-storage.h>
#include <mach/dma.h>
/**************************************************************************/
/* Debug                                                                  */
/**************************************************************************/
#undef RAID_DEBUG
#undef RAID_CONFIG_DEBUG
#define ERROR_INJECTION
#undef DEBUG_REGISTER_ACCESSES
#undef RECORD_REGISTER_ACCESSES

#ifdef DPRINTK
#undef DPRINTK
#endif

#ifdef RAID_DEBUG
#define DPRINTK(fmt, args...) \
    printk(KERN_ERR "%d %s: " fmt, raw_smp_processor_id(), __FUNCTION__, ## args);
#else
#define DPRINTK(fmt, args...) {while(0);}
#endif

#ifdef RAID_CONFIG_DEBUG
#define CPRINTK(fmt, args...) \
    printk(KERN_ERR "%d %s: " fmt, raw_smp_processor_id(), __FUNCTION__, ## args);
#else
#define CPRINTK(fmt, args...) {while(0);}
#endif

#if (defined(RAID_DEBUG) || defined(RAID_CONFIG_DEBUG))
#define DEBUG_CODE 1
#else
#define DEBUG_CODE 1
#endif

#if defined(DEBUG_REGISTER_ACCESSES) || defined(RECORD_REGISTER_ACCESSES)
    #ifdef writel
    #undef writel
    #endif

    #ifdef readl
    #undef readl
    #endif
    #define readl(a) (myreadl(a))

    #ifdef RECORD_REGISTER_ACCESSES
        #define NUMBER_OF_RECORDED_REGISTERS (1 << 6)    

        typedef struct {
            u32 a;
            u32 d;
            u32 w;
        } regaccess;
        static u32 regindex = 0;
        static regaccess regarray[NUMBER_OF_RECORDED_REGISTERS];

        static inline void writel(u32 vv,u32 aa)
        {
            regarray[regindex].a=(aa);
            regarray[regindex].d=(vv);
            regarray[regindex].w=1 | raw_smp_processor_id() << 8;
            ++regindex;
            regindex &= (NUMBER_OF_RECORDED_REGISTERS - 1);
            *((volatile u32*)(aa)) = (vv);
        }
        static inline u32 myreadl(u32 a)
        {
            u32 v =(*((volatile u32*)(a)));
            regarray[regindex].a=a;
            regarray[regindex].d=v;
            regarray[regindex].w=0 | raw_smp_processor_id() << 8;;
            ++regindex;
            regindex &= (NUMBER_OF_RECORDED_REGISTERS - 1);
            return v;
        }
        static void dump_register_accesses(void)
        {
            int count;
            for(count = 0;count < NUMBER_OF_RECORDED_REGISTERS;++count) {
                if (regarray[regindex].w & 2) { 
                    printk("new sata command\n");
                }
                printk("[%08x]%s%08x cpu %d\n",
                    regarray[regindex].a,
                    (regarray[regindex].w & 1) ? "<=" : "=>",
                    regarray[regindex].d,
                    (regarray[regindex].w >> 8)
                    );
                ++regindex;
                regindex &= (NUMBER_OF_RECORDED_REGISTERS - 1);
            }
        }
    #else /* RECORD_REGISTER_ACCESSES */
        static inline void writel(u32 v,u32 a)
        {
            printk("[%08x]<=%08x\n",a,v);
            *((volatile u32*)(a)) = v;
        }
        static inline u32 myreadl(u32 a)
        {
            u32 v =(*((volatile u32*)(a)));
            printk("[%08x]=>%08x\n",a,v);
            return v;
        }
    #endif /* RECORD_REGISTER_ACCESSES */

#endif /* defined(DEBUG_REGISTER_ACCESSES) || defined(RECORD_REGISTER_ACCESSES) */

/**************************************************************************/
/* Constants                                                              */
/**************************************************************************/


MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

/* the largest size of transfer we will send to the disks */
#define MAX_CMD_SIZE (0x8000)

#define NUMBER_OF_DISKS 2

/** RAID-1 read-balancing penalties for backwards seeking */
#define SEEK_BACKWARDS_MULTIPLE     1
#define SEEK_BACKWARDS_ADDITIONAL   0

/** how many times a command can be sent before the system degrades the raid
array or gives up */
#define MAX_TRIES 4
#define DEFINITELY_BROKEN (MAX_TRIES + 1)

/** how long the request queue will wait for access to the sata hardware */
#define COMMAND_TIMEOUT (HZ * 5)

#define NUMBER_OF_COMMANDS 2

#define MAX_PRD_ELEMENT_SIZE 0x10000

#define NUMBER_OF_DMA_ELEMENTS CONFIG_ODRB_WRITER_PRD_ARRAY_SIZE

/** cause requests to complete in a bottom half rather than irq */
#undef REQUEST_DONE_BH

/** The maximum number of pages that can be synchronised at once is limited
 * by the maximum number of elements in a scatterlist. (Unless we do
 * contiguous page coalesceing (and we can barely even spell that)) */
#define RESYNC_PAGES (NUMBER_OF_DMA_ELEMENTS)
#define CHECK_PAGES (16)
#define HWRAID1_RESYNC_SECTORS ((RESYNC_PAGES * PAGE_SIZE) >> 9)
#define HWRAID1_CHECK_SECTORS ((CHECK_PAGES * PAGE_SIZE) >> 9)
#define REQUEST_QUEUE_UID   ((void*)0x11524551)

/** The software RAID will get several small sync commands going at the same
 * time and rely on the io queue to merge them in order to make the sync 
 * process efficient. HWRAID can't do that so restrict it to a maximum of
 * one (large) command at a time. */
#define HWRAID1_RESYNC_DEPTH 2

/**************************************************************************/
/* Types                                                                  */
/**************************************************************************/
struct ox820_raiddev_s;
struct port_s;
struct directcmd_s;
struct command_s;
struct raidset_s;
struct sataport_s;
struct component_partition_s;
struct sync_process_s;
struct dma_table_s;
struct host_s;
struct sync_activity_s;

typedef struct command_s* (biotocmd_t)(struct raidset_s*, struct request_queue*, struct request*);
typedef int (directcmd_t)(struct direct_access_context*, int, sector_t, sector_t, dma_addr_t, void (*done)(int error, void *data), void *data);
typedef int (execute_fn_t)( struct command_s* cmd, struct host_s* host);

struct raidset_s {
    mddev_t* mddev;

    /** Indictates that the hwRAID is handling commands */
    int hw_raid_active;

    sector_t start; /** start of the raid set on the disks */
    struct port_s* last_port; /** last port we sent a read command to */

    struct host_s* host;

    /* The partitions that the RAID set is implemented on */
    struct component_partition_s* partition[NUMBER_OF_DISKS];

    struct request_queue* queue;

    /** will convert a bio request into a command */
    biotocmd_t* biotocmd;

    /** functon for converting a direct command into a raid command */
    directcmd_t* direct_cmd;

    /* function pointers that connect to functions that call the RAID-1
     * barriers and md_write_start/end. On RAID-0 these should be NULL */
    int (*wait_barrier)(struct command_s*, struct raidset_s*, int);
    void (*allow_barrier)(struct raidset_s*, int);

    /* function pointers that connect to functions that call the RAID-1
     * barriers and md_write_start/end. On RAID-0 these should be NULL */
    void (*direct_start_barrier)(struct raidset_s*, int);
    void (*direct_end_barrier)(struct raidset_s*, int);

    struct workqueue_struct* wait_event_anticipation;
    struct work_struct work;

    struct sync_activity_s* sync_process;
};

enum {
    in_use      = 0, /* the cmd struct has stuff in it we want */
    started_md_write,
    passed_md_write_checks,
    started_wait_barrier,
    passed_wait_barrier,
};

enum error_flags_e {
    command_timedout, /* the command timed out */
};

struct command_s {
    /* a set bit-zero indicates that the command is already being used */
    unsigned long flags;

    /** links back to the source of the command */
    struct request* req;
    struct directcmd_s* direct;

    u8  ata_cmd;
    sector_t sector;
    sector_t nsects;
    struct port_s* port;
    unsigned int write;

    /** scatter list for request queue commands */
    struct scatterlist sg[NUMBER_OF_DMA_ELEMENTS];

    /* pointers to the dma tables */
    odrb_prd_list_t* odrb_prd_list;

    /** a function that will program the command into hardware */
    execute_fn_t* execute_command;

    /** reports the result of the command to the bio layer or direct io thing */
    done_fn_t* done;

    /** data for the done() callback */
    void* data;

    /** when the command completes, this works out what happened */
    void (*analyse)(struct command_s* cmd, int timeout);

    struct timer_list command_timeout;
    struct work_struct eh_work;
    struct raidset_s* raid;
    unsigned long error_flags;

    /* the number of failed attempts at executing this command */
    unsigned int faults;
};

/** try not to instanciate this directly, create a sataport or a raidport */
struct port_s {
    unsigned int* base;

    /** estimated head position */
    sector_t headpos;

    /** number of times a command has failed on the drive */
    int faults;

    /** indicates that this port is write only, possibly due to syncing */
    int write_only;

    int debug_portno;
};

/* specialisation of a port */
struct sataport_s {
    struct port_s port;
};

/* specialisation of a port */
struct raidport_s {
    struct port_s port;

    /* this raid port is implemented on these two ports */
    struct sataport_s* host_port[2];
};

/* check this matches the space reserved in hardware.h */
typedef struct {
    volatile u32 qualifier;
    volatile u32 control;
    dma_addr_t src_pa;
    dma_addr_t dst_pa;
} __attribute ((aligned(4),packed)) sgdma_request_t;

/* hardware */
struct host_s {
    struct raidport_s raid_port;
    struct sataport_s sata_port[2];

    struct command_s* active_cmd;
    spinlock_t active_cmd_lock;

#ifdef ERROR_INJECTION
    unsigned int error_period;
    unsigned long next_error;
#endif
};

struct ox820_raiddev_s {
    struct host_s* host;
    struct raidset_s* raidset;
};

struct component_partition_s {
    /* port the partition is accessed through */
    struct port_s* port;

    /* start sector */
    sector_t start;

    /* size */
    sector_t size;

    /* mirror */
    int mirror;
};

typedef struct ox820_raiddev_s ox820_raiddev_t;

struct raid_context_s {
    struct direct_access_context base_context;
    struct raidset_s* raid;
};

struct sync_activity_s {
    struct raidset_s* raid;
    struct page* sync_page[RESYNC_PAGES];
    struct page** sync_buffer[NUMBER_OF_DISKS];
    /* pointers to the dma tables */
    odrb_prd_list_t* odrb_prd_list;
    unsigned long source;
    unsigned long source_ok;
    unsigned long dest;

    sector_t nr_sectors;
    sector_t start_sector;
    
    unsigned long   recovery_flags;
    
    /* target of the current command */
    int mirror;
    
    struct workqueue_struct* workqueue;
    struct work_struct work;
};
/**************************************************************************/
/* Prototypes                                                             */
/**************************************************************************/
struct ata_port;
static void sg_to_prd(struct scatterlist* , int, odrb_prd_list_t* );

static void hwraid_log(struct command_s* cmd);
static void hwraid_cleanup(struct command_s* cmd);

/* RAID-0 */
static struct command_s* hwraid0_bio_to_cmd(struct raidset_s* raid, struct request_queue* queue, struct request* req);
static void hwraid0_analysis(struct command_s* cmd, int timeout);
static int hwraid0_direct_cmd(struct direct_access_context* ,int ,sector_t ,sector_t ,dma_addr_t ,void (*)(int, void*),void*);
static int hwraid0_execute_command( struct command_s* cmd, struct host_s* host);

/* RAID-1 */
static inline int port_is_reliable(struct port_s* port) ;
static struct port_s* hwraid1_read_balance(struct raidset_s* raid, sector_t start);
static struct command_s* hwraid1_bio_to_cmd(struct raidset_s* raid, struct request_queue* queue, struct request* req);
static void hwraid1_double_analysis(struct command_s* cmd, int timeout);
static void hwraid1_degraded_analysis(struct command_s* cmd, int timeout);
static void hwraid1_read_analysis(struct command_s* cmd, int timeout);
static void hwraid1_single_analysis(struct command_s* cmd, int timeout);
static int hwraid1_direct_cmd(struct direct_access_context* ,int ,sector_t ,sector_t ,dma_addr_t ,void (*)(int, void*),void*);
static int hwraid1_execute_command_double(struct command_s* cmd, struct host_s* host);
static int hwraid1_execute_command_single(struct command_s* cmd, struct host_s* host);

static void hwraid1_sbwrite_waiting(struct work_struct *work);
static int hwraid1_wait_barrier(struct command_s*, struct raidset_s*, int);
static void hwraid1_end_barrier(struct raidset_s* raid, int write);
static void hwraid1_direct_start_barrier(struct raidset_s* raid, int write);
static void hwraid1_direct_end_barrier(struct raidset_s* raid, int write);

static int ox820hwraid_disk_up(struct raidset_s* raid);
static void ox820hwraid_disk_down(struct raidset_s* raid);
static int __init ox820hwraid_init(void);
static inline int injectErrors(struct port_s* port, int write);
void ox820hwraid_restart_queue(void);
static void request_processor(struct request_queue* queue);
static void request_done(int error, void* data);
#ifdef REQUEST_DONE_BH
static void request_done_bh(unsigned long data);
#endif
static struct host_s* get_host(void);
irqreturn_t ox820hwraid_irq_handler(int irq_status, unsigned long);
static void ox820hwraid_timeout(unsigned long unused);
static struct command_s* form_sync_read(struct sync_activity_s* );
sector_t hwraid1_sync_request(mddev_t*, sector_t, int*, int);

#ifdef ERROR_INJECTION
static int ox820direct_error_inject_show(char *page, char **start, off_t off, int count, int *eof, void *data);
static int ox820direct_error_inject_store(struct file *file,const char __user *buffer,unsigned long count,void *data);
#endif

/**************************************************************************/
/* Structures                                                             */
/**************************************************************************/

static ox820_raiddev_t* raiddev = NULL;

static struct raidset_s hwraid1_raidset = {
    .last_port = NULL,
    .biotocmd = hwraid1_bio_to_cmd,
    .direct_cmd = hwraid1_direct_cmd,
    .wait_barrier = hwraid1_wait_barrier,
    .allow_barrier = hwraid1_end_barrier,
    .direct_start_barrier = hwraid1_direct_start_barrier,
    .direct_end_barrier = hwraid1_direct_end_barrier,
};

static struct raidset_s hwraid0_raidset = {
    .biotocmd = hwraid0_bio_to_cmd,
    .direct_cmd = hwraid0_direct_cmd,
    .wait_barrier = NULL,
    .allow_barrier = NULL,
    .direct_start_barrier = NULL,
    .direct_end_barrier = NULL,
};

static struct command_s cmd_for_direct_access;
static struct command_s cmd_for_request_q;
static struct command_s cmd_for_sync;

/**
 * The command barrier is used to stop and new commands from being started
 * the system cannot change the RAID geometry if there are any commands in use
 * cmds_out is the count of commands in use.
 */
static atomic_t cmds_out = ATOMIC_INIT(0);
static volatile int command_barrier = 0;

static struct host_s host = {
    .raid_port = {
        .port.base = (u32*)SATARAID_REGS_BASE,
    },
    .sata_port = {
        { .port.base = (u32*)SATA0_REGS_BASE, .port.debug_portno = 0 },
        { .port.base = (u32*)SATA1_REGS_BASE, .port.debug_portno = 1 }
    },

    .active_cmd = NULL,
    .active_cmd_lock = __SPIN_LOCK_UNLOCKED(host.active_cmd_lock),
#ifdef ERROR_INJECTION
    .error_period = 0,
#endif
};

/** lock for the request queue */
spinlock_t rq_lock;

/* work queue for error handling */
static struct workqueue_struct* error_handler_wq;

/* record of pending hotplug events. A bitfield, bit per port */
static unsigned long ox820hwraid_hotplug_events = 0;

#ifdef REQUEST_DONE_BH
/* tasklet for completing requests */
static DECLARE_TASKLET(ox820hwraid_req_completion_bh, request_done_bh, 0 );
#endif

/**************************************************************************/
/* Functions                                                              */
/**************************************************************************/

/**
 * Used by request queue commands to release the sata core. It will inform
 * The libATA stack if any hotplug events have occured
 */
void ox820hwraid_release_sata_core(sata_locker_t locker_type)
{
    DPRINTK("\n");
    if (unlikely(ox820hwraid_hotplug_events)) {
        int i;
        for (i = 0; i < NUMBER_OF_DISKS; ++i) {
            if (test_and_clear_bit(i, &ox820hwraid_hotplug_events)) {
                ox820sata_checkforhotplug(i);
            }
        }
    }
    release_sata_core_without_restart(locker_type);
}

/* also used by the resync command initialisation that happens at runtime
so cannot be __init */
static inline void init_command_basic(struct command_s* cmd)
{
    /* clear flags */
    cmd->error_flags = 0;
    cmd->flags = 0;
    cmd->faults = 0;

    /* init the command timeout timer */
    init_timer(&cmd->command_timeout);
}

/* Exists for consistency */
static inline void exit_command_basic(struct command_s* cmd)
{
}

/**
 * Used to prepare a command for use by the request queue.
 */
static void __init init_command_request_queue(struct command_s* cmd)
{
    init_command_basic(cmd);

    /* allocate a list */
    BUG_ON(odrb_alloc_prd_array(&cmd->odrb_prd_list, 1, 1));
    /* setup sg table in the request queue command */
    sg_init_table(cmd->sg, NUMBER_OF_DMA_ELEMENTS);

}

static void __exit exit_command_request_queue(struct command_s* cmd)
{
    /* free allocated sg prd table or sg list */
    odrb_free_prd_array(cmd->odrb_prd_list);
    cmd->odrb_prd_list = NULL;
    exit_command_basic(cmd);
}

/**
 * Used to prepare a command for use by the direct_code
 */
static void __init init_command_direct(struct command_s* cmd)
{
    init_command_basic(cmd);

    /* the direct code will be supplying a physical pointer to the sgdma
     * list, but need to allocate this structure so it can be stored in the
     * same place as the request command's sgdma stuff */
    cmd->odrb_prd_list = kzalloc( sizeof(odrb_prd_list_t), GFP_KERNEL);
}

static void __exit exit_command_direct(struct command_s* cmd)
{
    kfree(cmd->odrb_prd_list);
    exit_command_basic(cmd);
}

/**
 * returns the first free comand that the request queue can use or null if
 * there are none available
 */
static inline struct command_s* get_command_req(void)
{
    /* The incrementing and decrementing is used like this to avoid a race
    condition with the command_barrier */
    smp_mb__before_atomic_inc();
    atomic_inc(&cmds_out);
    smp_mb__after_atomic_inc();
    if (likely(!command_barrier) &&
        !test_and_set_bit(in_use, &(cmd_for_request_q.flags))) {
        DPRINTK("got cmd\n");
        return &cmd_for_request_q;
    } else {
        smp_mb__before_atomic_dec();
        atomic_dec(&cmds_out);
        smp_mb__after_atomic_dec();
        return NULL;
    }
}

/**
 * returns the first free comand that the direct command path can use or null if
 * there are none available
 */
static inline struct command_s* get_command_direct(void)
{
    struct command_s* chosen_cmd = NULL;

    if (!test_and_set_bit(in_use, &(cmd_for_direct_access.flags))) {
        DPRINTK("got cmd\n");
        chosen_cmd = &cmd_for_direct_access;

        /* The locking used by the above fn is done differently for direct
        commands, but as the same put_command function is used at the end
        of every command, the cmds_out variable must still be incremented */
        smp_mb__before_atomic_inc();
        atomic_inc(&cmds_out);
        smp_mb__after_atomic_inc();
    }
    return chosen_cmd;
}

static inline struct command_s* get_sync_cmd(void)
{
    struct command_s* chosen_cmd = NULL;

    if (!test_and_set_bit(in_use, &(cmd_for_sync.flags))) {
        chosen_cmd = &cmd_for_sync;

        /* The locking used by the above fn is done differently for direct
        commands, but as the same put_command function is used at the end
        of every command, the cmds_out variable must still be incremented */
        smp_mb__before_atomic_inc();
        atomic_inc(&cmds_out);
        smp_mb__after_atomic_inc();
    }
    DPRINTK("returning %p, flags = %08x\n",chosen_cmd, cmd_for_sync.flags);
    return chosen_cmd;
}
/**
 * marks a previously used command as available for use again.
 */
static inline void put_command(struct command_s* cmd)
{
    cmd->error_flags = 0;
    cmd->faults = 0;
    cmd->flags = 0;
    smp_mb__before_atomic_dec();
    atomic_dec(&cmds_out);
    smp_mb__after_atomic_dec();
}

int hwraid_stop_new_commands(void)
{
    /* stop any new commands from being issued */
    command_barrier = 1;

    smp_mb();
    /* if there are any outstanding commands then quit an try again later */
    if (atomic_read(&cmds_out)) {
        CPRINTK("commands busy\n");
        return -EBUSY;
    }
    return 0;
}

void hwraid_start_new_commands(void)
{
    /* start commands and restart queue */
    command_barrier = 0;
    smp_wmb();
    CPRINTK(" done\n");
}

/******************************************************************************/

/**
 * returns a pointer to the host structure.
 */
static inline struct host_s* get_host(void)
{
    return &host;
}

/**
 * Returns the active command if there is one, clearing it in the process.
 */
static inline struct command_s* get_and_clear_active_command(void)
{
    unsigned long flags;
    struct command_s* cmd;
    struct host_s* host = get_host();
    if (!host) {
        return NULL;
    }

    spin_lock_irqsave(&host->active_cmd_lock, flags);
    cmd = host->active_cmd;
    host->active_cmd = NULL;
    spin_unlock_irqrestore(&host->active_cmd_lock, flags);

    return cmd;
}

/**
 * sets the active command in the host
 */
static inline void set_active_command(struct command_s* cmd)
{
    unsigned long flags;
    struct host_s* host = get_host();

    BUG_ON(!cmd);
    spin_lock_irqsave(&host->active_cmd_lock, flags);
    BUG_ON(host->active_cmd);
    host->active_cmd = cmd;
    spin_unlock_irqrestore(&host->active_cmd_lock, flags);
}

/**
 * fills in a PRD table from a scatterlist.
 */
static void sg_to_prd
    (struct scatterlist* sg_list, int nents, odrb_prd_list_t* dma_table)
{
    int i;
    int count;
    struct scatterlist* sg_element;
    prd_table_entry_t* prd;

    prd = dma_table->prds;
    /* loop through all the sg elements */
    count = 0;
    for_each_sg(sg_list, sg_element, nents, i) {
        unsigned int addr = sg_dma_address(sg_element);
        unsigned int len = sg_dma_len(sg_element);

        /* chop the transfer into managable chunks */
        while (len) {
            prd->adr = cpu_to_le32(addr);
            if (unlikely(len > MAX_PRD_ELEMENT_SIZE)) {
                prd->flags_len = cpu_to_le32(MAX_PRD_ELEMENT_SIZE & 0xffff);
                prd++;
                count++;
                len -= MAX_PRD_ELEMENT_SIZE;
                addr += MAX_PRD_ELEMENT_SIZE;
            } else {
                prd->flags_len = cpu_to_le32(len & 0xffff);
                len = 0;
            }
            DPRINTK("prd addr %p addr %08x len/flag %08x\n",
                prd, prd->adr, prd->flags_len);
        }

        /* Last one? set flag */
        if (unlikely((i+1) == nents)) {
            prd->flags_len |= PRD_EOF_MASK;
        }
        prd++;
        count++;
    }

    BUG_ON(count > NUMBER_OF_DMA_ELEMENTS);
}

/**
 * as sg_to_prd(), but assumes that all sg entries are less than 64k
 */
static inline void sg_to_prd_fast
    (struct scatterlist* sg_list, int nents, odrb_prd_list_t* dma_table)
{
    int i;
    struct scatterlist* sg_element;
    prd_table_entry_t* prd = dma_table->prds;

    DPRINTK("%d elements\n",nents);
    /* loop through all the sg elements */
    for_each_sg(sg_list, sg_element, nents, i) {
        prd->adr = cpu_to_le32(sg_dma_address(sg_element));
        prd->flags_len = cpu_to_le32(sg_dma_len(sg_element));

        /* Last one? set flag */
        if (unlikely((i+1) == nents)) {
            prd->flags_len |= PRD_EOF_MASK;
        }
        prd++;
    }
}

/**
 * Called when there may have been a hotplug event, this function will
 * examine the SERROR registers and confirm the hotplug. If there is one, then
 * it will set a flag the should cause the HWRAID code to release the sata core
 * asap and get libata to start its hotplug processing
 */
void ox820hwraid_checkforhotplug(unsigned int port_no)
{
    u32 serror;
    serror = ox820sata_link_read(get_host()->sata_port[port_no].port.base, 0x24);
    if(serror & (SERR_DEV_XCHG | SERR_PHYRDY_CHG)) {
        /* log that a hotplug event has occured */
        set_bit(port_no, &ox820hwraid_hotplug_events);
        smp_wmb();
    }
}

/**
 * Called from the irq handler if the normal driver doesn't own up to the
 * interrupt.
 */
irqreturn_t ox820hwraid_irq_handler(int irq, unsigned long arg)
{
    struct command_s* cmd;
    int int_status = readl(OX820SATA_CORE_INT_STATUS);

    /* clear any interrupt */
    writel(int_status, OX820SATA_CORE_INT_CLEAR);

    /* look for hotplug events on both ports */
    if (unlikely(readl((u32*)SATA0_REGS_BASE + OX820SATA_INT_STATUS) & OX820SATA_INT_LINK_SERROR)) {
        ox820hwraid_checkforhotplug(0);
    }
    if (unlikely(readl((u32*)SATA1_REGS_BASE + OX820SATA_INT_STATUS) & OX820SATA_INT_LINK_SERROR)) {
        ox820hwraid_checkforhotplug(1);
    }

    /* If a command hasn't finished we don't want to know */
    if ( likely(int_status & OX820SATA_COREINT_END) ) {
        /* get the active command */
        cmd = get_and_clear_active_command();
        if (likely(cmd)) {
            /* call the command's analyse function to analyse the interrupt */
            cmd->analyse(cmd, 0);
            return IRQ_HANDLED;
        }
    }
    return IRQ_NONE;
}
EXPORT_SYMBOL( ox820hwraid_irq_handler );

/**
 * At any time we could be interrupted by aa completing command.
 */
static void ox820hwraid_timeout(unsigned long unused)
{
    struct command_s* cmd;

    DPRINTK("\n");
    /* get the active command */
    cmd = get_and_clear_active_command();
    if (cmd) {
        /* call the command's analyse function to analyse the error */
        cmd->analyse(cmd, 1);
    } else {
        printk(KERN_ERR"ox820hwraid timeout, no command\n");
        BUG();
    }
}

/**************************************************************************/
/* RAID-0                                                                 */
/**************************************************************************/
/**
 * Assesses if the current raid configuration is suitable for implementation
 * by the raid HW, if so, will enable it in the mddev
 */
int hwraid0_compatible(mddev_t *mddev, struct raid0_private_data* conf)
{
    mdk_rdev_t *rdev0;
    mdk_rdev_t *rdev1;

    DPRINTK("\n");
    /* if this drive is suitable for HW raid then enable it */
    if (mddev->raid_disks != 2) {
        printk(KERN_NOTICE"raid0 not hw raidable %d disks (needs to be 2)\n",
            mddev->raid_disks);
        return 0;
    }

    if (conf->nr_strip_zones != 1) {
        printk(KERN_NOTICE"raid0 not hw raidable %d zone (needs to be 1)\n",
            conf->nr_strip_zones);
        return 0;
    }

    if (conf->strip_zone->nb_dev != 2) {
        printk(KERN_NOTICE"raid0 not hw raidable %d disks (needs to be 2)\n",
            conf->strip_zone->nb_dev);
        return 0;
    }

    /* snhould be able to do this safely as we know there is only one stripe
    zone and two devices. */
    rdev0 = conf->devlist[0];
    rdev1 = conf->devlist[1];

    /* are there two working disks */
    if (!rdev0 ||
        !rdev1 ||
        test_bit(Faulty, &rdev0->flags) ||
        test_bit(Faulty, &rdev1->flags) ) {
        printk(KERN_NOTICE"raid0 not hw raidable, needs two working disks.\n");
        return 0;
    }

    if (!rdev0->bdev ||
        !rdev1->bdev ||
        !rdev0->bdev->bd_part ||
        !rdev1->bdev->bd_part ) {
        printk(KERN_NOTICE"raid0 not hw raidable, disks not ready\n");
        return 0;
	}

	if (rdev0->bdev->bd_part->start_sect != rdev1->bdev->bd_part->start_sect) {
#ifdef CONFIG_LBDAF
        printk(KERN_NOTICE"raid0 not hw raidable, partition start sectors differ %llu, %llu\n",
#else
        printk(KERN_NOTICE"raid0 not hw raidable, partition start sectors differ %lu, %lu\n",
#endif // CONFIG_LBDAF
            rdev0->bdev->bd_part->start_sect,
            rdev1->bdev->bd_part->start_sect);
        return 0;
    }
	if (rdev0->bdev->bd_part->start_sect & 255) {
            printk(KERN_NOTICE"raid0 not hw raidable, partitions need"
                " to start on a multiple of 256 sectors\n");
        return 0;
    }

    if (!rdev0->bdev->bd_disk ||
		!rdev0->bdev->bd_disk->queue ||
		(oxnassata_get_port_no(rdev0->bdev->bd_disk->queue) != 0)) {
        printk(KERN_NOTICE"raid0 not hw raidable, RAID disk 0 not on internal SATA port.\n");
        return 0;
    }

    if (!rdev1->bdev->bd_disk ||
		!rdev1->bdev->bd_disk->queue ||
		(oxnassata_get_port_no(rdev1->bdev->bd_disk->queue) != 1)) {
        printk(KERN_NOTICE"raid1 not hw raidable, RAID disk 1 not on internal SATA port.\n");
        return 0;
    }

	/* cannot mix 28 and 48-bit LBA devices */
	if (!oxnassata_LBA_schemes_compatible()) {
        printk(KERN_NOTICE"raid0 not hw raidable, disks need to use same LBA size (28 vs 48)\n");
		return 0;
	}

    printk(KERN_NOTICE"raid0 using hardware RAID\n");
    return 1;
}

/**
 * This is called after first checking that the RAID layout is suitable for
 * the hardware.
 */
int hwraid0_assemble(struct mddev_s* mddev, struct raid0_private_data* conf )
{
    struct raidset_s* raid;
    int ret;
    mdk_rdev_t *rdev0;
    mdk_rdev_t *rdev1;

    CPRINTK("\n");
    /* create the raid0 data structure */
    if (raiddev->raidset) {
        printk(KERN_ERR"raid set already seems to be present\n");
        return -ENOMEM;
    }

    raid = kmalloc( sizeof(struct raidset_s),GFP_KERNEL);
    if (!raid) {
        printk(KERN_ERR"Could not allocate memory for a RAID object.\n");
        return -ENOMEM;
    }
    raiddev->raidset = raid;

    /* initialise the commands */
    cmd_for_direct_access.raid = raid;
    cmd_for_request_q.raid = raid;

    /* copy in standard settings */
    memcpy(raid, &hwraid0_raidset, sizeof(struct raidset_s));

    raid->host = raiddev->host;

    rdev0 = conf->devlist[0];
    rdev1 = conf->devlist[1];

    /* allocate the partition structures and fill in */
    raid->partition[0] = NULL;
    raid->partition[0] = kmalloc( sizeof(struct component_partition_s),GFP_KERNEL);
    raid->partition[1] = NULL;
    raid->partition[1] = kmalloc( sizeof(struct component_partition_s),GFP_KERNEL);

    if (!(raid->partition[0] && raid->partition[1])) {
        printk(KERN_ERR"Could not allocate memory for a RAID object.\n");

        /* free partition storage */
        if (raid->partition[0]) {
            kfree(raid->partition[0]);
            raid->partition[0] = NULL;
        }
        if (raid->partition[1]) {
            kfree(raid->partition[1]);
            raid->partition[1] = NULL;
        }

        return -ENOMEM;
    }

    /* know that the host ports are this way around from checks */
    raid->partition[0]->port = &raiddev->host->sata_port[0].port;
    raid->partition[1]->port = &raiddev->host->sata_port[1].port;

    /* know that the start sectors are the same from the checks */
    raid->start = rdev0->bdev->bd_part->start_sect;
    raid->mddev = mddev;

    /* setup block device */
    ret = ox820hwraid_disk_up(raiddev->raidset);
    if (ret >=0 ) {
        mddev->hw_raid = raid;
    }

    return ret;
}

void hwraid0_stop(struct mddev_s* mddev, struct raid0_private_data* conf )
{
    struct raidset_s* raid = mddev->hw_raid;

    DPRINTK("\n");
    /* check that all direct access is done */

    ox820hwraid_disk_down(raid);

    /* free partition storage */
    if (raid->partition[0]) {
        kfree(raid->partition[0]);
        raid->partition[0] = NULL;
    }
    if (raid->partition[1]) {
        kfree(raid->partition[1]);
        raid->partition[1] = NULL;
    }

    mddev->hw_raid = NULL;
    raiddev->raidset = NULL;
    kfree(raid);
}

/**
 * @return A command to be run on the SATA hardware, must be cleaned up with
 *         put_command()
 */
static int hwraid0_direct_cmd(struct direct_access_context* context,
                              int write,
                              sector_t start,
                              sector_t nsect,
                              dma_addr_t dma_list_pa,
                              void (*done)(int error, void *data),
                              void *data)
{
    struct command_s* cmd;
    struct raidset_s* raid;
	struct raid_context_s* raid_context =
	    container_of( context, struct raid_context_s, base_context);

    DPRINTK("\n");
    raid = raid_context->raid;
    cmd = get_command_direct();
    if (cmd) {
        cmd->write = write;
        /* form  a command */
        if ( write ) {
            cmd->ata_cmd = ATA_CMD_WRITE_EXT;
        } else {
            cmd->ata_cmd = ATA_CMD_READ_EXT;
        }

        cmd->sector = (raid->start * 2) + start;
        cmd->nsects = nsect;
        cmd->port = &raid->host->raid_port.port;

        cmd->execute_command = hwraid0_execute_command;
        cmd->done = done;
        cmd->data = data;
        cmd->analyse = hwraid0_analysis;
        cmd->odrb_prd_list->phys = dma_list_pa;

        DPRINTK("cmd %x, loc %llx, num %llx\n",
            cmd->ata_cmd, cmd->sector, cmd->nsects );
        DPRINTK("sglst %x, sgdma %p, port %p\n",
            cmd->odrb_prd_list->phys, cmd->odrb_prd_list, cmd->port );
        return 0;
    } else {
        return -EIO;
    }
}

static struct command_s* hwraid0_bio_to_cmd(struct raidset_s* raid,
                          struct request_queue* queue,
                          struct request* req)
{
    int nents;
    struct command_s* cmd = get_command_req();

    DPRINTK("\n");
    if (cmd) {
        /* form  a command */
        if ( rq_data_dir(req) ) {
            cmd->write = 1;
            cmd->ata_cmd = ATA_CMD_WRITE_EXT;
        } else {
            cmd->write = 0;
            cmd->ata_cmd = ATA_CMD_READ_EXT;
        }

        cmd->sector = (raid->start * 2) + blk_rq_pos(req);
        cmd->nsects = blk_rq_sectors(req);
        cmd->port = &raid->host->raid_port.port;

        cmd->execute_command = hwraid0_execute_command;
        cmd->done = request_done;
        cmd->data = req;
        cmd->analyse = hwraid0_analysis;

        /* convert the bio vectors into a scatter list/prd table */
        nents = blk_rq_map_sg(queue, req, cmd->sg);

        /* map the scatterlist */
        dma_map_sg(NULL, cmd->sg, nents,
            rq_data_dir(req) ? DMA_TO_DEVICE : DMA_FROM_DEVICE );

        /* convert into a prd table */
        sg_to_prd(cmd->sg, nents, cmd->odrb_prd_list);
        DPRINTK("cmd %x, loc %llx, num %llx\n",
            cmd->ata_cmd, cmd->sector, cmd->nsects );
        DPRINTK("sglst %x, sgdma %p, port %p\n",
            cmd->odrb_prd_list->phys, cmd->odrb_prd_list, cmd->port );
    }
    return cmd;
}

static void hwraid0_eh_work(struct work_struct *work)
{
    struct command_s* cmd = container_of(work, struct command_s, eh_work);
    struct host_s* host = get_host();

    /* report and cleanup the error */
    hwraid_log(cmd);
    hwraid_cleanup(cmd);

    if ( cmd->faults < MAX_TRIES ) {
        DPRINTK("retrying cmd\n");
        cmd->execute_command(cmd, host);
    } else {
        done_fn_t* done = cmd->done;
        void* data = cmd->data;
        put_command(cmd);
        done(1, data);
    }
}

/**
 * Called by the interrupt handler or the timeout gubbins
 */
static void hwraid0_analysis(struct command_s* cmd, int timeout)
{
    int error = timeout;

    DPRINTK("\n");

    /* Try and stop the timer (we are in interrupt context, so can't use
     * the sync versions) */
    if (likely(!timeout)) {
        del_timer(&cmd->command_timeout);
    }

    /* check for an error */
    error |= readl(cmd->port->base + OX820SATA_INT_STATUS) & OX820SATA_RAW_ERROR;

    if( unlikely(error) ) {
        cmd->faults++;
        /* freeze the host so to stop kblockd from polling us to death */
        ox820sata_freeze_host(0);
        if (timeout)
            set_bit(command_timedout, &cmd->error_flags);
        smp_wmb();
        INIT_WORK(&cmd->eh_work, hwraid0_eh_work);
        queue_work(error_handler_wq, &cmd->eh_work);
    } else {
        done_fn_t* done = cmd->done;
        void* data = cmd->data;
        put_command(cmd);
        done(0, data);
    }
}

static void hwraid_log(struct command_s* cmd)
{
    u32 error;
    int port_no;
    struct host_s* host = get_host();

    DPRINTK("\n");
    if (test_bit(command_timedout, &cmd->error_flags)) {
        printk(KERN_ERR"Timed out waiting for hardware RAID SATA command to finish\n");
        printk("cmd port base = %p\n",cmd->port->base);
    } else {
        printk(KERN_ERR"Error found during hardware RAID SATA command\n");
    }

    printk(KERN_ERR"Cmd=%02x Sector=%012llx NSect=%04x\n",
        cmd->ata_cmd, cmd->sector, (int)cmd->nsects );

    for (port_no = 0; port_no < NUMBER_OF_DISKS; ++port_no) {
        u32 serror;
        u32* base = host->sata_port[port_no].port.base;
        /* which error bits are set in the port */
        error = readl(base + OX820SATA_INT_STATUS);
        if (error & OX820SATA_RAW_ERRORS) {
            int orb_err;

            printk(KERN_ERR"Error on port %d\n", port_no);

            /* examine the orb error bytes for each port */
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
        serror = ox820sata_link_read(host->sata_port[port_no].port.base, 0x24);
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
        if(serror & (SERR_DEV_XCHG | SERR_PHYRDY_CHG)) {
            /* log that a hotplug event has occured */
            set_bit(port_no, &ox820hwraid_hotplug_events);
            smp_wmb();
        }
    }

    /* which error bits are set in the dma */
    odrb_decode_sg_error();
}


static void hwraid_cleanup(struct command_s* cmd)
{
    int action;
    DPRINTK("\n");

    action = ox820sata_cleanup();
    while (action) {
        /* send reset */
        sata_std_hardreset( &ox820sata_get_ap(0)->link, NULL, jiffies + HZ);
        sata_std_hardreset( &ox820sata_get_ap(1)->link, NULL, jiffies + HZ);
        action = ox820sata_cleanup();
    }

    /* all errors gone */
    cmd->error_flags = 0;
	ox820sata_thaw_host(0);
}


/**
 * When this is called, we have control of the sata core, the command has
 * been decided, all we need to do is write the command into hardware then
 * return.
 *
 * May be called from an interrupt context.
 */
static int hwraid0_execute_command( struct command_s* cmd, struct host_s* host)
{
    struct port_s* port = cmd->port;
    unsigned int* base = port->base;

    u32 Orb1 = 0;
    u32 Orb2 = 0;
    u32 Orb3 = 0;
    u32 Orb4 = 0;
    u32 reg;

    DPRINTK("\n");

    /* check the core is idle */
    if (DEBUG_CODE &&
        (readl(base + OX820SATA_SATA_COMMAND) & CMD_CORE_BUSY))
    {
        int count = 0;
        DPRINTK(KERN_ERR"core busy for a raid-0 command\n");
        do {
            mdelay(1);
            if (++count > 100) {
                DPRINTK(KERN_ERR"core busy for a raid-0 command\n");
                break;
            }
        } while (readl(base + OX820SATA_SATA_COMMAND) & CMD_CORE_BUSY);
    }

    /* check both cables are connected */
    if (likely(ox820sata_check_link(0) &&
               ox820sata_check_link(1) &&
               !(readl(base + OX820SATA_SATA_COMMAND) & CMD_CORE_BUSY))) 
    {
        /* mark the command as active */
        set_active_command(cmd);
    
        /* set raid mode if required */
        ox820sata_set_mode(OXNASSATA_RAID0, 0);
    
        /* Work around for a work around for a bug in another chip that uses a
        * similar SATA core. Set error mask bits, the RAID micro-code inverts
        * these bits to turn them off again */
        writel( 0x3, OX820SATA_PORT_ERROR_MASK);
    
        /* Prepare the DMA controller for a PRD driven transfer */
        odrb_dma_sata_prd_nogo(cmd->write ? OXNAS_DMA_TO_DEVICE : OXNAS_DMA_FROM_DEVICE,
            cmd->nsects, cmd->odrb_prd_list->phys);
    
        /* enable passing of error signals to DMA sub-core by clearing the
        appropriate bit (all transfers are on dma channel 0)*/
        reg = readl(OX820SATA_DATA_PLANE_CTRL);
        reg &= ~(OX820SATA_DPC_ERROR_MASK_BIT);
        writel(reg, OX820SATA_DATA_PLANE_CTRL);
    
        /* clear any pending interrupts */
        writel(~0, (u32*)SATA0_REGS_BASE + OX820SATA_INT_CLEAR);
        writel(~0, (u32*)SATA1_REGS_BASE + OX820SATA_INT_CLEAR);
        writel(~0, (u32*)SATARAID_REGS_BASE + OX820SATA_INT_CLEAR);
        writel(~0, OX820SATA_CORE_INT_CLEAR);
    
        /* enable End of command interrupt */
        writel(~OX820SATA_INT_WANT, base + OX820SATA_INT_DISABLE);
        writel(OX820SATA_INT_WANT, base + OX820SATA_INT_ENABLE);
        writel(OX820SATA_COREINT_END, OX820SATA_CORE_INT_ENABLE);
    
        /* write in command registers */
        Orb1 = ATA_LBA << 24;
    
        Orb2 |= cmd->nsects & 0xffff ;
        Orb2 |= cmd->ata_cmd << 24;
    
        Orb3 = cmd->sector & (u32)0xffffffff;
    
        Orb4 |= (cmd->sector >> 32) & (u32)0x0000ffff ;
    
        /* write values to registers */
        writel(Orb1, base + OX820SATA_ORB1 );
        writel(Orb2, base + OX820SATA_ORB2 );
        writel(Orb3, base + OX820SATA_ORB3 );
        writel(Orb4, base + OX820SATA_ORB4 );
    
        /* start DMA */
        odrb_dma_sata_prd_go();
    
        reg = injectErrors(&host->sata_port[0].port, cmd->write) ;
        reg |= injectErrors(&host->sata_port[1].port, cmd->write);
        if (reg) {
            printk(KERN_ERR"Error injected on Cmd=%02x Sector=%012llx NSect=%04x\n",
                cmd->ata_cmd, cmd->sector, (int)cmd->nsects );
        }
    
        /* start timeout timer */
        cmd->command_timeout.function = ox820hwraid_timeout;
        mod_timer(&cmd->command_timeout, jiffies + COMMAND_TIMEOUT);
    
        /* start command */
        reg = readl(base + OX820SATA_SATA_COMMAND);
        reg &= ~SATA_OPCODE_MASK;
        reg |= CMD_WRITE_TO_ORB_REGS;
        writel(reg , base + OX820SATA_SATA_COMMAND);
    } else {
        /* call the analyse function directly with an error */
        cmd->analyse(cmd, 1);
    }
    return 0;
}

/**************************************************************************/
/* RAID-1                                                                 */
/**************************************************************************/

/**
 * Finds a mirror number when given a port, returns -1 if it can't find a
 * match
 */
static int mirror_no_from_port(struct raidset_s* raid, struct port_s* port)
{
    int mirror = -1;
    if ((raid) && (port)) {
        int i;
        for(i = 0; i < NUMBER_OF_DISKS; ++i) {
            if ((raid->partition[i]) &&
                (raid->partition[i]->port == port))
            {
                mirror = raid->partition[i]->mirror;
            }
        }
    }

    return mirror;
}

/* returns true if both ports are working */
static inline int both_ports_work(struct raidset_s* raid)
{
    smp_rmb();
    return raid->partition[0] &&
           raid->partition[1] &&
           port_is_reliable(raid->partition[0]->port) &&
           port_is_reliable(raid->partition[1]->port) ;
}

/**
 * Assesses if the current raid configuration is suitable for implementation
 * by the raid HW. It doesn't indicate writes should be sent to two drives
 * further checks are required to determine that.
 */
static int hwraid1_compatible(mddev_t *mddev)
{
    mdk_rdev_t *rdev;
    int disk;
    int disks_working = 0;
    int disks_insync = 0;
    int disks_ready = 0;
    sector_t start_sect = ~0;
    conf_t *conf = mddev->private;

    CPRINTK("\n");
    /* if this drive is suitable for HW raid then enable it */
    if (mddev->raid_disks != 2) {
        printk(KERN_NOTICE"%s has %d disks, needs to be 2 for HW RAID-1\n",
            mdname(mddev), mddev->raid_disks);
        return 0;
    }

    /* cannot mix 28 and 48-bit LBA devices */
	if (!oxnassata_LBA_schemes_compatible()) {
        printk(KERN_NOTICE"%s not suitable for HW RAID-1 as disks need to "
            "use same LBA size (28 vs 48)\n", mdname(mddev));
		return 0;
	}

    rcu_read_lock();
    for (disk = 0; disk < 2; disk++) {
        rdev = rcu_dereference(conf->mirrors[disk].rdev);

        /* is the disk working */
        if (!rdev)
            continue;

        if  (!test_bit(Faulty, &rdev->flags)) {
            CPRINTK(" disk working\n");
            disks_working++;
            if  (test_bit(In_sync, &rdev->flags)) {
                CPRINTK(" disk in sync\n");
                disks_insync++;
            }
        }

        /* is the disk ready */
        if (!rdev->bdev || !rdev->bdev->bd_part ) {
            continue;
        }
        CPRINTK(" disk ready\n");
        disks_ready++;

        if (!rdev->bdev->bd_disk ||
            !rdev->bdev->bd_disk->queue ||
            (oxnassata_get_port_no(rdev->bdev->bd_disk->queue) < 0))
        {
            printk(KERN_NOTICE"%s not suitable for HW RAID-1 as, disk %s is "
                "not connected to the RAID_controller.\n",
                mdname(mddev),
                rdev->bdev->bd_disk->disk_name);
            goto not_suitable;
        }

        if (start_sect == ~0) {
            /* this is the first disk with a start sector */
            start_sect = rdev->bdev->bd_part->start_sect;
        } else if (rdev->bdev->bd_part->start_sect != start_sect) {
            printk(KERN_NOTICE"%s not suitable for HW RAID-1 as start of"
                " partitions for differ "
#ifdef CONFIG_LBDAF
                "(%llu, %llu)\n",
#else
                "(%lu, %lu)\n",
#endif // CONFIG_LBDAF
                mdname(mddev),
                start_sect,
                rdev->bdev->bd_part->start_sect);
            goto not_suitable;
        }
    }
	rcu_read_unlock();

    if (!disks_working) {
        printk(KERN_NOTICE"%s not suitable for HW RAID-1 as no disks are "
            "working.\n", mdname(mddev));
        return 0;
    }

    if (!disks_ready) {
        printk(KERN_NOTICE"%s not suitable for HW RAID-1 as no disks are "
            "ready.\n", mdname(mddev));
        return 0;
    }

    if (!disks_insync) {
        printk(KERN_NOTICE"%s not suitable for HW RAID-1 as no ready disks "
            "are in sync.\n", mdname(mddev));
        return 0;
    }

    printk(KERN_NOTICE"%s using HW RAID-1\n", mdname(mddev));

    return 1;
not_suitable:
	rcu_read_unlock();
    return 0;
}

/**
 * Determine if the port is still reliable
 */
static inline int port_is_reliable(struct port_s* port)
{
    int reliable;
    smp_rmb();
    reliable = (port->faults < MAX_TRIES);

    DPRINTK(" %d/%d = %s\n",
        port->faults,
        MAX_TRIES,
        reliable ? "reliable" : "unreliable" );
    return reliable;
}

/**
 * pick the most reliable, fastest disk for reading, only use if there are
 * two disks
 */
static struct port_s* hwraid1_read_balance(struct raidset_s* raid, sector_t start)
{
    int i;
    sector_t closest_distance = ~0;
    struct port_s* closest_port = NULL;
    struct port_s* last_port;
    static int first_disk_to_examine = 0;
    int disk;

    DPRINTK("\n");
    /* if consecutive sectors, send to the same disk as last time */
    last_port = raid->last_port;
    
    /* this should force the one read above and prevent multiple reads of 
     * raid->last_port which may be changed by other proceses during this fn
     */
    rmb(); 
    if ( last_port &&
        (last_port->headpos == start) &&
        !last_port->write_only &&
        port_is_reliable(last_port) )
    {
        DPRINTK("concurrent commands \n");
        return last_port;
    }

    /* pseudo randomize our initial search position */
    first_disk_to_examine++;
    if (first_disk_to_examine >= NUMBER_OF_DISKS) {
        first_disk_to_examine = 0;
    }
    disk = first_disk_to_examine;
    
    /* pick the reliable port with it's head closest to the start of the data */
    for (i = 0;i < NUMBER_OF_DISKS; ++i) {
        s64 distance;
        struct port_s* port;

        disk++;
        if (disk >= NUMBER_OF_DISKS) {
            disk = 0;
        }

        /* if there is a partition entry */
        if (raid->partition[disk]) {
            port = raid->partition[disk]->port;

            /* if we can't read from the disk, skip it... */
            smp_rmb();
            if (unlikely(!port_is_reliable(port))) {
                continue;
            }
            
            if (unlikely(port->write_only)) {
                /* ...but check more thoroughly first, in case recovery has
                 * made the disk rw again */
                struct r1_private_data_s* conf = raid->mddev->private;
                int mirror_no = raid->partition[disk]->mirror;
                mdk_rdev_t* rdev = rcu_dereference(conf->mirrors[mirror_no].rdev);
                if (!test_bit(Faulty, &rdev->flags) &&
                     test_bit(In_sync, &rdev->flags))
                {
                    char b[BDEVNAME_SIZE];

                    port->write_only = 0;
                    printk(KERN_INFO"HW-RAID1: %s, is now read-write.\n",
                        bdevname(rdev->bdev, b));
                    rcu_read_unlock();
                } else {
                    rcu_read_unlock();
                    continue;
                }
            }

            /* calc distance, with extra penalty for backwards head movement */
            distance = start - port->headpos;

#if ((SEEK_BACKWARDS_MULTIPLE == 1) && (SEEK_BACKWARDS_ADDITIONAL == 0))
            distance = abs(distance);
#else            
            if (distance < 0) {
                distance = abs(distance);
                distance *= (SEEK_BACKWARDS_MULTIPLE);
                distance += SEEK_BACKWARDS_ADDITIONAL;
            }
#endif
            DPRINTK(" port %d head at %llu cmd at %llu cost %llu\n",
                disk, port->headpos, start, distance);

            if (distance < closest_distance) {
                closest_distance = distance;
                closest_port = port;
            }
        }
    }

    WARN_ON(!closest_port);
    return closest_port;
}

/**
 * Returns a pointer to a reliable port, or null if it cannot find one.
 */
static struct port_s* find_reliable_port(struct raidset_s* raid)
{
    int disk;

    DPRINTK("\n");

    /* iterate through the partitions looking for a reliable port */
    for (disk = 0;disk < NUMBER_OF_DISKS; ++disk) {
        struct component_partition_s* partition = raid->partition[disk];
        if ((partition) &&
            (partition->port) &&
            port_is_reliable(partition->port))
        {
            return partition->port;
        }
    }
    return NULL;
}

static struct command_s* hwraid1_bio_to_cmd
    (struct raidset_s* raid, struct request_queue* queue, struct request* req)
{
    int nents;
    struct command_s* cmd = get_command_req();
    DPRINTK("\n");

    if (cmd) {
        /* form  a command */
        cmd->sector  = raid->start + blk_rq_pos(req);;
        cmd->nsects  = blk_rq_sectors(req);
        cmd->done    = request_done;
        cmd->data    = req;

        if ( rq_data_dir(req) ) {
            /* write */
            cmd->write = 1;
            cmd->ata_cmd = ATA_CMD_WRITE_EXT;

            if (both_ports_work(raid)) {
                /* two ports working */
                DPRINTK("two ports working\n");
                cmd->port = &raid->host->raid_port.port;
                cmd->execute_command = hwraid1_execute_command_double;
                cmd->analyse = hwraid1_double_analysis;
            } else {
                /* one port */
                DPRINTK("one port working\n");
                cmd->port = find_reliable_port(raid);
                cmd->execute_command = hwraid1_execute_command_single;
                cmd->analyse = hwraid1_single_analysis;
            }
        } else {
            /* read */
            cmd->write = 0;
            cmd->ata_cmd = ATA_CMD_READ_EXT;
            cmd->port = hwraid1_read_balance(raid, cmd->sector);
            raid->last_port = cmd->port;

            cmd->execute_command = hwraid1_execute_command_single;
            cmd->analyse = hwraid1_read_analysis;
        }
        /* convert the bio vectors into a scatter list/prd table */
        nents = blk_rq_map_sg(queue, req, cmd->sg);

        /* map the scatterlist */
        dma_map_sg(NULL, cmd->sg, nents,
            rq_data_dir(req) ? DMA_TO_DEVICE : DMA_FROM_DEVICE );

        /* convert into a prd table */
        sg_to_prd(cmd->sg, nents, cmd->odrb_prd_list);
        DPRINTK("cmd %x, loc %llx, num %llx\n", cmd->ata_cmd, cmd->sector, cmd->nsects );
    }
    return cmd;
}

static int hwraid1_direct_cmd(
    struct direct_access_context* context,
    int write,
    sector_t start,
    sector_t nsect,
    dma_addr_t dma_list_pa,
    void (*done)(int error, void *data),
    void *data)
{
    struct command_s* cmd;
    struct raidset_s* raid;
	struct raid_context_s* raid_context =
	    container_of( context, struct raid_context_s, base_context);

    DPRINTK("\n");
    raid = raid_context->raid;
    cmd = get_command_direct();
    if (cmd) {
        cmd->sector  = raid->start + start;
        cmd->nsects  = nsect;
        cmd->done    = done;
        cmd->data    = data;
        cmd->write   = write;
        cmd->odrb_prd_list->phys = dma_list_pa;

        if ( write ) {
            /* write */
            cmd->ata_cmd = ATA_CMD_WRITE_EXT;

            if (both_ports_work(raid)) {
                /* two ports working */
                cmd->port = &raid->host->raid_port.port;
                cmd->execute_command = hwraid1_execute_command_double;
                cmd->analyse = hwraid1_double_analysis;
            } else {
                /* one port */
                cmd->port = find_reliable_port(raid);
                cmd->execute_command = hwraid1_execute_command_single;
                cmd->analyse = hwraid1_single_analysis;
            }
        } else {
            /* read */
            cmd->ata_cmd = ATA_CMD_READ_EXT;
            cmd->port = hwraid1_read_balance(raid, cmd->sector);
            raid->last_port = cmd->port;

            cmd->execute_command = hwraid1_execute_command_single;
            cmd->analyse = hwraid1_read_analysis;
        }
        DPRINTK("cmd %x, loc %llx, num %llx\n", cmd->ata_cmd, cmd->sector, cmd->nsects );
        return 0;
    } else {
        return -EIO;
    }
}

/* only want one partition to be considered for hardware raid at this time */
static int hwraid1_basic_checks(struct mddev_s* mddev)
{
    CPRINTK("minor no %d\n", mddev->md_minor);
    return (mddev->md_minor == 4);
}

static int hwraid1_disassemble(struct mddev_s* mddev)
{
    int i;
    struct raidset_s* raid = mddev->hw_raid;

    CPRINTK("\n");
    ox820hwraid_disk_down(raid) ;

    /* clear existing partitons */
    for (i = 0; i < NUMBER_OF_DISKS; i++) {
        if (raid->partition[i]) {
            kfree(raid->partition[i]);
            raid->partition[i] = 0;
        }
    }

    return 0;
}

/**
 * called after establishing that the configuration of mddev is suitable for
 * hardware raid. mddev->raid must point to a raidset. will return < 0 if
 * there is an error during assembly
 */
static int hwraid1_assemble(struct mddev_s* mddev)
{
    mdk_rdev_t* rdev;
    int mirror;
    int ret;
    struct raidset_s* raid = mddev->hw_raid;
    conf_t* conf = mddev->private;

    CPRINTK("\n");
    /* Loop through the mirrors */
    rcu_read_lock();
    for (mirror = 0; mirror < mddev->raid_disks; mirror++) {
        int port_no;
        struct component_partition_s* partition;

        rdev = rcu_dereference(conf->mirrors[mirror].rdev);
        if (rdev) {
            char b[BDEVNAME_SIZE];
            int write_only = 0;

            /* check if the rdev has a disk connected to the internal
             * sata port */
            port_no = oxnassata_get_port_no(rdev->bdev->bd_disk->queue);
            if (port_no < 0) {
                printk(KERN_INFO"HW-RAID1 not using %s, disk not on internal sata port\n",
                    bdevname(rdev->bdev, b));
                continue;
            }

            /* check for faulty disks */
            if (test_bit(Faulty, &rdev->flags)) {
                printk(KERN_INFO"HW-RAID1 not using %s, partition is faulty.\n",
                    bdevname(rdev->bdev, b));
                continue;
            }

            /* check for in-sync disks */
            if (!test_bit(In_sync, &rdev->flags))
            {
                printk(KERN_INFO"HW-RAID1 %s, is write only.\n",
                    bdevname(rdev->bdev, b));
                write_only = 1;
            } else {
                printk(KERN_INFO"HW-RAID1 %s, is read/write.\n", bdevname(rdev->bdev, b));
            }

            /* check for existing partition */
            if (raid->partition[mirror]) {
                CPRINTK("Already a disk setup for mirror no %d\n",mirror);
                CPRINTK("it is  %s , start %llu, size %llu, mirror %d\n",
                    raid->partition[mirror]->port->write_only ? "wo" : "rw",
                    raid->partition[mirror]->start,
                    raid->partition[mirror]->size,
                    raid->partition[mirror]->mirror );
                continue;
            }
            
            /* create partition object */
            partition =
                kmalloc( sizeof(struct component_partition_s), GFP_KERNEL);
            if (!partition) {
                printk(KERN_ERR"Could not allocate memory for a partition object.\n");
                ret = -ENOMEM;
                /* free everything and exit with null*/
                break;
            }

            printk(KERN_INFO"HW-RAID1 using disk %s on port %d mirror %d\n",
                bdevname(rdev->bdev, b), port_no, mirror);
            /* Have now found that rdev describes a disk connected to a
            SATA port on our HWRAID controller, set up the partition info */
            partition->start = rdev->bdev->bd_part->start_sect;
            partition->size = mddev->array_sectors;
            partition->port = &raid->host->sata_port[port_no].port;
            partition->port->write_only = write_only;
            partition->mirror = mirror;
            raid->partition[mirror] = partition;
            raid->start = rdev->bdev->bd_part->start_sect;
        }
    }
	rcu_read_unlock();

    /* setup block device so that requests go through our request handler */
    ret = ox820hwraid_disk_up(raiddev->raidset);
    return ret;
}


/**
 * Will creates a hardware raid-1 device if it can
 */
int hwraid1_run(struct mddev_s* mddev, struct r1_private_data_s* conf)
{
    struct raidset_s* raid;
    int err = 0;
    int use_hwraid;

    DPRINTK("\n");
    /* basic checks to see if this is the user data partition */
    if (!hwraid1_basic_checks(mddev)) {
        return 0;
    }

    /* create the RAID-1 data structure */
    raid = kmalloc( sizeof(struct raidset_s),GFP_KERNEL);
    if (!raid) {
        printk(KERN_ERR"Could not allocate memory for a RAID object.\n");
        return -ENOMEM;
    }
    raiddev->raidset = raid;

    /* initialise the commands */
    cmd_for_direct_access.raid = raid;
    cmd_for_request_q.raid = raid;

    memcpy(raid, &hwraid1_raidset, sizeof(struct raidset_s));

    /* setup work queue and a work item for waiting for superblock writes */
    raid->wait_event_anticipation = create_workqueue("hwraid1-wait");
    INIT_WORK(&raid->work, hwraid1_sbwrite_waiting);

    raid->host = get_host();
    raid->mddev = mddev;
    mddev->hw_raid = raid;

    /* Is the array suitable for hardware raid */
    use_hwraid = hwraid1_compatible(mddev);
    if (use_hwraid) {
        /* yes, build with hwraid */
        err = hwraid1_assemble(mddev);
    }

    return err;
}

/**
 * A new disk has been added to the raid array, this function should
 * re-assess the HWRAID compatibility. We can assume that the array is not
 * already in the array.
 */
int hwraid1_add_disk(struct mddev_s* mddev, mdk_rdev_t* rdev)
{
    char b[BDEVNAME_SIZE];
    int port_no;
    struct component_partition_s* partition;
    int err = 0;
    struct raidset_s* raid = mddev->hw_raid;
    int mirror = rdev->raid_disk;

    bdevname(rdev->bdev, b);
    CPRINTK("%s\n", bdevname(rdev->bdev, b) );

    if (mirror < 0) {
        CPRINTK("passed a spare\n");
        BUG();
    }

    if (hwraid1_compatible(mddev)) {
        int write_only = 0;

        /* check for existing partition */
        if (raid->partition[mirror]) {
            CPRINTK("Already a disk setup for mirror no %d\n",mirror);
            CPRINTK("it is  %s , start %llu, size %llu, mirror %d\n",
                raid->partition[mirror]->port->write_only ? "wo" : "rw",
                raid->partition[mirror]->start,
                raid->partition[mirror]->size,
                raid->partition[mirror]->mirror );
            return 0;
        }

        /* check if the rdev has a disk connected to the internal
         * sata port */
        port_no = oxnassata_get_port_no(rdev->bdev->bd_disk->queue);
        if (port_no < 0) {
            printk(KERN_INFO"HW-RAID1 not using %s, disk not on internal sata port\n",
                bdevname(rdev->bdev, b));
            return 0;
        }

        /* check for faulty disks */
        if (test_bit(Faulty, &rdev->flags)) {
            printk(KERN_INFO"HW-RAID1 not using %s, partition is faulty.\n",
                bdevname(rdev->bdev, b));
            return 0;
        }

        /* check for in-sync disks */
        if (!test_bit(In_sync, &rdev->flags)) {
            printk(KERN_INFO"HW-RAID1 %s, is write only.\n", bdevname(rdev->bdev, b));
            write_only = 1;
        } else {
            printk(KERN_INFO"HW-RAID1 %s, is read/write.\n", bdevname(rdev->bdev, b));
        }

        partition = kmalloc( sizeof(struct component_partition_s), GFP_KERNEL);
        if (!partition) {
            printk(KERN_ERR"HW-RAID1 Could not allocate memory for a partition object.\n");
            return -ENOMEM;
        }

        printk(KERN_INFO"HW-RAID1 using disk %s on port %d mirror %d\n",
            bdevname(rdev->bdev, b), port_no, mirror);

        /* Have now found that rdev describes a disk connected to a
        SATA port on our HWRAID controller, set up the partition info */
        partition->start = rdev->bdev->bd_part->start_sect;
        partition->size = mddev->array_sectors;
        partition->port = &raid->host->sata_port[port_no].port;
        partition->port->write_only = write_only;
        partition->port->faults = 0;
        partition->mirror = mirror;
        raid->partition[mirror] = partition;
        smp_wmb();

        /* can turn on hardware RAID now if it wasn't already on */
        if (!raid->hw_raid_active) {
            ox820hwraid_disk_up(raid);
        }
    }

    ox820hwraid_restart_queue();
    CPRINTK("done\n");

    return err;
}

int hwraid1_remove_disk(struct mddev_s* mddev, int mirror, mdk_rdev_t* rddev)
{
    int use_hwraid;
    int err = 0;
    struct raidset_s* raid = mddev->hw_raid;

    CPRINTK("\n");

    /* find the partition */
    if (raid->partition[mirror]) {
        CPRINTK("removing partition %d\n",mirror);
        /* remove it */
        kfree(raid->partition[mirror]);
        raid->partition[mirror] = NULL;
    } else {
        CPRINTK("partition %d not present\n",mirror);
    }

    /* Is the array suitable for hardware raid */
    use_hwraid = hwraid1_compatible(mddev);

    /* if suitability has changed, start or stop hardware raid */
    if (use_hwraid != raid->hw_raid_active) {
        if (use_hwraid) {
            ox820hwraid_disk_up(raid);
        } else {
            ox820hwraid_disk_down(raid);
        }
    }
    hwraid_start_new_commands();
    return err;
}

/**
 * Called when the array has changed size (same no. of disks, different
 * disk size.
 */
int  hwraid1_resize(struct mddev_s* mddev, sector_t sectors)
{
    CPRINTK("\n");
    return hwraid1_add_disk(mddev, NULL);
}

/**
 * Called when the array is about to be reshaped
 */
void hwraid1_reshape_begin(struct mddev_s* mddev)
{
    struct raidset_s* raid = mddev->hw_raid;

    CPRINTK("\n");
    /* were we using hardware raid */
    if (raid->hw_raid_active) {
        /* if yes dismantle */
        hwraid1_disassemble(mddev);
    }
}

/**
 * called when the array has been reshaped and accesses can resume.
 */
void hwraid1_reshape_end(struct mddev_s* mddev)
{
    int use_hwraid;
    int err = 0;

    CPRINTK("\n");

    /* Is the array suitable for hardware raid */
    use_hwraid = hwraid1_compatible(mddev);
    if (use_hwraid) {
        /* yes, build with hwraid */
        err = hwraid1_assemble(mddev);
    }
}

/**
 * Called when the RAID array is about to stop.
 */
void hwraid1_stop(struct mddev_s* mddev)
{
    struct raidset_s* raid = mddev->hw_raid;

    CPRINTK("\n");

    /* were we using hardware raid */
    if (raid->hw_raid_active) {
        /* if yes dismantle */
        hwraid1_disassemble(mddev);
        raid->hw_raid_active = 0;
    }
    smp_wmb();

    if (raid->wait_event_anticipation) {
        destroy_workqueue(raid->wait_event_anticipation);
        raid->wait_event_anticipation = NULL;
    }
    smp_wmb();

    if (raid) {
        mddev->hw_raid = NULL;
        raiddev->raidset = NULL;
        kfree(raid);
    }
}

/**
 * In response to one disk being marked as faulty, will go through all the
 * disks mark the faulty ones as unreliable.
 */
void hwraid1_error(struct mddev_s* mddev, mdk_rdev_t* rdev_that_failed)
{
    int disk;
    mdk_rdev_t* rdev;
    struct raidset_s* raid = mddev->hw_raid;
	conf_t *conf = mddev->private;

    CPRINTK("\n");

    rcu_read_lock();
    /* check that rdev is faulty */
    if (test_bit(Faulty, &rdev_that_failed->flags)) {
        /* go through all partitions to find those that are faulty */
        for (disk = 0; disk < NUMBER_OF_DISKS; disk++) {
            if (raid->partition[disk]) {
                int mirror_no = raid->partition[disk]->mirror;
                rdev = rcu_dereference(conf->mirrors[mirror_no].rdev);
                if (test_bit(Faulty, &rdev->flags)) {
#ifdef RAID_CONFIG_DEBUG
                    int old_faults = raid->partition[disk]->port->faults;
#endif
                    /* mark the fault as unreliable */
                    raid->partition[disk]->port->faults = DEFINITELY_BROKEN;
                    CPRINTK("Changing partition %d faults from %d to %d\n",
                        disk, old_faults, raid->partition[disk]->port->faults);
                }
            }
        }
    }
	rcu_read_unlock();
}

/**
 * When this is called, we have control of the sata core, the command has
 * been decided, all we need to do is write the command into hardware then
 * return.
 *
 * May be called from an interrupt context.
 */
static int hwraid1_execute_command_double( struct command_s* cmd, struct host_s* host)
{
    struct port_s* port = cmd->port;
    unsigned int* base = port->base;

    u32 Orb1 = 0;
    u32 Orb2 = 0;
    u32 Orb3 = 0;
    u32 Orb4 = 0;
    u32 reg;

    DPRINTK("\n");

    /* check the core is idle */
    if((readl((u32*)SATA0_REGS_BASE + OX820SATA_SATA_COMMAND) & CMD_CORE_BUSY) || 
       (readl((u32*)SATA1_REGS_BASE + OX820SATA_SATA_COMMAND) & CMD_CORE_BUSY) ) 
    {
        int count = 0;
        DPRINTK(KERN_ERR"core busy hw-raid-1 command\n");
        do {
            mdelay(1);
            if (++count > 100) {
                DPRINTK(KERN_ERR"core busy hw-raid-1 command\n");
                break;
            }
        } while (
            (readl((u32*)SATA0_REGS_BASE + OX820SATA_SATA_COMMAND) & CMD_CORE_BUSY) || 
            (readl((u32*)SATA1_REGS_BASE + OX820SATA_SATA_COMMAND) & CMD_CORE_BUSY) );
    }

    /* check both cables are connected */
    if (likely(ox820sata_check_link(0) &&
               ox820sata_check_link(1) &&
       !(readl((u32*)SATA0_REGS_BASE + OX820SATA_SATA_COMMAND) & CMD_CORE_BUSY) && 
       !(readl((u32*)SATA1_REGS_BASE + OX820SATA_SATA_COMMAND) & CMD_CORE_BUSY) ))
    {
        /* mark the command as active */
        set_active_command(cmd);

        /* set raid mode if required */
        ox820sata_set_mode(OXNASSATA_RAID1, 0);

        /* Work around for a work around for a bug in another chip that uses a
        * similar SATA core. Set error mask bits, the RAID micro-code inverts
        * these bits to turn them off again */
        writel( 0x3, OX820SATA_PORT_ERROR_MASK);

        /* Prepare the DMA controller for a PRD driven transfer */
        odrb_dma_sata_prd_nogo(cmd->write ? OXNAS_DMA_TO_DEVICE : OXNAS_DMA_FROM_DEVICE,
            cmd->nsects, cmd->odrb_prd_list->phys);

        /* enable passing of error signals to DMA sub-core by clearing the
        appropriate bit (all transfers are on dma channel 0)*/
        reg = readl(OX820SATA_DATA_PLANE_CTRL);
        reg &= ~(OX820SATA_DPC_ERROR_MASK_BIT);
        writel(reg, OX820SATA_DATA_PLANE_CTRL);

        /* clear any pending interrupts */
        writel(~0, (u32*)SATA0_REGS_BASE + OX820SATA_INT_CLEAR);
        writel(~0, (u32*)SATA1_REGS_BASE + OX820SATA_INT_CLEAR);
        writel(~0, (u32*)SATARAID_REGS_BASE + OX820SATA_INT_CLEAR);
        writel(~0, OX820SATA_CORE_INT_CLEAR);

        /* enable End of command interrupt */
        writel(~OX820SATA_INT_WANT, base + OX820SATA_INT_DISABLE);
        writel(OX820SATA_INT_WANT, base + OX820SATA_INT_ENABLE);
        writel(OX820SATA_COREINT_END, OX820SATA_CORE_INT_ENABLE);

        /* write in command registers */
        Orb1 = ATA_LBA << 24;

        Orb2 |= cmd->nsects & 0xffff ;
        Orb2 |= cmd->ata_cmd << 24;

        Orb3 = cmd->sector & (u32)0xffffffff;

        Orb4 |= (cmd->sector >> 32) & (u32)0x0000ffff ;

        /* write values to registers */
        writel(Orb1, base + OX820SATA_ORB1 );
        writel(Orb2, base + OX820SATA_ORB2 );
        writel(Orb3, base + OX820SATA_ORB3 );
        writel(Orb4, base + OX820SATA_ORB4 );

        /* start DMA */
        odrb_dma_sata_prd_go();

        reg = injectErrors(&host->sata_port[0].port, cmd->write) ;
        reg |= injectErrors(&host->sata_port[1].port, cmd->write);
        if (reg) {
            printk(KERN_ERR"Error injected on Cmd=%02x Sector=%012llx NSect=%04x\n",
                cmd->ata_cmd, cmd->sector, (int)cmd->nsects );
        }

        /* start timeout timer */
        cmd->command_timeout.function = ox820hwraid_timeout;
        mod_timer(&cmd->command_timeout, jiffies + COMMAND_TIMEOUT);

        /* start command */
        reg = readl(base + OX820SATA_SATA_COMMAND);
        reg &= ~SATA_OPCODE_MASK;
        reg |= CMD_WRITE_TO_ORB_REGS;
        writel(reg , base + OX820SATA_SATA_COMMAND);
    } else {
        DPRINTK(KERN_ERR"cable disconnect detected.\n");
        /* call the analyse function directly with an error */
        cmd->analyse(cmd, 1);
    }
    return 0;
}

/**
 * When this is called, we have control of the sata core, the command has
 * been decided, all we need to do is write the command into hardware then
 * return.
 *
 * May be called from an interrupt context.
 */
static int hwraid1_execute_command_single( struct command_s* cmd, struct host_s* host)
{
    struct port_s* port = cmd->port;
    unsigned int* base = port->base;

    u32 Orb1 = 0;
    u32 Orb2 = 0;
    u32 Orb3 = 0;
    u32 Orb4 = 0;
    u32 reg;

    DPRINTK("\n");

    /* check the core is idle */
    if((readl((u32*)SATA0_REGS_BASE + OX820SATA_SATA_COMMAND) & CMD_CORE_BUSY) || 
       (readl((u32*)SATA1_REGS_BASE + OX820SATA_SATA_COMMAND) & CMD_CORE_BUSY) )
    {
        int count = 0;
        DPRINTK(KERN_ERR"core busy for a raid cmd on port %d\n", cmd->port->debug_portno);
        do {
            mdelay(1);
            if (++count > 100) {
                DPRINTK(KERN_ERR"core busy for a raid command\n");
                break;
            }
        } while (
            (readl((u32*)SATA0_REGS_BASE + OX820SATA_SATA_COMMAND) & CMD_CORE_BUSY) || 
            (readl((u32*)SATA1_REGS_BASE + OX820SATA_SATA_COMMAND) & CMD_CORE_BUSY) );
    }

    if (likely(ox820sata_check_link(cmd->port->debug_portno) &&
       !(readl((u32*)SATA0_REGS_BASE + OX820SATA_SATA_COMMAND) & CMD_CORE_BUSY) && 
       !(readl((u32*)SATA1_REGS_BASE + OX820SATA_SATA_COMMAND) & CMD_CORE_BUSY) ))
    {
        /* mark the command as active */
        set_active_command(cmd);
    
        /* set raid mode if required */
        ox820sata_set_mode(OXNASSATA_RAID1, 0);
    
        /* Turn the work around off */
        writel( 0x0, OX820SATA_PORT_ERROR_MASK);

        /* Prepare the DMA controller for a PRD driven transfer */
        odrb_dma_sata_prd_nogo(cmd->write ? OXNAS_DMA_TO_DEVICE : OXNAS_DMA_FROM_DEVICE,
            cmd->nsects, cmd->odrb_prd_list->phys);
    
        /* enable passing of error signals to DMA sub-core by clearing the
        appropriate bit (all transfers are on dma channel 0)*/
        reg = readl(OX820SATA_DATA_PLANE_CTRL);
        reg &= ~(OX820SATA_DPC_ERROR_MASK_BIT);
        writel(reg, OX820SATA_DATA_PLANE_CTRL);
    
        /* clear any pending interrupts */
        writel(~0, (u32*)SATA0_REGS_BASE + OX820SATA_INT_CLEAR);
        writel(~0, (u32*)SATA1_REGS_BASE + OX820SATA_INT_CLEAR);
        writel(~0, (u32*)SATARAID_REGS_BASE + OX820SATA_INT_CLEAR);
        writel(~0, OX820SATA_CORE_INT_CLEAR);
    
        /* enable End of command interrupt */
        writel(~OX820SATA_INT_WANT, base + OX820SATA_INT_DISABLE);
        writel(OX820SATA_INT_WANT, base + OX820SATA_INT_ENABLE);
        writel(OX820SATA_COREINT_END, OX820SATA_CORE_INT_ENABLE);
    
        /* write in command registers */
        Orb1 = ATA_LBA << 24;
    
        Orb2 |= cmd->nsects & 0xffff ;
        Orb2 |= cmd->ata_cmd << 24;
    
        Orb3 = cmd->sector & (u32)0xffffffff;
    
        Orb4 |= (cmd->sector >> 32) & (u32)0x0000ffff ;
    
        /* write values to registers */
        writel(Orb1, base + OX820SATA_ORB1 );
        writel(Orb2, base + OX820SATA_ORB2 );
        writel(Orb3, base + OX820SATA_ORB3 );
        writel(Orb4, base + OX820SATA_ORB4 );
    
        /* start DMA */
        odrb_dma_sata_prd_go();
    
        if (injectErrors(port, cmd->write)) {
            printk(KERN_ERR"Error injected on Cmd=%02x Sector=%012llx NSect=%04x\n",
                cmd->ata_cmd, cmd->sector, (int)cmd->nsects );
        }
    
        /* start timeout timer */
        cmd->command_timeout.function = ox820hwraid_timeout;
        mod_timer(&cmd->command_timeout, jiffies + COMMAND_TIMEOUT);
    
        /* start command */
        reg = readl(base + OX820SATA_SATA_COMMAND);
        reg &= ~SATA_OPCODE_MASK;
        reg |= CMD_WRITE_TO_ORB_REGS;
        writel(reg , base + OX820SATA_SATA_COMMAND);
    } else {
        DPRINTK(KERN_ERR"cable disconnect detected.\n");
        /* call the analyse function directly with an error */
        cmd->analyse(cmd, 1);
    }
    return 0;
}

/**
 * Tells the raid layer that commands to both disks have failed.
 */
static void hwraid1_report_double_error(struct command_s* cmd, int not_used)
{
    struct mddev_s* mddev = cmd->raid->mddev;
    conf_t *conf = mddev->private;

    /* would normally do this before a command starts, but do it here to
    save time by only processing bitmap info for failed commands */
    bitmap_startwrite(mddev->bitmap, cmd->sector, cmd->nsects, 0);

    /* have to assume that this will only be called for raid arrays that have
    two mirrors */
    md_error(mddev, conf->mirrors[0].rdev);
    md_error(mddev, conf->mirrors[1].rdev);

    /* end with failure and not behind */
    bitmap_endwrite(mddev->bitmap, cmd->sector, cmd->nsects, 0, 0);
}

/**
 * Tells the raid layer of a write error/failure to "mirror".
 */
static void hwraid1_report_single_error(struct command_s* cmd, int mirror)
{
    struct mddev_s* mddev = cmd->raid->mddev;
    conf_t *conf = mddev->private;

    /* would normally do this before a command starts, but do it here to
    save time by only processing bitmap info for failed commands */
    bitmap_startwrite(mddev->bitmap, cmd->sector, cmd->nsects, 0);

    /* have to assume that this will only be called for raid arrays that have
    two mirrors */
    md_error(mddev, conf->mirrors[mirror].rdev);

    /* end with failure and not behind */
    bitmap_endwrite(mddev->bitmap, cmd->sector, cmd->nsects, 0, 0);
}

/**
 * Tells the raid layer of an read error/failure to "mirror".
 */
static void hwraid1_report_read_error(struct command_s* cmd, int mirror)
{
    struct mddev_s* mddev = cmd->raid->mddev;
    conf_t *conf = mddev->private;

    /* have to assume that this will only be called for raid arrays that have
    two mirrors */
    md_error(mddev, conf->mirrors[mirror].rdev);
}

static void hwraid1_double_eh_work(struct work_struct *work)
{
    int i;
    struct port_s* working_port = NULL;
    int working_ports = 0;
    struct command_s* cmd = container_of(work, struct command_s, eh_work);
    struct host_s* host = get_host();
    /* Check the ports to see which port was the one that errd.
     * Note: "i" is the physical port number, as opposed to an index
     * to the order that partition structures are created
     */
    for(i = 0; i < NUMBER_OF_DISKS; ++i) {
        struct port_s* port = &(host->sata_port[i].port);
        int port_error = 0;
        int int_status;

        if (!ox820sata_check_link(i)) {
            DPRINTK(KERN_ERR"cable disconnected on port %d\n",i);
            port_error |= 1;
        }

        int_status = readl(port->base + OX820SATA_INT_STATUS);
        if (int_status & OX820SATA_RAW_ERROR) {
            DPRINTK("error found on port %d\n",i);
            port_error |= 1;
        }

        if (port_error) {
            port->faults++;
        } else {
            /* update head position */
            port->headpos = cmd->sector + cmd->nsects;
        }

        /* is this port still working */
        if (port->faults < MAX_TRIES) {
            working_ports++;
            working_port = port;
        }
    }

    hwraid_log(cmd);
    hwraid_cleanup(cmd);

    /* Keep trying until a port gives up */
    if (working_ports == NUMBER_OF_DISKS) {
        printk(KERN_ERR"retrying command\n");
        cmd->execute_command(cmd, host);
    } else {
        host->sata_port[0].port.faults = 0;
        host->sata_port[1].port.faults = 0;
        if (working_port) {
            printk(KERN_ERR"retrying command on one port\n");
            /* re-encode command so that it executes on the remaining port */
            cmd->port = working_port;
            cmd->faults = 0;
            cmd->execute_command = hwraid1_execute_command_single;
            cmd->analyse = hwraid1_degraded_analysis;
            cmd->execute_command(cmd, host);
        } else {
            /* no ports left, give up */
            done_fn_t* done = cmd->done;
            void* data = cmd->data;
            printk(KERN_ERR"no working ports left, reporting errors\n");
            hwraid1_report_double_error(cmd, 0);
            put_command(cmd);
            done(1, data);
        }
    }
}

/**
 * Called by the interrupt handler or the timeout gubbins for write to two
 * disks
 */
static void hwraid1_double_analysis(struct command_s* cmd, int timeout)
{
    struct host_s* host = get_host();
    int error = timeout;

    DPRINTK("\n");

    /* Try and stop the timer (we are in interrupt context, so can't use
     * the sync versions) */
    if (likely(!timeout)) {
        del_timer(&cmd->command_timeout);
    }

    /* check for an error */
    error |= readl(cmd->port->base + OX820SATA_INT_STATUS) & OX820SATA_RAW_ERROR;

    if( unlikely(error) ) {
        cmd->faults++;
        /* freeze the host so to stop kblockd from polling us to death */
        ox820sata_freeze_host(0);
        if (timeout)
            set_bit(command_timedout, &cmd->error_flags);
        smp_wmb();
        INIT_WORK(&cmd->eh_work, hwraid1_double_eh_work);
        queue_work(error_handler_wq, &cmd->eh_work);
    } else {
        /* no errors, record head positions and complete */
        done_fn_t* done = cmd->done;
        void* data = cmd->data;

        /* update head positions */
        host->sata_port[0].port.headpos = cmd->sector + cmd->nsects;
        host->sata_port[1].port.headpos = cmd->sector + cmd->nsects;

        /* all good */
        host->sata_port[0].port.faults = 0;
        host->sata_port[1].port.faults = 0;

        put_command(cmd);
        done(0, data);
    }
}

static void hwraid1_sync_eh_work(struct work_struct* work)
{
    struct command_s* cmd = container_of(work, struct command_s, eh_work);
    struct host_s* host = get_host();

    /* report and cleanup the error */
    hwraid_log(cmd);
    hwraid_cleanup(cmd);

    smp_rmb();
    if( cmd->faults < MAX_TRIES ) {
        /* can retry on the same disk */
        printk(KERN_ERR"retrying cmd\n");
        cmd->execute_command(cmd, host);
    } else {
        /* exit with an error */
        done_fn_t* done = cmd->done;
        void* data = cmd->data;
        put_command(cmd);
        done(1, data);
    }
}

static void __hwraid1_single_eh_work(
    struct command_s* cmd, void (report_error)(struct command_s*, int))
{
    struct host_s* host = get_host();

    /* report and cleanup the error */
    hwraid_log(cmd);
    hwraid_cleanup(cmd);

    smp_rmb();
    if( cmd->faults < MAX_TRIES ) {
        /* can retry on the same disk */
        printk(KERN_ERR"retrying cmd\n");
        cmd->execute_command(cmd, host);
    } else {
        struct port_s* working_port;
        int mirror = mirror_no_from_port(cmd->raid, cmd->port);

        /* report the error on the broken disk */
        report_error(cmd, mirror);

        /* if there are other disks, retry on them */
        working_port = find_reliable_port(cmd->raid);

        /* the system won't fail the last remaining port, but we don't want
        to retry the command on it indefinitely, so fail the request if it
        suggests using the same port again */
        if (working_port == cmd->port) {
            working_port = NULL;
        }

        if (working_port) {
            /* re-encode command so that it executes on the remaining port */
            cmd->port = working_port;
            cmd->faults = 0;
            cmd->execute_command(cmd, host);
        } else {
            /* already reported the error to raid, can now exit with an error */
            done_fn_t* done = cmd->done;
            void* data = cmd->data;
            put_command(cmd);
            done(1, data);
        }
    }
}

static void hwraid1_single_eh_work(struct work_struct *work)
{
    struct command_s* cmd = container_of(work, struct command_s, eh_work);
    __hwraid1_single_eh_work(cmd, hwraid1_report_single_error);
}

static void hwraid1_read_eh_work(struct work_struct *work)
{
    struct command_s* cmd = container_of(work, struct command_s, eh_work);
    __hwraid1_single_eh_work(cmd, hwraid1_report_read_error);
}

/**
 * Called for reads or writes that were sent to one physical disk.
 *
 * If writes are using this, then there is only one disk left in the array
 *
 *
 */
static inline void __hwraid1_single_analysis(
    struct command_s* cmd,
    int timeout,
    void error_handler_work(struct work_struct *work))
{
    int error = timeout;

    DPRINTK("\n");

    /* Try and stop the timer (we are in interrupt context, so can't use
     * the sync versions) */
    if (likely(!timeout)) {
        del_timer(&cmd->command_timeout);
    }

    /* check for an error */
    error |= readl(cmd->port->base + OX820SATA_INT_STATUS) & OX820SATA_RAW_ERROR;

    if ( unlikely(error) ) {
        cmd->faults++;
        /* freeze the host so to stop kblockd from polling us to death */
        ox820sata_freeze_host(0);
        if (timeout)
            set_bit(command_timedout, &cmd->error_flags);
        smp_wmb();
        INIT_WORK(&cmd->eh_work, error_handler_work);
        queue_work(error_handler_wq, &cmd->eh_work);
    } else {
        done_fn_t* done = cmd->done;
        void* data = cmd->data;

        /* update head position */
        cmd->port->headpos = cmd->sector + cmd->nsects;

        put_command(cmd);
        done(error, data);
    }
}

/**
 * Used for commands that start when the array only has a single disk
 */
static void hwraid1_single_analysis(struct command_s* cmd, int timeout)
{
    __hwraid1_single_analysis(cmd, timeout, hwraid1_single_eh_work);
}

/**
 * Used for commands that start when the array only has a single disk
 */
static void hwraid1_read_analysis(struct command_s* cmd, int timeout)
{
    __hwraid1_single_analysis(cmd, timeout, hwraid1_read_eh_work);
}

/**
 * Used for commands that start when the array only has a single disk
 */
static void hwraid1_sync_analysis(struct command_s* cmd, int timeout)
{
    __hwraid1_single_analysis(cmd, timeout, hwraid1_sync_eh_work);
}

static void hwraid1_degraded_eh_work(struct work_struct *work)
{
    struct command_s* cmd = container_of(work, struct command_s, eh_work);
    struct host_s* host = get_host();

    /* report and cleanup the error */
    hwraid_log(cmd);
    hwraid_cleanup(cmd);

    smp_rmb();
    if( cmd->faults < MAX_TRIES ) {
        /* can retry on the same disk */
        printk(KERN_ERR"retrying cmd\n");
        cmd->execute_command(cmd, host);
    } else {
        done_fn_t* done = cmd->done;
        void* data = cmd->data;

        printk(KERN_ERR"giving up\n");
        /* the other disk has already failed, report error on both disks */
        hwraid1_report_double_error(cmd, 0);

        put_command(cmd);
        done(1, data);
    }
}


/**
 * Used when an array was two-disk when the command started, but is now
 * single disk.
 * when the command succeeds, it has to report the error that caused it to
 * switch to a single disk.
 */
static void hwraid1_degraded_analysis(struct command_s* cmd, int timeout)
{
    int error = timeout;

    DPRINTK("\n");

    /* Try and stop the timer (we are in interrupt context, so can't use
     * the sync versions) */
    if (likely(!timeout)) {
        del_timer(&cmd->command_timeout);
    }

    /* check for an error */
    error |= readl(cmd->port->base + OX820SATA_INT_STATUS) & OX820SATA_RAW_ERROR;

    if ( unlikely(error) ) {
        cmd->faults++;
        if (timeout)
            set_bit(command_timedout, &cmd->error_flags);
        smp_wmb();
        INIT_WORK(&cmd->eh_work, hwraid1_degraded_eh_work);
        queue_work(error_handler_wq, &cmd->eh_work);
    } else {
        done_fn_t* done = cmd->done;
        void* data = cmd->data;
        int mirror = mirror_no_from_port(cmd->raid, cmd->port);
        BUG_ON( mirror == -1 );

        /* update head position */
        cmd->port->headpos = cmd->sector + cmd->nsects;

        /* report the error that got us into the degraded state */
        mirror = 1 - mirror; /* (0->1, 1->0) */
        hwraid1_report_single_error(cmd, mirror);

        put_command(cmd);
        done(error, data);
    }
}

static void hwraid1_sbwrite_waiting(struct work_struct* ws)
{
    unsigned long flags;
    struct raidset_s* raid = container_of(ws, struct raidset_s, work);
    mddev_t* mddev = (mddev_t* )raid->mddev;

    DPRINTK("sb write work started\n");
	wait_event(mddev->sb_wait,
		   !test_bit(MD_CHANGE_CLEAN, &mddev->flags) &&
		   !test_bit(MD_CHANGE_PENDING, &mddev->flags));

    DPRINTK("conditions met, continuing\n");
    if (blk_queue_stopped(mddev->queue)) {
        /* make the request queue re-examine the barrier */
        spin_lock_irqsave(mddev->queue->queue_lock, flags);
        blk_start_queue(mddev->queue);
        spin_unlock_irqrestore(mddev->queue->queue_lock, flags);
	}
}


static void hwraid1_raise_barrier(conf_t *conf)
{
    mddev_t* mddev = (mddev_t* )conf->mddev;
	spin_lock_irq(&conf->resync_lock);

	/* Wait until no block IO is waiting */
	wait_event_lock_irq(conf->wait_barrier, !conf->nr_waiting,
			    conf->resync_lock,
			    md_wakeup_thread(mddev->thread));

	/* block any new IO from starting */
	conf->barrier++;

	/* No wait for all pending IO to complete */
	wait_event_lock_irq(conf->wait_barrier,
			    !conf->nr_pending && conf->barrier < HWRAID1_RESYNC_DEPTH,
			    conf->resync_lock,
			    md_wakeup_thread(mddev->thread));

	spin_unlock_irq(&conf->resync_lock);
}

/* will return -1 if the request is blocked */
static int
hwraid1_wait_barrier(struct command_s* cmd, struct raidset_s* raid, int write)
{
    unsigned long flags;
    mddev_t *mddev = raid->mddev;
	conf_t *conf = raid->mddev->private;

	/* This implements the code from md_write_start without the wait_event
    * which can cause problems when running atomically. The wait queue is
    * implemented by returning requests onto the request queue to be retried
    * later. A work item will ensure that the request queue is woken up when
    * conditions are right */
	if (write) {
	    DPRINTK("write\n");
	    /* if we've done this before then skip it */
	    if (!test_and_set_bit(started_md_write, &cmd->flags) ) {
            int did_change = 0;

            /* shouldn't get here if read-only */
            BUG_ON(mddev->ro == 1);

            DPRINTK("started md_write_start code\n");
            /* if read-auto, switch to rw, with changes to superblock and
            recovery states */
            if (mddev->ro == 2) {
                /* need to switch to read/write */
                mddev->ro = 0;
                set_bit(MD_RECOVERY_NEEDED, &mddev->recovery);
                md_wakeup_thread(mddev->thread);
                md_wakeup_thread(mddev->sync_thread);
                did_change = 1;
                DPRINTK("switching from read-auto to rw\n");
            }

            /* increment writes in progress so can't go back into safe mode
            * until there all done */
            atomic_inc(&mddev->writes_pending);

            /* if in safemode, switch out as we're writing */
            if (mddev->safemode == 1)
                mddev->safemode = 0;

            /* in sync will only be set back to one if we re-enter safe mode
            * plus meet other conditions */
            if (mddev->in_sync) {
                spin_lock_irqsave(&mddev->write_lock,flags);
                if (mddev->in_sync) {
                    DPRINTK("not in sync, changing sb\n");
                    mddev->in_sync = 0;
                    set_bit(MD_CHANGE_CLEAN, &mddev->flags);
                    md_wakeup_thread(mddev->thread);
                    did_change = 1;
                }
                spin_unlock_irqrestore(&mddev->write_lock,flags);
            }

            /* things(!) watch the sysfs files. notify them that something
            * has changed */
            if (did_change)
                sysfs_notify_dirent(mddev->sysfs_state);
        }

        /* The write can continue when there is no required changes to the
        * clean status of the superblock and there are no superblock writes
        * pending. If we've caused and changes to the superblock, let them
        * happen */
        if (!test_bit(passed_md_write_checks, &cmd->flags)) {
            if (!test_bit(MD_CHANGE_CLEAN, &mddev->flags) &&
                !test_bit(MD_CHANGE_PENDING, &mddev->flags) )
            {
                DPRINTK("passed md_write_start checks\n");
                set_bit(passed_md_write_checks, &cmd->flags);
                smp_wmb();
            } else {
                DPRINTK("getting ready to wait on sb updates\n");
                /* start a task that waits on the sb_wait stuff, then starts
                * the queue as soon as there's a wake up */
                PREPARE_WORK(&raid->work, hwraid1_sbwrite_waiting);
                queue_work(raid->wait_event_anticipation, &raid->work);

                DPRINTK("waiting queued, continuing\n");
                /* try again later */
                return -1;
            }
        }
	}
	
    /* this implements the code from raid1_wait_barrier without the wait_event
    * which can cause problems when running atomically. */
    if (!test_and_set_bit(started_wait_barrier, &cmd->flags) ) {
	    DPRINTK("entered wait barrier\n");
        spin_lock_irqsave(&conf->resync_lock,flags);
        if (conf->barrier) {
            conf->nr_waiting++;
        } else {
            /* no barrier, can continue */
            conf->nr_pending++;
            set_bit(passed_wait_barrier, &cmd->flags);
            DPRINTK("barrier continuing immediately\n");
            DPRINTK("pending = %d\n",conf->nr_pending);
        }
        spin_unlock_irqrestore(&conf->resync_lock,flags);
    }

    if (!test_bit(passed_wait_barrier, &cmd->flags) ) {
        spin_lock_irqsave(&conf->resync_lock,flags);
        if (conf->barrier) {
            DPRINTK("waiting at barrier\n");
            DPRINTK("waiting = %d\n",conf->nr_waiting);
            DPRINTK("barrier = %d\n",conf->barrier);
            DPRINTK("pending = %d\n",conf->nr_pending);
            /* try again later, the request queue should wake up when
            raid1_lower_barrier is called */

            md_wakeup_thread(mddev->thread);

            spin_unlock_irqrestore(&conf->resync_lock,flags);
            return -1;
        } else {
            DPRINTK("barrier continuing\n");
            conf->nr_waiting--;
            conf->nr_pending++;
            set_bit(passed_wait_barrier, &cmd->flags);
            DPRINTK("waiting = %d\n",conf->nr_waiting);
            DPRINTK("pending = %d\n",conf->nr_pending);
        }
        spin_unlock_irqrestore(&conf->resync_lock,flags);
    }

	return 0;
}

/**
 * Should be identical to raid1_allow_barrier in raid1.c except for extra
 * debug which we don't want output during "normal" raid1 operations
 */
void hwraid1_allow_barrier(conf_t *conf)
{
	unsigned long flags;
	spin_lock_irqsave(&conf->resync_lock, flags);
    DPRINTK("pending = %d\n",conf->nr_pending);
	BUG_ON(!conf->nr_pending);
	conf->nr_pending--;
	spin_unlock_irqrestore(&conf->resync_lock, flags);
	wake_up(&conf->wait_barrier);
}



static void hwraid1_end_barrier(struct raidset_s* raid, int write)
{
    mddev_t *mddev = raid->mddev;
	conf_t *conf = raid->mddev->private;

    if (write) {
        md_write_end(mddev);
    }

    hwraid1_allow_barrier(conf);
}

static void hwraid1_direct_end_barrier(struct raidset_s* raid, int write)
{
    if (0 && write) {
        md_write_end(raid->mddev);
    }
}

static void hwraid1_direct_start_barrier(struct raidset_s* raid, int write)
{
	DPRINTK("write = %d\n",write);
    if (0 && write) {
        /* assume that md_write_start only uses bi for direction */
        struct bio bi = { .bi_rw = 1};
        md_write_start(raid->mddev, &bi);
    }
}

/**
 * Allocates resources needed for the synchronisation process.
 */
static int hwraid1_init_resync(struct raidset_s* raid)
{
    int i;
    struct page* page;
    struct sync_activity_s* sync_process;
    BUG_ON(raid->sync_process);

    /* try and allocate a sync process object */
    raid->sync_process = kzalloc( sizeof(struct sync_activity_s), GFP_KERNEL);
    if (!raid->sync_process) {
        return -ENOMEM;
    }
    sync_process = raid->sync_process;
    sync_process->raid = raid;

    /* allocate pages to hold the data */
    for (i = 0; i < RESYNC_PAGES; i++) {
        page = alloc_page( GFP_KERNEL | __GFP_DMA );
        if (unlikely(!page)) {
            goto out_free_pages;
        }

        /* add page to a list */
        sync_process->sync_page[i] = page;
        
        /* clear out anything that might be in cpu caches for this page in case
        it dribbles out inbetween reads and writes */
        dma_map_single(NULL, page_address(page), PAGE_SIZE, DMA_FROM_DEVICE );
    }

    /* allocate a DMA table of the pages */
    BUG_ON(sync_process->odrb_prd_list);
    if (odrb_alloc_prd_array(&sync_process->odrb_prd_list, 1, 1)) {
        goto out_free_pages;
    }

    /* create a workqueue for the comparison */
    sync_process->workqueue = create_workqueue("hwraid-sync");
    if (!sync_process->workqueue) {
        goto out_free_odrb;
    }
    
    return 0;

out_free_odrb:
    /* free DMA table */
    if(sync_process->odrb_prd_list) {
        odrb_free_prd_array(sync_process->odrb_prd_list);
        sync_process->odrb_prd_list = NULL;
    }
    
out_free_pages:
    DPRINTK("couldn't get enough pages for a sync buffer!\n");
    while(i--) {
        if (sync_process->sync_page[i]) {
            __free_page(sync_process->sync_page[i]);
            sync_process->sync_page[i] = NULL;
        }
    }
    kfree(sync_process);
    raid->sync_process = NULL;
    return -ENOMEM;
}

/**
 * Called after synchronisation has finished, frees resourses used by the 
 * sync process.
 */
static void hwraid1_close_sync(struct raidset_s* raid)
{
    struct sync_activity_s* sync_process = raid->sync_process;
    int i = RESYNC_PAGES;

    if (sync_process->workqueue) {
        flush_workqueue(sync_process->workqueue);
        destroy_workqueue(sync_process->workqueue);
        sync_process->workqueue = NULL;
    }
    
    /* free DMA table */
    if(sync_process->odrb_prd_list) {
        odrb_free_prd_array(sync_process->odrb_prd_list);
        sync_process->odrb_prd_list = NULL;
    }
    
    /* free sync buffer pages */
    while(i--) {
        /* dma unmap is for completeness */
        dma_unmap_single(
            NULL, 
            page_to_dma(NULL, sync_process->sync_page[i]),
            PAGE_SIZE,
            DMA_FROM_DEVICE );
    
        __free_page(sync_process->sync_page[i]);
        sync_process->sync_page[i] = NULL;
    }
    kfree(sync_process);
    raid->sync_process = NULL;
}

/** 
 * This maps straight from the page list to a physical PRD table or
 * hardware scatterlist. It does not do any cache coherency operations, so
 * cannot be used for check operations
 */
static void map_sync_buffer(struct sync_activity_s* sync_process, enum dma_data_direction dir)
{
    int i = 0;
    int length_left = sync_process->nr_sectors << 9;
    int mirror = sync_process->mirror;
    int coherent = test_bit(MD_RECOVERY_CHECK, &sync_process->recovery_flags);
    struct page* page;

    /* point to the first prd table entry */
    prd_table_entry_t* prd = sync_process->odrb_prd_list->prds;
    while (length_left) {
        BUG_ON(i >= NUMBER_OF_DMA_ELEMENTS);
        /* set it to the page address */
        page = sync_process->sync_buffer[mirror][i];
        if (coherent) {
            prd->adr = dma_map_single(NULL, page_address(page), PAGE_SIZE, dir);
        } else {
            prd->adr = cpu_to_le32(page_to_dma(NULL, page));
        }
        
        /* set the size, taking page sized chunks out of the total size then 
        put the final sub-page sized chunk into the last entry */
        if (length_left > PAGE_SIZE) {
            prd->flags_len = cpu_to_le32(PAGE_SIZE);
            length_left -= PAGE_SIZE;
        } else {
            prd->flags_len = cpu_to_le32(length_left) | PRD_EOF_MASK;
            length_left = 0;
        }
        
        /* next prd table entry and next page */
        prd++;
        i++;
    }

    smp_wmb();
}

/**
 * called after the write for a sync command, it should report the success
 * of the write to the sync process
 */
static void post_sync_write(int error, void* data)
{
    struct sync_activity_s* sync_process;
    mddev_t *mddev;
	conf_t *conf;

    DPRINTK("\n");
    sync_process = (struct sync_activity_s* )data;
    mddev = sync_process->raid->mddev;
	conf = mddev->private;

    if (error) {
		int sync_blocks = 0;
		sector_t s = sync_process->start_sector;
		long sectors_to_go = sync_process->nr_sectors;
		/* make sure these bits doesn't get cleared. */
		do {
			bitmap_end_sync(mddev->bitmap, s, &sync_blocks, 1);
			s += sync_blocks;
			sectors_to_go -= sync_blocks;
		} while (sectors_to_go > 0);
		md_error(mddev, conf->mirrors[sync_process->mirror].rdev);
	}

    raid1_lower_barrier(conf);
	/* release sata core lock */
    ox820hwraid_release_sata_core(SATA_REBUILD);
    ox820hwraid_restart_queue();
    md_done_sync(mddev, sync_process->nr_sectors, !error);
}

/**
 * Forms a write command either for a simple sync or a check and sync
 */
static struct command_s* form_sync_write(struct sync_activity_s* sync_process)
{
    struct command_s* cmd;
    int port_no;
    unsigned long sources = sync_process->source_ok & ~sync_process->dest;
    struct raidset_s* raid = sync_process->raid;

    /* find a port with good data */
    smp_rmb();
    port_no = find_first_bit(&sources, NUMBER_OF_DISKS);
    DPRINTK("from port no %d\n",port_no);
    sync_process->mirror = port_no;
    smp_wmb();

    /* create a scatterlist to a buffer to hold the resynced data */
    map_sync_buffer(raid->sync_process, DMA_TO_DEVICE);

    /* find the port to write */
    smp_rmb();
    port_no = find_first_bit(&sync_process->dest, NUMBER_OF_DISKS);
    BUG_ON((port_no != 0) && (port_no != 1));
    DPRINTK("to port no %d\n",port_no);
    clear_bit(port_no, &sync_process->dest);
    sync_process->mirror = port_no;
    smp_wmb();

    /* form the write command */
    cmd = get_sync_cmd();
    BUG_ON(!cmd);
    cmd->write = 1;
    cmd->ata_cmd = ATA_CMD_WRITE_EXT;
    cmd->sector  = raid->start + sync_process->start_sector;
    cmd->nsects  = sync_process->nr_sectors;
    cmd->port = raid->partition[port_no]->port;
    cmd->execute_command = hwraid1_execute_command_single;
    cmd->analyse = hwraid1_sync_analysis;
    cmd->data    = sync_process ;
    cmd->done    = post_sync_write;
    cmd->odrb_prd_list = sync_process->odrb_prd_list;
    DPRINTK("sglst %x, sgdma %p\n", cmd->odrb_prd_list->phys, cmd->odrb_prd_list);
    DPRINTK("cmd %x, loc %llx, num %llx\n",
        cmd->ata_cmd, cmd->sector, cmd->nsects );
    return cmd;
}

static void sync_work(struct work_struct *work)
{
    struct command_s* cmd;
    struct sync_activity_s* sync_process;
    mddev_t *mddev;
	conf_t *conf;
    
    DPRINTK("\n");
    sync_process = (struct sync_activity_s* )
        container_of(work, struct sync_activity_s, work);
    mddev = sync_process->raid->mddev;
	conf = mddev->private;

    /* if there are any other disks to read, read them */
    if (sync_process->source) {
        cmd = form_sync_read(sync_process);
        cmd->execute_command(cmd, get_host());
    } else {
        int write_needed = 1;
        
        /* Cannot read from anywhere, array is toast */
        if (!sync_process->source_ok) {
            printk(KERN_ALERT "hwraid1: unable to read block %llu for synchronisation\n",
                   (unsigned long long)sync_process->start_sector);
            md_done_sync(mddev, sync_process->nr_sectors, 0);
    
            /* release sata core lock */
            ox820hwraid_release_sata_core(SATA_REBUILD);
            ox820hwraid_restart_queue();
            raid1_lower_barrier(conf);
            return;
        }
        
        /* no value in reading and writing the same disk */
        if (sync_process->source_ok == sync_process->dest) {
            DPRINTK("source and dest are now the same disk, no need to write\n");
            write_needed = 0;
        }
        
        /* if this was a recovery check, then compare buffers */
        if (test_bit(MD_RECOVERY_REQUESTED, &sync_process->recovery_flags) ||
            test_bit(MD_RECOVERY_CHECK, &sync_process->recovery_flags))
        {
            int nr_pages = sync_process->nr_sectors >> (PAGE_SHIFT - 9);
            
            DPRINTK("checking data\n");
            /* assumes only two disks in the array, but they must both work */
            if (sync_process->source_ok == 3) {
                int i = 0;
                while (nr_pages) {
                    struct page* p = sync_process->sync_buffer[0][i];
                    struct page* s = sync_process->sync_buffer[1][i];
                    if (memcmp(page_address(p), page_address(s), PAGE_SIZE))
                        break;
                    nr_pages--;
                    i++;
                }
            }
            
            /* if any missmatches are found then add them to the count */
            if (nr_pages) {
                DPRINTK("found %d pages don't match\n", nr_pages);
                mddev->resync_mismatches += nr_pages << (PAGE_SHIFT - 9);
            } else {
                /* data matches, no write needed */
                write_needed = 0;
            }
            
            if (test_bit(MD_RECOVERY_CHECK, &sync_process->recovery_flags)) {
                write_needed = 0;
            }
        }
        
        if (write_needed) {
            /* form a command for the write */
            cmd = form_sync_write(sync_process);
            cmd->execute_command(cmd, get_host());
        } else {
            DPRINTK("this bit done\n");
            /* this part of the sync is done */
            raid1_lower_barrier(conf);
            ox820hwraid_release_sata_core(SATA_REBUILD);
            ox820hwraid_restart_queue();
            md_done_sync(mddev, sync_process->nr_sectors, 1);
        }
    }
}

/**
 * called after the read for a sync command,
 */
static void post_sync_read(int error, void* data)
{
    struct sync_activity_s* sync_process;
    mddev_t *mddev;
	conf_t *conf;

    DPRINTK("\n");
    sync_process = (struct sync_activity_s* )data;
    mddev = sync_process->raid->mddev;
	conf = mddev->private;
    if (error) {
        /* report error */
        md_error(mddev, conf->mirrors[sync_process->mirror].rdev);
    } else {
        set_bit(sync_process->mirror, &sync_process->source_ok);
    }

    /* queue a work item to do some thinking */
    INIT_WORK(&sync_process->work, sync_work);
    queue_work(sync_process->workqueue, &sync_process->work);
}

/**
 * Forms a read command either for a simple sync or a check and sync
 */
static struct command_s* form_sync_read(struct sync_activity_s* sync_process)
{
    struct command_s* cmd;
    int port_no;
    struct raidset_s* raid = sync_process->raid;

    /* find the port */
    smp_rmb();
    port_no = find_first_bit(&sync_process->source, NUMBER_OF_DISKS);
    BUG_ON((port_no != 0) && (port_no != 1));
    DPRINTK("port no %d\n",port_no);
    clear_bit(port_no, &sync_process->source);
    sync_process->mirror = port_no;
    smp_wmb();

    /* create a scatterlist to a buffer to hold the resynced data */
    map_sync_buffer(raid->sync_process, DMA_FROM_DEVICE);

    /*  */
    cmd = get_sync_cmd();
    BUG_ON(!cmd);
    cmd->write = 0;
    cmd->ata_cmd = ATA_CMD_READ_EXT;
    cmd->sector  = raid->start + sync_process->start_sector;
    cmd->nsects  = sync_process->nr_sectors;
    cmd->port = raid->partition[port_no]->port;
    sync_process->raid->last_port = cmd->port;
    cmd->execute_command = hwraid1_execute_command_single;
    cmd->analyse = hwraid1_sync_analysis;
    cmd->data    = sync_process ;
    cmd->done    = post_sync_read;
    cmd->odrb_prd_list = sync_process->odrb_prd_list;
    DPRINTK("sglst %x, sgdma %p\n", cmd->odrb_prd_list->phys, cmd->odrb_prd_list);
    DPRINTK("cmd %x, loc %llx, num %llx\n",
        cmd->ata_cmd, cmd->sector, cmd->nsects );
    return cmd;
}

/**
 * Atomic RAID-1 rebuild
 *
 * Based on the code in /drivers/md/raid1.c it performs a sync of a "block"
 * whilst holding the SATA core lock. This negates the need for barriers
 * that can cause deadlock problems when used in conjunction with the three
 * other routes into the SATA core.
 */
sector_t
hwraid1_sync_request(mddev_t *mddev, sector_t start_sector, int *skipped, int go_faster)
{
    int i;
    struct command_s* cmd;
    unsigned long source;
    unsigned long dest;
    sector_t max_sector;
    sector_t position;
    sector_t nr_sectors;
    int length_pages;
    int sync_blocks;
    int wait_count;
    int ret;
    void* contrived_uid;
    int max_nr_sectors;
    int still_degraded = 0;

    struct raidset_s* raid = mddev->hw_raid;
	conf_t *conf = mddev->private;

    /* check for a pool of buffers, create one if there isn't and quit if we
     * can't */
    if (!raid->sync_process)
    {
        DPRINTK("sync start - bitmap %p\n", mddev->bitmap);
        if (hwraid1_init_resync(raid))
            return 0;
    }

    max_sector = mddev->dev_sectors;
    if (start_sector >= max_sector) {
        /* If we aborted, we need to abort the
         * sync on the 'current' bitmap chunk (there will
         * only be one in raid1 resync.
         * We can find the current addess in mddev->curr_resync
         */
        if (mddev->curr_resync < max_sector) /* aborted */
            bitmap_end_sync(mddev->bitmap, mddev->curr_resync,
                        &sync_blocks, 1);
        else /* completed sync */
            conf->fullsync = 0;

        bitmap_close_sync(mddev->bitmap);
        hwraid1_close_sync(raid);
        return 0;
    }

    /* don't need to do this bit? */
    if (mddev->bitmap == NULL &&
        mddev->recovery_cp == MaxSector &&
        !test_bit(MD_RECOVERY_REQUESTED, &mddev->recovery) &&
        conf->fullsync == 0)
    {
        *skipped = 1;
        return max_sector - start_sector;
    }

    /* before building a request, check if we can skip these blocks..
     * This call the bitmap_start_sync doesn't actually record anything
     */
    if (!bitmap_start_sync(mddev->bitmap, start_sector, &sync_blocks, 1) &&
        !conf->fullsync &&
        !test_bit(MD_RECOVERY_REQUESTED, &mddev->recovery))
    {
        /* We can skip this block, and probably several more */
        *skipped = 1;
        return sync_blocks;
    }

    /*
     * If there is non-resync activity waiting for a turn,
     * and resync is going fast enough,
     * then let it though before starting on this new sync request.
     */

    /* TODO: MODIFY THIS TO WORK WITH FAST CODE */
    if (!go_faster && conf->nr_waiting ) {
        DPRINTK("slowing \n");
        msleep_interruptible(1000);
    }

    bitmap_cond_end_sync(mddev->bitmap, start_sector);

    hwraid1_raise_barrier(conf);

    DPRINTK("resync task waiting for sata core\n");
    
    /* Get sata core lock, should wait indefinitely if not available */
    contrived_uid = (void* )(((u32)start_sector & 0x00ffffff) | 0x10000000);
    wait_count = 0;
    ret = 0;
    do {
        /* other tasks are always a priority, wait for a bit */
        if ( sata_core_has_waiters() ) {
            if (++wait_count > 10) {
                md_wakeup_thread(mddev->thread);
            }
            msleep_interruptible(50);
        } else {
            ret = acquire_sata_core_direct(ox820hwraid_irq_handler, 0, HZ, contrived_uid, SATA_REBUILD);
        }
    } while (!ret);
    DPRINTK("resync task got sata core\n");

    conf->next_resync = start_sector;

	/* identify the read and write disks */
    rcu_read_lock();
    source = 0;
    dest = 0;
    for (i = 0; i < NUMBER_OF_DISKS; ++i) {
        mdk_rdev_t* rdev;
        int mirror_no;

        /* exclude non-existant partitions */
        if (!raid->partition[i]) {
            DPRINTK("partition %d not present\n",i);
            continue;
        }

        /* get the rdev for this partition */
        mirror_no = raid->partition[i]->mirror;
        rdev = rcu_dereference(conf->mirrors[mirror_no].rdev);

        if (rdev == NULL || test_bit(Faulty, &rdev->flags)) {
            /* if faulty, ignore */
            DPRINTK("partition %d faulty\n",i);
            still_degraded = 1;
            continue;
        } else if (!test_bit(In_sync, &rdev->flags)) {
            /* if not in sync, write to it */
            DPRINTK("partition %d write disk\n",i);
            dest |= 1 << i;
        } else {
            /* If a sync rather than a recovery, extra read targets are
             * write targets, (this implementation only allows one read target)
             */
            if (test_bit(MD_RECOVERY_SYNC, &mddev->recovery) && source) {
                DPRINTK("partition %d write disk due to multiple read disks\n",i);
                dest |= 1 << i;
            }
            /* this must be our source */
            DPRINTK("partition %d read disk\n",i);
            source |= 1 << i;
        }
    }
    rcu_read_unlock();

    /* if there is no source or dest, something's broken - finish */
    if (!source || !dest ) {
        sector_t rv = max_sector - start_sector;
        DPRINTK("no source or destination, exiting\n");
        *skipped = 1;
        /* release sata core lock */
        ox820hwraid_release_sata_core(SATA_REBUILD);
        ox820hwraid_restart_queue();
        
        raid1_lower_barrier(conf);
        return rv;
    }

    /* obey limits on where we can sync to */
    if (max_sector > mddev->resync_max) {
        DPRINTK("sectors limited by resync_max\n");
        max_sector = mddev->resync_max;
    }

    /* Amount we can sync is limited when we check the data to reduce the
     * time spent holding the SATA lock */
    if (test_bit(MD_RECOVERY_CHECK, &mddev->recovery) ||
        test_bit(MD_RECOVERY_REQUESTED, &mddev->recovery))
    {
        max_nr_sectors = HWRAID1_CHECK_SECTORS;
        raid->sync_process->sync_buffer[0] = &raid->sync_process->sync_page[0];
        raid->sync_process->sync_buffer[1] = &raid->sync_process->sync_page[CHECK_PAGES];
    } else {
        max_nr_sectors = HWRAID1_RESYNC_SECTORS;
        raid->sync_process->sync_buffer[0] = &raid->sync_process->sync_page[0];
        raid->sync_process->sync_buffer[1] = &raid->sync_process->sync_page[0];
    }
    
    /* repeatedly look for bits of the raidset to sync up until limits are
     * met */
    position = start_sector;
    sync_blocks = 0;
    nr_sectors = 0;
    do {
        length_pages = PAGE_SIZE;

        /* limit to inside the disk */
        if (position + (length_pages >> 9) > max_sector) {
            DPRINTK("limit to inside of the disk\n");
            length_pages = (max_sector - position) << 9;
        }

        /* if nothing left stop looking for more */
        if (!length_pages) {
            DPRINTK("done\n");
            break;
        }

        /* do we need to do this bit of the disks, if not stop looking for
         * more */
        if (sync_blocks == 0) {
            if (!bitmap_start_sync(mddev->bitmap, position, &sync_blocks, still_degraded) &&
                !conf->fullsync &&
                !test_bit(MD_RECOVERY_REQUESTED, &mddev->recovery) )
            {
                DPRINTK("break in bitmap\n");
                break;
            }
    
            /* limit to the amount reccomended by bitmap fns */
            BUG_ON(sync_blocks < (PAGE_SIZE >> 9));
            if (length_pages > (sync_blocks << 9)) {
                DPRINTK("limit to the amount reccomended by bitmap fns\n");
                length_pages = sync_blocks << 9;
            }
        }

        /* update positions based on length */
        nr_sectors += length_pages >> 9;
        position +=  length_pages >> 9;
        sync_blocks -= (length_pages >> 9);

    } while (nr_sectors < max_nr_sectors);
    DPRINTK("resync of %llu sectors at %llu\n", nr_sectors, start_sector);
    raid->sync_process->start_sector = start_sector;
    raid->sync_process->nr_sectors = nr_sectors;
    raid->sync_process->source = source;
    raid->sync_process->source_ok = 0;
    raid->sync_process->dest = dest;
    raid->sync_process->recovery_flags = mddev->recovery;
    smp_wmb();

    /* form a raid rebuild activity */
    cmd = form_sync_read(raid->sync_process);
    cmd->execute_command(cmd, get_host());

    DPRINTK("exiting\n");
	return nr_sectors;
}

/**************************************************************************/
/* REQUEST QUEUE STUFF                                                    */
/**************************************************************************/

/*
 * If the request queue is stopped, restart it
 */
void ox820hwraid_restart_queue(void)
{
    if (raiddev &&
        raiddev->raidset &&
        raiddev->raidset->queue)
    {
        struct request_queue* q = raiddev->raidset->queue;
        spin_lock_irq(q->queue_lock);
        if (blk_queue_stopped(q)) {
            blk_start_queue(q);
        }
        spin_unlock_irq(q->queue_lock);
    }
}


/**
 * Prepares requests, performing command translaition and DMA mapping
 * irrespective of the host being busy.
 */
static int prep_request(struct request_queue* queue, struct request* req)
{
    struct raidset_s* raidset;
    mddev_t* mddev;
    struct command_s* cmd;

    DPRINTK("\n");
    if (unlikely(!blk_fs_request(req))) {
        printk(KERN_NOTICE"raid driver sent a non-fs request\n");
        return BLKPREP_KILL;
    }

    if (!req->special) {
        mddev = req->rq_disk->private_data;
        raidset = mddev->hw_raid;
        cmd = raidset->biotocmd(raidset, queue, req);
        if (!cmd) {
            DPRINTK("All commands busy\n");
            blk_stop_queue(queue);
            return BLKPREP_DEFER;
        }
        req->special = cmd;
    }

    return BLKPREP_OK;
}

/**
 * The job of this function is to take requests from the queue and process them
 * It is called and must return in atomic context.
 *
 */
static void request_processor(struct request_queue* queue)
{
    struct request* req;

    DPRINTK("\n");

    /* get a request from the elevator */
    while ((req = blk_fetch_request(queue)) != NULL) {
        struct raidset_s* raid;
        mddev_t* mddev;
        struct command_s* cmd;
        struct host_s* host;

        spin_unlock_irq(queue->queue_lock);

        host = get_host();
        if (unlikely(!host)) {
            DPRINTK("couldn't get the host\n");
            /* error */
            spin_lock_irq(queue->queue_lock);
            elv_requeue_request(queue, req);
            return;
        }

        /* use either a pre-prepared command or prepare it here */
        if (req->special) {
            DPRINTK("using prepared command\n");
            cmd = req->special;
        } else {
            struct raidset_s* raidset;
            mddev_t* mddev;
            /* check that the request is a filesystem request */
            if (!blk_fs_request(req)) {
                printk(KERN_NOTICE"raid driver sent a non-fs request\n");
                spin_lock_irq(queue->queue_lock);
                blk_end_request_all(req, 0);
                continue;
            }

            /* turn the request into a command and begin the issuing process */
            mddev = req->rq_disk->private_data;
            raidset = mddev->hw_raid;
            cmd = raidset->biotocmd(raidset, queue, req);
            if (!cmd) {
                DPRINTK("error creating a command\n");
                /* error */
                spin_lock_irq(queue->queue_lock);
                elv_requeue_request(queue, req);
                return;
            }

            /* save what we've done in the command in case we need to requeue
            it */
            req->special = cmd;
        }

        /* perform any raid-1 waiting here */
        mddev = req->rq_disk->private_data;
        raid = mddev->hw_raid;
        if (raid->wait_barrier &&
            raid->wait_barrier(cmd, raid, rq_data_dir(req)))
        {
            spin_lock_irq(queue->queue_lock);
            elv_requeue_request(queue, req);
            blk_stop_queue(queue);
            return;
        }

        if (!acquire_sata_core_hwraid(ox820hwraid_irq_handler, 0, REQUEST_QUEUE_UID)) {
            /* didn't get it */
            DPRINTK("hw busy\n");
            spin_lock_irq(queue->queue_lock);
            elv_requeue_request(queue, req);
            blk_stop_queue(queue);
            return;
        }

        /* send the command to the hardware */
        cmd->execute_command(cmd, host);

        spin_lock_irq(queue->queue_lock);
    }
    DPRINTK("all requests done, exiting\n");
}

/**
 * completes a bio command
 */
static void request_done(int error, void* data)
{
    struct raidset_s* raid;
    mddev_t* mddev;
    struct request* req = (struct request*)data;
    DPRINTK("\n");

    ox820hwraid_release_sata_core(SATA_HWRAID);
    mddev = req->rq_disk->private_data;
    raid = mddev->hw_raid;
    if (raid->allow_barrier) {
        raid->allow_barrier(raid, rq_data_dir(req));
    }
    blk_end_request_all(req, error);
#ifndef REQUEST_DONE_BH
    ox820hwraid_restart_queue();
    DPRINTK("done\n");
}
#else
    tasklet_schedule(&ox820hwraid_req_completion_bh);
    DPRINTK("done\n");
}

static void request_done_bh(unsigned long data)
{
	ox820hwraid_restart_queue();
    DPRINTK("done\n");
}
#endif
/**************************************************************************/
/* DIRECT COMMAND STUFF                                                   */
/**************************************************************************/

/**
 * Used by direct access code to acquire the sata core. This is conditional
 * on the RAID code not needing to add or remove disks
 * @return 0 = got it, -ve = error code
 */
static int direct_aquire(
	direct_access_context_t *context,
	int                      timeout_jiffies,
	void                    *uid,
	int                      is_reader)
{
    int ret =  -EBUSY;
    unsigned long end = jiffies + timeout_jiffies;
	DPRINTK("context %p jiffies %d\n", context, timeout_jiffies);
    do {
        /* the atomic_inc/atomic_dec is done like this to avoid race conditions
        with the command barrier */
        smp_mb__before_atomic_inc();
        atomic_inc(&cmds_out);
        smp_mb__after_atomic_inc();
        if (likely(!command_barrier) &&
            acquire_sata_core_direct(ox820hwraid_irq_handler, 0,
            	timeout_jiffies, uid, is_reader ? SATA_READER : SATA_WRITER))
        {
            DPRINTK("got the sata core lock\n");
            ret = 0;
            break;
        } else {
            DPRINTK("didn't get the sata core lock\n");
            smp_mb__before_atomic_dec();
            atomic_dec(&cmds_out);
            smp_mb__after_atomic_dec();
            msleep_interruptible(100);
        }
    } while (time_before(jiffies, end));
    return ret;
}

/**
 * Used by the direct access code to release the sata core lock. It also
 * co-operates with the RAID code to release locks around adding/removing
 * disks.
 */
static void direct_release(int is_reader)
{
	DPRINTK("\n");
    /* release the hardware */
    smp_rmb();
    if (unlikely(ox820hwraid_hotplug_events)) {
        int i;
        for (i = 0; i < NUMBER_OF_DISKS; ++i) {
            if (test_and_clear_bit(i, &ox820hwraid_hotplug_events)) {
                ox820sata_checkforhotplug(i);
            }
        }
    }
    release_sata_core(is_reader ? SATA_READER : SATA_WRITER);
    smp_mb__before_atomic_dec();
    atomic_dec(&cmds_out);
    smp_mb__after_atomic_dec();
	DPRINTK("done\n");
}

/**
 * Used by the direct access code to determine if another SATA core user
 * needs access to the sata core or if there has been some kind of event
 * like a hotplug that requires the direct access code to hand over its
 * lock
 */
static int direct_has_waiters(void)
{
    int ret;
	DPRINTK("\n");
    ret = sata_core_has_waiters();
    ret += command_barrier;
    if (unlikely(ox820hwraid_hotplug_events)) {
        ret++;
    }
    return ret;
}

static int direct_execute(void)
{
    struct command_s* cmd = (struct command_s*)&cmd_for_direct_access;

	DPRINTK("\n");
    /* send the command to the hardware */
    return cmd->execute_command(cmd, cmd->raid->host);
}

static void direct_free(struct direct_access_context* context)
{
	struct raid_context_s* raid_context =
	    container_of( context, struct raid_context_s, base_context);

	DPRINTK("raid_context %p\n", raid_context);
    /* perform end of command barriers */
    if (raid_context->raid->direct_end_barrier) {
        raid_context->raid->direct_end_barrier(raid_context->raid, !context->read_only);
    }

	kfree(raid_context);
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
    struct raidset_s* raid;
	struct raid_context_s* raid_context;
	int retval = 0;

	DPRINTK("\n");
	/* only give a context if the file is on the raid partition */
	if ((MAJOR(inode->i_sb->s_dev) != 9) ||
	    (MINOR(inode->i_sb->s_dev) != 4))
	{
	    printk(KERN_ERR"This version of the direct access code only works on md4\n");
		retval = -ENODEV;
		goto out;
	}

	/* Need to track direct access context for this inode */
	raid_context = kzalloc(sizeof(struct raid_context_s), GFP_KERNEL);
	if (!raid_context) {
	    printk(KERN_ERR"Direct access code unable to allocate memory\n");
		retval = -ENOMEM;
		goto out;
	}

	raid = raiddev->raidset;
    /* Setup function pointers etc. for hwraid access */
    raid_context->base_context.read_only       = read_only;
    raid_context->base_context.acquire         = direct_aquire;
    raid_context->base_context.prepare_command = raid->direct_cmd;
    raid_context->base_context.execute_command = direct_execute;
    raid_context->base_context.release         = direct_release;
    raid_context->base_context.free            = direct_free;
    raid_context->base_context.has_waiters     = direct_has_waiters;

    raid_context->raid = raid;
    smp_wmb();

	/* set the context pointer to point to the base class in our superclass */
	*context = &( raid_context->base_context );

    if (raid->direct_start_barrier) {
        raid->direct_start_barrier(raid, !read_only);
    }

out:
	DPRINTK("raid_context %p (containing context %p) %s for %s\n",
	    raid_context,
	    *context,
	    retval ? "failure" : "success",
	    read_only ? "reader" : "writer");
	return retval;
}

void free_direct_sata_context(direct_access_context_t *context)
{
    context->free(context);
}

/******************************************************************************/
/**
 * A method in the raid-set object
 *
 * when we have all the geometry we need, this creates the disk structures with
 * the correct size geometry
 */
static int ox820hwraid_disk_up(struct raidset_s* raid)
{
    CPRINTK("\n");

    /* initialise the request queue so that it can be used with our strategy*/
    spin_lock_init(&rq_lock);
    raid->queue = raid->mddev->queue ;

	if (blk_reinit_queue(raid->queue, request_processor, &rq_lock) < 0 ) {
	    printk(KERN_ERR"elevator initialisation failed\n");
        return -EIO;
	}
	smp_wmb();
    raid->hw_raid_active = 1;

	blk_queue_prep_rq(raid->queue, prep_request);

    /* how may sg elelments we can handle */
    blk_queue_max_hw_segments(raid->queue, NUMBER_OF_DMA_ELEMENTS);
    blk_queue_max_phys_segments(raid->queue, NUMBER_OF_DMA_ELEMENTS);

    /* set the size of the biggest command we can handle */
    blk_queue_max_sectors(raid->queue, MAX_CMD_SIZE);

    return  0;
}

static void ox820hwraid_disk_down(struct raidset_s* raid)
{
    CPRINTK("\n");
    /* queue shouly remain plugged after this */
    blk_sync_queue(raid->mddev->queue);
}


/**
 *
 */
static int __init ox820hwraid_init(void)
{
    ox820_raiddev_t* temp;

    temp = kmalloc(sizeof(ox820_raiddev_t), GFP_KERNEL );
    if (!temp) {
        return -ENOMEM;
    }
    memset(temp, 0, sizeof(ox820_raiddev_t));
    temp->host = get_host();

    raiddev = temp;

    /* initialise the commands */
    init_command_request_queue(&cmd_for_request_q);
    init_command_direct(&cmd_for_direct_access);
    init_command_basic(&cmd_for_sync);

    /* create a workqueue for error handling */
    error_handler_wq = create_singlethread_workqueue("hwraid_eh");

#ifdef ERROR_INJECTION
    {
        struct proc_dir_entry *res=create_proc_entry("ox820direct_errorinject",0,NULL);
        if (res) {
            res->read_proc =ox820direct_error_inject_show;
            res->write_proc=ox820direct_error_inject_store;
            res->data=NULL;
        }
    }
#endif

    return 0;
}

module_init(ox820hwraid_init);

static void __exit ox820hwraid_exit( void )
{
    /* remove procfs entries */
    remove_proc_entry("ox820direct_errorinject", NULL);
    
    /* stop workqueue */
    flush_workqueue(error_handler_wq);
    destroy_workqueue(error_handler_wq);
    error_handler_wq = NULL;
    
    /* free resources associated with commands */
    exit_command_basic(&cmd_for_sync);
    exit_command_direct(&cmd_for_direct_access);
    exit_command_request_queue(&cmd_for_request_q);
    
    if (raiddev) {
        kfree(raiddev);
        raiddev = NULL;
    }
}

module_exit(ox820hwraid_exit);

static inline int injectErrors(struct port_s* port, int write)
#ifdef ERROR_INJECTION
{
    int ret = 0;
    struct host_s* host = get_host();

    if (host->error_period) {
        u32 val;

		/* if it is time to inject another error, then do so */
		if (time_after(jiffies, host->next_error))
		{
            host->next_error = jiffies + host->error_period;
			printk(".");
            val = (write) ? 0x4 : 0x8; 
            val |= 1;
		} else {
			val = 1;
		}
        ox820sata_link_write(port->base, 0x14, val );
    }
    return ret;
}
#else
{
    return 0;
}
#endif

#ifdef ERROR_INJECTION

/**
 * @param kobj Not Used
 * @param attr Used to determine which file is being accessed
 * @param buffer Space to put the file contents
 * @return The number of bytes transferred or an error
 */
static int ox820direct_error_inject_show(
    char *page, char **start, off_t off, int count, int *eof, void *data)
{
    struct host_s* host = get_host();

    if (page)
    {
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

static int ox820direct_error_inject_store(struct file *file,
                                        const char __user *buffer,
                                        unsigned long count,
                                        void *data)
{
    struct host_s* host = get_host();
    if (count) {
        sscanf(buffer, "%d", &(host->error_period));
        host->next_error = jiffies + host->error_period;
        return count;
    }

    /* if we get here, there's been an error */
    return -EIO;
}
#endif /* ERROR_INJECTION */
