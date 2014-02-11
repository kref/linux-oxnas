/*
 * arch/arm/mach-ox820/include/mach/direct_storage.h
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#ifndef _DIRECT_STORAGE_H
#define _DIRECT_STORAGE_H

struct raidset_s;

typedef void (done_fn_t)(int error, void* data);

typedef struct direct_access_context {
    int      read_only;

	/* Acquire exclusive access to the SATA core */
	int (*acquire)(struct direct_access_context *context, int timeout_jiffies, void *uid, int is_reader);

	/* Release exclusive access to the SATA core */
	void (*release)(int is_reader);

	/* returns non zero if you should release your sata core lock */
	int (*has_waiters)(void);    

    /**
     * This will prepare a command for the SATA core in the current setup.
     * after calling this, you shouldn't change the data.
     *
     * @param prd_pa The _physical_ address of the PRD_table. 
     * @param data This pointer will be be passed to the done function when
     *             the command completes.
     * @param done This function will be called when your command completes.
     *             Success of your command is usually indicated by an error
     *             value of zero, though a non-zero value can indicate that 
     *             the command completed successfully, but you should finish
     *             and give up control of the SATA core. See the enum
     *             done_errors for a list of possible values.
     *             NB: done() may be called in atomic context.
     */
	int (*prepare_command)(
	    struct direct_access_context *context,
	    int                           write,
		sector_t                      start,
		sector_t                      nsect,
		dma_addr_t                    dma_list_pa,
		void (*done)(int error, void* data),
		void                         *data);

	/** 
	 * Call this to make the disk do your command. make sure there is only
	 * one command operational at a time.
	 * @return error code.
	 */
	int (*execute_command)(void);

	void (*free)(struct direct_access_context* context);
} direct_access_context_t;

struct inode;

/**
 * @param read_only = if not set, will assume that writes will be done.
 */
extern int alloc_direct_sata_context(
	struct inode             *inode,
	int                       read_only,
	direct_access_context_t **context);

extern void free_direct_sata_context(direct_access_context_t *context);

#endif        /*  #ifndef _DIRECT_STORAGE_H */
