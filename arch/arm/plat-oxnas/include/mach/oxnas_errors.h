/*
 * arch/arm/plat-oxnas/include/mach/oxnas_errors.h
 *
 * Copyright (C) 2010 Oxford Semiconductor Ltd
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
 */
 /* this file defines the error values returned by the oxnas bypass write and backup software */

#ifndef __OXNAS_ERRORS_H__
#define __OXNAS_ERRORS_H__

enum {
	OXERR_VALIDATION = -19999,
	OXERR_BKP_NOTPRESENT,
	OXERR_DISKIO,
	OXERR_TIMEOUT,
	OXERR_NETWORK,
	OXNAS_FALLBACK,
};

#endif /* __OXNAS_ERRORS_H__ */
