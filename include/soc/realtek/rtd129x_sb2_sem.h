/*
 * rtd129x_sb2_sem.h - Realtek SB2 HW semaphore API
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 * Copyright (C) 2017 Cheng-Yu Lee <cylee12@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __SOC_REALTEK_RTD129X_SB2_SEM_H
#define __SOC_REALTEK_RTD129X_SB2_SEM_H

struct sb2_sem;
struct sb2_sem * sb2_sem_get(unsigned int index);
int sb2_sem_try_lock(struct sb2_sem *sem, unsigned int caller);
void sb2_sem_lock(struct sb2_sem *sem, unsigned int caller);
void sb2_sem_unlock(struct sb2_sem *sem);
const char * sb2_sem_id_to_name(unsigned int id);

int sb2_sem_set_dbg_mem(unsigned int index, void * mem);
unsigned int *sb2_sem_get_dbg_mem(unsigned int index);


enum {
	SB2_SEM_ID_UNKNOWN,
	SB2_SEM_ID_SCPU_KERNEL,
	SB2_SEM_ID_SCPU_CLOCK,
	SB2_SEM_ID_SCPU_RESET,
	SB2_SEM_ID_MAX
};

#endif /* __SOC_REALTEK_RTD129X_SB2_SEM_H */
