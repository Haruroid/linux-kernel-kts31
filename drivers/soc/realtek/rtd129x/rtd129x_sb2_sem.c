/*
 * rtd129x_sb2_sem.c - Realtek SB2 HW semaphore API
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 * Copyright (C) 2017 Cheng-Yu Lee <cylee12@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <soc/realtek/rtd129x_sb2_sem.h>

#define SB2_SEM_TRYLOCK_MAX 1024
#define SB2_SEM_RETRY_MAX 10
#define SB2_SEM_NUM 9

static const char * sb2_sem_id_str[SB2_SEM_ID_MAX] = {
	[SB2_SEM_ID_UNKNOWN] = "unknown",
	[SB2_SEM_ID_SCPU_KERNEL] = "kernel",
	[SB2_SEM_ID_SCPU_CLOCK] = "clock",
	[SB2_SEM_ID_SCPU_RESET] = "reset",
};

const char * sb2_sem_id_to_name(unsigned int id)
{
	if (id >= SB2_SEM_NUM)
		return NULL;
	else return sb2_sem_id_str[id];
}

static __init int init_sb2_sem(void);

struct sb2_sem {
	void __iomem *reg;
	unsigned int *cal;
};

static __initdata int __should_init = 1;
static __initdata DEFINE_SPINLOCK(__lock);
static __initdata unsigned int __reg[SB2_SEM_NUM] = {
	0x9801A000,
	0x9801A620,
	0x9801A624,
	0x9801A628,
	0x9801A62C,
	0x9801A630,
	0x9801A634,
	0x9801A638,
	0x9801A63C
};
static struct sb2_sem __sem[SB2_SEM_NUM];
static unsigned int __dummy[SB2_SEM_NUM];

int sb2_sem_try_lock(struct sb2_sem *sem, unsigned int caller)
{
	unsigned int val;
	val = readl(sem->reg);

	if (val) 
		*sem->cal = caller;

	return val;

}

void sb2_sem_lock(struct sb2_sem *sem, unsigned int caller)
{
	int i;
	unsigned val;
	int cnt  = 0;

retry:
	for (i = 0; i < SB2_SEM_RETRY_MAX; i++) {
		val = sb2_sem_try_lock(sem, caller);
		if (val)
			return;
	}

	pr_err("%s: trylock fail (locked by %s)\n",
		sb2_sem_id_str[caller], sb2_sem_id_str[*sem->cal]);
	cnt ++;
	if (cnt < SB2_SEM_RETRY_MAX)
		goto retry;
	
	pr_err("%s: ignore lock (locked by %s)\n",
		sb2_sem_id_str[caller], sb2_sem_id_str[*sem->cal]);

}

void sb2_sem_unlock(struct sb2_sem *sem)
{
	*sem->cal = 0;
	writel(0, sem->reg);
}

struct sb2_sem * sb2_sem_get(unsigned int index)
{
	if (__should_init)
		init_sb2_sem();

	if (index >= SB2_SEM_NUM)
		return ERR_PTR(-EINVAL);
	return &__sem[index];
}

int sb2_sem_set_dbg_mem(unsigned int index, void * mem)
{
	if (index >= SB2_SEM_NUM)
		return -EINVAL;

	__sem[index].cal = mem;
	return 0;
}

unsigned int *sb2_sem_get_dbg_mem(unsigned int index)
{
	if (index >= SB2_SEM_NUM)
		return ERR_PTR(-EINVAL);

	return __sem[index].cal;
}


static __init int init_sb2_sem(void)
{
	int i;
	unsigned long flags;

	spin_lock_irqsave(&__lock, flags);
	if (!__should_init)
		goto unlock;

	for (i = 0; i < SB2_SEM_NUM; i++) {
		__sem[i].reg = ioremap(__reg[i], 0x4);
		__sem[i].cal = &__dummy[i];
	}
	__should_init = 0;

unlock:
	spin_unlock_irqrestore(&__lock, flags);

	return 0;
}
early_initcall(init_sb2_sem);
