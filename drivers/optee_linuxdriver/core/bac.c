static int tee_shm_va_get(struct tee_context *ctx, struct tee_shm *shm,
		void *buffer, unsigned int flags, size_t size, int offset)
{
	int ret = 0;
	struct mm_struct *mm = current->mm;
	unsigned long va = 0;
	unsigned int pa = 0;
	unsigned long virt_base = 0;
	unsigned int offset_in_page = 0;
	unsigned int offset_total = 0;
	struct vm_area_struct *vma;
	struct tee *tee = ctx->tee;

	dev_dbg(_DEV(tee), "%s: > %p\n", __func__, buffer);
	/* if the caller is the kernel api, active_mm is mm */
	if (!mm)
	{
		printk(KERN_ERR "mm = current->active_mm\n");
		mm = current->active_mm;
	}
	
	pa = *(unsigned int*)buffer;
	printk(KERN_ERR "pa =%x\n",pa);
	va = (unsigned long)ioremap(pa, size + offset);


	virt_base = (va / PAGE_SIZE) * PAGE_SIZE;
	offset_in_page = va - virt_base;
	offset_total = offset_in_page + offset;

	BUG_ON(!mm);
	vma = find_vma(mm, virt_base);
	printk(KERN_ERR "va =%lx\n",va);
	printk(KERN_ERR "virt_base =%lx\n",virt_base);
	shm->flags |= TEEC_MEM_DMABUF;
 	shm->paddr = (long int)pa;
#if 0
	if (vma) {	
		unsigned long pfn;
		printk(KERN_ERR "It's a VMA\n");
		/* It's a VMA => consider it a a user address */
		vma->vm_flags=VM_PFNMAP;

		if (follow_pfn(vma, virt_base, &pfn)) {
			//dev_err(_DEV(tee), "%s can't get pfn for %p\n",
			//	__func__, buffer);
			printk(KERN_ERR "failed 1\n");
			ret = -EINVAL;
			goto out;
		}

		shm->paddr = PFN_PHYS(pfn) + offset_total;
		printk(KERN_ERR "pa2 =%x\n",(unsigned int)shm->paddr);
		if (vma->vm_end - vma->vm_start - offset_total < size) {
			//dev_err(_DEV(tee), "%s %p:%x not big enough: %lx - %d < %x\n",
			//		__func__, buffer, shm->paddr,
			//		vma->vm_end - vma->vm_start,
			//		offset_total, size);
			printk(KERN_ERR "failed 2\n");
			shm->paddr = 0;
			ret = -EINVAL;
			goto out;
		}
	} else if (!ctx->usr_client) {

		
		/* It's not a VMA => consider it as a kernel address
		 * And look if it's an internal known phys addr
		 * Note: virt_to_phys is not usable since it can be a direct
		 * map or a vremap address
		*/
		unsigned int phys_base;

		int nb_page = (PAGE_SIZE - 1 + size + offset_total) / PAGE_SIZE;
		int i;
		printk(KERN_ERR "It's not VMA\n");
		spin_lock(&mm->page_table_lock);
		phys_base = tee_shm_get_phy_from_kla(mm, virt_base);

		if (!phys_base) {
			spin_unlock(&mm->page_table_lock);
			printk(KERN_ERR "failed 3\n");
			//dev_err(_DEV(tee), "%s can't get physical address for %p\n",
			//		__func__, buffer);
			goto err;
		}

		/* Check continuity on size */
		for (i = 1; i < nb_page; i++) {
			unsigned int pa = tee_shm_get_phy_from_kla(mm,
					virt_base + i*PAGE_SIZE);
			if (pa != phys_base + i*PAGE_SIZE) {
				printk(KERN_ERR "failed 4\n");
				spin_unlock(&mm->page_table_lock);
			//	dev_err(_DEV(tee), "%s %p:%x not big enough: %lx - %d < %x\n",
			//			__func__, buffer, phys_base,
			//			i*PAGE_SIZE,
			//			offset_total, size);
				goto err;
			}
		}
		spin_unlock(&mm->page_table_lock);

		shm->paddr = phys_base + offset_total;
		goto out;
err:
		ret = -EINVAL;
	}

out:
//	dev_dbg(_DEV(tee), "%s: < %d shm=%p vaddr=%p paddr=%x\n",
//			__func__, ret, (void *)shm, buffer, shm->paddr);
#endif
	return ret;
}
#endif