#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>

#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <asm/mutex.h>
#include <asm/io.h>

#include <linux/interrupt.h>
#include <linux/wait.h>

#include <linux/uaccess.h>  //for copy_form_user

//#include "debug.h"
#include <linux/compat.h>

#include "rtk_mcp.h"

#define MCP_DEVICE_NAME         "mcp"

#define _mcp_map_single(p_data, len, dir)      dma_map_single(mcp_device, (void*) p_data, (size_t) len, dir)
#define _mcp_unmap_single(p_data, len, dir)    dma_unmap_single(mcp_device, (dma_addr_t) p_data, (size_t)  len, dir)

//#define nop() __asm__ __volatile__("mov\tr0,r0\t@ nop\n\t");
//#define nop() __asm__ __volatile__("nop");



void mcp_dump_all_regieters(void);
void mcp_dump_descriptor(mcp_desc *p_desc);

int mcp_do_command(
    mcp_desc*               p_desc, 
    int                     n_desc
    );

void MCP_AES_128_ECB_DataEncryptTest(void);

int MCP_AES_Encryption(    
    unsigned char           mode,
    unsigned char           key[16],
    unsigned char           iv[16],
    unsigned char*          p_in,
    unsigned char*          p_out,
    unsigned long           len
    );

void mcp_dump_mem(unsigned char* data, unsigned int len);


//static spinlock_t       mcp_buffer_lock = SPIN_LOCK_UNLOCKED; //removed on Linux 3.x
static DEFINE_SPINLOCK(mcp_buffer_lock);
static DEFINE_MUTEX(mcp_mutex);

void __iomem *mcp_base = 0;
void __iomem *tp_base = 0;

static struct device*   mcp_device;
static struct cdev      mcp_dev;
static dev_t            devno;

struct class * mcp_dev_class = NULL;

/*
 *  writeSRAM
 *  Parameter :
 *      id : offset in cw
 *      data : data array to be written
 *      cnt : entry count for data writing in SRAM
 *  Return :
 *      true : successful
 *      false : false
 */
unsigned char writeSRAM(unsigned int id, unsigned int data[8], unsigned int cnt)
{
    int i=0,y=0;

    #if 0
    if ((id&0x7f)>0){   
                printf("*** %s %s %d\n", __FILE__, __FUNCTION__, __LINE__ );
                return FALSE;
        }
    if (((id+cnt)&0x7f)>0){ 
                printf("*** %s %s %d\n", __FILE__, __FUNCTION__, __LINE__ );
                return FALSE;
        }
    #endif
    for (i=0;i<cnt;i++)
    {
        //dbg_info("%s %s %d==>data: %x",__FILE__, __func__,__LINE__,data[y]);
        SET_TP_KEYINFO_0(data[y++],tp_base);
            //dbg_info("%s %s %d==>data: %x",__FILE__, __func__,__LINE__,data[y]);
        SET_TP_KEYINFO_1(data[y++],tp_base);
        SET_TP_KEY_CTRL((id+i) | 0x80,tp_base);  // write 8 bytes
        nop();nop();nop();nop();nop();nop();nop();nop();nop();nop();nop();nop();nop();nop();nop();nop();
    }
    SET_TP_KEYINFO_0(0x0,tp_base);
    SET_TP_KEYINFO_1(0x0,tp_base);
    return 0;
}

static int mcp_probe(struct platform_device *dev)
{
   // dbg_info("%s %s %d",__FILE__, __func__,__LINE__);
    //mcp_warning("mcp_probe\n");

    return 0;
}

static int mcp_remove(struct platform_device *dev)
{
    return 0;
}

static int mcp_suspend(struct platform_device *dev, pm_message_t state)
{
    return 0;
}

static int mcp_resume(struct platform_device *dev)
{
    return 0;
}

static struct of_device_id rtk_mcp_ids[] = {
    { .compatible = "Realtek,rtk-mcp" },
    { /* Sentinel */ },
};

static struct platform_driver rtk_mcp_driver = {
    .probe      = mcp_probe,
    .remove     = mcp_remove,
    .suspend    = mcp_suspend,
    .resume     = mcp_resume,
    .driver     = {
        .name   = "rtk129x_mcp",
        .owner  = THIS_MODULE,
        .of_match_table = rtk_mcp_ids,
    },
};

/***************************************************************************
            ------------------- MISC ----------------
****************************************************************************/



/*------------------------------------------------------------------ 
 * Func : mcp_malloc
 *
 * Desc : allocate memory 
 *
 * Parm : size      : size of data 
 *
 * Retn : start address of data
 *------------------------------------------------------------------*/
void* mcp_malloc(unsigned long size)
{    
#ifdef USE_DVR_MALLOC
    if (system_state == SYSTEM_BOOTING)
        return (void*) kmalloc(size, GFP_KERNEL);
    else
        return (size >= PAGE_SIZE) ? (void*) dvr_malloc(size) 
                                   : (void*) kmalloc(size, GFP_KERNEL);         
#else
    return (size >= PAGE_SIZE) ? (void*) __get_free_pages(GFP_KERNEL, get_order(size)) 
                               : (void*) kmalloc(size, GFP_KERNEL) ;     
#endif            
}




/*------------------------------------------------------------------ 
 * Func : mcp_free
 *
 * Desc : release memory 
 *
 * Parm : addr      : Start address of data
 *        size      : size of data 
 *
 * Retn : N/A
 *------------------------------------------------------------------*/
void mcp_free(void* addr, unsigned long size)
{   
    if (system_state == SYSTEM_BOOTING) {
        kfree(addr) ;
        return;
    }

    if (size >= PAGE_SIZE) 
#ifdef USE_DVR_MALLOC         
        dvr_free(addr);
#else
        free_pages((unsigned long)addr, get_order(size));
#endif    
    else        
        kfree(addr) ;     
}



/***************************************************************************
         ------------------- File Operations ----------------
****************************************************************************/



/*------------------------------------------------------------------
 * Func : mcp_dev_open
 *
 * Desc : open function of mcp dev
 *
 * Parm : inode : inode of dev
 *        file  : context of file
 *         
 * Retn : 0 : success, others fail  
 *------------------------------------------------------------------*/
static 
int mcp_dev_open(struct inode *inode, struct file *file)
{       
    //mcp_warning("mcp_dev_open\n");

    file->private_data = mcp_malloc(sizeof(mcp_desc) * MCP_DESC_ENTRY_COUNT);    
    if (file->private_data==NULL) 
    {
        mcp_warning("open mcp failed - allocate mcp descriptor buffer failed\n");
        return -ENOMEM;
    }
    
    return 0;    
}


/*------------------------------------------------------------------
 * Func : mcp_dev_release
 *
 * Desc : release function of mcp dev
 *
 * Parm : inode : inode of dev
 *        file  : context of file
 *         
 * Retn : 0 : success, others fail  
 *------------------------------------------------------------------*/
static 
int mcp_dev_release(
    struct inode*           inode, 
    struct file*            file
    )
{               
    if (file->private_data)
        mcp_free(file->private_data, sizeof(mcp_desc) * MCP_DESC_ENTRY_COUNT);
    return 0;
}

typedef struct 
{
    compat_uint_t flags;    
    compat_uint_t key[6];  
    compat_uint_t iv[4];  
    compat_uint_t data_in;      // data in : physical address
    compat_uint_t data_out;     // data out : physical address 
    compat_uint_t length;       // data length
}compat_mcp_desc_t;

static int compat_get_mcp_desc_data(
    compat_mcp_desc_t __user *data32,
    mcp_desc __user *data)
{
    //compat_uint_t s;
    compat_uint_t c;
    compat_uint_t b[6];
	compat_uint_t k[4];
	compat_uint_t t;
	compat_uint_t p;
	compat_uint_t q;
	
    int err;

    err = get_user(c, &data32->flags);
    err |= put_user(c, &data->flags);
    err |= copy_from_user(b, data32->key, sizeof(b));
    err |= copy_to_user(data->key, b, sizeof(b));
    err |= copy_from_user(k, data32->iv, sizeof(k));
    err |= copy_to_user(data->iv, k, sizeof(k));
    err |= get_user(t, &data32->data_in);
    err |= put_user(t, &data->data_in);
    err |= get_user(p, &data32->data_out);
    err |= put_user(p, &data->data_out);
    err |= get_user(q, &data32->length);
    err |= put_user(q, &data->length);


    return err;
}

static int compat_put_mcp_desc_data(
    compat_mcp_desc_t __user *data32,
    mcp_desc __user *data)
{
    //compat_uint_t s;
    compat_uint_t c;
    compat_uint_t b[6];
	compat_uint_t k[4];
	compat_uint_t t;
	compat_uint_t p;
	compat_uint_t q;
	
    int err;

    err = get_user(c, &data->flags);
    err |= put_user(c, &data32->flags);
    err |= copy_from_user(b, data->key, sizeof(b));
    err |= copy_to_user(data32->key, b, sizeof(b));
    err |= copy_from_user(k, data->iv, sizeof(k));
    err |= copy_to_user(data32->iv, k, sizeof(k));
    err |= get_user(t, &data->data_in);
    err |= put_user(t, &data32->data_in);
    err |= get_user(p, &data->data_out);
    err |= put_user(p, &data32->data_out);
    err |= get_user(q, &data->length);
    err |= put_user(q, &data32->length);


    return err;
}


/*------------------------------------------------------------------
 * Func : mcp_dev_ioctl
 *
 * Desc : ioctl function of mcp dev
 *
 * Parm : inode : inode of dev
 *        file  : context of file
 *        cmd   : control command
 *        arg   : arguments
 *         
 * Retn : 0 : success, others fail  
 *------------------------------------------------------------------*/
static
long compat_mcp_dev_ioctl(
    struct file*            file,
    unsigned int            cmd, 
    unsigned long           arg
    )
{                   
    mcp_desc_set desc_set;
    //mcp_desc*    desc_list = (mcp_desc*) KSEG1ADDR(file->private_data);
    mcp_desc*    desc_list = (mcp_desc*)(file->private_data);
    mcp_desc*    p_desc;
    int          n_desc;
    void __user *arg64;

    int key_cnt = 0;

    //mcp_info("[MCP] =====>compat_mcp_dev_ioctl:\n");

    switch (cmd)
    {
    case MCP_IOCTL_DO_COMMAND:
        arg64 = compat_ptr(arg);

        if (copy_from_user(&desc_set, (void *)arg64, sizeof(mcp_desc_set)))
        {
            mcp_warning("do ioctl command failed - copy desc_set from user failed 1\n");
            return -EFAULT;
        }
        p_desc = compat_ptr(desc_set.p_desc);


        while(desc_set.n_desc)
        {
            n_desc = (desc_set.n_desc >= MCP_DESC_ENTRY_COUNT) ? MCP_DESC_ENTRY_COUNT
                                                               : desc_set.n_desc;

            if (copy_from_user(desc_list, (void *)p_desc, sizeof(mcp_desc) * n_desc))
            {
                mcp_warning("do ioctl command failed - copy desc from user failed 2\n");
                return -EFAULT;
            }

            if(desc_list->flags & ((0x1)<<13))
            {
                mcp_info("%s %s %d==>flags s:%x\n", __FILE__, __func__, __LINE__, desc_list->flags);

                if(desc_list->flags&((0x1)<<3)) {
                    mcp_info("%s %s %d==>AES 192 mode key in CW\n", __FILE__, __func__, __LINE__);
                    key_cnt=3;
                } else {
                    mcp_info("%s %s %d==>AES 256 mode key in CW\n", __FILE__, __func__, __LINE__);
                    key_cnt=4;
                }


                if (writeSRAM(0, desc_list->key, key_cnt) == -1)       // write key from cw offset 0x00
                {
                    mcp_warning("%s %s %d==>writeSRAM fail\n",__FILE__, __func__,__LINE__);
                    return  -EFAULT;
                }


                desc_list->key[0]=0x0;
            }

            if (mcp_do_command(desc_list, n_desc)<0)
            {
                mcp_warning("do command failed\n");
                return -EFAULT;
            }

            p_desc          += n_desc;
            desc_set.n_desc -= n_desc;
        }                             
        break;

    //-----------------------------------
    // for testing only        
    //-----------------------------------                
    case MCP_IOCTL_TEST_AES_H:
        //MCP_AES_H_DataHashTest();
        break;
                                        
    default:        
        mcp_warning("do ioctl command failed - unknown ioctl command(%d)\n", cmd);
        return -EFAULT;          
    }

    return 0;          
}

/*------------------------------------------------------------------
 * Func : mcp_dev_ioctl
 *
 * Desc : ioctl function of mcp dev
 *
 * Parm : inode : inode of dev
 *        file  : context of file
 *        cmd   : control command
 *        arg   : arguments
 *         
 * Retn : 0 : success, others fail  
 *------------------------------------------------------------------*/
static 
long mcp_dev_ioctl(
    struct file*            file,
    unsigned int            cmd, 
    unsigned long           arg
    )
{                   
    mcp_desc_set desc_set;    
    //mcp_desc*    desc_list = (mcp_desc*) KSEG1ADDR(file->private_data);
    mcp_desc*    desc_list = (mcp_desc*)(file->private_data);
    mcp_desc*    p_desc;    
    int          n_desc;

    int key_cnt = 0;

    //mcp_info("[MCP] =====>mcp_dev_ioctl:\n");
    
    switch (cmd)
    {
    case MCP_IOCTL_DO_COMMAND:                
        if (copy_from_user(&desc_set, (mcp_desc_set __user *)arg, sizeof(mcp_desc_set)))
        {
            mcp_warning("do ioctl command failed - copy desc_set from user failed 1\n");
            return -EFAULT;
        }           
        p_desc = desc_set.p_desc;

        
        while(desc_set.n_desc)                      
        {            
            n_desc = (desc_set.n_desc >= MCP_DESC_ENTRY_COUNT) ? MCP_DESC_ENTRY_COUNT 
                                                               : desc_set.n_desc;         

            if (copy_from_user(desc_list, (mcp_desc __user *)p_desc, sizeof(mcp_desc) * n_desc))
            {                
                mcp_warning("do ioctl command failed - copy desc from user failed 2\n");
                return -EFAULT;            
            }

            if(desc_list->flags & ((0x1)<<13)) 
            {
                mcp_info("%s %s %d==>flags s:%x\n", __FILE__, __func__, __LINE__, desc_list->flags);

                if(desc_list->flags&((0x1)<<3)) {
                    mcp_info("%s %s %d==>AES 192 mode key in CW\n", __FILE__, __func__, __LINE__);
                    key_cnt=3;
                } else {
                    mcp_info("%s %s %d==>AES 256 mode key in CW\n", __FILE__, __func__, __LINE__);
                    key_cnt=4;
                }


                if (writeSRAM(0, desc_list->key, key_cnt) == -1)       // write key from cw offset 0x00
                {
                    mcp_warning("%s %s %d==>writeSRAM fail\n",__FILE__, __func__,__LINE__);
                    return  -EFAULT;
                }


                desc_list->key[0]=0x0;
            }

            if (mcp_do_command(desc_list, n_desc)<0)                                            
            {
                mcp_warning("do command failed\n");
                return -EFAULT;
            }

            p_desc          += n_desc;
            desc_set.n_desc -= n_desc;
        }                         
        break;
        
    //-----------------------------------
    // for testing only        
    //-----------------------------------                
    case MCP_IOCTL_TEST_AES_H:
        //MCP_AES_H_DataHashTest();
        break;
                                        
    default:        
        mcp_warning("do ioctl command failed - unknown ioctl command(%d)\n", cmd);
        return -EFAULT;          
    }

    return 0;          
}

static struct file_operations mcp_ops = 
{
    .owner      = THIS_MODULE,  
    //.ioctl      = mcp_dev_ioctl,
    .unlocked_ioctl = mcp_dev_ioctl,
    .compat_ioctl = compat_mcp_dev_ioctl,
    .open       = mcp_dev_open,
    .release    = mcp_dev_release,
};

/*------------------------------------------------------------------
 * Func : _mcp_load_otp
 *
 * Desc : load otp key
 *
 * Parm : N/A
 *         
 * Retn : N/A
 *------------------------------------------------------------------*/
static void _mcp_load_otp(void)
{
    int i = 0;
    
    SET_MCP_OTP_LOAD(1,mcp_base);
    
    while(GET_MCP_OTP_LOAD(mcp_base))
    {
        if (i++ > 100)
        {
            mcp_warning("Load OTP Key Timeout\n");              
        }          

        udelay(10);                                                
    }            
}

/*------------------------------------------------------------------
 * Func : _mcp_phy_init
 *
 * Desc : init mcp engine
 *
 * Parm : N/A
 *         
 * Retn : N/A
 *------------------------------------------------------------------*/
static int _mcp_phy_init(void)
{                          
    _mcp_load_otp();
    
    SET_MCP_CTRL(MCP_GO,mcp_base);       // dessert go bit                        
    SET_MCP_EN(0xFE,mcp_base);           // disable all interrupts
    SET_MCP_STATUS(0xFE,mcp_base);       // clear interrupts status        
    SET_MCP_BASE (0,mcp_base);
    SET_MCP_LIMIT(0,mcp_base);
    SET_MCP_RDPTR(0,mcp_base);
    SET_MCP_WRPTR(0,mcp_base);        
                         
    //SET_MCP_CTRL(MCP_ARB_MODE(1) | MCP_WRITE_DATA);     // set arbitraction mode to mode 1

    //SET_MCP_CTRL1(MCP_AES_PAD_OFF(1) |                  // disable AES_H auto padding
     //             MCP_CSA_ENTROPY(ORIGIONAL_MODE) |     // setup csa mode
     //             MCP_ROUND_NO(0));                     // set round number of multi-2 to 1

    return 0;
}

/*------------------------------------------------------------------
 * Func : mcp_init
 *
 * Desc : init mcp engine
 *
 * Parm : N/A
 *         
 * Retn : N/A
 *------------------------------------------------------------------*/
static int mcp_init(void)
{           
    if (_mcp_phy_init()<0)            
        return -1;    
    
#if 0//def MCP_INTERRUPT_ENABLE
    if (request_irq(MCP_IRQ, mcp_isr, SA_INTERRUPT | SA_SHIRQ, "MCP", &mcp_wait_queue) < 0) 
    {
        mcp_warning("Request irq %d failed\n", MCP_IRQ);                
        return -ENODEV;
    }
#endif           

    return 0;    
}

/*------------------------------------------------------------------
 * Func : mcp_uninit
 *
 * Desc : uninit mcp engine
 *
 * Parm : N/A
 *         
 * Retn : N/A
 *------------------------------------------------------------------*/
static void mcp_uninit(void)
{        
    SET_MCP_CTRL(MCP_GO,mcp_base);           // dessert go bit
    SET_MCP_EN(0xFE,mcp_base);               // disable all interrupts        
    msleep(10);                     // wait for hw stop            
    SET_MCP_BASE (0,mcp_base);
    SET_MCP_LIMIT(0,mcp_base);
    SET_MCP_RDPTR(0,mcp_base);
    SET_MCP_WRPTR(0,mcp_base);                
#if 0 //def MCP_INTERRUPT_ENABLE    
    free_irq(MCP_IRQ, &mcp_wait_queue);
#endif    
}

/*---------------------------------------------------------------------- 
 * Func : mcp_dump_mem 
 *
 * Desc : dump data in memory
 *
 * Parm : data : start address of data
 *        len  : length of data
 *
 * Retn : N/A
 *----------------------------------------------------------------------*/ 
void mcp_dump_mem(unsigned char* data, unsigned int len)
{
    int i;                  
    for (i=0; i<len; i++)
    {
        if ((i & 0xF)==0)
            printk("\n %04x | ", i);                    
        printk("%02x ", data[i]);                    
    }    
    printk("\n");
}

/*---------------------------------------------------------------------- 
 * Func : mcp_dump_all_registers 
 *
 * Desc : dump mcp registers
 *
 * Parm : N/A
 *
 * Retn : N/A
 *----------------------------------------------------------------------*/ 
void mcp_dump_all_registers(void)
{    
    mcp_info("****** MCP Registers ******\n");
    mcp_info("MCP_CTRL addr:0x%x, value:0x%x\n", MCP_CTRL, GET_MCP_CTRL(mcp_base));
    mcp_info("MCP_STATUS addr:0x%x, value:0x%x\n", MCP_STATUS, GET_MCP_STATUS(mcp_base));
    mcp_info("MCP_EN addr:0x%x, value:0x%x\n", MCP_EN, GET_MCP_EN(mcp_base));
    mcp_info("MCP_BASE addr:0x%x, value:0x%x\n", MCP_BASE, GET_MCP_BASE(mcp_base));
    mcp_info("MCP_LIMIT addr:0x%x, value:0x%x\n", MCP_LIMIT, GET_MCP_LIMIT(mcp_base));
    mcp_info("MCP_RDPTR addr:0x%x, value:0x%x\n", MCP_RDPTR, GET_MCP_RDPTR(mcp_base));
    mcp_info("MCP_WRPTR addr:0x%x, value:0x%x\n", MCP_WRPTR, GET_MCP_WRPTR(mcp_base));
    mcp_info("MCP_DES_COUNT addr:0x%x, value:0x%x\n", MCP_DES_COUNT, GET_MCP_DES_COUNT(mcp_base));
    mcp_info("MCP_DES_COMPARE addr:0x%x, value:0x%x\n", MCP_DES_COMPARE, GET_MCP_DES_COMPARE(mcp_base));
}


/*---------------------------------------------------------------------- 
 * Func : mcp_dump_descriptor 
 *
 * Desc : dump mcp descriptor
 *
 * Parm : p_desc: pointer  to descriptor
 *
 * Retn : N/A
 *----------------------------------------------------------------------*/ 
void mcp_dump_descriptor(mcp_desc *p_desc)
{
    int i = 0;

    mcp_info("****** MCP Descriptor ******\n");
    mcp_info("p_desc->flags:0x%0x\n", p_desc->flags);

    for (i = 0; i < sizeof(p_desc->key)/sizeof(p_desc->key[0]); i++) {
        mcp_info("p_desc->key[%d]:0x%0x\n", i, p_desc->key[i]);
    }

    for (i = 0; i < sizeof(p_desc->iv)/sizeof(p_desc->iv[0]); i++) {
        mcp_info("p_desc->iv[%d]:0x%0x\n", i, p_desc->iv[i]);
    }

    mcp_info("p_desc->data_in:0x%0x\n", p_desc->data_in);
    mcp_info("p_desc->data_out:0x%0x\n", p_desc->data_out);
    mcp_info("p_desc->length:0x%0x\n", p_desc->length);
}


/*------------------------------------------------------------------ 
 * Func : _mcp_set_desc_buffer
 *
 * Desc : set descriptors buffer
 *
 * Parm : base  : base address of descriptor buffer
 *        limit : limit address of descriptor buffer
 *        rp    : read pointer of descriptor buffer
 *        wp    : write pointer of descriptor buffer
 *
 * Retn : 0
 *------------------------------------------------------------------*/
int _mcp_set_desc_buffer(
    unsigned long           base, 
    unsigned long           limit, 
    unsigned long           rp, 
    unsigned long           wp
    )
{
    unsigned long event;
    spin_lock_irqsave(&mcp_buffer_lock, event);                           
                                  
    SET_MCP_BASE (base, mcp_base);

    SET_MCP_LIMIT(limit, mcp_base);

    SET_MCP_RDPTR(rp, mcp_base);

    SET_MCP_WRPTR(wp, mcp_base);
            
    spin_unlock_irqrestore(&mcp_buffer_lock, event);   
    
    return 0;
}


/*------------------------------------------------------------------ 
 * Func : _mcp_start_xfer
 *
 * Desc : Start Xfer
 *
 * Parm : N/A
 *
 * Retn : S_OK /  S_FALSE 
 *------------------------------------------------------------------*/
int _mcp_start_xfer(void)
{   
    int ret = -1;                   
    int WaitTime = 0x3FF << 2;          
    int wiat_clear_timeout = 0; 

    SET_MCP_DES_COUNT(0x0, mcp_base);   // descriptor in ddr
    
    SET_MCP_CTRL(MCP_CLEAR | MCP_WRITE_DATA, mcp_base);    // issue clear   

    while ((GET_MCP_CTRL(mcp_base) & MCP_CLEAR) && wiat_clear_timeout++ < 30);     

    if (GET_MCP_CTRL(mcp_base) & MCP_CLEAR) 
    {       
        mcp_warning("wait clear bit deassert timeout,  force unset clear bit, (CTRL=%08x, STATUS=%08x)\n", 
                    GET_MCP_CTRL(mcp_base), GET_MCP_STATUS(mcp_base));
        SET_MCP_CTRL(MCP_CLEAR, mcp_base);    // issue clear        
        mcp_warning("CTRL=%08x, STATUS=%08x)\n",GET_MCP_CTRL(mcp_base), GET_MCP_STATUS(mcp_base));
    }            
    
    SET_MCP_EN(0xFE, mcp_base);               

    SET_MCP_STATUS(0xFE, mcp_base);    // clear status

    //mcp_info("before go:\n");
    //mcp_dump_all_registers();

    SET_MCP_CTRL(MCP_GO | MCP_WRITE_DATA, mcp_base);

    while (WaitTime--)    
    {        
        mcp_debug("STATUS=%08x, CTRL=%08x\n",GET_MCP_STATUS(mcp_base), GET_MCP_CTRL(mcp_base));
        if (!(GET_MCP_CTRL(mcp_base) & MCP_GO)) {
            //mcp_info("ctrl break\n");
            break;
        }

        if (GET_MCP_STATUS(mcp_base)&0x6) {
            //mcp_info("status break\n");
            break;
        }

        msleep(1);
    }

    ret = ((GET_MCP_STATUS(mcp_base) & ~(MCP_RING_EMPTY | MCP_COMPARE))) ? -1 : 0;

    //mcp_info("after go:\n");
    //mcp_dump_all_registers();

    if (ret <0)
        mcp_warning("do mcp command failed, (MCP_Status %08x)\n", GET_MCP_STATUS(mcp_base));
        
    SET_MCP_CTRL(MCP_GO, mcp_base);               // clear go bit        
    
    SET_MCP_STATUS(0xFE, mcp_base);               // clear ring empty    

    return ret;
}

/*------------------------------------------------------------------ 
 * Func : mcp_do_command 
 *
 * Desc : Do Command
 *
 * Parm : p_desc : number of Descriptor to be Execute
 *        n_desc  : number of Descriptor to be Execute
 *
 * Retn : 0 : success, others fail  
 *------------------------------------------------------------------*/
int mcp_do_command(
    mcp_desc*               p_desc, 
    int                     n_desc
    )
{                
    int         ret = 0;    
    int         len = sizeof(mcp_desc) * n_desc;

    int         i, j;
    unsigned long desc_dma_vaddr;
    dma_addr_t desc_dma;
    mcp_desc *p_desc_vaddr;

    if (n_desc) {
        
        mutex_lock(&mcp_mutex);

	    desc_dma_vaddr = (unsigned long)dma_alloc_coherent(NULL,
			    n_desc * sizeof(mcp_desc) + sizeof(mcp_desc),
			    &desc_dma, GFP_ATOMIC);

        if (desc_dma_vaddr) {
            
            p_desc_vaddr = (mcp_desc *)desc_dma_vaddr;

            for (i = 0; i < n_desc; i++) {
                p_desc_vaddr->flags = p_desc->flags;

                for (j = 0; j < sizeof(p_desc->key)/sizeof(p_desc->key[0]); j++) {
                    p_desc_vaddr->key[j] = p_desc->key[j];
                }

                for (j = 0; j < sizeof(p_desc->iv)/sizeof(p_desc->iv[0]); j++) {
                    p_desc_vaddr->iv[j] = p_desc->iv[j];
                }

                p_desc_vaddr->data_in = p_desc->data_in;
                p_desc_vaddr->data_out = p_desc->data_out;
                p_desc_vaddr->length = p_desc->length;

                mcp_dump_descriptor(p_desc_vaddr);

                p_desc_vaddr++;
                p_desc++;
            }
            
            p_desc_vaddr = p_desc_vaddr - n_desc;
            p_desc = p_desc - n_desc;

            _mcp_set_desc_buffer(desc_dma, desc_dma + len + sizeof(mcp_desc), desc_dma, desc_dma + len);

            ret = _mcp_start_xfer();
            
        }
        else {
            mcp_warning("no dma buffer for descriptor!\n");
            ret = -1;   // error, no dma buffer for descriptor
        }

        dma_free_coherent(NULL, n_desc * sizeof(mcp_desc),
		    (void *)desc_dma_vaddr, desc_dma);
           
        mutex_unlock(&mcp_mutex);
    }   

    return ret;
}

/********************************************************************************
 AES
 ********************************************************************************/



/*====================================================================== 
 * Func : MCP_AES_DESC_INIT
 *
 * Desc : Init AES Descriptor
 *
 * Parm : pDesc      : Descriptor to be Load
 *        EnDe       : Encryption/Descryption
 *              0 for Decryption / 1 for Encryption
 *        Mode       : Operation Mode
 *        Key        : Key Value
 *        IV         : Initial Vector 
 *
 * Retn : S_OK /  S_FALSE 
 *======================================================================*/
int MCP_AES_DESC_INIT(
    mcp_desc*               pDesc, 
    unsigned char           EnDe,
    unsigned char           Mode,
    unsigned char           Key[16],
    unsigned char           IV[16]
    )
{            
    memset(pDesc, 0, sizeof(mcp_desc));

    switch (Mode)
    {
    case MCP_BCM_ECB:        
    case MCP_BCM_CBC:
    case MCP_BCM_CTR:                
        
        pDesc->flags = MARS_MCP_MODE(MCP_ALGO_AES) | MARS_MCP_BCM(Mode) | MARS_MCP_ENC(EnDe);
                
        if (Key)                                        
        {            
            pDesc->key[0]  = (Key[ 0]<<24) + (Key[ 1]<<16) + (Key[ 2]<<8) + Key[ 3];  
            pDesc->key[1]  = (Key[ 4]<<24) + (Key[ 5]<<16) + (Key[ 6]<<8) + Key[ 7];  
            pDesc->key[2]  = (Key[ 8]<<24) + (Key[ 9]<<16) + (Key[10]<<8) + Key[11];  
            pDesc->key[3]  = (Key[12]<<24) + (Key[13]<<16) + (Key[14]<<8) + Key[15];  
        }                    
        else
            pDesc->flags |= MARS_MCP_KEY_SEL(MCP_KEY_SEL_OTP);
                     
        if (IV)        
        {
            pDesc->iv[0]   = (IV[ 0]<<24) + (IV[ 1]<<16) + (IV[ 2]<<8) + IV[ 3];  
            pDesc->iv[1]   = (IV[ 4]<<24) + (IV[ 5]<<16) + (IV[ 6]<<8) + IV[ 7];  
            pDesc->iv[2]   = (IV[ 8]<<24) + (IV[ 9]<<16) + (IV[10]<<8) + IV[11];  
            pDesc->iv[3]   = (IV[12]<<24) + (IV[13]<<16) + (IV[14]<<8) + IV[15]; 
        }
        else
            pDesc->flags |= MARS_MCP_IV_SEL(MCP_IV_SEL_REG);
    
        return 0;                
        
    default:        
        mcp_warning("Init AES descriptor failed - invalid mode (%d)\n", Mode);
        return -1;
    }        
}



/*====================================================================== 
 * Func : MCP_AES_Decryption
 *
 * Desc : Do AES Decryption
 *
 * Parm : mode       : Operation Mode
 *        key        : Key Value
 *        iv         : Initial Vector 
 *        p_in       : Data In 
 *        p_out      : Data Out
 *        len        : Data Length
 *
 * Retn : 0 for success, others failed
 *======================================================================*/
int MCP_AES_Decryption(    
    unsigned char           Mode,
    unsigned char           Key[16],
    unsigned char           IV[16],
    unsigned char*          p_in,
    unsigned char*          p_out,
    unsigned long           len
    )
{    
    mcp_desc    desc;
    int         ret;    
    dma_addr_t  addr1;
    dma_addr_t  addr2;

    MCP_AES_DESC_INIT(&desc, 0, Mode, Key, IV);

    addr1 = _mcp_map_single(p_in, len, DMA_TO_DEVICE);    
    addr2 = _mcp_map_single(p_out, len, DMA_FROM_DEVICE);    

    desc.data_in  = addr1;
    desc.data_out = addr2;
    desc.length   = len & ~0xF;    
    
    ret = mcp_do_command(&desc, 1);   
    
    _mcp_unmap_single(addr1, len, DMA_TO_DEVICE);                    
    _mcp_unmap_single(addr2, len, DMA_FROM_DEVICE);  
    
    return ret;
}




/*====================================================================== 
 * Func : MCP_AES_Encryption
 *
 * Desc : Do AES Encryption
 *
 * Parm : mode       : Operation Mode
 *        key        : Key Value
 *        iv         : Initial Vector 
 *        p_in       : Data In 
 *        p_out      : Data Out
 *        len        : Data Length
 *
 * Retn : 0 for success, others failed
 *======================================================================*/
int MCP_AES_Encryption(    
    unsigned char           mode,
    unsigned char           key[16],
    unsigned char           iv[16],
    unsigned char*          p_in,
    unsigned char*          p_out,
    unsigned long           len
    )
{    
    mcp_desc    desc;
    int         ret;    
    dma_addr_t  addr1;    
    dma_addr_t  addr2;    

    MCP_AES_DESC_INIT(&desc, 1, mode, key, iv);

    addr1 = _mcp_map_single(p_in, len, DMA_TO_DEVICE);    
    addr2 = _mcp_map_single(p_out, len, DMA_FROM_DEVICE);    
    
    desc.data_in  = addr1;
    desc.data_out = addr2;
    desc.length   = len & ~0xF;

    ret = mcp_do_command(&desc, 1);   

    _mcp_unmap_single(addr1, len, DMA_TO_DEVICE);                

    _mcp_unmap_single(addr2, len, DMA_FROM_DEVICE);  

    return ret; 
}

/*------------------------------------------------------------------ 
 * Func : MCP_AES_128_ECB_DataEncryptTest
 *
 * Desc : Test AES 128 ECB
 *
 * Parm : N/A
 *
 * Retn : N/A
 *------------------------------------------------------------------*/
void MCP_AES_128_ECB_DataEncryptTest(void)
{
    unsigned char Data[16]={0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 
                            0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x00};        
    unsigned char Key[16]; 
    
    memset(Key, 'b', sizeof(Key));

    MCP_AES_ECB_Encryption(Key, Data, Data, 16);

    mcp_dump_data_with_text(Data, 16, "encrypted value : ");    

    MCP_AES_ECB_Decryption(Key, Data, Data, 16);

    mcp_dump_data_with_text(Data, 16, "decrypted value : ");    
}


static int __init rtk_mcp_init(void)
{ 
    mcp_base =  ioremap(0x98015000, 0x1000);
    tp_base =  ioremap(0x98014000, 0x1000);

    mcp_info("mcp_base: 0x%0lx\n", (unsigned long)mcp_base);
    mcp_info("tp_base: 0x%0lx\n", (unsigned long)tp_base);
#if 1
    if (mcp_init()<0)    
        return -ENODEV;    

    cdev_init(&mcp_dev, &mcp_ops);            
                
    if (alloc_chrdev_region(&devno, 0, 1, MCP_DEVICE_NAME)!=0)    
    {
        cdev_del(&mcp_dev);
        printk("%s %s %d",__FILE__, __func__,__LINE__);
        return -EFAULT;
    }
    if (cdev_add(&mcp_dev, devno, 1)<0)
        return -EFAULT;      
    
    mcp_dev_class = class_create(THIS_MODULE, "mcp");       // create a new class for mcp
    
    mcp_device = device_create(
                    mcp_dev_class,      // class 
                    NULL,               // parent
                    devno,              // dev number
                    NULL,               // driver data
                    "mcp_core");      // device name                    

    //MCP_AES_128_ECB_DataEncryptTest();
                                
    //MCP_AES_128_CTR_DataEncryptTest();   
    //MCP_AES_H_DataHashTest();
    //MCP_SHA1_DataHashTest();
    return 0;        
#endif

    //return platform_driver_register(&rtk_mcp_driver);
}

static void __exit rtk_mcp_exit(void)
{
        device_destroy(mcp_dev_class, mcp_device->devt);       // remove device    

        cdev_del(&mcp_dev);
   
        unregister_chrdev_region(devno, 1);            
    
        mcp_uninit();

    //platform_driver_unregister(&rtk_mcp_driver);
}

module_init(rtk_mcp_init);
module_exit(rtk_mcp_exit);


