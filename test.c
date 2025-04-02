//#include <linux/module.h>
//#include <linux/init.h>
//#include <linux/types.h>
//#include <linux/kernel.h>
//#include <linux/fs.h>
#include <linux/cdev.h>
//#include <linux/sched.h>
//#include <linux/slab.h>
//#include <linux/delay.h>
//#include <linux/device.h>
#include <linux/pci.h>
#include <linux/eventfd.h>
#include <linux/version.h>
#include <linux/dma-mapping.h>

//#include <linux/interrupt.h>
//#include <asm/dma.h>
//#include <asm/current.h>
//#include <asm/uaccess.h>
//#include <linux/uaccess.h>
//#include <linux/nvme.h>
//#include <linux/io-64-nonatomic-lo-hi.h>
//#include <linux/wait.h>
//#include "nvme.h"





MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PCIe UserSpace Driver");

#define DEVICE_NAME "test"
#define TEST_DEV_MINORS		(1U << MINORBITS)



static DEFINE_IDA(test_instance_ida);

static struct class* test_class = NULL;
static dev_t test_device_number = 0;   //major番号を動的に割り当てるために必要

#define PCI_VENDOR_ID_TEST 0x144d
#define PCI_DEVICE_ID_TEST 0xa808

#define TEST_PCI_DRIVER_DEBUG
#ifdef  TEST_PCI_DRIVER_DEBUG
#define tprintk(fmt, ...) printk(KERN_ALERT "** (%3d) %-20s: " fmt, __LINE__,  __func__,  ## __VA_ARGS__)
#else
#define tprintk(fmt, ...) 
#endif



struct dma_entry {
    struct list_head entry;
    void *alloc_ptr;
    dma_addr_t dma_addr;
    long size;
};

struct test_dev{

    struct  device *sys_dev;
    struct  pci_dev *pdev;
    struct  cdev cdev;
    dev_t   device_number;
    int     instance;
    bool    is_open;

    struct list_head dmaheads;


};



struct test_params{
    int vector;
    int fd;

};




// ioctls
#define TEST 'T'
#define IOCTL_SIGNAL_TEST           _IOW(TEST, 1, struct test_params*)
#define IOCTL_SET_SIGNAL            _IOW(TEST, 2, struct test_params*)
#define IOCTL_MAP_HOST_MEMORY       _IOW(TEST, 3, struct test_params*)
#define IOCTL_UNMAP_HOST_MEMORY     _IOW(TEST, 4, struct test_params*)
#define IOCTL_MAP_KENRNEL_MEMORY    _IOW(TEST, 5, struct test_params*)
#define IOCTL_UNMAP_KENRNEL_MEMORY  _IOW(TEST, 6, struct test_params*)
#define IOCTL_GET_BDF               _IOW(TEST, 7, struct test_params*)

static struct pci_device_id test_ids[] =
{
    { PCI_DEVICE(PCI_VENDOR_ID_TEST, PCI_DEVICE_ID_TEST) },
    { 0, },
};

MODULE_DEVICE_TABLE(pci, test_ids);


static int test_open(struct inode* inode, struct file* filp){
    printk(KERN_INFO "%s\n", __func__);
    
    struct test_dev *p = container_of(inode->i_cdev, struct test_dev, cdev);
    filp->private_data = p;
    p -> is_open = 1;
    return 0;
}

static int test_close(struct inode* inode, struct file* filp){
    printk(KERN_INFO "%s\n", __func__);
    struct test_dev *p = filp->private_data;
    
    struct dma_entry *h;
    struct dma_entry *e = NULL;
    
    list_for_each_entry_safe(h, e, &p->dmaheads, entry) {
        list_del(&h->entry);
		dma_free_coherent (NULL, h->size, h->alloc_ptr, h->dma_addr);
        kfree(h);
        
    }

    p -> is_open = 0;
    return 0;
}


static long test_ioctl(struct file *filp, unsigned int ioctlnum, unsigned long ioctlparam){

    int rc;
    struct test_params cmd = {0};
    struct test_dev *p = filp->private_data;

    switch(ioctlnum){
        case IOCTL_SIGNAL_TEST:  /* epoll signal test */
            if((rc = copy_from_user(&cmd, (void  __user*)ioctlparam, sizeof(struct test_params)))){
                return rc;
            }

            struct eventfd_ctx *trigger;
            trigger = eventfd_ctx_fdget(cmd.fd);
	        if (IS_ERR(trigger)) {
                printk(KERN_ERR "trigger");
                return 1;
	        }
            eventfd_signal(trigger);
            eventfd_ctx_put(trigger);
            break;
        case IOCTL_SET_SIGNAL :
      

            break;
        case IOCTL_MAP_HOST_MEMORY:




            break;
        case IOCTL_UNMAP_HOST_MEMORY:
            
            
            
            break;

        case IOCTL_MAP_KENRNEL_MEMORY:
            
            
            break;
        case IOCTL_UNMAP_KENRNEL_MEMORY:
            
            
            break;

        case IOCTL_GET_BDF:
        


            break;

        default:
            break;
    }

    return 0;
}


int mmap_dma(struct file *filp, struct vm_area_struct *vma)
{
    int ret = 0;
    struct test_dev *p = filp->private_data;

    struct dma_entry *head = kzalloc(sizeof(struct dma_entry), GFP_KERNEL);

    if(IS_ERR_OR_NULL(head)){
        ret = PTR_ERR(head);
        goto out_kzalloc;
    }

    long length = vma->vm_end - vma->vm_start;

    head -> size = ((length + ((size_t)1 << PAGE_SHIFT) -1) >> PAGE_SHIFT) << PAGE_SHIFT;

    printk(KERN_INFO "length: %d\n", length);

    printk(KERN_INFO "vma->vm_pgoff: %d\n", vma->vm_pgoff);


#if 1
    head -> alloc_ptr = dma_alloc_coherent (&p->pdev->dev, head -> size, &head->dma_addr, GFP_KERNEL);

    if(IS_ERR_OR_NULL(head -> alloc_ptr)){
        ret = PTR_ERR(head -> alloc_ptr);
        goto out_dma_alloc_coherent;
    }

    list_add_tail(&head->entry, &p-> dmaheads);


	if (vma->vm_pgoff == 0) {
		ret = dma_mmap_coherent(NULL, vma, head->alloc_ptr, head-> dma_addr, head->size);
	} else
	{
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	    vm_flags_set(vma, VM_IO);

        ret = remap_pfn_range(vma, vma->vm_start,
			      PFN_DOWN(head->dma_addr) +
			      vma->vm_pgoff, head->size, vma->vm_page_prot);
	}

    if (ret < 0) {
		goto out;
    }
#endif

    return ret;

out:
    dma_free_coherent (NULL, head->size, head->alloc_ptr, head->dma_addr);

out_dma_alloc_coherent :
    kfree(head);

out_kzalloc:

    return ret;
}

static const struct file_operations fops = {
    .owner          = THIS_MODULE,
    .open           = test_open,
    .release        = test_close,
    .unlocked_ioctl = test_ioctl,
    .mmap           = mmap_dma,

};



static int test_probe(struct pci_dev *pdev, const struct pci_device_id *id){

    int ret = -ENOMEM;
    struct test_dev *p;

    // Allocate device structure
    p = kmalloc(sizeof(struct test_dev), GFP_KERNEL);
    if(IS_ERR_OR_NULL(p)){
        ret = PTR_ERR(p);
        goto out_alloc_test_dev;
    }

    ret = ida_simple_get(&test_instance_ida, 0, 0, GFP_KERNEL);
    if (ret < 0){
        goto out_ida_simple_get;
    }
	
    p -> instance = ret;
    p -> device_number = MKDEV(MAJOR(test_device_number), p -> instance);

    cdev_init(&p->cdev, &fops); 
    p->cdev.owner = THIS_MODULE;

    if ((ret = cdev_add(&p->cdev, p -> device_number , TEST_DEV_MINORS)) != 0) {
        goto out_cdev_add;
    }

    p-> sys_dev = device_create(test_class, 
                    NULL, 
                    p -> device_number, 
                    NULL, 
                    "test%d", 
                    p -> instance);

    if(IS_ERR_OR_NULL(p->sys_dev)){
        ret = PTR_ERR(p->sys_dev);
        p->sys_dev = NULL;

        goto out_device_create;
    }

    INIT_LIST_HEAD(&p->dmaheads);   


    // Device has only Memory mapped space, so pci_enable_device_mem is used.
    if (pci_enable_device_mem(pdev)){
        ret = -ENOMEM; 
        //goto out_pci_enable_device;
    }
		
    pci_set_master(pdev);

    if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64)) && dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32))){
        ret = -ENOMEM;
        //goto out_dma_set_mask_and_coherent;
    }

    ret = pci_request_regions(pdev, DEVICE_NAME);

    p->pdev = pci_dev_get(pdev);
	pci_set_drvdata(pdev, p);


    return 0;

out_device_create:
    cdev_del(&p->cdev);

out_cdev_add:
    ida_simple_remove(&test_instance_ida, p -> instance );

out_ida_simple_get:
    kfree(p);

out_alloc_test_dev:

    return ret;

}

static void test_remove(struct pci_dev* pdev){
    struct test_dev *p = pci_get_drvdata(pdev);


     pci_set_drvdata(pdev, NULL);
	pci_release_regions(pdev);
    pci_disable_device(pdev);


    device_destroy(test_class, p-> device_number );
    cdev_del(&p->cdev); 
    ida_simple_remove(&test_instance_ida, p->instance );
    kfree(p);

}


// PCをシャットダウンしたときに呼ばれる
static void test_shutdown(struct pci_dev *pdev)
{
  
}

static struct pci_driver test_driver = {
    .name = DEVICE_NAME,
    .id_table = test_ids,
    .probe = test_probe,
    .remove = test_remove,
    .shutdown = test_shutdown,
};

/* インストール時に実行 */
static int test_init(void) {
    int ret;

    ida_init(&test_instance_ida);

	ret = alloc_chrdev_region(&test_device_number, 0, TEST_DEV_MINORS, DEVICE_NAME);
	if (ret != 0){
        printk(KERN_ERR "%d: %s()   %d",__LINE__, __FUNCTION__, ret ); 
        return ret;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,4,0)
    test_class = class_create(DEVICE_NAME);
#else
	test_class = class_create(THIS_MODULE, DEVICE_NAME);
#endif

	if (IS_ERR(test_class)) {
		ret = PTR_ERR(test_class);  
        unregister_chrdev_region(test_device_number, TEST_DEV_MINORS);
		return ret;
	}

    ret = pci_register_driver(&test_driver); //ドライバー名、table, コールバック関数(probe, remove)を登録
    if(ret != 0){
        printk(KERN_ERR "%d: %s()   %d",__LINE__, __FUNCTION__, ret );     
        return ret;
    }

    return 0; 
}

/* アンインストール時に実行 */
static void test_exit(void) {

    pci_unregister_driver(&test_driver);  //ドライバー名、table, コールバック関数(probe, remove)を削除  
    if(test_class != NULL) class_destroy(test_class);
    if(test_device_number != 0) unregister_chrdev_region(test_device_number, TEST_DEV_MINORS);
    ida_destroy(&test_instance_ida);
}

module_init(test_init);
module_exit(test_exit);