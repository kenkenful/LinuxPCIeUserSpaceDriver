
#include <linux/cdev.h>
#include <linux/pci.h>
#include <linux/eventfd.h>
#include <linux/version.h>
#include <linux/dma-mapping.h>

//#include <linux/device.h>
#include <linux/blk_types.h>
#include <linux/blkdev.h>

#include <linux/part_stat.h>
#include <linux/jiffies.h>
#include <linux/proc_fs.h>
#include "common.h"

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Simple NVMe Device Driver");


#define DEVICE_NAME "nvmet"
#define NVME_MINORS		(1U << MINORBITS)

#define PROC_NAME "jiffies"

#undef USE_DUMMYBLK_LIST  /* This define is not complete, and already not mentenance. */
#define NUM_OF_DUMMY_FOR_EACH    (128)

static unsigned int vectors = 16;
module_param(vectors, uint, 0644);
MODULE_PARM_DESC(vectors,
		"interrupt vector num.");

static DEFINE_IDA(nvme_instance_ida);

static struct class* nvme_class;
static dev_t nvme_chr_devt;   

static int blk_major;


#define PCI_VENDOR_ID_TEST 0x144d
#define PCI_DEVICE_ID_TEST 0xa808

int jiffies_creater;
bool set_jiffies = false;
struct page **jiffies_pages;

struct dma_entry {
    struct list_head list;
    void *virt_addr;
    dma_addr_t dma_addr;
    long size;
};

struct signal_entry{
    struct list_head list;
    int irq_no;
    void* id;
    struct eventfd_ctx   *trigger;  
};


struct dummyblk_entry {

#if defined(USE_DUMMYBLK_LIST)
    struct list_head list;
    int number;
#endif
    struct page ** pages;
    long npages;

    struct gendisk *gendisk;
};


struct nvme_dev{
    struct pci_dev *pdev;
    struct cdev cdev;
    dev_t  devt;
    int    instance;
    bool   is_open;

    struct device *sysdev;

    struct list_head memlist;
    struct list_head signallist;

    struct eventfd_ctx   *trigger;   /* use when INT-X */

#if defined(USE_DUMMYBLK_LIST)
    struct list_head dummyblklist;
#else    
    struct dummyblk_entry *dummyblk_entry_p;
#endif

};

static const struct pci_device_id test_nvme_ids[] =
{
   // { PCI_DEVICE(PCI_VENDOR_ID_TEST, PCI_DEVICE_ID_TEST) },
   { PCI_DEVICE_CLASS(PCI_CLASS_STORAGE_EXPRESS, 0xffffff) },
    { 0, },
};

MODULE_DEVICE_TABLE(pci, test_nvme_ids);


static int nvmet_pci_open(struct inode* inode, struct file* filp){
    struct nvme_dev *pnvme_dev =container_of(inode->i_cdev, struct nvme_dev, cdev);
    filp->private_data = pnvme_dev;
    pnvme_dev->is_open = 1;
    return 0;
}

/*
    release resource allocated after probe.

*/
static int nvmet_pci_close(struct inode* inode, struct file* filp){
    pr_info("%s\n", __func__);

    struct nvme_dev *pnvme_dev = filp->private_data;

    struct dma_entry *dma_list_h;
    struct dma_entry *dma_list_e = NULL;

    if(set_jiffies && jiffies_creater == pnvme_dev ->instance){
        put_page(jiffies_pages[0]);
        kfree(jiffies_pages);
        set_jiffies = false;
    }
    
    list_for_each_entry_safe(dma_list_h, dma_list_e, &pnvme_dev->memlist, list) {
        pr_info("safe remove in close\n");

        list_del(&dma_list_h->list);
		dma_free_coherent (&pnvme_dev -> pdev->dev, dma_list_h->size, dma_list_h->virt_addr, dma_list_h->dma_addr);
        kfree(dma_list_h);
        
    }

    struct signal_entry *single_list_h;
    struct signal_entry *single_list_e = NULL;

    list_for_each_entry_safe(single_list_h, single_list_e, &pnvme_dev->signallist, list) {
        list_del(&single_list_h->list);
        free_irq(single_list_h->irq_no, single_list_h -> id);
        eventfd_ctx_put(single_list_h -> trigger);
        kfree(single_list_h);
        
    }

#if defined(USE_DUMMYBLK_LIST)
    struct dummyblk_entry *dummyblk_list_h;
    struct dummyblk_entry *dummyblk_list_e = NULL;

    list_for_each_entry_safe(dummyblk_list_h, dummyblk_list_e, &pnvme_dev->dummyblklist, list) {
        pr_info("%s: remove dummyblk\n", __func__);
        list_del(&dummyblk_list_h->list);
        
        del_gendisk(dummyblk_list_h->gendisk);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,19,12)
        blk_put_queue(dummyblk_list_h->gendisk->queue);
#else 
        blk_cleanup_queue(dummyblk_list_h->gendisk->queue);
#endif
	    put_disk(dummyblk_list_h->gendisk);
        
        kfree(dummyblk_list_h);
    }
#else

    for(int i=0; i< NUM_OF_DUMMY_FOR_EACH; ++i){
        if(pnvme_dev -> dummyblk_entry_p[i].gendisk != NULL){
            //pr_info("%s: safe remove dummyblk\n", __func__);
            del_gendisk(pnvme_dev -> dummyblk_entry_p[i].gendisk);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,19,12)
            blk_put_queue(pnvme_dev -> dummyblk_entry_p[i].gendisk->queue);
#else 
            blk_cleanup_queue(pnvme_dev -> dummyblk_entry_p[i].gendisk->queue);
#endif
	        put_disk(pnvme_dev -> dummyblk_entry_p[i].gendisk);

            /* This is test code.*/
            //uint8_t *data = (uint8_t*) page_address(pnvme_dev->dummyblk_entry_p[i].pages[0]);
            //pr_info("receivce data: %x\n", data[0]);
            //pr_info("receivce data: %x\n", data[1]);
            //pr_info("receivce data: %x\n", data[2]);

            for(int j=0; j< pnvme_dev->dummyblk_entry_p[i].npages; ++j) put_page(pnvme_dev->dummyblk_entry_p[i].pages[j]);
            kfree(pnvme_dev->dummyblk_entry_p[i].pages);

            pnvme_dev -> dummyblk_entry_p[i].gendisk = NULL;
        }
    }
    
#endif
    pnvme_dev->is_open = 0;
    return 0;
}

int mmap_dma(struct file *filp, struct vm_area_struct *vma)
{
    int ret = 0;
    struct nvme_dev *pnvme_dev = filp->private_data;

    dma_addr_t dma_addr; 
    void* virt_addr = NULL;  

    struct dma_entry *head = kzalloc(sizeof(struct dma_entry), GFP_KERNEL);

    if(IS_ERR_OR_NULL(head)){
        ret = PTR_ERR(head);
        goto out_kzalloc;
    }

    long size = vma->vm_end - vma->vm_start;

    size = ((size + PAGE_SIZE -1) >> PAGE_SHIFT) << PAGE_SHIFT;

    pr_info("length: %d\n", size);

    pr_info("vma->vm_pgoff: %d\n", vma->vm_pgoff);

    virt_addr = dma_alloc_coherent(&pnvme_dev -> pdev->dev, size, &dma_addr, GFP_KERNEL);

    if(IS_ERR_OR_NULL(virt_addr)){
        ret = PTR_ERR(virt_addr);
        goto out_alloc_data_buffer;
    }

    head -> size = size;
    head -> dma_addr = dma_addr;
    head -> virt_addr = virt_addr;

    pr_info("%p, %lx\n", virt_addr, dma_addr);

    list_add(&head->list, &pnvme_dev-> memlist);

	if (vma->vm_pgoff == 0) {
		ret = dma_mmap_coherent(&pnvme_dev -> pdev->dev, vma, virt_addr, dma_addr, size);
	} else
	{
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	    vm_flags_set(vma, VM_IO);

        ret = remap_pfn_range(vma, vma->vm_start, PFN_DOWN(dma_addr) + vma->vm_pgoff, size, vma->vm_page_prot);
	}

    if (ret < 0) {
		goto out;
    }

    return 0;

out:
    dma_free_coherent (&pnvme_dev -> pdev->dev, size, virt_addr, dma_addr);

out_alloc_data_buffer :
    kfree(head);

out_kzalloc:

    return ret;
}

static irqreturn_t msix_irq(int irq, void *arg)
{
    struct eventfd_ctx *trigger = arg;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,7,12)
	eventfd_signal(trigger);
#else
    eventfd_signal(trigger, 1);
#endif
	return IRQ_HANDLED; 
}

static irqreturn_t intx_irq(int irq, void *arg)
{
	struct nvme_dev *pnvme_dev = arg;

	if (pci_check_and_mask_intx(pnvme_dev->pdev)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,7,12)
		eventfd_signal(pnvme_dev->trigger);
#else
        eventfd_signal(pnvme_dev->trigger, 1);
#endif
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static const struct block_device_operations bops = {
	.owner		= THIS_MODULE,
};

int dummyblk_add(int major, struct nvme_dev * pnvme_dev, int number, struct page **pages, long npages)
{
	struct dummyblk_entry *entry = NULL;
	int ret = 0;
	struct gendisk *gdisk;

	entry = kzalloc(sizeof(struct dummyblk_entry), GFP_KERNEL);
	if (!entry) {
		 return PTR_ERR(entry);
	}

	gdisk = blk_alloc_disk(NUMA_NO_NODE);
	if (IS_ERR(gdisk)) {
		pr_err("Failed to allocate disk\n");
        kfree(entry);
		return PTR_ERR(gdisk);
	}

	entry->gendisk = gdisk;

	gdisk->flags = GENHD_FL_NO_PART;
	gdisk->major = major;
	gdisk->first_minor = number;
	gdisk->minors = 1;

	gdisk->fops = &bops;

	gdisk->private_data = entry;

    snprintf(gdisk->disk_name, sizeof( gdisk->disk_name), "ctrl%dn%d", pnvme_dev->instance, number);

	set_capacity(gdisk, 0);

	ret = add_disk(gdisk);
	//ret = device_add_disk(p->sysdev, gdisk, NULL);
    
    if (ret) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,19,12)
        blk_put_queue(gdisk->queue);
#else 
        blk_cleanup_queue(gdisk->queue);
#endif
	    put_disk(gdisk);
        
        kfree(entry);
        return ret;
	}

#if defined(USE_DUMMYBLK_LIST)

    entry->number = number;
    list_add(&entry->list, &pnvme_dev-> dummyblklist);
#else
    if( pnvme_dev -> dummyblk_entry_p[number].gendisk == NULL){
        pnvme_dev -> dummyblk_entry_p[number].pages = pages;    
        pnvme_dev -> dummyblk_entry_p[number].gendisk = gdisk;
        pnvme_dev -> dummyblk_entry_p[number].npages = npages;    
    }else {   // alreay set 
        del_gendisk(gdisk);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,19,12)
        blk_put_queue(pnvme_dev -> dummyblk_entry_p[number].gendisk->queue);
#else 
        blk_cleanup_queue(pnvme_dev -> dummyblk_entry_p[number].gendisk->queue);
#endif
	    put_disk(gdisk);
        
        kfree(entry);
        return -ENOMEM;
    }
#endif

    pr_info("add device '%s'\n", gdisk->disk_name);
	return 0;
}

void dummyblk_remove(struct dummyblk_entry *dev)
{
	del_gendisk(dev->gendisk);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,19,12)
    blk_put_queue(dev->gendisk->queue);
#else 
    blk_cleanup_queue(dev->gendisk->queue);
#endif

	put_disk(dev->gendisk);

	kfree(dev);

	pr_info("simple block device was removed\n");
}

static long nvme_ioctl(struct file *filp, unsigned int ioctlnum, unsigned long ioctlparam){
    int ret = 0;
    struct nvme_dev *pnvme_dev = filp->private_data;

    struct eventfd_ctx *trigger;
    char *name = NULL;

    struct dma_entry *ptr;
    struct test_params params = {0};
   // int irq_no;

    switch(ioctlnum){
        case IOCTL_SET_SIGNAL:
            if(ret = copy_from_user(&params, (void  __user*)ioctlparam, sizeof(struct test_params))){
                return ret;
            }

            name = kasprintf(GFP_KERNEL, KBUILD_MODNAME "[%d](%s)", params.s.vector, pci_name(pnvme_dev->pdev));

            if(name == NULL){
                return -EFAULT;
            }

            trigger = eventfd_ctx_fdget(params.s.fd);
	        if (IS_ERR(trigger)) {
                printk(KERN_ERR "trigger");
                kfree(name);
                return -EFAULT;
	        }    
            
            if(params.s.msix){
                pr_info("set msix: %s\n",  name);
                ret = request_irq(pci_irq_vector(pnvme_dev->pdev, params.s.vector), 
                                      msix_irq, 
                                      0, 
                                      name, 
                                        trigger
                                      );
            }else{
                pnvme_dev ->trigger = trigger;    
                ret = request_irq(pci_irq_vector(pnvme_dev->pdev, 0),
                                      intx_irq, 
                                      IRQF_SHARED, 
                                      name, 
                                      pnvme_dev
                                      );
            }

            if (ret) {
                dev_notice(&pnvme_dev->pdev->dev, "request irq failed: %d\n", ret);
                kfree(name);
                eventfd_ctx_put(trigger);
                return ret;
            }

            kfree(name);

            struct signal_entry *head = kzalloc(sizeof(struct signal_entry), GFP_KERNEL);
            
            if(IS_ERR_OR_NULL(head)){
                eventfd_ctx_put(trigger);
                return PTR_ERR(head);
            }       
    
            head ->irq_no = (params.s.msix == true) ? pci_irq_vector(pnvme_dev->pdev, params.s.vector) : pci_irq_vector(pnvme_dev->pdev, 0);
            head -> id = (params.s.msix == true) ? trigger : pnvme_dev;
            head -> trigger = trigger;

            list_add(&head->list, &pnvme_dev-> signallist);
            pr_info("ioctl done\n");

            break;
        
        case IOCTL_GET_MEMFINFO:
            ptr = list_entry(pnvme_dev->memlist.next, struct dma_entry, list);

            //pr_info("ioctl : %p\n", ptr ->virt_addr);
            //pr_info("virtual address2: %lx", (uint32_t)ptr ->virt_addr);

            params.m.dma_addr = ptr ->dma_addr;
            params.m.kernel_virtaddr = (uint64_t)(ptr->virt_addr);
            params.m.size = ptr->size;

            //pr_info("virtual address3: %p", params.m.kernel_virtaddr);
            
            if (copy_to_user((void __user *)ioctlparam, &params, sizeof(struct test_params))) {
			    ret =  -EFAULT;
		    }

            break;

        case IOCTL_GET_BDF:
            params.b.domain = pci_domain_nr(pnvme_dev->pdev->bus);
            params.b.bus = pnvme_dev->pdev->bus->number;
            params.b.dev = PCI_SLOT(pnvme_dev->pdev->devfn);
            params.b.func = PCI_FUNC(pnvme_dev->pdev->devfn);    

            if (copy_to_user((void __user *)ioctlparam, &params, sizeof(struct test_params))) {
			    ret =  -EFAULT;
		    }

            break;

        case IOCTL_RELEASE_MEM:
            if(copy_from_user(&params, (void  __user*)ioctlparam, sizeof(struct test_params))){
                return -EFAULT;
            }

            pr_info("recv : %lx\n", params.m.kernel_virtaddr);
            pr_info("recv : %lx\n", params.m.dma_addr);

            struct dma_entry *dma_list_h;
            struct dma_entry *dma_list_e = NULL;
                
            list_for_each_entry_safe(dma_list_h, dma_list_e, &pnvme_dev->memlist, list) {
                if((uint64_t)dma_list_h->virt_addr == params.m.kernel_virtaddr && dma_list_h->dma_addr == params.m.dma_addr){
                    pr_info("safe remove in ioctl\n");
                    list_del(&dma_list_h->list);
		            dma_free_coherent (&pnvme_dev -> pdev->dev, dma_list_h->size, dma_list_h->virt_addr, dma_list_h->dma_addr);
                    kfree(dma_list_h);
                    break;
                }
            }

            break;
        
        case IOCTL_ALLOC_DUMMYBLK:
            if(copy_from_user(&params, (void  __user*)ioctlparam, sizeof(struct test_params))){
                return -EFAULT;
            }

            if( params.d.number >= NUM_OF_DUMMY_FOR_EACH ){
                return -EFAULT;
            }

            // get_user_page
            unsigned long udata = (unsigned long)params.d.buf;
            pr_info("udata: %x\n", udata);

            long npages = 0;
            unsigned long npages_req = ((udata + params.d.len - 1)>>PAGE_SHIFT) - (udata>>PAGE_SHIFT) + 1;

            struct page **pages;

            unsigned fp_offset = (udata & (~PAGE_MASK));

            pr_info("first page offset: %d\n", fp_offset);

            pr_info("request page size: %d\n", npages_req);

            if ((pages =  (struct page**) kvcalloc(npages_req, sizeof(struct pages*), GFP_KERNEL)) == NULL){
                pr_err("could not allocate memory for pages array\n");
                return -ENOMEM;
            }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,8,0)
            mmap_read_lock(current->mm);
#else
            down_read(&current->mm->mmap_sem);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,5,0)
            npages = get_user_pages(udata, npages_req, FOLL_WRITE , pages);
#else 
            npages = get_user_pages(udata, npages_req, FOLL_WRITE , pages, NULL);
#endif

            if (npages <= 0){
                pr_err("unable to pin any pages in memory\n");
                kfree(pages);
                return -ENOMEM;
            }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,8,0)
            mmap_read_unlock(current->mm);
#else
            up_read(&current->mm->mmap_sem);
#endif

            dummyblk_add(blk_major, pnvme_dev, params.d.number, pages, npages);

            break;

        case IOCTL_RECORD_STATS:

#if defined(USE_DUMMYBLK_LIST)
            
#else 
            part_stat_lock();
            //part_stat_inc(&pnvme_dev->dummyblklist[0], ios[0]);



            part_stat_unlock();

#endif

            break;

        case IOCTL_SETUP_JIFFIES:
            if(set_jiffies) return -EEXIST;

            if(copy_from_user(&params, (void  __user*)ioctlparam, sizeof(struct test_params))){
                return -EFAULT;
            }
#if 1
            unsigned long jiffies_data = (unsigned long)params.j.buf;
            
            long jiffies_npages = 0;
            unsigned long jiffies_npages_req = 1;

            if ((jiffies_pages =  (struct page**) kvcalloc(jiffies_npages_req, sizeof(struct pages*), GFP_KERNEL)) == NULL){
                pr_err("could not allocate memory for pages array\n");
                return -ENOMEM;
            }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,8,0)
            mmap_read_lock(current->mm);
#else
            down_read(&current->mm->mmap_sem);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,5,0)
            jiffies_npages = get_user_pages(jiffies_data, jiffies_npages_req, FOLL_WRITE , jiffies_pages);
#else 
            jiffies_npages = get_user_pages(jiffies_data, jiffies_npages_req, FOLL_WRITE , jiffies_pages, NULL);
#endif

            if (jiffies_npages <= 0){
                pr_err("unable to pin any pages in memory\n");
                kfree(jiffies_pages);
                return -ENOMEM;
            }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,8,0)
            mmap_read_unlock(current->mm);
#else
            up_read(&current->mm->mmap_sem);
#endif

#endif
            set_jiffies = true;
            jiffies_creater = pnvme_dev ->instance;

            break;

        case IOCTL_GET_JIFFIES:
            *(unsigned long*)page_address(jiffies_pages[0]) = jiffies;
            pr_info("jiffies virt addr: %lx\n",&jiffies);
            pr_info("jiffies phys addr: %lx\n",virt_to_phys(&jiffies));
            break;

        default:
            break;

        case IOCTL_GET_PHYSADDR_JIFFIES:

            params.j.phys_addr = virt_to_phys(&jiffies_64);

            if (copy_to_user((void __user *)ioctlparam, &params, sizeof(struct test_params))) {
			    ret =  -EFAULT;
		    }

            break;

    }

    return ret;
}

ssize_t proc_read(struct file *file, char __user *ubuf, size_t count, loff_t *pos)
{
    char buf[32] = {0};

    static int completed = 0;
    if (completed) {
            completed = 0;
            return 0;
    }
    completed = 1;

    int len = sprintf(buf, "%lu\n", jiffies);
    copy_to_user(ubuf, buf, len);
	return len;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
static const struct proc_ops proc_ops = {
    .proc_read	 = proc_read,
};
#else 
static const struct file_operations proc_ops = {
    .read = proc_read,
};
#endif

static const struct file_operations nvme_chr_fops = {
    .owner          = THIS_MODULE,
    .open           = nvmet_pci_open,
    .release        = nvmet_pci_close,
    .unlocked_ioctl = nvme_ioctl,
    .mmap           = mmap_dma,
};

static int nvmet_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id){

    int ret = 0;
    struct nvme_dev *pnvme_dev;
    int n;
   
    pnvme_dev = kmalloc(sizeof(struct nvme_dev), GFP_KERNEL);
    if(!pnvme_dev){
        ret = -ENOMEM;
        goto out_alloc_pnvme_dev;
    }

    ret = ida_simple_get(&nvme_instance_ida, 0, 0, GFP_KERNEL);
    if (ret < 0){
        ret = -ENODEV;
        goto out_ida_simple_get;
    }
		
    pnvme_dev -> instance = ret;

    pnvme_dev -> devt = MKDEV(MAJOR(nvme_chr_devt), pnvme_dev -> instance);

    cdev_init(&pnvme_dev->cdev, &nvme_chr_fops);
    pnvme_dev->cdev.owner = THIS_MODULE;

    if ((ret = cdev_add(&pnvme_dev->cdev, pnvme_dev -> devt , NVME_MINORS)) != 0) {
        goto out_cdev_add;
    }

    pnvme_dev -> sysdev = device_create(nvme_class, NULL, MKDEV(MAJOR(nvme_chr_devt), pnvme_dev -> instance), NULL, "nvmet%d", pnvme_dev -> instance);
    
    if (IS_ERR( pnvme_dev -> sysdev)) {
		pr_err("couldn't create dev\n");
		ret = PTR_ERR( pnvme_dev -> sysdev);
		goto out_device_create;
	}

    if (ret = pci_enable_device_mem(pdev)){
        goto out_pci_enable_device;
    }

    pci_set_master(pdev);

    if (ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64)) /*&& dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32))*/){
        pr_err("Error dma_set_mask_and_coherent\n");
        goto out_dma_set_mask_and_coherent;
    }

    if(ret= pci_request_regions(pdev, DEVICE_NAME)) {
        goto out_pci_request_regions;
    }

    pnvme_dev->pdev = pci_dev_get(pdev);
	pci_set_drvdata(pdev, pnvme_dev);

    n = pci_alloc_irq_vectors(pdev, 1, vectors, PCI_IRQ_ALL_TYPES | PCI_IRQ_AFFINITY);

    pr_info("vector num: %d\n", n);

    if (n < 0) {
        ret = -ENOMEM;
        goto out_pci_alloc_irq_vectors;
	}

    INIT_LIST_HEAD(&pnvme_dev->memlist);   
    INIT_LIST_HEAD(&pnvme_dev->signallist);   

#if defined(USE_DUMMYBLK_LIST)

    INIT_LIST_HEAD(&pnvme_dev->dummyblklist);
#else
    pnvme_dev-> dummyblk_entry_p = kzalloc(sizeof(struct dummyblk_entry) * NUM_OF_DUMMY_FOR_EACH, GFP_KERNEL);
            
    if(IS_ERR_OR_NULL(pnvme_dev->dummyblk_entry_p)){
        ret = PTR_ERR(pnvme_dev->dummyblk_entry_p);
        goto out_kzalloc;
    }       

    for(int i=0; i< NUM_OF_DUMMY_FOR_EACH; ++i){
        pnvme_dev -> dummyblk_entry_p[i].gendisk = NULL;
    }
#endif
    return ret;


out_kzalloc:
    pci_free_irq_vectors(pdev);


out_pci_alloc_irq_vectors:
    pci_set_drvdata(pdev, NULL);
    pci_release_regions(pdev);  

out_pci_request_regions:

out_dma_set_mask_and_coherent:
    pci_disable_device(pdev);

out_pci_enable_device:
    device_destroy(nvme_class, pnvme_dev -> devt );

out_device_create:
    cdev_del(&pnvme_dev->cdev); 

out_cdev_add:
    ida_simple_remove(&nvme_instance_ida, pnvme_dev -> instance );

out_ida_simple_get:
    kfree(pnvme_dev);

out_alloc_pnvme_dev:

    return ret;
}

static void nvmet_pci_remove(struct pci_dev* pdev){
    pr_info("%s\n", __func__);

    struct nvme_dev *pnvme_dev = pci_get_drvdata(pdev);
    pnvme_dev->is_open = 0;

#if defined(USE_DUMMYBLK_LIST)

    struct dummyblk_entry *dummyblk_list_h;
    struct dummyblk_entry *dummyblk_list_e = NULL;

    list_for_each_entry_safe(dummyblk_list_h, dummyblk_list_e, &pnvme_dev->dummyblklist, list) {
        pr_info("%s: dummyblk\n", __func__);
        list_del(&dummyblk_list_h->list);
        
        del_gendisk(dummyblk_list_h->gendisk);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,19,12)
        blk_put_queue(dummyblk_list_h->gendisk->queue);
#else 
        blk_cleanup_queue(dummyblk_list_h->gendisk->queue);
#endif
	    put_disk(dummyblk_list_h->gendisk);
        
        kfree(dummyblk_list_h);
    }
#else
    for(int i=0; i< NUM_OF_DUMMY_FOR_EACH; ++i){
        if(pnvme_dev -> dummyblk_entry_p[i].gendisk != NULL){
            //pr_info("%s: safe remove dummyblk\n", __func__);
            del_gendisk(pnvme_dev -> dummyblk_entry_p[i].gendisk);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,19,12)
            blk_put_queue(pnvme_dev -> dummyblk_entry_p[i].gendisk->queue);
#else 
            blk_cleanup_queue(pnvme_dev -> dummyblk_entry_p[i].gendisk->queue);
#endif
	        put_disk(pnvme_dev -> dummyblk_entry_p[i].gendisk);
        
            for(int j=0; j< pnvme_dev->dummyblk_entry_p[i].npages; ++j) put_page(pnvme_dev->dummyblk_entry_p[i].pages[j]);
            kfree(pnvme_dev->dummyblk_entry_p[i].pages);
            pnvme_dev -> dummyblk_entry_p[i].gendisk = NULL;
        }
    }

    kfree( pnvme_dev->dummyblk_entry_p);

#endif

    struct dma_entry *dma_list_h;
    struct dma_entry *dma_list_e = NULL;

    list_for_each_entry_safe(dma_list_h, dma_list_e, &pnvme_dev->memlist, list) {
        pr_info("%s: remove dma memory\n", __func__);

        list_del(&dma_list_h->list);
		dma_free_coherent (&pnvme_dev -> pdev->dev, dma_list_h->size, dma_list_h->virt_addr, dma_list_h->dma_addr);
        kfree(dma_list_h);
        
    }

    struct signal_entry *single_list_h;
    struct signal_entry *single_list_e = NULL;

    list_for_each_entry_safe(single_list_h, single_list_e, &pnvme_dev->signallist, list) {
        list_del(&single_list_h->list);
        free_irq(single_list_h->irq_no , single_list_h->id);
        eventfd_ctx_put(single_list_h -> trigger);
        kfree(single_list_h);        
    }

    pci_free_irq_vectors(pnvme_dev->pdev);
    pci_set_drvdata(pdev, NULL);
	pci_release_regions(pdev);
    pci_disable_device(pdev);

    device_destroy(nvme_class, pnvme_dev->devt );
    cdev_del(&pnvme_dev->cdev); 
    ida_simple_remove(&nvme_instance_ida, pnvme_dev->instance );
    kfree(pnvme_dev);

}

static void nvme_shutdown(struct pci_dev *pdev)
{

}

static struct pci_driver simple_nvme_driver = {
    .name = DEVICE_NAME,
    .id_table = test_nvme_ids,
    .probe = nvmet_pci_probe,
    .remove = nvmet_pci_remove,
    .shutdown = nvme_shutdown,
};

static int nvmet_init(void) {
    int ret;
    pr_info("%s\n", __func__);

    ida_init(&nvme_instance_ida);

	ret = alloc_chrdev_region(&nvme_chr_devt, 0, NVME_MINORS, DEVICE_NAME);
	if (ret != 0){
        printk(KERN_ERR "%d: %s()   %d",__LINE__, __FUNCTION__, ret ); 
        ida_destroy(&nvme_instance_ida);
        return ret;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,4,0)
    nvme_class = class_create(DEVICE_NAME);
#else
	nvme_class = class_create(THIS_MODULE, DEVICE_NAME);
#endif

	if (IS_ERR(nvme_class)) {
		ret = PTR_ERR(nvme_class);
        unregister_chrdev_region(nvme_chr_devt, NVME_MINORS);
        ida_destroy(&nvme_instance_ida);
		return ret;
	}

    blk_major = register_blkdev(0, "dummyblk");

    if (blk_major < 0) {
        class_destroy(nvme_class);
        unregister_chrdev_region(nvme_chr_devt, NVME_MINORS);
        ida_destroy(&nvme_instance_ida);

		return blk_major;
	}

    ret = pci_register_driver(&simple_nvme_driver); 
    if(ret != 0){
        pr_err("%d: %s()   %d",__LINE__, __FUNCTION__, ret );     
        unregister_blkdev(blk_major, "dummyblk");

        class_destroy(nvme_class);
        unregister_chrdev_region(nvme_chr_devt, NVME_MINORS);
        ida_destroy(&nvme_instance_ida);

        return ret;
    }

    //struct proc_dir_entry *entry = proc_create(PROC_NAME, S_IRUGO, NULL, &proc_ops);
	//if (!entry ) {
	//	pr_err("proc_create\n");

    //  pci_unregister_driver(&simple_nvme_driver);
    //  unregister_blkdev(blk_major, "dummyblk");
    //  class_destroy(nvme_class);
    //  unregister_chrdev_region(nvme_chr_devt, NVME_MINORS);
    //  ida_destroy(&nvme_instance_ida);
	//	return -ENOENT;
	//}

    return ret; 
}

static void nvmet_exit(void) {

    pr_info("%s\n", __func__);

    //remove_proc_entry(PROC_NAME, NULL);
    pci_unregister_driver(&simple_nvme_driver);  //ドライバー名、table, コールバック関数(probe, remove)を削除  

    unregister_blkdev(blk_major, "dummyblk");

    class_destroy(nvme_class);
    unregister_chrdev_region(nvme_chr_devt, NVME_MINORS);
    ida_destroy(&nvme_instance_ida);
}

module_init(nvmet_init);
module_exit(nvmet_exit);