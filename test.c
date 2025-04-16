
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
MODULE_DESCRIPTION("Generic PCIe Device Driver");


#define DEVICE_NAME "genpci"
#define genpci_MINORS		(1U << MINORBITS)

#define PROC_NAME "jiffies"

#undef USE_DUMMYBLK_LIST  /* This define is not complete, and already not mentenance. */
#define NUM_OF_DUMMY_FOR_EACH    (128)

static unsigned int vectors = 32;
module_param(vectors, uint, 0644);
MODULE_PARM_DESC(vectors,
		"interrupt vector num.");

static DEFINE_IDA(genpci_instance_ida);

static struct class* genpci_class;
static dev_t genpci_chr_devt;   

static int blk_major;

struct dma_entry {
    struct list_head list;
    void *virt_addr;
    dma_addr_t dma_addr;
    long size;
};

struct signal_entry{
    char* irqname;
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
    //struct page ** pages;
    //long npages;

    struct gendisk *gendisk;
};

struct genpci_dev{
    struct pci_dev *pdev;
    struct cdev cdev;
    dev_t  devt;
    int    instance;
    bool   is_open;

    struct device *sysdev;

    struct list_head memlist;
    struct list_head signallist;

    struct eventfd_ctx   *trigger;   /* use when INT-X */
    int    num_irq_vector;

#if defined(USE_DUMMYBLK_LIST)
    struct list_head dummyblklist;
#else    
    struct dummyblk_entry *dummyblk_entry_p;
#endif

    //bool set_jiffies;
    struct page **jiffies_pages;

    //bool set_statsbuffer;
    struct page **stats_pages;
};

static const struct pci_device_id genpci_ids[] =
{
   // { PCI_DEVICE(PCI_VENDOR_ID_TEST, PCI_DEVICE_ID_TEST) },
   { PCI_DEVICE_CLASS(PCI_CLASS_STORAGE_EXPRESS, 0xffffff) },
    { 0, },
};

MODULE_DEVICE_TABLE(pci, genpci_ids);


static int genpci_open(struct inode* inode, struct file* filp){
    struct genpci_dev *pgenpci_dev =container_of(inode->i_cdev, struct genpci_dev, cdev);
    filp->private_data = pgenpci_dev;
    pgenpci_dev->is_open = 1;
    return 0;
}

/*
    release resource allocated after probe.
*/
static int genpci_close(struct inode* inode, struct file* filp){
    pr_info("%s\n", __func__);

    struct genpci_dev *pgenpci_dev = filp->private_data;

    if(pgenpci_dev->stats_pages != NULL){
        put_page(pgenpci_dev -> stats_pages[0]);
        kfree(pgenpci_dev-> stats_pages);
        pgenpci_dev-> stats_pages = NULL;
    }

    struct dma_entry *dma_list_h;
    struct dma_entry *dma_list_e = NULL;

    if(pgenpci_dev -> jiffies_pages != NULL){
        put_page(pgenpci_dev -> jiffies_pages[0]);
        kfree(pgenpci_dev -> jiffies_pages);
        pgenpci_dev -> jiffies_pages = NULL;
    }
    
    list_for_each_entry_safe(dma_list_h, dma_list_e, &pgenpci_dev->memlist, list) {
        pr_info("safe remove dma in close\n");

        list_del(&dma_list_h->list);
		dma_free_coherent (&pgenpci_dev -> pdev->dev, dma_list_h->size, dma_list_h->virt_addr, dma_list_h->dma_addr);
        kfree(dma_list_h);
    }

    struct signal_entry *signal_list_h;
    struct signal_entry *signal_list_e = NULL;

    list_for_each_entry_safe(signal_list_h, signal_list_e, &pgenpci_dev->signallist, list) {
        pr_info("safe remove signal in close\n");
        list_del(&signal_list_h->list);
        free_irq(signal_list_h->irq_no, signal_list_h -> id);
        eventfd_ctx_put(signal_list_h -> trigger);
        kfree(signal_list_h->irqname);
                
        kfree(signal_list_h);
    }

    pci_free_irq_vectors(pgenpci_dev->pdev);

#if defined(USE_DUMMYBLK_LIST)
    struct dummyblk_entry *dummyblk_list_h;
    struct dummyblk_entry *dummyblk_list_e = NULL;

    list_for_each_entry_safe(dummyblk_list_h, dummyblk_list_e, &pgenpci_dev->dummyblklist, list) {
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
        if(pgenpci_dev -> dummyblk_entry_p[i].gendisk != NULL){
            //pr_info("%s: safe remove dummyblk\n", __func__);
            del_gendisk(pgenpci_dev -> dummyblk_entry_p[i].gendisk);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,19,12)
            blk_put_queue(pgenpci_dev -> dummyblk_entry_p[i].gendisk->queue);
#else 
            blk_cleanup_queue(pgenpci_dev -> dummyblk_entry_p[i].gendisk->queue);
#endif
	        put_disk(pgenpci_dev -> dummyblk_entry_p[i].gendisk);

            /* This is test code.*/
            //uint8_t *data = (uint8_t*) page_address(pgenpci_dev->dummyblk_entry_p[i].pages[0]);
            //pr_info("receivce data: %x\n", data[0]);
            //pr_info("receivce data: %x\n", data[1]);
            //pr_info("receivce data: %x\n", data[2]);

            //for(int j=0; j< pgenpci_dev->dummyblk_entry_p[i].npages; ++j) put_page(pgenpci_dev->dummyblk_entry_p[i].pages[j]);
            //kfree(pgenpci_dev->dummyblk_entry_p[i].pages);

            pgenpci_dev -> dummyblk_entry_p[i].gendisk = NULL;
        }
    }
    
#endif
    pgenpci_dev->is_open = 0;
    return 0;
}

int mmap_dma(struct file *filp, struct vm_area_struct *vma)
{
    int ret = 0;
    struct genpci_dev *pgenpci_dev = filp->private_data;

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

    virt_addr = dma_alloc_coherent(&pgenpci_dev -> pdev->dev, size, &dma_addr, GFP_KERNEL);

    if(IS_ERR_OR_NULL(virt_addr)){
        ret = PTR_ERR(virt_addr);
        goto out_alloc_data_buffer;
    }

    head -> size = size;
    head -> dma_addr = dma_addr;
    head -> virt_addr = virt_addr;

    pr_info("%p, %lx\n", virt_addr, dma_addr);

    list_add(&head->list, &pgenpci_dev-> memlist);

	if (vma->vm_pgoff == 0) {
		ret = dma_mmap_coherent(&pgenpci_dev -> pdev->dev, vma, virt_addr, dma_addr, size);
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
    dma_free_coherent (&pgenpci_dev -> pdev->dev, size, virt_addr, dma_addr);

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
	struct genpci_dev *pgenpci_dev = arg;

	//if (pci_check_and_mask_intx(pgenpci_dev->pdev)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,7,12)
		eventfd_signal(pgenpci_dev->trigger);
#else
        eventfd_signal(pgenpci_dev->trigger, 1);
#endif
		return IRQ_HANDLED;
	//}

	//return IRQ_NONE;
}

static const struct block_device_operations bops = {
	.owner		= THIS_MODULE,
};

int dummyblk_add(int major, struct genpci_dev * pgenpci_dev, int number/*, struct page **pages, long npages*/)
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

    snprintf(gdisk->disk_name, sizeof( gdisk->disk_name), "dmmyblk%dn%d", pgenpci_dev->instance, number);

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
    list_add(&entry->list, &pgenpci_dev-> dummyblklist);
#else
    if( pgenpci_dev -> dummyblk_entry_p[number].gendisk == NULL){
        //pgenpci_dev -> dummyblk_entry_p[number].pages = pages;    
        pgenpci_dev -> dummyblk_entry_p[number].gendisk = gdisk;
        //pgenpci_dev -> dummyblk_entry_p[number].npages = npages;    
    }else {    
        pr_err("dummyblk is already set.\n");
        del_gendisk(gdisk);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,19,12)
        blk_put_queue(pgenpci_dev -> dummyblk_entry_p[number].gendisk->queue);
#else 
        blk_cleanup_queue(pgenpci_dev -> dummyblk_entry_p[number].gendisk->queue);
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

static long genpci_ioctl(struct file *filp, unsigned int ioctlnum, unsigned long ioctlparam){
    int ret = 0;
    struct genpci_dev *pgenpci_dev = filp->private_data;

    struct eventfd_ctx *trigger;
    struct dma_entry *ptr;
    struct test_params params = {0};

    switch(ioctlnum){
        case IOCTL_SET_SIGNAL:
            if(ret = copy_from_user(&params, (void  __user*)ioctlparam, sizeof(struct test_params))){
                return ret;
            }

            struct signal_entry *signal_list_h;
            struct signal_entry *signal_list_e = NULL;

            list_for_each_entry_safe(signal_list_h, signal_list_e, &pgenpci_dev->signallist, list) {
                list_del(&signal_list_h->list);
                free_irq(signal_list_h->irq_no, signal_list_h -> id);
                eventfd_ctx_put(signal_list_h -> trigger);
                kfree(signal_list_h->irqname);
                kfree(signal_list_h);

            }

            pci_free_irq_vectors(pgenpci_dev->pdev);

            if(params.s.irq_mode == MSIX){
               
                pgenpci_dev-> num_irq_vector = pci_alloc_irq_vectors( pgenpci_dev-> pdev, 1, vectors, PCI_IRQ_MSIX | PCI_IRQ_AFFINITY);
                pr_info("msix vector num: %d\n",  pgenpci_dev-> num_irq_vector );
                if ( pgenpci_dev-> num_irq_vector  < 0) {
                    pgenpci_dev-> num_irq_vector = 0;
                    return -ENOMEM;
	            }
               
                int num = (params.s.vectornum > pgenpci_dev-> num_irq_vector) ? pgenpci_dev-> num_irq_vector : params.s.vectornum;

                for(int i=0; i< num; ++i){

                    struct signal_entry *head = kzalloc(sizeof(struct signal_entry), GFP_KERNEL);

                    if(IS_ERR_OR_NULL(head)){
                        return PTR_ERR(head);
                    }       

                    //head->irqname = kasprintf(GFP_KERNEL, KBUILD_MODNAME "[%d](%s)", i, pci_name(pgenpci_dev->pdev));
                    head->irqname = kasprintf(GFP_KERNEL, "genpci%d.%d", pgenpci_dev ->instance, i);
                    pr_info("%s\n",  head->irqname);
                    if(!head->irqname){
                        kfree(head);
                        return -EFAULT;
                    }

                    trigger = eventfd_ctx_fdget(params.s.fd[i]);
	                if (IS_ERR(trigger)){
                        pr_err("Failed to create trigger\n");
                        kfree(head->irqname);
                        kfree(head);
                        return -EFAULT;
	                }    

                    ret = request_irq(pci_irq_vector(pgenpci_dev->pdev, i), 
                                        msix_irq, 
                                        0, 
                                        head->irqname, 
                                        trigger
                                      );

                    if (ret) {
                        dev_notice(&pgenpci_dev->pdev->dev, "request irq failed: %d\n", ret);
                        eventfd_ctx_put(trigger);
                        kfree(head->irqname);
                        kfree(head);
                        return ret;
                    }
                  
                    head -> irq_no =  pci_irq_vector(pgenpci_dev->pdev, i);
                    head -> id = trigger;
                    head -> trigger = trigger;

                    list_add(&head->list, &pgenpci_dev-> signallist);

                }

            }else if(params.s.irq_mode == INTX){

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,8,0) 
                pgenpci_dev-> num_irq_vector = pci_alloc_irq_vectors( pgenpci_dev-> pdev, 1, vectors, PCI_IRQ_INTX | PCI_IRQ_AFFINITY);
#else
                pgenpci_dev-> num_irq_vector = pci_alloc_irq_vectors( pgenpci_dev-> pdev, 1, vectors, PCI_IRQ_LEGACY | PCI_IRQ_AFFINITY);
#endif

                pr_info("intx vector num: %d\n",  pgenpci_dev-> num_irq_vector );
                if ( pgenpci_dev-> num_irq_vector < 0) {
                    pgenpci_dev-> num_irq_vector = 0;
                    return -ENOMEM;
                }
                
                int num = (params.s.vectornum > pgenpci_dev-> num_irq_vector) ? pgenpci_dev-> num_irq_vector : params.s.vectornum;

                for(int i=0; i< num; ++i){
                    struct signal_entry *head = kzalloc(sizeof(struct signal_entry), GFP_KERNEL);

                    if(IS_ERR_OR_NULL(head)){
                        return PTR_ERR(head);
                    }       

                    //head->irqname = kasprintf(GFP_KERNEL, KBUILD_MODNAME "[%d](%s)", i, pci_name(pgenpci_dev->pdev));
                    head->irqname = kasprintf(GFP_KERNEL, "genpci%d.%d", pgenpci_dev ->instance  , i);
                    pr_info("%s\n",  head->irqname);
                    if(!head->irqname){
                        kfree(head);
                        return -EFAULT;
                    }

                    trigger = eventfd_ctx_fdget(params.s.fd[i]);
	                if (IS_ERR(trigger)){
                        pr_err("Failed to create trigger\n");
                        kfree(head->irqname);
                        kfree(head);
                        return -EFAULT;
	                }    

                    pgenpci_dev ->trigger = trigger;    
                    ret = request_irq(pci_irq_vector(pgenpci_dev->pdev, i),
                                        intx_irq, 
                                        IRQF_SHARED, 
                                        head->irqname, 
                                        pgenpci_dev
                                        );

                    if (ret) {
                        dev_notice(&pgenpci_dev->pdev->dev, "request irq failed: %d\n", ret);
                        eventfd_ctx_put(trigger);
                        kfree(head->irqname);
                        kfree(head);
                        return ret;
                    }

                    head -> irq_no = pci_irq_vector(pgenpci_dev->pdev, i);
                    head -> id = pgenpci_dev;
                    head -> trigger = trigger;

                    list_add(&head->list, &pgenpci_dev-> signallist);
                }
            }

            params.s.vectornum = pgenpci_dev-> num_irq_vector;

            if (copy_to_user((void __user *)ioctlparam, &params, sizeof(struct test_params))) {
			    ret =  -EFAULT;
		    }

            pr_info("ioctl done\n");

            break;
        
        case IOCTL_GET_MEMFINFO:
            ptr = list_entry(pgenpci_dev->memlist.next, struct dma_entry, list);

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
            params.b.domain = pci_domain_nr(pgenpci_dev->pdev->bus);
            params.b.bus = pgenpci_dev->pdev->bus->number;
            params.b.dev = PCI_SLOT(pgenpci_dev->pdev->devfn);
            params.b.func = PCI_FUNC(pgenpci_dev->pdev->devfn);    

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
                
            list_for_each_entry_safe(dma_list_h, dma_list_e, &pgenpci_dev->memlist, list) {
                if((uint64_t)dma_list_h->virt_addr == params.m.kernel_virtaddr && dma_list_h->dma_addr == params.m.dma_addr){
                    pr_info("safe remove in ioctl\n");
                    list_del(&dma_list_h->list);
		            dma_free_coherent (&pgenpci_dev -> pdev->dev, dma_list_h->size, dma_list_h->virt_addr, dma_list_h->dma_addr);
                    kfree(dma_list_h);
                    break;
                }
            }

            break;

        case IOCTL_SEUTP_STATS_BUFFER:
            if(pgenpci_dev -> stats_pages != NULL) return -EEXIST;

            if(copy_from_user(&params, (void  __user*)ioctlparam, sizeof(struct test_params))){
                return -EFAULT;
            }

            // get_user_page
            unsigned long udata = (unsigned long)params.a.buf;
            pr_info("udata: %x\n", udata);

            long npages = 0;
            unsigned long npages_req = 1;

            struct page **pages;

            unsigned fp_offset = (udata & (~PAGE_MASK));

            pr_info("first page offset: %d\n", fp_offset);

            pr_info("request page size: %d\n", npages_req);

            if ((pgenpci_dev -> stats_pages  =  (struct page**) kvcalloc(npages_req, sizeof(struct pages*), GFP_KERNEL)) == NULL){
                pr_err("could not allocate memory for pages array\n");
                return -ENOMEM;
            }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,8,0)
            mmap_read_lock(current->mm);
#else
            down_read(&current->mm->mmap_sem);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,5,0)
            npages = get_user_pages(udata, npages_req, FOLL_WRITE , pgenpci_dev -> stats_pages );
#else 
            npages = get_user_pages(udata, npages_req, FOLL_WRITE , pgenpci_dev -> stats_pages , NULL);
#endif

            if (npages <= 0){
                pr_err("unable to pin any pages in memory\n");
                kfree(pgenpci_dev -> stats_pages );
                return -ENOMEM;
            }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,8,0)
            mmap_read_unlock(current->mm);
#else
            up_read(&current->mm->mmap_sem);
#endif

            break;

        case IOCTL_ALLOC_DUMMYBLK:
            if(copy_from_user(&params, (void  __user*)ioctlparam, sizeof(struct test_params))){
                return -EFAULT;
            }

            if( params.d.number >= NUM_OF_DUMMY_FOR_EACH ){
                return -EFAULT;
            }


            dummyblk_add(blk_major, pgenpci_dev, params.d.number/*, pages, npages*/);

            break;

        case IOCTL_RECORD_STATS:


#if defined(USE_DUMMYBLK_LIST)
            // not impemented
#else 
            part_stat_lock();
            part_stat_inc(pgenpci_dev->dummyblk_entry_p[0].gendisk->part0, ios[0]);

		    //part_stat_add(req->part, sectors[sgrp], bytes >> 9);


            part_stat_unlock();

#endif

            break;

        case IOCTL_SETUP_JIFFIES:
            if(pgenpci_dev -> jiffies_pages != NULL) return -EEXIST;

            if(copy_from_user(&params, (void  __user*)ioctlparam, sizeof(struct test_params))){
                return -EFAULT;
            }

            unsigned long jiffies_data = (unsigned long)params.j.buf;
            
            long jiffies_npages = 0;
            unsigned long jiffies_npages_req = 1;

            if ((pgenpci_dev -> jiffies_pages =  (struct page**) kvcalloc(jiffies_npages_req, sizeof(struct pages*), GFP_KERNEL)) == NULL){
                pr_err("could not allocate memory for pages array\n");
                return -ENOMEM;
            }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,8,0)
            mmap_read_lock(current->mm);
#else
            down_read(&current->mm->mmap_sem);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,5,0)
            jiffies_npages = get_user_pages(jiffies_data, jiffies_npages_req, FOLL_WRITE , pgenpci_dev -> jiffies_pages);
#else 
            jiffies_npages = get_user_pages(jiffies_data, jiffies_npages_req, FOLL_WRITE , pgenpci_dev -> jiffies_pages, NULL);
#endif

            if (jiffies_npages <= 0){
                pr_err("unable to pin any pages in memory\n");
                kfree(pgenpci_dev -> jiffies_pages);
                return -ENOMEM;
            }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,8,0)
            mmap_read_unlock(current->mm);
#else
            up_read(&current->mm->mmap_sem);
#endif
            
            break;

        case IOCTL_GET_JIFFIES:
            *(unsigned long*)page_address(pgenpci_dev -> jiffies_pages[0]) = jiffies_64;

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

static const struct file_operations genpci_chr_fops = {
    .owner          = THIS_MODULE,
    .open           = genpci_open,
    .release        = genpci_close,
    .unlocked_ioctl = genpci_ioctl,
    .mmap           = mmap_dma,
};

static int genpci_probe(struct pci_dev *pdev, const struct pci_device_id *id){

    int ret = 0;
    struct genpci_dev *pgenpci_dev;
    int n;
   
    pgenpci_dev = kmalloc(sizeof(struct genpci_dev), GFP_KERNEL);
    if(!pgenpci_dev){
        ret = -ENOMEM;
        goto out_alloc_pgenpci_dev;
    }

    ret = ida_simple_get(&genpci_instance_ida, 0, 0, GFP_KERNEL);
    if (ret < 0){
        ret = -ENODEV;
        goto out_ida_simple_get;
    }
		
    pgenpci_dev -> instance = ret;

    pgenpci_dev -> devt = MKDEV(MAJOR(genpci_chr_devt), pgenpci_dev -> instance);

    cdev_init(&pgenpci_dev->cdev, &genpci_chr_fops);
    pgenpci_dev->cdev.owner = THIS_MODULE;

    if ((ret = cdev_add(&pgenpci_dev->cdev, pgenpci_dev -> devt , genpci_MINORS)) != 0) {
        goto out_cdev_add;
    }

    pgenpci_dev -> sysdev = device_create(genpci_class, NULL, MKDEV(MAJOR(genpci_chr_devt), pgenpci_dev -> instance), NULL, "genpci%d", pgenpci_dev -> instance);
    
    if (IS_ERR( pgenpci_dev -> sysdev)) {
		pr_err("couldn't create dev\n");
		ret = PTR_ERR( pgenpci_dev -> sysdev);
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

    pgenpci_dev->pdev = pci_dev_get(pdev);
	pci_set_drvdata(pdev, pgenpci_dev);

    pgenpci_dev->num_irq_vector = 0;

    INIT_LIST_HEAD(&pgenpci_dev->memlist);   
    INIT_LIST_HEAD(&pgenpci_dev->signallist);   

    pgenpci_dev->is_open = 0;
    pgenpci_dev -> jiffies_pages = NULL;
    pgenpci_dev -> stats_pages = NULL;


#if defined(USE_DUMMYBLK_LIST)

    INIT_LIST_HEAD(&pgenpci_dev->dummyblklist);
#else
    pgenpci_dev-> dummyblk_entry_p = kzalloc(sizeof(struct dummyblk_entry) * NUM_OF_DUMMY_FOR_EACH, GFP_KERNEL);
            
    if(IS_ERR_OR_NULL(pgenpci_dev->dummyblk_entry_p)){
        ret = PTR_ERR(pgenpci_dev->dummyblk_entry_p);
        goto out_kzalloc;
    }       

    for(int i=0; i< NUM_OF_DUMMY_FOR_EACH; ++i){
        pgenpci_dev -> dummyblk_entry_p[i].gendisk = NULL;
    }
#endif
    return ret;


out_kzalloc:

    pci_set_drvdata(pdev, NULL);
    pci_release_regions(pdev);  

out_pci_request_regions:

out_dma_set_mask_and_coherent:
    pci_disable_device(pdev);

out_pci_enable_device:
    device_destroy(genpci_class, pgenpci_dev -> devt );

out_device_create:
    cdev_del(&pgenpci_dev->cdev); 

out_cdev_add:
    ida_simple_remove(&genpci_instance_ida, pgenpci_dev -> instance );

out_ida_simple_get:
    kfree(pgenpci_dev);

out_alloc_pgenpci_dev:

    return ret;
}

static void genpci_remove(struct pci_dev* pdev){
    pr_info("%s\n", __func__);

    struct genpci_dev *pgenpci_dev = pci_get_drvdata(pdev);
    pgenpci_dev->is_open = 0;

    if(pgenpci_dev->stats_pages != NULL){
        put_page(pgenpci_dev -> stats_pages[0]);
        kfree(pgenpci_dev-> stats_pages);
        pgenpci_dev->stats_pages = NULL;
    }

    if(pgenpci_dev -> jiffies_pages != NULL){
        put_page(pgenpci_dev -> jiffies_pages[0]);
        kfree(pgenpci_dev -> jiffies_pages);
        pgenpci_dev -> jiffies_pages = NULL;
    }

#if defined(USE_DUMMYBLK_LIST)

    struct dummyblk_entry *dummyblk_list_h;
    struct dummyblk_entry *dummyblk_list_e = NULL;

    list_for_each_entry_safe(dummyblk_list_h, dummyblk_list_e, &pgenpci_dev->dummyblklist, list) {
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
        if(pgenpci_dev -> dummyblk_entry_p[i].gendisk != NULL){
            //pr_info("%s: safe remove dummyblk\n", __func__);
            del_gendisk(pgenpci_dev -> dummyblk_entry_p[i].gendisk);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,19,12)
            blk_put_queue(pgenpci_dev -> dummyblk_entry_p[i].gendisk->queue);
#else 
            blk_cleanup_queue(pgenpci_dev -> dummyblk_entry_p[i].gendisk->queue);
#endif
	        put_disk(pgenpci_dev -> dummyblk_entry_p[i].gendisk);
        
            //for(int j=0; j< pgenpci_dev->dummyblk_entry_p[i].npages; ++j) put_page(pgenpci_dev->dummyblk_entry_p[i].pages[j]);
            //kfree(pgenpci_dev->dummyblk_entry_p[i].pages);
            pgenpci_dev -> dummyblk_entry_p[i].gendisk = NULL;
        }
    }

    kfree( pgenpci_dev->dummyblk_entry_p);

#endif

    struct dma_entry *dma_list_h;
    struct dma_entry *dma_list_e = NULL;

    list_for_each_entry_safe(dma_list_h, dma_list_e, &pgenpci_dev->memlist, list) {
        pr_info("%s: remove dma memory\n", __func__);

        list_del(&dma_list_h->list);
		dma_free_coherent (&pgenpci_dev -> pdev->dev, dma_list_h->size, dma_list_h->virt_addr, dma_list_h->dma_addr);
        kfree(dma_list_h);
        
    }

    struct signal_entry *signal_list_h;
    struct signal_entry *signal_list_e = NULL;

    list_for_each_entry_safe(signal_list_h, signal_list_e, &pgenpci_dev->signallist, list) {
        list_del(&signal_list_h->list);
        free_irq(signal_list_h->irq_no , signal_list_h->id);
        eventfd_ctx_put(signal_list_h -> trigger);
        kfree(signal_list_h->irqname);
                
        kfree(signal_list_h);        
    }

    pci_free_irq_vectors(pgenpci_dev->pdev);

    pci_set_drvdata(pdev, NULL);
	pci_release_regions(pdev);
    pci_disable_device(pdev);

    device_destroy(genpci_class, pgenpci_dev->devt );
    cdev_del(&pgenpci_dev->cdev); 
    ida_simple_remove(&genpci_instance_ida, pgenpci_dev->instance );
    kfree(pgenpci_dev);

}

static void genpci_shutdown(struct pci_dev *pdev)
{

}

static struct pci_driver genpci_driver = {
    .name = DEVICE_NAME,
    .id_table = genpci_ids,
    .probe = genpci_probe,
    .remove = genpci_remove,
    .shutdown = genpci_shutdown,
};

static int genpci_init(void) {
    int ret;
    pr_info("%s\n", __func__);

    ida_init(&genpci_instance_ida);

	ret = alloc_chrdev_region(&genpci_chr_devt, 0, genpci_MINORS, DEVICE_NAME);
	if (ret != 0){
        printk(KERN_ERR "%d: %s()   %d",__LINE__, __FUNCTION__, ret ); 
        ida_destroy(&genpci_instance_ida);
        return ret;
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,4,0)
    genpci_class = class_create(DEVICE_NAME);
#else
	genpci_class = class_create(THIS_MODULE, DEVICE_NAME);
#endif

	if (IS_ERR(genpci_class)) {
		ret = PTR_ERR(genpci_class);
        unregister_chrdev_region(genpci_chr_devt, genpci_MINORS);
        ida_destroy(&genpci_instance_ida);
		return ret;
	}

    blk_major = register_blkdev(0, "dummyblk");

    if (blk_major < 0) {
        class_destroy(genpci_class);
        unregister_chrdev_region(genpci_chr_devt, genpci_MINORS);
        ida_destroy(&genpci_instance_ida);

		return blk_major;
	}

    ret = pci_register_driver(&genpci_driver); 
    if(ret != 0){
        pr_err("%d: %s()   %d",__LINE__, __FUNCTION__, ret );     
        unregister_blkdev(blk_major, "dummyblk");

        class_destroy(genpci_class);
        unregister_chrdev_region(genpci_chr_devt, genpci_MINORS);
        ida_destroy(&genpci_instance_ida);

        return ret;
    }

    //struct proc_dir_entry *entry = proc_create(PROC_NAME, S_IRUGO, NULL, &proc_ops);
	//if (!entry ) {
	//	pr_err("proc_create\n");

    //  pci_unregister_driver(&genpci_driver);
    //  unregister_blkdev(blk_major, "dummyblk");
    //  class_destroy(genpci_class);
    //  unregister_chrdev_region(genpci_chr_devt, genpci_MINORS);
    //  ida_destroy(&genpci_instance_ida);
	//	return -ENOENT;
	//}

    return ret; 
}

static void genpci_exit(void) {

    pr_info("%s\n", __func__);

    //remove_proc_entry(PROC_NAME, NULL);
    pci_unregister_driver(&genpci_driver);  
    
    unregister_blkdev(blk_major, "dummyblk");

    class_destroy(genpci_class);
    unregister_chrdev_region(genpci_chr_devt, genpci_MINORS);
    ida_destroy(&genpci_instance_ida);
}

module_init(genpci_init);
module_exit(genpci_exit);