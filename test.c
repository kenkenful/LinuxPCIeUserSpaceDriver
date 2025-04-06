
#include <linux/cdev.h>
#include <linux/pci.h>
#include <linux/eventfd.h>
#include <linux/version.h>
#include <linux/dma-mapping.h>



#include <linux/blkdev.h>
#include <linux/blk-mq.h>   
#include "common.h"

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Simple NVMe Device Driver");

#define DEVICE_NAME "nvmet"
#define NVME_MINORS		(1U << MINORBITS)

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

#define TEST_PCI_DRIVER_DEBUG
#ifdef  TEST_PCI_DRIVER_DEBUG
#define tprintk(fmt, ...) printk(KERN_ALERT "** (%3d) %-20s: " fmt, __LINE__,  __func__,  ## __VA_ARGS__)
#else
#define tprintk(fmt, ...) 
#endif


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


//struct block_dev {
    //sector_t capacity;
    //u8 *data;  
//    struct blk_mq_tag_set tag_set;
//    struct request_queue *queue;
//    struct gendisk *gendisk;
//};

struct nvme_dev{
    struct pci_dev *pdev;
    struct cdev cdev;
    dev_t  devt;
    int instance;
    bool is_open;

    struct list_head memlist;
    struct list_head signallist;

    struct eventfd_ctx   *trigger;   /* use when INT-X */
    
    struct blk_mq_tag_set tag_set;
    struct request_queue *queue;
    struct gendisk *gendisk;

    spinlock_t lock;	
   
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
    probe処理以降で取得したリソースに関しては、この関数で後始末する。

*/
static int nvmet_pci_close(struct inode* inode, struct file* filp){
    tprintk("%s\n", __func__);

    struct nvme_dev *pnvme_dev = filp->private_data;

    struct dma_entry *dma_list_h;
    struct dma_entry *dma_list_e = NULL;
    
    list_for_each_entry_safe(dma_list_h, dma_list_e, &pnvme_dev->memlist, list) {
        tprintk("safe remove in close\n");

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

    tprintk("length: %d\n", size);

    tprintk("vma->vm_pgoff: %d\n", vma->vm_pgoff);

    virt_addr = dma_alloc_coherent(&pnvme_dev -> pdev->dev, size, &dma_addr, GFP_KERNEL);

    if(IS_ERR_OR_NULL(virt_addr)){
        ret = PTR_ERR(virt_addr);
        goto out_alloc_data_buffer;
    }

    head -> size = size;
    head -> dma_addr = dma_addr;
    head -> virt_addr = virt_addr;

    tprintk("%p, %lx\n", virt_addr, dma_addr);

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
	eventfd_signal(trigger);
	return IRQ_HANDLED; 
}

static irqreturn_t intx_irq(int irq, void *arg)
{
	struct nvme_dev *pnvme_dev = arg;

	if (pci_check_and_mask_intx(pnvme_dev->pdev)) {
		eventfd_signal(pnvme_dev->trigger);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
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

            name = kasprintf(GFP_KERNEL,
				                    KBUILD_MODNAME "[%d](%s)",
				                    params.s.vector, pci_name(pnvme_dev->pdev));

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
                tprintk("set msix: %s\n",  name);

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
            tprintk("ioctl done\n");

            break;
        
        case IOCTL_GET_MEMFINFO:

            ptr = list_entry(pnvme_dev->memlist.next, struct dma_entry, list);

            tprintk("ioctl : %p\n", ptr ->virt_addr);

           // uint64_t temp = (uint64_t)((uintptr_t)(ptr ->virt_addr));
           // tprintk("ioctl cast: %lx\n", temp);

            params.m.dma_addr = ptr ->dma_addr;
            params.m.kernel_virtaddr = ptr ->virt_addr;
            params.m.size = ptr->size;
            
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

            tprintk("recv : %lx\n", params.m.kernel_virtaddr);
            tprintk("recv : %lx\n", params.m.dma_addr);

            struct dma_entry *dma_list_h;
            struct dma_entry *dma_list_e = NULL;
                
            list_for_each_entry_safe(dma_list_h, dma_list_e, &pnvme_dev->memlist, list) {
                if((uint64_t)dma_list_h->virt_addr == params.m.kernel_virtaddr && dma_list_h->dma_addr == params.m.dma_addr){
                    tprintk("safe remove in ioctl\n");
                    list_del(&dma_list_h->list);
		            dma_free_coherent (&pnvme_dev -> pdev->dev, dma_list_h->size, dma_list_h->virt_addr, dma_list_h->dma_addr);
                    kfree(dma_list_h);
                }
            }

            break;

        default:
            break;
    }

    return ret;
}

static blk_status_t queue_rq(struct blk_mq_hw_ctx *hctx, const struct blk_mq_queue_data *bd)
{
    unsigned int nr_bytes = 0;
    blk_status_t status = BLK_STS_OK;
    struct request *rq = bd->rq;
    blk_mq_start_request(rq);

    if (blk_update_request(rq, status, nr_bytes)) {
        BUG();
    }

    __blk_mq_end_request(rq, status);

    return status;
}


static const struct file_operations nvme_chr_fops = {
    .owner          = THIS_MODULE,
    .open           = nvmet_pci_open,
    .release        = nvmet_pci_close,
    .unlocked_ioctl = nvme_ioctl,
    .mmap           = mmap_dma,
};

static struct blk_mq_ops mq_ops = {
    .queue_rq = queue_rq,

};

#if 0
static void dummy_request(struct request_queue *q)
{
	struct request *req;
	int ret = 0;

	while ((req = blk_fetch_request(q)) != NULL) {
		__blk_end_request_all(req, ret);
	}
}
#endif

static const struct block_device_operations blockdev_ops = {
	.owner		= THIS_MODULE,
   // .open       = blockdev_open,
   // .release    = blockdev_release,
   // .ioctl      = blockdev_ioctl
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

    struct device * dev = device_create(nvme_class, NULL, MKDEV(MAJOR(nvme_chr_devt), pnvme_dev -> instance), NULL, "nvmet%d", pnvme_dev -> instance);
    

    if (IS_ERR(dev)) {
		pr_err("couldn't create dev\n");
		ret = PTR_ERR(dev);
		goto out_device_create;
	}






    /* Here is block device setting */
    //pnvme_dev -> bdev = kzalloc(sizeof(struct block_dev), GFP_KERNEL);

    //if (!pnvme_dev->bdev){
	//	ret = -ENOMEM;
    //   goto out_kmalloc;
    //}

    //memset(&pnvme_dev->tag_set.ops, 0, sizeof(pnvme_dev ->tag_set.ops));
    pnvme_dev->tag_set.cmd_size = 0;
    pnvme_dev->tag_set.ops = &mq_ops;
    pnvme_dev->tag_set.driver_data =  pnvme_dev;
    pnvme_dev->tag_set.flags = BLK_MQ_F_SHOULD_MERGE;
    pnvme_dev->tag_set.nr_hw_queues = 1;
    pnvme_dev->tag_set.queue_depth = 128;
    pnvme_dev->tag_set.numa_node = NUMA_NO_NODE;
    printk("%s, %d\n", __func__ , __LINE__);

   // ret = blk_mq_alloc_tag_set(&pnvme_dev ->bdev->tag_set);
     
   // if(ret = blk_mq_alloc_sq_tag_set(&pnvme_dev->tag_set, &mq_ops, 1, BLK_MQ_F_SHOULD_MERGE)){
    if(ret = blk_mq_alloc_tag_set(&pnvme_dev ->tag_set)){
        tprintk("Failed to allocate tag set\n");
       ret = -ENOMEM; 
        goto out_blk_mq_alloc_tag_set;
    }

    tprintk("%s, %d\n", __func__ , __LINE__);


   // pnvme_dev->queue = blk_mq_init_queue(&pnvme_dev ->tag_set);

 //   if (IS_ERR(pnvme_dev->queue )) {
   //      ret = -ENOMEM;
   // 	goto out_blk_mq_init_queue;
   // }
    
   // blk_queue_logical_block_size(pnvme_dev->queue, 512);
   // pnvme_dev->queue->queuedata = pnvme_dev;


    //pnvme_dev->queue = queue;

   // struct gendisk* gendisk = blk_mq_alloc_disk(&pnvme_dev->tag_set, NULL);

    pnvme_dev->gendisk = blk_alloc_disk(NUMA_NO_NODE);

    if (IS_ERR( pnvme_dev->gendisk )) {
        tprintk("Failed to allocate gendisk\n");
		ret = PTR_ERR( pnvme_dev->gendisk );
		goto out_blk_mq_alloc_disk;
	}

    pnvme_dev->queue = pnvme_dev->gendisk->queue; 


    //pnvme_dev ->queue = gendisk->queue;

    //spin_lock_init(&pnvme_dev ->lock);

    //struct request_queue *queue =  blk_init_queue(dummy_request, &pnvme_dev ->lock)

    //pnvme_dev->queue = queue;


    //struct request_queue *queue =  gendisk->queue;

    //blk_queue_max_hw_sectors(queue, 409600000 >> 9);
	//blk_queue_dma_alignment(queue, 4096-1);
	//blk_queue_logical_block_size(queue, 4096);
	//blk_queue_write_cache(queue, true, false);
	//blk_queue_max_segments(queue, -1);
	//blk_queue_max_segment_size(queue, 409600000);

    //pnvme_dev ->bdev->queue = pnvme_dev ->bdev->gendisk->queue;

    //pnvme_dev ->gendisk = gendisk;

    //gendisk->flags |= GENHD_FL_NO_PART;
    pnvme_dev->gendisk->major = blk_major;
    pnvme_dev->gendisk->first_minor = 0;
    pnvme_dev->gendisk->minors = 1;
    pnvme_dev->gendisk->fops = &blockdev_ops;
   // pnvme_dev->gendisk->queue =  pnvme_dev ->queue ;
    pnvme_dev->gendisk->private_data = pnvme_dev;

    snprintf( pnvme_dev->gendisk->disk_name, sizeof( pnvme_dev->gendisk->disk_name), "dummyblk%d", pnvme_dev -> instance);

    set_capacity(pnvme_dev->gendisk, 102400);

    tprintk("%s, %d\n", __func__ , __LINE__);

    ret = add_disk( pnvme_dev->gendisk);
    if (ret)
        tprintk("Failed to adddisk\n");
        ret = -ENOMEM; 
		goto out_add_disk;

    /* End of block device setting.　*/

    tprintk("%s, %d\n", __func__ , __LINE__);


#if 0

    struct queue_limits lim = {0};

    pnvme_dev -> bdev = kmalloc(sizeof(struct block_dev), GFP_KERNEL);
    if (pnvme_dev -> bdev == NULL) {
        printk("Failed to allocate struct block_dev\n");
        //unregister_blkdev(pnvme_dev ->blk_major, pnvme_dev ->blkname);
        //kfree(pnvme_dev ->blkname);
        return -ENOMEM;
    }

    memset(&pnvme_dev -> bdev->tag_set, 0, sizeof(struct blk_mq_tag_set));

#if 1
    pnvme_dev ->bdev->tag_set.ops = &mq_ops;
    pnvme_dev ->bdev->tag_set.flags = BLK_MQ_F_SHOULD_MERGE;
    pnvme_dev ->bdev->tag_set.nr_hw_queues = 1;
    pnvme_dev ->bdev->tag_set.queue_depth = 128;
    pnvme_dev ->bdev->tag_set.numa_node = NUMA_NO_NODE;
    pnvme_dev ->bdev->tag_set.cmd_size = 64;
    pnvme_dev ->bdev->tag_set.driver_data =  pnvme_dev -> bdev;
#endif

    ret = blk_mq_alloc_tag_set(&pnvme_dev -> bdev->tag_set);
    if(ret){//このエラー処理を入れておかないとrmmodをしたときにひどいことになる。
        printk("Failed to allocate tag set\n");
        
      //  unregister_blkdev(pnvme_dev ->blk_major, pnvme_dev ->blkname);
      //  kfree(pnvme_dev ->blkname);
        kfree( pnvme_dev ->bdev);
        
         return -ENOMEM;
    }

    pnvme_dev ->bdev->queue = blk_mq_init_queue(&pnvme_dev ->bdev->tag_set);

    if (pnvme_dev ->bdev->queue == NULL) {
        printk("Failed to allocate device queue\n");
        blk_mq_free_tag_set(&pnvme_dev ->bdev->tag_set);
       // unregister_blkdev(pnvme_dev ->blk_major, pnvme_dev ->blkname);
       // kfree(pnvme_dev ->blkname);
        kfree( pnvme_dev ->bdev);
       
        return -ENOMEM;
    }

    pnvme_dev ->bdev->queue->queuedata = pnvme_dev ->bdev;

    pnvme_dev ->bdev->gendisk = blk_mq_alloc_disk(&priv->tag_set, &lim, dev);
    pnvme_dev ->bdev->gendisk->flags = GENHD_FL_NO_PART_SCAN;
    pnvme_dev ->bdev->gendisk->major =  blk_major;
    pnvme_dev ->bdev->gendisk->first_minor = 0;
    pnvme_dev ->bdev->gendisk->fops = &blockdev_ops;
    pnvme_dev ->bdev->gendisk->queue = pnvme_dev ->bdev->queue;
    pnvme_dev ->bdev->gendisk->private_data = pnvme_dev ->bdev;

    snprintf (pnvme_dev ->bdev->gendisk->disk_name, 32, "blkdev");

    set_capacity(pnvme_dev ->bdev->gendisk, 4096);

    add_disk(bdev->gendisk);


#endif









    if (ret = pci_enable_device_mem(pdev)){
        goto out_pci_enable_device;
    }

    pci_set_master(pdev);

    if (ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64)) && dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32))){
        goto out_dma_set_mask_and_coherent;
    }

    if(ret= pci_request_regions(pdev, DEVICE_NAME)) {
        goto out_pci_request_regions;
    }

    pnvme_dev->pdev = pci_dev_get(pdev);
	pci_set_drvdata(pdev, pnvme_dev);

    n = pci_alloc_irq_vectors(pdev, 1, vectors, PCI_IRQ_ALL_TYPES | PCI_IRQ_AFFINITY);

    tprintk("vector num: %d\n", n);

    if (n < 0) {
        ret = -ENOMEM;
        goto out_pci_alloc_irq_vectors;
	}

    INIT_LIST_HEAD(&pnvme_dev->memlist);   
    INIT_LIST_HEAD(&pnvme_dev->signallist);   

    return ret;


out_pci_alloc_irq_vectors:
    pci_set_drvdata(pdev, NULL);
    pci_release_regions(pdev);

out_pci_request_regions:

out_dma_set_mask_and_coherent:
    pci_disable_device(pdev);

out_pci_enable_device:
    del_gendisk(pnvme_dev ->gendisk);

out_add_disk:
    put_disk(pnvme_dev ->gendisk);

out_blk_mq_alloc_disk:
 //   blk_mq_destroy_queue(pnvme_dev -> queue);

out_blk_mq_init_queue:
 //   blk_mq_free_tag_set(&pnvme_dev ->tag_set);

out_blk_mq_alloc_tag_set:
//    kfree(pnvme_dev -> bdev );

//out_kmalloc:
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
    tprintk("%s\n", __func__);

    struct nvme_dev *pnvme_dev = pci_get_drvdata(pdev);
    pnvme_dev->is_open = 0;

    struct dma_entry *dma_list_h;
    struct dma_entry *dma_list_e = NULL;

    list_for_each_entry_safe(dma_list_h, dma_list_e, &pnvme_dev->memlist, list) {
        tprintk("safe remove in remove\n");

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

    del_gendisk(pnvme_dev ->gendisk);
    put_disk(pnvme_dev ->gendisk);



  ///  	blk_cleanup_queue(pnvme_dev ->queue);
   // blk_mq_destroy_queue(pnvme_dev -> queue);

   // blk_mq_free_tag_set(&pnvme_dev ->tag_set);
    //kfree(pnvme_dev -> bdev );


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

/* インストール時に実行 */
static int nvmet_init(void) {
    int ret;
    tprintk("%s\n", __func__);

    ida_init(&nvme_instance_ida);

	ret = alloc_chrdev_region(&nvme_chr_devt, 0, NVME_MINORS, DEVICE_NAME);
	if (ret != 0){
        printk(KERN_ERR "%d: %s()   %d",__LINE__, __FUNCTION__, ret ); 
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
		return ret;
	}


    blk_major = register_blkdev(0, "testblk");

    if (blk_major < 0) {
		return blk_major;
	}



    ret = pci_register_driver(&simple_nvme_driver); //ドライバー名、table, コールバック関数(probe, remove)を登録
    if(ret != 0){
        printk(KERN_ERR "%d: %s()   %d",__LINE__, __FUNCTION__, ret );     
        return ret;
    }

    return ret; 
}

/* アンインストール時に実行 */
static void nvmet_exit(void) {

    tprintk("%s\n", __func__);

    pci_unregister_driver(&simple_nvme_driver);  //ドライバー名、table, コールバック関数(probe, remove)を削除  

    unregister_blkdev(blk_major, "testblk");

    class_destroy(nvme_class);
    unregister_chrdev_region(nvme_chr_devt, NVME_MINORS);
    ida_destroy(&nvme_instance_ida);
}

module_init(nvmet_init);
module_exit(nvmet_exit);