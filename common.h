#ifndef _COMMON_H
#define _COMMON_H

#define support_vector_num (32)

enum irq_mode{
    INTX = (1 << 0), 
    MSI	 = 	(1 << 1) ,
    MSIX =	(1 << 2) ,
};

struct signal{
    int vectornum;
    int fd[support_vector_num];
    enum irq_mode irq_mode;
//
};

struct mem{
    uint64_t kernel_virtaddr;
    uint64_t user_virtaddr;
    uint64_t dma_addr;
    long size;
};

struct bus{
    int     domain;
    uint8_t bus;
    uint8_t dev;
    uint8_t func;

};

struct jiffies{
    __le64  buf;
    __le64  phys_addr;
};


struct dummyblk{
    int     number;
    __le64  buf;
    unsigned long len;
};

struct test_params{
    union{
        struct signal s;
        struct mem m;
        struct bus b;
        struct dummyblk d;
        struct jiffies j;
    };

};

struct stats {
    unsigned long sectors[2];   /* READs and WRITEs */  // rsec/s wsec/s avgrq-sz
    unsigned long ios[2];  // r/s w/s
    unsigned long merges[2]; // rrqm/s wrqm/s
    unsigned long ticks[2];  // await r_wait w_wait
    unsigned long io_ticks; // %util svctm
    unsigned long time_in_queue; // avgqu-sz
};



// ioctls
#define NVME 'N'
#define IOCTL_SET_SIGNAL            _IOW(NVME, 1, struct test_params*)
#define IOCTL_GET_MEMFINFO          _IOW(NVME, 2, struct test_params*)
#define IOCTL_GET_BDF               _IOW(NVME, 3, struct test_params*)
#define IOCTL_RELEASE_MEM           _IOW(NVME, 4, struct test_params*)    
#define IOCTL_ALLOC_DUMMYBLK        _IOW(NVME, 5, struct test_params*)    
#define IOCTL_RECORD_STATS          _IOW(NVME, 6, struct test_params*)    
#define IOCTL_SETUP_JIFFIES         _IOW(NVME, 7, struct test_params*)
#define IOCTL_GET_JIFFIES           _IOW(NVME, 8, struct test_params*)
#define IOCTL_GET_PHYSADDR_JIFFIES  _IOW(NVME, 9, struct test_params*)


#endif