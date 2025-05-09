#ifndef _COMMON_H
#define _COMMON_H

#define SUPPORT_VECTOR_NUM (32)

enum irq_mode{
    INTX = (1 << 0), 
    MSI	 = 	(1 << 1) ,
    MSIX =	(1 << 2) ,
};

struct signal{
    int vectornum;
    int fd[SUPPORT_VECTOR_NUM];
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

struct ktime{
    __le64  buf;
};

struct dummyblk{
    int     number;
};

struct allocate_statsbuffer{
    __le64  buf;
};

struct test_params{
    union{
        struct signal s;
        struct mem m;
        struct bus b;
        struct dummyblk d;
        struct ktime k;
        struct allocate_statsbuffer a;
    };

};

enum iotype{
    Read = 0,
    Write = 1
};


struct stats {
    int id;
    enum iotype iotype;     /* 0: read, 1: write */
    unsigned int bytes;
    unsigned long start_time_ns;

};



// ioctls
#define NVME 'N'
#define IOCTL_SET_SIGNAL            _IOW(NVME, 1, struct test_params*)
#define IOCTL_GET_MEMFINFO          _IOW(NVME, 2, struct test_params*)
#define IOCTL_GET_BDF               _IOW(NVME, 3, struct test_params*)
#define IOCTL_RELEASE_MEM           _IOW(NVME, 4, struct test_params*)    
#define IOCTL_ALLOC_DUMMYBLK        _IOW(NVME, 5, struct test_params*)    
#define IOCTL_RECORD_STATS          _IOW(NVME, 6, struct test_params*)    
#define IOCTL_SETUP_KTIME            _IOW(NVME, 7, struct test_params*)
#define IOCTL_GET_KTIME             _IOW(NVME, 8, struct test_params*)
//#define IOCTL_GET_PHYSADDR_JIFFIES  _IOW(NVME, 9, struct test_params*)
#define IOCTL_SEUTP_STATS_BUFFER    _IOW(NVME, 10, struct test_params*)

#endif