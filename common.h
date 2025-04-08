#ifndef _COMMON_H
#define _COMMON_H

struct signal{
    int vector;
    int fd;
    int msix;
    char name[32];
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
    };

};


struct stats{



};




// ioctls
#define NVME 'N'
#define IOCTL_SET_SIGNAL    _IOW(NVME, 1, struct test_params*)
#define IOCTL_GET_MEMFINFO  _IOW(NVME, 2, struct test_params*)
#define IOCTL_GET_BDF       _IOW(NVME, 3, struct test_params*)
#define IOCTL_RELEASE_MEM   _IOW(NVME, 4, struct test_params*)

#define IOCTL_ALLOC_DUMMYBLK _IOW(NVME, 5, struct test_params*)

#define IOCTL_REGISTER_STATS _IOW(NVME, 6, struct test_params*)


#endif