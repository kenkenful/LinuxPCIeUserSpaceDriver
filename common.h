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

struct test_params{
    union{
        struct signal s;
        struct mem m;
        struct bus b;
    };

};


// ioctls
#define NVME 'N'
#define IOCTL_SET_SIGNAL _IOW(NVME, 1, struct test_params*)
#define IOCTL_GET_MEMFINFO _IOW(NVME, 2, struct test_params*)
#define IOCTL_GET_BDF _IOW(NVME, 3, struct test_params*)
#define IOCTL_RELEASE_MEM _IOW(NVME, 4, struct test_params*)




#endif