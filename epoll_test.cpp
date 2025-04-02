#include <iostream>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <thread>
#include <unistd.h>


#define ASSERT(expr, msg, ...)                                             \
    do {                                                                   \
        if (!(expr)) {                                                     \
            fprintf(stderr, "[Error] %s:%3d %15s(): ", __FILE__, __LINE__, \
                    __func__);                                             \
            fprintf(stderr, msg "\n", ##__VA_ARGS__);                      \
            exit(1);                                                       \
        }                                                                  \
    } while (0)


// ioctls
#define TEST 'T'
#define IOCTL_SIGNAL_TEST   _IOW(TEST, 1, struct test_params*)
#define IOCTL_SET_SIGNAL_FD _IOW(TEST, 2, struct test_params*)
#define IOCTL_MAP_USERPAGE  _IOW(TEST, 3, struct test_params*)

struct test_params{
    int vector;
    int fd;

};

void func(int *efd) {
    struct epoll_event evs;
    printf("waiting interrupts...\n");
    int rc = epoll_wait(*efd, &evs, 1, -1);
    printf("raise interrupts...\n");
};


int main(){


    int fd = open("/dev/test0", O_RDWR|O_SYNC);
    if(fd < 0){
        printf("open error\n");
        return 1;
    }

    int efd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    ASSERT(efd >= 0, "efd init failed");

    int epfd = epoll_create1(EPOLL_CLOEXEC);
    ASSERT(epfd >= 0, "failed to create epoll fd");


    // Add eventfd to epoll
    struct epoll_event ev = {0}; //{.events = EPOLLIN | EPOLLPRI,
                                 //.data.fd = dev->efds[i]};
    ev.events = EPOLLIN | EPOLLPRI;
    ev.data.fd = efd;

    int ret = epoll_ctl(epfd, EPOLL_CTL_ADD, efd, &ev);
    ASSERT(ret == 0, "cannot add fd to epoll");

    std::thread th(func, &epfd);

    sleep(3);

    struct test_params params;
    params.fd = efd;

    int rc = ioctl(fd, IOCTL_SIGNAL_TEST  , &params);

    th.join();

    
}