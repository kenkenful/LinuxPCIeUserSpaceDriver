#include <stdio.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <cstdint>
#include <sys/ioctl.h>
#include "common.h"


#define NPAGES 16

int main(void)
{
	int fd;
	unsigned int *kadr;

	struct test_params param;

	int len = NPAGES * getpagesize();

	if ((fd=open("/dev/nvmet0", O_RDWR|O_SYNC)) < 0) {
		perror("open");
		exit(-1);
	}
	fprintf(stderr, "mmap_alloc: open OK\n");

	kadr = (unsigned int *)mmap(0, len, PROT_READ|PROT_WRITE, MAP_SHARED| MAP_LOCKED,fd, 0);

	if (kadr == MAP_FAILED)	{
		perror("mmap");
		exit(-1);
	}
	fprintf(stderr, "mmap_alloc: mmap OK\n");

	int rc = ioctl(fd, IOCTL_GET_MEMFINFO, &param);


	//printf("%lx, %lx\n", param.m.kernel_virtaddr, param.m.dma_addr);

	rc = ioctl(fd, IOCTL_RELEASE_MEM, &param);

	void *p;
	int ret = posix_memalign(&p, 4096, 4096*5);

	printf("udata: %p\n", p);


	param.d.number = 0;
	param.d.len = 4096*5;
	param.d.buf = (__le64)p;;

	rc = ioctl(fd, IOCTL_ALLOC_DUMMYBLK, &param);

	uint8_t* p8= (uint8_t*)p;
	p8[0] = 0xAA;	
	p8[1] = 0xBB;	
	p8[2] = 0xCC;	
	p8[3] = 0xDD;	

	//param.d.number = 1;
	//rc = ioctl(fd, IOCTL_ALLOC_DUMMYBLK, &param);


	//param.d.number = 2;
	//rc = ioctl(fd, IOCTL_ALLOC_DUMMYBLK, &param);


//	param.d.number = 3;
//	rc = ioctl(fd, IOCTL_ALLOC_DUMMYBLK, &param);




//	sleep(10);



#if 0
	rc = ioctl(fd, IOCTL_GET_BDF, &param);
	printf("%x:%x.%x\n", param.b.bus, param.b.dev, param.b.func);

	kadr = (unsigned int *)mmap(0, len, PROT_READ|PROT_WRITE, MAP_SHARED| MAP_LOCKED,fd, 0);

	if (kadr == MAP_FAILED)	{
		perror("mmap");
		exit(-1);
	}
	fprintf(stderr, "mmap_alloc: mmap OK\n");

	rc = ioctl(fd, IOCTL_GET_MEMFINFO, &param);
	printf("%lx, %lx\n", param.m.kernel_virtaddr, param.m.dma_addr);
#endif

	free(p);
	close(fd);
	return(0);
}