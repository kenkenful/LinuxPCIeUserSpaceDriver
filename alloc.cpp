#include <stdio.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>

#define NPAGES 16



int main(void)
{
	int fd;
	unsigned int *kadr;

	int len = NPAGES * getpagesize();

	if ((fd=open("/dev/test0", O_RDWR|O_SYNC)) < 0) {
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


	kadr = (unsigned int *)mmap(0, len, PROT_READ|PROT_WRITE, MAP_SHARED| MAP_LOCKED,fd, 0);

	if (kadr == MAP_FAILED)	{
		perror("mmap");
		exit(-1);
	}
	fprintf(stderr, "mmap_alloc: mmap OK\n");

	close(fd);
	return(0);
}