#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <cstdint>
#include <sys/ioctl.h>
#include <memory>
#include <pthread.h>
#include <cassert>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <cstring>
#include "common.h"
#include "nvme.h"

class GenPci{
public:
    GenPci(int ctrl_no){

        instance_no = ctrl_no;
        bar0_virt = nullptr;
        config_virt = nullptr;
        pthread_mutex_init(&alloc_dma_m, NULL);


        get_bdf();
        get_pci_config_addr();
        get_bar0();
        map_ctrlreg();
        
        printf("pci_addr: %lx\n", pci_addr);
        printf("bar0: %lx\n", bar0);
    }

    ~GenPci(){
        close(efd);
        close(epfd);


        pthread_mutex_destroy(&alloc_dma_m);

        if(bar0_virt != nullptr) munmap(bar0_virt, 8192);
        if(config_virt != nullptr) munmap(config_virt, 4096);
        close(genpcifd);
    }

    void get_bdf(){
      
        sprintf(devname, "/dev/genpci%d", instance_no);
        std::cout << devname << std::endl;
        if ((genpcifd=open(devname, O_RDWR|O_SYNC)) < 0) {
		    perror("open");
		    exit(-1);
	    }

        struct test_params param = {0};
        int rc = ioctl(genpcifd, IOCTL_GET_BDF, &param);

        if(rc){
            perror("ioctl");
            exit(-1);
        }

	    printf("%x:%x.%x\n", param.b.bus, param.b.dev, param.b.func);

        busn = param.b.bus; 
        devn = param.b.dev;
        funcn = param.b.func;

    }

    void get_pci_config_addr(){
        uint32_t buf[60] = {0};
      
        int fd = open("/sys/firmware/acpi/tables/MCFG", O_RDONLY);
        if(fd == -1){
            printf("Error open /sys/firmware/acpi/tables/MCFG  %d\n",fd);
              exit(-1);
        }

        int sz = read(fd, buf, 60);
        if(sz != 60){
            printf("Error read\n");
             exit(-1);
        }

        close(fd);

        pci_addr = buf[11] + 4096 * ((uint32_t)funcn + 8 * ((uint32_t)devn + 32 * (uint32_t)busn));

    }

    void get_bar0(){
        int fd = open("/dev/mem", O_RDWR | O_DSYNC);
        if(fd == -1){
            printf("Error open\n");
            exit(-1);
        }

        config_virt = (uint8_t*)mmap(NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pci_addr);
        if(config_virt == MAP_FAILED){
            printf("Error map\n");
            close(fd);
        }

        uint32_t bar_lower = *(uint32_t*)((uint8_t*)config_virt + 0x10);
        uint32_t bar_upper = *(uint32_t*)((uint8_t*)config_virt + 0x14);

        bar0 = ((uint64_t)bar_upper << 32) | (bar_lower & 0xfffffff0);
    }

    void map_ctrlreg(){
        int fd = open("/dev/mem", O_RDWR|O_SYNC);    
        if(fd == -1){
            printf("Error open\n");
            exit(-1);
        }

        bar0_virt =  mmap(NULL, 8192, PROT_READ|PROT_WRITE, MAP_SHARED, fd, bar0);
        if(bar0_virt == MAP_FAILED){
            printf("Error map\n");
            close(fd);
            exit(-1);
        }

        close(fd);
    }

    void* alloc_dma(size_t len, uint64_t &dma_addr){
        struct test_params param;
        pthread_mutex_lock(&alloc_dma_m);	    
        void* virt_addr = (unsigned int *)mmap(0, len, PROT_READ | PROT_WRITE, MAP_SHARED| MAP_LOCKED, genpcifd, 0);

	    if (virt_addr == MAP_FAILED)	{
		    perror("mmap");
		    exit(-1);
	    }

        int rc = ioctl(genpcifd, IOCTL_GET_MEMFINFO, &param);
        if(rc){
            perror("ioctl");
            exit(-1);
        }

        dma_addr = param.m.dma_addr;
        pthread_mutex_unlock(&alloc_dma_m);
        return virt_addr;

    }

    void set_MSIX(char* name, int vector){
        efd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
        //ASSERT(efd >= 0, "efd init failed");
        if(efd < 0){
            exit(-1);
        }

        epfd = epoll_create1(EPOLL_CLOEXEC);
        //ASSERT(epfd >= 0, "failed to create epoll fd");
        if(epfd < 0){
            exit(-1);
        }

        // Add eventfd to epoll
        struct epoll_event ev = {0}; //{.events = EPOLLIN | EPOLLPRI,
                                     //.data.fd = dev->efds[i]};
        ev.events = EPOLLIN | EPOLLPRI;
        ev.data.fd = efd;

        int ret = epoll_ctl(epfd, EPOLL_CTL_ADD, efd, &ev);
        //ASSERT(ret == 0, "cannot add fd to epoll");
        if(ret != 0){
            exit(-1);
        }

        //std::thread th(func, &epfd);
        sleep(3);

        struct test_params params;
        params.s.fd = efd;
        params.s.msix = true;
        strncpy(params.s.name, name, 32); 

        params.s.vector = vector;

        int rc = ioctl(genpcifd, IOCTL_SET_SIGNAL  , &params);
        if(rc){
            perror("ioctl");
            exit(-1);
        }
    }

private:
    int efd;
    int epfd;

    pthread_mutex_t	alloc_dma_m;

    char devname[20] = {0};
    int instance_no;
    int genpcifd;
    uint16_t busn; 
    uint16_t devn;
    uint16_t funcn;

    uint64_t pci_addr;
    uint64_t bar0;

protected:
    void*    bar0_virt;
    void*    config_virt;
};

class NVMeCtrl: public GenPci{
public:

    NVMeCtrl(int instance_no):GenPci(instance_no){
        ctrl_reg = (nvme_controller_reg_t*)bar0_virt;
        init_ctrl();
        char name[32] = {0};
        sprintf(name, "testirq%d", instance_no);
        set_MSIX(name, 0);
   
    }

    ~NVMeCtrl(){}

    bool wait_ready(){
        int cnt = 0;
        ctrl_reg->cc.a.en = 1;

        while (ctrl_reg->csts.rdy == 0) {
		    if (cnt++ >  ctrl_reg->cap.a.to) {
		    	std::cerr << "timeout: controller enable" << std::endl;
		    	return false;
		    }
            usleep(500000);         
        }

        return true;
    }

    bool wait_not_ready(){
        int cnt = 0;
        ctrl_reg->cc.a.en = 0;
        while (ctrl_reg->csts.rdy == 1) {
		    printf("Waiting  controller disable: %d\n", ctrl_reg->csts.rdy);
		    if (cnt++ > ctrl_reg->cap.a.to) {
		    	std::cerr << "timeout: controller disable" << std::endl;
		    	return false;
		    }
            usleep(500000);         
        }
        std::cout << "controller is not ready" << std::endl;

        return true;
    }

    void init_adminQ(int cq_depth, int sq_depth){
        nvme_adminq_attr_t	aqa = { 0 };
        admin_sq_tail = 0;
	    admin_cq_head = 0;
	    admin_cq_phase = 1;

	    admin_cq_size = cq_depth;
	    admin_sq_size = sq_depth;
        
        ctrl_reg->aqa.a.acqs = admin_cq_size -1;
        ctrl_reg->aqa.a.asqs = admin_sq_size -1;

        admin_cq_pvu = alloc_dma(sizeof(nvme_cq_entry_t) * admin_cq_size, admin_cq_amd_addr);
        if(admin_cq_pvu == nullptr){
            exit(1);
        }
        
        admin_sq_pvu = alloc_dma(sizeof(nvme_sq_entry_t) * admin_sq_size, admin_sq_amd_addr);
        if(admin_sq_pvu == nullptr){
            exit(1);
        }

        ctrl_reg->acq = admin_cq_amd_addr;
	    ctrl_reg->asq = admin_sq_amd_addr;

        admin_sq_doorbell = &ctrl_reg->sq0tdbl[0];
	    admin_cq_doorbell = &ctrl_reg->sq0tdbl[0] + (1 << ctrl_reg->cap.a.dstrd);

    }

    bool init_ctrl(){
        bool ret = true;

        ret = wait_not_ready();

        init_adminQ(64,64);

        ctrl_reg ->cc.val = NVME_CC_CSS_NVM;
	    ctrl_reg ->cc.val |= 0 << NVME_CC_MPS_SHIFT;
	    ctrl_reg ->cc.val |= NVME_CC_AMS_RR | NVME_CC_SHN_NONE;
	    ctrl_reg ->cc.val |= NVME_CC_IOSQES | NVME_CC_IOCQES;

        ret = wait_ready();
        if(ret == true){
            std::cout << "controller is ready" << std::endl;
        }

        return ret;
    }

private:
    nvme_controller_reg_t* ctrl_reg;

    int admin_cq_size;
	int admin_sq_size;
	int admin_sq_tail;
	int admin_cq_head;
	int admin_cq_phase;

    uint64_t admin_cq_amd_addr;
    uint64_t admin_sq_amd_addr;
    void* admin_cq_pvu;
    void* admin_sq_pvu;

    void* admin_cq_doorbell;
	void* admin_sq_doorbell;

};






int main(){
    std::shared_ptr<NVMeCtrl> p = std::make_shared<NVMeCtrl>(0);
    sleep(10);

}