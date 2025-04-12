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
    }

    ~GenPci(){
        printf("pci_addr: %lx\n", pci_addr);
        printf("bar0: %lx\n", bar0);

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


private:
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
        cap = &ctrl_reg->cap;

        uint64_t dma_addr;
        void* p = alloc_dma(4096, dma_addr);
        printf("virtual addr: %p\n", p);
        printf("dma addr: %lx\n", dma_addr);

    }

    ~NVMeCtrl(){


    }

    bool wait_ready(){
        int cnt = 0;
        nvme_controller_cap_t cap = { 0 };
        cap.val = ctrl_reg->cap.val;

        while (ctrl_reg->csts.rdy == 0) {
		    if (cnt++ > cap.a.to) {
		    	std::cerr << "timeout: controller enable" << std::endl;
		    	return false;
		    }
        }
		sleep(500);

        return true;
    }

    bool wait_not_ready(){
        int cnt = 0;
      
        cap->val = ctrl_reg->cap.val;

        ctrl_reg->cc.a.en = 0;
        while (ctrl_reg->csts.rdy == 1) {
		printf("Waiting  controller disable: %d\n", ctrl_reg->csts.rdy);
		    if (cnt++ > cap->a.to) {
		    	std::cerr << "timeout: controller disable" << std::endl;
		    	return false;
		    }
        }
		sleep(500);

        return true;
    }

    void init_adminQ(int cq_sz, int sq_sz){
        nvme_adminq_attr_t	aqa = { 0 };
        admin_sq_tail = 0;
	    admin_cq_head = 0;
	    admin_cq_phase = 1;

	    admin_cq_size = cq_sz;
	    admin_sq_size = sq_sz;
        
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
	    nvme_controller_config_t cc = { 0 };

       


        ret = wait_not_ready();

        //aqa.a.acqs = admin_cq_size - 1;
	    //aqa.a.asqs = admin_sq_size - 1;
	    //ctrl_reg->aqa.val = aqa.val;

        //ctrl_reg->acq = (u64)adminCQ.phyAddr;
	    //ctrl_reg->asq = (u64)adminSQ.phyAddr;

        ret = wait_ready();



        return ret;
    }

private:
    nvme_controller_reg_t* ctrl_reg;
    nvme_controller_cap_t* cap;

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

}