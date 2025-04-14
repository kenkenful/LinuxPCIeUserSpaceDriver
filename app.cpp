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
# include <errno.h>
#include "common.h"
#include "nvme.h"

class GenPci{
public:
    GenPci(int ctrl_no){

        instance_no = ctrl_no;
        bar0_virt = nullptr;
        config_virt = nullptr;
        jiffies_p = nullptr;
        pthread_mutex_init(&alloc_dma_m, NULL);

        get_bdf();
        get_pci_config_addr();
        get_bar0();
        map_ctrlreg();
        
        printf("pci_addr: %lx\n", pci_addr);
        printf("bar0: %lx\n", bar0);
    }

    ~GenPci(){
        for(int i= 0; i<support_vector_num; ++i)close(efd[i]);
        close(epfd);

        pthread_mutex_destroy(&alloc_dma_m);

        if(bar0_virt != nullptr) munmap(bar0_virt, 8192);
        if(config_virt != nullptr) munmap(config_virt, 4096);

        if(jiffies_p != nullptr) free(jiffies_p);
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

    void set_INTx(int vectornum){
        if(vectornum > support_vector_num) return;
        efd[0] = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
        //ASSERT(efd >= 0, "efd init failed");
        if(efd[0] < 0){
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
        ev.data.fd = efd[0];

        int ret = epoll_ctl(epfd, EPOLL_CTL_ADD, efd[0], &ev);
        //ASSERT(ret == 0, "cannot add fd to epoll");
        if(ret != 0){
            exit(-1);
        }

        struct test_params params;
        params.s.fd[0] = efd[0];
        params.s.irq_mode = INTX;
        params.s.vectornum = vectornum;

        int rc = ioctl(genpcifd, IOCTL_SET_SIGNAL, &params);
        if(rc){
            perror("ioctl");
            exit(-1);
        }
    }

    void set_MSIX(int vectornum){
        struct test_params params;

        if(vectornum > support_vector_num) return;

        vector_num = vectornum;

        for(int i=0; i< vector_num; ++i){
            efd[i] = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
            //ASSERT(efd >= 0, "efd init failed");
            if(efd[i] < 0){
                std::cerr << "error: eventfd" << std::endl;
                exit(-1);
            }

            epfd = epoll_create1(EPOLL_CLOEXEC);
            //ASSERT(epfd >= 0, "failed to create epoll fd");
            if(epfd < 0){
                std::cerr << "error: create1" << std::endl;
                exit(-1);
            }

            // Add eventfd to epoll
            struct epoll_event ev = {0}; //{.events = EPOLLIN | EPOLLPRI,
                                         //.data.fd = dev->efds[i]};
            ev.events = EPOLLIN | EPOLLPRI;
            ev.data.fd = efd[i];

            int ret = epoll_ctl(epfd, EPOLL_CTL_ADD, efd[i], &ev);
            //ASSERT(ret == 0, "cannot add fd to epoll");
            if(ret != 0){
                std::cerr << "error: epoll_ctl" << std::endl;
                exit(-1);
            }

            params.s.fd[i] = efd[i];
        
        }

        params.s.irq_mode = MSIX;
        params.s.vectornum = vectornum;

        int rc = ioctl(genpcifd, IOCTL_SET_SIGNAL, &params);
        if(rc){
            perror("ioctl");
            exit(-1);
        }
    }



    void set_jiffies(){
        struct test_params params;
        int ret = posix_memalign(&jiffies_p, 4096, 4096);
        if(ret != 0){
            perror("posix_memalign");
            exit(-1);
        }

	    params.j.buf = (__le64)jiffies_p;

	    int rc = ioctl(genpcifd, IOCTL_SETUP_JIFFIES, &params);
        if(rc){
            perror("ioctl");
            exit(-1);
        }
    }

    uint64_t get_jiffies(){
        struct test_params params = {0};
        int rc = ioctl(genpcifd, IOCTL_GET_JIFFIES, &params);
        if(rc){
            perror("ioctl");
            exit(-1);
        }

        return *(uint64_t*)jiffies_p;
    }

    void create_dummy_blk(int num){
        struct test_params params = {0};
        for(int i=0; i< num;++i){
            int ret = posix_memalign(&dummyblk_buf[i], 4096, 4096);
            if(ret != 0){
                perror("posix_memalign");
                exit(-1);
            }

            params.d.number = i;
	        params.d.len = 4096;
	        params.d.buf = (__le64)dummyblk_buf[i];

            int rc = ioctl(genpcifd, IOCTL_ALLOC_DUMMYBLK, &params);

            if(rc){
                perror("ioctl");
                exit(-1);
            }
        }

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
    void*   jiffies_p; 

    void*  dummyblk_buf[128];  

protected:
    void*   bar0_virt;
    void*   config_virt;
    int     efd[support_vector_num];
    int     epfd;
    int     vector_num;
};

class NVMeCtrl: public GenPci{
public:

    NVMeCtrl(int instance_no):GenPci(instance_no){
        ctrl_reg = (nvme_controller_reg_t*)bar0_virt;
        init_ctrl();
        
        //set_MSIX(1);
        //set_jiffies();
        //std::cout << get_jiffies() << std::endl;
        //create_dummy_blk(5);
        //kick_msix_intr_thread();
        
        
        set_INTx(1);
        kick_intx_intr_thread();
        
        
        sleep(1);
        identify();
        identify();
        identify();
        identify();


    }

    ~NVMeCtrl(){}


    void* msix_handler(void){

        struct epoll_event evs;
        u64 u;

        for (;;) {
            printf("waiting interrupts...\n");
            // blocking wait
            int rc = epoll_wait(epfd, &evs, 1, -1);
            if(rc <= 0){
                printf("epoll error\n");
                exit(1);
            }

            ssize_t s = read(evs.data.fd, &u, sizeof(u));
            if(s != sizeof(u)){
                printf("efd read failed\n");
            }

            for(int i=0; i<vector_num; ++i){
                if (evs.data.fd == efd[i]) {
                    if(i == 0){
                        printf("MSIx vector 0 interrupt ocuured\n");      
                        if (admin_cq_entry[admin_cq_head].u.a.p == admin_cq_phase) {
				            while (admin_cq_entry[admin_cq_head].u.a.p == admin_cq_phase) {
					            printf("Interrupt Occured\n");

					            //int head = p->admin_cq_head;
					            if (++admin_cq_head == admin_cq_size) {
					            	admin_cq_head = 0;
					            	admin_cq_phase = !admin_cq_phase;
					            }
					            *(volatile u32*)(admin_cq_doorbell) = admin_cq_head;
					            //ctrl_reg->sq0tdbl[1] = admin_cq_head;
					        }
		            	}

                    }else{
                        printf("MSIx vector %d interrupt ocuured\n", i);
                    }
                    break;
                }
            }

        }
    }

    static void* msix_handler_wrapper(void* p){
        return ((NVMeCtrl*)p) -> msix_handler();


    }


    void kick_msix_intr_thread(){
        if (pthread_create(&intr_th, NULL, msix_handler_wrapper, this) != 0) {
            printf("Error: pthread create\n");
            exit(1);
        }
        if (pthread_detach(intr_th) != 0) {
            printf("Error: pthread detach\n");
            exit(1);
        }
    }

#if 1
    void* intx_handler(void){
        struct epoll_event evs;
        u64 u;

        for (;;) {
            printf("waiting interrupts...\n");
            // blocking wait
            int rc = epoll_wait(epfd, &evs, 1, -1);
            if(rc <= 0){
                printf("epoll error\n");
                exit(1);
            }

            ssize_t s = read(evs.data.fd, &u, sizeof(u));
            if(s != sizeof(u)){
                printf("efd read failed\n");
            }

            if (evs.data.fd == efd[0]) {
                printf("INTx interrupt ocuured\n");      
                if (admin_cq_entry[admin_cq_head].u.a.p == admin_cq_phase) {
			        while (admin_cq_entry[admin_cq_head].u.a.p == admin_cq_phase) {
				        printf("Interrupt Occured\n");
				        //int head = p->admin_cq_head;
				        if (++admin_cq_head == admin_cq_size) {
				        	admin_cq_head = 0;
				        	admin_cq_phase = !admin_cq_phase;
				        }
				        *(volatile u32*)(admin_cq_doorbell) = admin_cq_head;
				        //ctrl_reg->sq0tdbl[1] = admin_cq_head;
				    }
                }
            }
  

        }
    }
#endif


    static void* intx_handler_wrapper(void* p){
        return ((NVMeCtrl*)p) -> intx_handler();
    }

    void kick_intx_intr_thread(){
        if (pthread_create(&intr_th, NULL, intx_handler_wrapper, this) != 0) {
            printf("Error: pthread create\n");
            exit(1);
        }
        if (pthread_detach(intr_th) != 0) {
            printf("Error: pthread detach\n");
            exit(1);
        }
    }


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

        admin_cq_entry = (nvme_cq_entry_t*)alloc_dma(sizeof(nvme_cq_entry_t) * admin_cq_size, admin_cq_amd_addr);
        if(admin_cq_entry == nullptr){
            std::cerr << "cannot allocate admin cq" << std::endl;
            exit(1);
        }
        
        admin_sq_entry = (nvme_sq_entry_t*)alloc_dma(sizeof(nvme_sq_entry_t) * admin_sq_size, admin_sq_amd_addr);
        if(admin_sq_entry == nullptr){
            std::cerr << "cannot allocate admin sq" << std::endl;
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

    void identify(){
        std::cout << __func__ << std::endl;
        int cid = admin_sq_tail;

        uint64_t data_addr;
        void* datap = alloc_dma(4096, data_addr);
        if(datap == nullptr){
            std::cerr << "cannot allocate data buffer" << std::endl;
            exit(-1);
        }

        admin_sq_entry[cid].identify.opcode = nvme_admin_identify;
		admin_sq_entry[cid].identify.command_id = (u16)cid;
		admin_sq_entry[cid].identify.cns = NVME_ID_CNS_CTRL;
		admin_sq_entry[cid].identify.prp1 = data_addr;
		admin_sq_entry[cid].identify.nsid = 0;

        if (++admin_sq_tail == admin_sq_size) admin_sq_tail = 0;
		*(volatile u32*)admin_sq_doorbell = admin_sq_tail;

    }


private:
    pthread_t intr_th;
    nvme_controller_reg_t* ctrl_reg;

    int admin_cq_size;
	int admin_sq_size;
	int admin_sq_tail;
	int admin_cq_head;
	int admin_cq_phase;

    uint64_t admin_cq_amd_addr;
    uint64_t admin_sq_amd_addr;
	nvme_sq_entry_t* admin_sq_entry;
	nvme_cq_entry_t* admin_cq_entry;

    void* admin_cq_doorbell;
	void* admin_sq_doorbell;

};







int main(){
    std::shared_ptr<NVMeCtrl> p = std::make_shared<NVMeCtrl>(0);
    sleep(10);   
}