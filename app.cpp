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
        vector_num = 0;

        pthread_mutex_init(&alloc_dma_mutex, NULL);

        get_bdf();
        get_pci_config_addr();
        get_bar0();
        map_ctrlreg();
        
        printf("pci_addr: %lx\n", pci_addr);
        printf("bar0: %lx\n", bar0);
    }

    ~GenPci(){
        for(int i= 0; i<SUPPORT_VECTOR_NUM; ++i)close(efd[i]);
        close(epfd);

        pthread_mutex_destroy(&alloc_dma_mutex);

        if(bar0_virt != nullptr) munmap(bar0_virt, 8192);
        if(config_virt != nullptr) munmap(config_virt, 4096);
        if(stats_buf != nullptr)free(stats_buf);
        
        if(jiffies_p != nullptr) free(jiffies_p);
        close(genpcifd);
    }

    void get_bdf(){
      
        sprintf(devname, "/dev/genpci%d", instance_no);
        std::cout << devname << std::endl;
        if ((genpcifd=open(devname, O_RDWR|O_SYNC)) < 0) {
		    perror("open in get_bdf");
		    exit(-1);
	    }

        struct test_params param = {0};
        int rc = ioctl(genpcifd, IOCTL_GET_BDF, &param);

        if(rc){
            perror("ioctl in get_bdf");
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
            perror("open in get_pci_config_addr");
            exit(-1);
        }

        int sz = read(fd, buf, 60);
        if(sz != 60){
            perror("read in get_pci_config_addr\n");
            exit(-1);
        }

        close(fd);

        pci_addr = buf[11] + 4096 * ((uint32_t)funcn + 8 * ((uint32_t)devn + 32 * (uint32_t)busn));

    }

    void get_bar0(){
        int fd = open("/dev/mem", O_RDWR | O_DSYNC);
        if(fd == -1){
            perror("open in get_bar0");
            exit(-1);
        }

        config_virt = (uint8_t*)mmap(NULL, 4096, PROT_READ|PROT_WRITE, MAP_SHARED, fd, pci_addr);
        if(config_virt == MAP_FAILED){
            perror("mmap in get_bar0");
            close(fd);
        }

        uint32_t bar_lower = *(uint32_t*)((uint8_t*)config_virt + 0x10);
        uint32_t bar_upper = *(uint32_t*)((uint8_t*)config_virt + 0x14);

        bar0 = ((uint64_t)bar_upper << 32) | (bar_lower & 0xfffffff0);
    }

    void map_ctrlreg(){
        int fd = open("/dev/mem", O_RDWR|O_SYNC);    
        if(fd == -1){
            perror("open in map_ctrlreg");
            exit(-1);
        }

        bar0_virt =  mmap(NULL, 8192, PROT_READ|PROT_WRITE, MAP_SHARED, fd, bar0);
        if(bar0_virt == MAP_FAILED){
            perror("mmap in map_ctrlreg");
            close(fd);
            exit(-1);
        }

        close(fd);
    }

    void* alloc_dma(size_t len, uint64_t &dma_addr){
        struct test_params param;
        pthread_mutex_lock(&alloc_dma_mutex);	    
        void* virt_addr = (unsigned int *)mmap(0, len, PROT_READ | PROT_WRITE, MAP_SHARED| MAP_LOCKED, genpcifd, 0);

	    if (virt_addr == MAP_FAILED)	{
		    perror("mmap in alloc_dma");
		    exit(-1);
	    }

        int rc = ioctl(genpcifd, IOCTL_GET_MEMFINFO, &param);
        if(rc){
            perror("ioctl in alloc_dma");
            exit(-1);
        }

        dma_addr = param.m.dma_addr;
        pthread_mutex_unlock(&alloc_dma_mutex);
        return virt_addr;

    }

    void set_INTx(int vectornum){
        struct test_params params;
        
        vector_num = (vectornum > SUPPORT_VECTOR_NUM) ? SUPPORT_VECTOR_NUM : vectornum;

        for(int i=0; i< vector_num; ++i){

            efd[i] = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
            //ASSERT(efd >= 0, "efd init failed");
            if(efd[i] < 0){
                perror("eventfd in set_INTx");
                exit(-1);
            }

            epfd = epoll_create1(EPOLL_CLOEXEC);
            //ASSERT(epfd >= 0, "failed to create epoll fd");
            if(epfd < 0){
                perror("create1 in set_INTx");
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
                perror("epoll_ctl in set_INTx");
                exit(-1);
            }

            params.s.fd[i] = efd[i];

        }

        params.s.irq_mode = INTX;
        params.s.vectornum = vectornum;

        int rc = ioctl(genpcifd, IOCTL_SET_SIGNAL, &params);
        if(rc){
            perror("ioctl in set_INTx");
            exit(-1);
        }
        vector_num = params.s.vectornum;
        printf("vector_num: %d\n", vector_num);
    }

    void set_MSIX(int vectornum){
        struct test_params params;

        vector_num = (vectornum > SUPPORT_VECTOR_NUM) ? SUPPORT_VECTOR_NUM : vectornum;

        for(int i=0; i< vector_num; ++i){
            efd[i] = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
            //ASSERT(efd >= 0, "efd init failed");
            if(efd[i] < 0){
                perror("eventfd in set_MSIX");
                exit(-1);
            }

            epfd = epoll_create1(EPOLL_CLOEXEC);
            //ASSERT(epfd >= 0, "failed to create epoll fd");
            if(epfd < 0){
                perror("create1 in set_MSIX");
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
                perror("epoll_ctl in set_MSIX");
                exit(-1);
            }

            params.s.fd[i] = efd[i];
        
        }

        params.s.irq_mode = MSIX;
        params.s.vectornum = vector_num;

        int rc = ioctl(genpcifd, IOCTL_SET_SIGNAL, &params);
        if(rc){
            perror("ioctl in set_MSIX");
            exit(-1);
        }

        vector_num = params.s.vectornum;
        printf("vector_num: %d\n", vector_num);
    }

    void setup_ktime(){
        struct test_params params;
        int ret = posix_memalign(&jiffies_p, 4096, 4096);
        if(ret != 0){
            perror("posix_memalign in setup_ktime");
            exit(-1);
        }

	    params.k.buf = (__le64)jiffies_p;

	    int rc = ioctl(genpcifd, IOCTL_SETUP_KTIME, &params);
        if(rc){
            perror("ioctl in setup_ktime");
            exit(-1);
        }
    }

    uint64_t get_ktime(){
        struct test_params params = {0};
        int rc = ioctl(genpcifd, IOCTL_GET_KTIME, nullptr);
        if(rc){
            perror("ioctl in get_ktime");
            exit(-1);
        }

        return *(uint64_t*)jiffies_p;
    }

    void allocate_stats_buffer(){
        struct test_params params = {0};
        int ret = posix_memalign(&stats_buf, 4096, 4096);
        if(ret != 0){
            perror("posix_memalign in create_dummy_blk");
            exit(-1);
        }
        params.a.buf = (__le64)stats_buf;

        int rc = ioctl(genpcifd, IOCTL_SEUTP_STATS_BUFFER, &params);
        if(rc){
              perror("ioctl in allocate_stats_buffer");
              exit(-1);
        }
    }

    void create_dummy_blk(int num){
        struct test_params params = {0};
        params.d.number = num;
        int rc = ioctl(genpcifd, IOCTL_ALLOC_DUMMYBLK, &params);
        if(rc){
            perror("ioctl in create_dummy_blk");
            exit(-1);
        }
    }

    void record_stats(int id, enum iotype iotype, unsigned int bytes, unsigned long start_time_ns){
        struct stats *stats = (struct stats *)stats_buf;
        stats->id = id;
        stats->iotype = iotype;
        stats->bytes = bytes;
        stats->start_time_ns = start_time_ns;

        int rc = ioctl(genpcifd, IOCTL_RECORD_STATS, nullptr);
        if(rc){
            perror("ioctl in record_stats");
            exit(-1);
        }
    }

private:

    pthread_mutex_t	alloc_dma_mutex;

    char devname[20] = {0};
    int instance_no;
    int genpcifd;
    uint16_t busn; 
    uint16_t devn;
    uint16_t funcn;

    uint64_t pci_addr;
    uint64_t bar0;
    void*   jiffies_p; 

    void*  stats_buf;  

protected:
    void*   bar0_virt;
    void*   config_virt;
    int     efd[SUPPORT_VECTOR_NUM];
    int     epfd;
    int     vector_num;
};

class NVMeCtrl: public GenPci{
public:

    NVMeCtrl(int instance_no):GenPci(instance_no){
        ctrl_reg = (nvme_controller_reg_t*)bar0_virt;
        init_ctrl();
        
        //set_MSIX(1);
        //kick_msix_intr_thread();

        set_INTx(1);
        kick_intx_intr_thread();

        //kick_polling_thread();

        allocate_stats_buffer();
        create_dummy_blk(0);
        setup_ktime();
        //create_dummy_blk(5);
        
        pthread_mutex_init(&cq_mutex, NULL);

    }

    ~NVMeCtrl(){
        pthread_mutex_destroy(&cq_mutex);
    }

    void* msix_handler(void){

        struct epoll_event evs;
        u64 u;

        for (;;) {
            printf("waiting interrupts...\n");
            // blocking wait
            int rc = epoll_wait(epfd, &evs, 1, -1);
            if(rc <= 0){
                perror("epoll_wait");
                exit(1);
            }

            ssize_t s = read(evs.data.fd, &u, sizeof(u));
            if(s != sizeof(u)){
                printf("efd read failed\n");
            }

            for(int i=0; i<vector_num; ++i){
                if (evs.data.fd == efd[i]) {
                    if(i == 0){
                        //printf("MSIx vector 0 interrupt ocuured\n");      
                        if (cq[0].entry[cq[0].head].u.a.p == cq[0].phase) {
				            while (cq[0].entry[cq[0].head].u.a.p == cq[0].phase) {
					            printf("MSIX Interrupt Occured\n");
                                std::cout << get_ktime() << std::endl;

					            //int head = p->admin_cq_head;
					            if (++cq[0].head == cq[0].size) {
					            	cq[0].head = 0;
					            	cq[0].phase = !cq[0].phase;
					            }
					            *(volatile u32*)(cq[0].doorbell) = cq[0].head;
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
            perror("pthread create\n");
            exit(-1);
        }
        if (pthread_detach(intr_th) != 0) {
            perror("pthread detach\n");
            exit(-1);
        }
    }

    void* intx_handler(void){
        struct epoll_event evs;
        u64 u;

        for (;;) {
            printf("waiting interrupts...\n");
            int rc = epoll_wait(epfd, &evs, 1, -1);
            if(rc <= 0){
                perror("epoll_wait");
                exit(1);
            }

            ssize_t s = read(evs.data.fd, &u, sizeof(u));
            if(s != sizeof(u)){
                perror("efd read");
            }

            for(int i=0; i<vector_num; ++i){
                if (evs.data.fd == efd[i]) {
                    if (cq[0].entry[cq[0].head].u.a.p == cq[0].phase) {
			            while (cq[0].entry[cq[0].head].u.a.p == cq[0].phase) {
			    	        printf("INT-X Interrupt Occured\n");
                            std::cout << get_ktime() << std::endl;

			    	        if (++cq[0].head == cq[0].size) {
			    	        	cq[0].head = 0;
			    	        	cq[0].phase = !cq[0].phase;
			    	        }
			    	        *(volatile u32*)(cq[0].doorbell) = cq[0].head;
			    	    }
                    }
                }
            }
        }
    }

    static void* intx_handler_wrapper(void* p){
        return ((NVMeCtrl*)p) -> intx_handler();
    }

    void kick_intx_intr_thread(){
        if (pthread_create(&intr_th, NULL, intx_handler_wrapper, this) != 0) {
            perror("pthread create");
            exit(-1);
        }
        if (pthread_detach(intr_th) != 0) {
            perror("pthread detach");
            exit(-1);
        }
    }

    void* polling_handler(void){
        for (;;) {
            if (cq[0].entry[cq[0].head].u.a.p == cq[0].phase) {
			    while (cq[0].entry[cq[0].head].u.a.p == cq[0].phase) {
			        printf("receive response\n");
                    std::cout << get_ktime() << std::endl;
			        if (++cq[0].head == cq[0].size) {
			        	cq[0].head = 0;
			        	cq[0].phase = !cq[0].phase;
			        }
			        *(volatile u32*)(cq[0].doorbell) = cq[0].head;
			    }
            }
            usleep(1);         
        }
    }

    static void* polling_handler_wrapper(void* p){
        return ((NVMeCtrl*)p) -> polling_handler();
    }

    void kick_polling_thread(){
        if (pthread_create(&intr_th, NULL, polling_handler_wrapper, this) != 0) {
            perror("pthread create");
            exit(-1);
        }
        if (pthread_detach(intr_th) != 0) {
            perror("pthread detach");
            exit(-1);
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
        sq[0].tail = 0;
	    cq[0].head = 0;
	    cq[0].phase = 1;

	    cq[0].size = cq_depth;
	    sq[0].size = sq_depth;
        
        ctrl_reg->aqa.a.acqs = cq[0].size -1;
        ctrl_reg->aqa.a.asqs = sq[0].size -1;

        cq[0].entry = (nvme_cq_entry_t*)alloc_dma(sizeof(nvme_cq_entry_t) * cq[0].size, cq[0].dma_addr);
        if(cq[0].entry == nullptr){
            std::cerr << "cannot allocate admin cq" << std::endl;
            exit(1);
        }
        
        sq[0].entry = (nvme_sq_entry_t*)alloc_dma(sizeof(nvme_sq_entry_t) * sq[0].size, sq[0].dma_addr);
        if(sq[0].entry == nullptr){
            std::cerr << "cannot allocate admin sq" << std::endl;
            exit(1);
        }

        ctrl_reg->acq = cq[0].dma_addr;
	    ctrl_reg->asq = sq[0].dma_addr;

        sq[0].doorbell = &ctrl_reg->sq0tdbl[0];
	    cq[0].doorbell = &ctrl_reg->sq0tdbl[0] + (1 << ctrl_reg->cap.a.dstrd);

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


    void issueCommand(nvme_sq_entry_t *entry, int data_len){
        int cid = sq[0].tail;
        uint64_t data_addr;

        if(data_len){
            void* datap = alloc_dma(data_len, data_addr);
            if(datap == nullptr){
                std::cerr << "cannot allocate data buffer" << std::endl;
                exit(-1);
            }
            entry->common.prp1 = data_addr;
        }

        entry->common.command_id = cid;
        memcpy(&sq[0].entry[cid], entry, sizeof(nvme_sq_entry_t));
        
        if (++sq[0].tail == sq[0].size) sq[0].tail = 0;
		*(volatile u32*)sq[0].doorbell = sq[0].tail;

    }

private:
    pthread_mutex_t	cq_mutex;

    pthread_t intr_th;
    nvme_controller_reg_t* ctrl_reg;

    typedef struct sq{
        int              size;
	    int              tail;
        uint64_t         dma_addr;
	    nvme_sq_entry_t* entry;
        void*            doorbell;
    }SQ_t;

    typedef struct cq{
        int              size;
        int              head;
    	int              phase;
        uint64_t         dma_addr;
        nvme_cq_entry_t* entry;
        void*            doorbell;    
    }CQ_t;

    SQ_t sq[32];
    CQ_t cq[32];

};

void issue_identify_ctrl(std::shared_ptr<NVMeCtrl> ctrl, nvme_sq_entry_t *entry){
	entry->identify.cns = NVME_ID_CNS_CTRL;
	entry->identify.nsid = 0;

    ctrl->issueCommand(entry, 4096);
}

int main(){
    std::shared_ptr<NVMeCtrl> ctrl = std::make_shared<NVMeCtrl>(0);
    sleep(1);   
    nvme_sq_entry_t entry = {0};
    std::cout << ctrl ->get_ktime() << std::endl;

    issue_identify_ctrl(ctrl, &entry);

     ctrl ->record_stats(0, iotype::Read, 4096 , ctrl ->get_ktime() );
    sleep(5);   
}