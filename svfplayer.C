#include "libsvfplayer.H"
#include <stdio.h>
#include <string>
#include <vector>
#include <string.h>
#include <limits.h>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <assert.h>
#include <poll.h>


using namespace std;



int lineNum=0;

//##########################################################################################
/***************** hardware accessing functions *****************/
//##########################################################################################


typedef unsigned long ul;
typedef unsigned int ui;
typedef uint32_t u32;
typedef uint32_t u16;
typedef uint64_t u64;
typedef uint8_t u8;

volatile u16* gpioIn=NULL;
volatile u16* gpioOut=NULL;
u64 ioDelayCycles=0;


//============== soc
#define SOCFPGA_H2F_BASE (0xC0000000) // axi_master
#define SOCFPGA_H2F_SPAN (0x40000000) // Bridge span
#define SOCFPGA_HW_REGS_BASE ( 0xFC000000 )     //misc. registers
#define SOCFPGA_HW_REGS_SPAN ( 0x04000000 )
#define SOCFPGA_LWH2F_OFFSET 52428800
#define SOCFPGA_GPIOOUT 0x0
#define SOCFPGA_GPIOIN 0x10

#define SOCFPGA_GPIOTCK 10
#define SOCFPGA_GPIOTMS 11
#define SOCFPGA_GPIOTDI 12
#define SOCFPGA_GPIOTDO 13

//============== orange pi zero
#define OPI_GPIO_BASE_BP	(0x01C20000)
#define OPI_GPIO_OFFSET (0x0800)

// use the gpio numbers listed in the "H2+" column of wiringOP "gpio readall" output
// ALL GPIOS MUST BE FROM THE SAME BANK!!!
// bank number is gpio number / 32
u32 OPI_GPIOTCK=14;
u32 OPI_GPIOTMS=16;
u32 OPI_GPIOTDI=13;
u32 OPI_GPIOTDO=2;
u32 OPI_GPIOMASK = -1;

volatile u32* opiGpio=NULL;

//============== usb
int ttydevice=-1;
int bitrepeat=1;
static const u8 usbtck=1<<0;
static const u8 usbtms=1<<1;
static const u8 usbtdi=1<<2;
static const u8 usbtdo=1<<3;
static const u8 usbsmp=1<<7;

int readAll(int fd,void* buf, int len) {
	u8* buf1=(u8*)buf;
	int off=0;
	int r;
	while(off<len) {
		if((r=read(fd,buf1+off,len-off))<=0) break;
		off+=r;
	}
	return off;
}

void doDelay(u64 cycles) {
	for(u64 i=0;i<cycles;i++) asm volatile("");
}
u64 tsToNs(const struct timespec& ts) {
	return u64(ts.tv_sec)*1000000000+ts.tv_nsec;
}
//returns time in ns
u64 measureTime(u64 cycles) {
	struct timespec t,t2;
	clock_gettime(CLOCK_MONOTONIC,&t);
	for(int i=0;i<50;i++) {
		doDelay(cycles); doDelay(cycles); doDelay(cycles); doDelay(cycles);
		doDelay(cycles); doDelay(cycles); doDelay(cycles); doDelay(cycles);
		doDelay(cycles); doDelay(cycles); doDelay(cycles); doDelay(cycles);
		doDelay(cycles); doDelay(cycles); doDelay(cycles); doDelay(cycles);
		doDelay(cycles); doDelay(cycles); doDelay(cycles); doDelay(cycles);
		doDelay(cycles); doDelay(cycles); doDelay(cycles); doDelay(cycles);
		doDelay(cycles); doDelay(cycles); doDelay(cycles); doDelay(cycles);
		doDelay(cycles); doDelay(cycles); doDelay(cycles); doDelay(cycles);
	}
	clock_gettime(CLOCK_MONOTONIC,&t2);
	return (tsToNs(t2)-tsToNs(t))/(32*50);
}
u64 measureTime2(u64 cycles) {
	u64 min=measureTime(cycles);
	u64 tmp;
	for(int i=0;i<10;i++)
		if((tmp=measureTime(cycles))<min) min=tmp;
	return min;
}
//desiredDelay is in ns
//returns cycles to be passed to doDelay()
u64 calibrateDelay(u64 desiredDelay=1000) {
	fprintf(stderr,"calibrating delay loop... ");
	fflush(stderr);
	
	u64 d=2;
	while(true) {
		if(measureTime2(d)>desiredDelay) break;
		d*=2;
	}
	fprintf(stderr,"d=%llu\n",(unsigned long long)d);
	//binary search
	u64 min=d/2,max=d;
	for(int i=0;i<32;i++) {
		u64 mid=(min+max)/2;
		if(measureTime2(mid)>desiredDelay) {
			//too long
			max=mid;
		} else {
			min=mid;
		}
	}
	fprintf(stderr,"done; %llu ns = %llu cycles\n",
		(unsigned long long)measureTime(max),(unsigned long long)max);
	return max;
}
void executeCommands_soc(const string& buf) {
	//struct timespec ts;
	//ts.tv_sec=0;
	//ts.tv_nsec=1000;
	for(int i=0;i<(int)buf.length();i++) {
		uchar cmd=(uchar)buf[i];
		bool tms=cmd&(1);
		bool tdi=cmd&(1<<1);
		bool tdo=cmd&(1<<2);
		bool tdiEnable=cmd&(1<<3);
		bool tdoEnable=cmd&(1<<4);
		
		//if(tdiEnable) printf("tdi=%d ",(int)tdi);
		//if(tdoEnable) printf("tdo=%d ",(int)tdo);
		//printf("tms=%d\n",(int)tms);
		
		//put data on the line and set tck low
		*gpioOut=(tms<<SOCFPGA_GPIOTMS)|(tdi<<SOCFPGA_GPIOTDI);
		//wait
		doDelay(ioDelayCycles);
		//assert tck and sample tdo, while holding other pins constant
		*gpioOut=(tms<<SOCFPGA_GPIOTMS)|(tdi<<SOCFPGA_GPIOTDI)|(1<<SOCFPGA_GPIOTCK);
		doDelay(ioDelayCycles);
		bool receivedTdo=((*gpioIn)&(1<<SOCFPGA_GPIOTDO))!=0;
		if(tdoEnable && receivedTdo!=tdo) {
			fprintf(stderr,"line %d: error: tdo should be %d, but got %d\n",lineNum,(int)tdo,(int)receivedTdo);
			_exit(1);
		}
	}
}


void executeCommands_opi(const string& buf) {
	u32 mask1 = ((*opiGpio) & (~OPI_GPIOMASK));
	for(int i=0;i<(int)buf.length();i++) {
		uchar cmd=(uchar)buf[i];
		/*u32 tms=(cmd&(1))?1:0;
		u32 tdi=(cmd&(1<<1))?1:0;
		u32 tdo=(cmd&(1<<2))?1:0;
		u32 tdiEnable=(cmd&(1<<3))?1:0;
		u32 tdoEnable=(cmd&(1<<4))?1:0;*/
		bool tms=cmd&(1);
		bool tdi=cmd&(1<<1);
		bool tdo=cmd&(1<<2);
		bool tdiEnable=cmd&(1<<3);
		bool tdoEnable=cmd&(1<<4);
		
		//put data on the line and set tck low
		*opiGpio=mask1 | (tms<<OPI_GPIOTMS) | (tdi<<OPI_GPIOTDI) | (1<<OPI_GPIOTCK);
		doDelay(ioDelayCycles);
		*opiGpio=mask1 | (tms<<OPI_GPIOTMS) | (tdi<<OPI_GPIOTDI);
		
		doDelay(ioDelayCycles);
		//assert tck and sample tdo, while holding other pins constant
		*opiGpio = mask1 | (tms<<OPI_GPIOTMS) | (tdi<<OPI_GPIOTDI) | (1<<OPI_GPIOTCK);
		doDelay(ioDelayCycles);
		bool receivedTdo=((*opiGpio)&(1<<OPI_GPIOTDO))!=0;
		
		if(tdoEnable && receivedTdo!=tdo) {
			fprintf(stderr,"line %d: error: tdo should be %d, but got %d\n",lineNum,(int)tdo,(int)receivedTdo);
			_exit(1);
		}
	}
}

void writebuffer_serial(const u8* buf, int len, bool readback) {
	{
		u8 outbuf[len*2*bitrepeat];
		for(int i=0;i<len;i++) {
			u8 cmd=(u8)buf[i];
			bool tms=cmd&(1);
			bool tdi=cmd&(1<<1);
			bool tdo=cmd&(1<<2);
			bool tdiEnable=cmd&(1<<3);
			bool tdoEnable=cmd&(1<<4);
			
		
			//put data on the line and set tck low
			u8 out1=(tdi?usbtdi:0)|(tms?usbtms:0);
			//tck high
			u8 out2=out1|usbtck;
			
			for(int r=0;r<bitrepeat;r++) {
				outbuf[i*2*bitrepeat+r]=out1;
				outbuf[(i*2+1)*bitrepeat+r]=out2;
			}
			outbuf[i*2*bitrepeat+bitrepeat-1]=out1|usbsmp;
		}
		assert(write(ttydevice,outbuf,len*2*bitrepeat)==len*2*bitrepeat);
	}
	if(!readback) return;
	u8 inbuf[len];
	assert(readAll(ttydevice,inbuf,len)==len);
	for(int i=0;i<len;i++) {
		u8 cmd=(u8)buf[i];
		bool tms=cmd&(1);
		bool tdi=cmd&(1<<1);
		bool tdo=cmd&(1<<2);
		bool tdiEnable=cmd&(1<<3);
		bool tdoEnable=cmd&(1<<4);
		
		bool receivedTms=(inbuf[i]&usbtms);
		bool receivedTdi=(inbuf[i]&usbtdi);
		bool receivedTdo=(inbuf[i]&usbtdo);
		
		//fprintf(stderr,"tdi=%d%s ",(int)tdi,tdiEnable?"":"?");
		//fprintf(stderr,"tms=%d ",(int)tms);
		//fprintf(stderr,"tdo=%d%s ",(int)tdo,tdoEnable?"":"?");
		
		//fprintf(stderr,"\n");
		//fprintf(stderr,"%d\n",inbuf[i]);
		
		//assert(tms==receivedTms);
		//assert(tdi==receivedTdi);
		
		if(tdoEnable && tdo!=receivedTdo) {
			fprintf(stderr,"line %d: error: tdo should be %d, but got %d\n",lineNum,(int)tdo,(int)receivedTdo);
			
			//_exit(1);
		}
	}
}

// write the commands to a serial device
void executeCommands_serial(const string& buf, bool readback=true) {
	int maxbuf=4096;
	for(int i=0;i<(int)buf.length();i+=maxbuf) {
		int len=(int)buf.length()-i;
		if(len>maxbuf) len=maxbuf;
		writebuffer_serial((u8*)buf.data()+i,len,readback);
	}
}

void drainfd(int fd) {
	pollfd pfd;
	pfd.fd = fd;
	pfd.events = POLLIN;
	while(poll(&pfd,1,100)>0) {
		if(!(pfd.revents&POLLIN)) continue;
		char buf[4096];
		read(fd,buf,sizeof(buf));
	}
}


int main(int argc, char** argv) {
	if(argc<2) {
	print_usage:
		fprintf(stderr,"usage: %s (-s CLKPERIOD_NS)|(-u /dev/ttyXXX BITREPEAT)|(-r BITREPEAT)\n"
						"-s: SoC mode\n-u: usb mode\n-r: raw mode (output bitbang data to stdout)\n",argv[0]);
		return 1;
	}
	if(argv[1][0]!='-') goto print_usage;
	switch(argv[1][1]) {
		case 's':
		{
			if(argc<3) goto print_usage;
			int halfperiod=atoi(argv[2])/2;
			int fd;
			if((fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
				fprintf(stderr, "ERROR: could not open \"/dev/mem\"...\n" ); return 1;
			}
			
			//u8* h2f = (u8*)mmap( NULL, H2F_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, H2F_BASE);
			u8* hwreg = (u8*)mmap( NULL, SOCFPGA_HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, SOCFPGA_HW_REGS_BASE);
			gpioIn=(volatile u16*)(hwreg+SOCFPGA_LWH2F_OFFSET+SOCFPGA_GPIOIN);
			gpioOut=(volatile u16*)(hwreg+SOCFPGA_LWH2F_OFFSET+SOCFPGA_GPIOOUT);
			*gpioOut=1<<SOCFPGA_GPIOTCK;
			
			volatile u16* gpiooe=gpioOut+1;
			*gpiooe = u16(1<<SOCFPGA_GPIOTCK)|u16(1<<SOCFPGA_GPIOTMS)|u16(1<<SOCFPGA_GPIOTDI);		//enable write for jtag output pins
			
			ioDelayCycles=calibrateDelay(halfperiod);
			break;
		}
		case 'o':
		{
			if(argc<3) goto print_usage;
			int halfperiod=atoi(argv[2])/2;
			int fd;
			if((fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
				fprintf(stderr, "ERROR: could not open \"/dev/mem\"...\n" ); return 1;
			}
			
			u8* hwreg = (u8*)mmap( NULL, 4096, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, OPI_GPIO_BASE_BP);
			
			int bank = OPI_GPIOTCK >> 5;
			// make sure all gpios are on the same bank
			assert(bank == (OPI_GPIOTDI>>5));
			assert(bank == (OPI_GPIOTDO>>5));
			assert(bank == (OPI_GPIOTMS>>5));
			
			OPI_GPIOTCK -= bank;
			OPI_GPIOTDI -= bank;
			OPI_GPIOTDO -= bank;
			OPI_GPIOTMS -= bank;
			
			OPI_GPIOMASK = (1<<OPI_GPIOTCK)|(1<<OPI_GPIOTDI)|(1<<OPI_GPIOTDO)|(1<<OPI_GPIOTMS);
			
			opiGpio=(volatile u32*)(hwreg + OPI_GPIO_OFFSET + bank*36 + 0x10);
			int mask1 = (*opiGpio) & (~OPI_GPIOMASK);
			*opiGpio = mask1 | (1<<OPI_GPIOTCK);
			
			auto setGpioMode = [&](int pin, char mode) {
				volatile u32* opiGpioMode = (volatile u32*)(hwreg + OPI_GPIO_OFFSET + bank*36 + ((pin>>3)<<2));
				int offset = ((pin - ((pin >> 3) << 3)) << 2);
				
				(*opiGpioMode) &= ~(7 << offset);
				if(mode=='o') {
					(*opiGpioMode) |= (1 << offset);
				}
			};
			setGpioMode(OPI_GPIOTDO, 'i');
			setGpioMode(OPI_GPIOTCK, 'o');
			setGpioMode(OPI_GPIOTDI, 'o');
			setGpioMode(OPI_GPIOTMS, 'o');
			
			
			ioDelayCycles=calibrateDelay(halfperiod);
			break;
		}
		case 'u':
		{
			if(argc<4) goto print_usage;
			bitrepeat=atoi(argv[3]);
			int fd;
			if((fd = open(argv[2], O_RDWR)) < 0) {
				perror("open");
				fprintf(stderr, "ERROR: could not open %s\n", argv[2]);
				return 1;
			}
			/* Set TTY mode. */
			struct termios tc;
			if (tcgetattr(fd, &tc) < 0) {
				perror("tcgetattr");
			} else {
				tc.c_iflag &= ~(INLCR|IGNCR|ICRNL|IGNBRK|IUCLC|INPCK|ISTRIP|IXON|IXOFF|IXANY);
				tc.c_oflag &= ~OPOST;
				tc.c_cflag &= ~(CSIZE|CSTOPB|PARENB|PARODD|CRTSCTS);
				tc.c_cflag |= CS8 | CREAD | CLOCAL;
				tc.c_lflag &= ~(ICANON|ECHO|ECHOE|ECHOK|ECHONL|ISIG|IEXTEN);
				tc.c_cc[VMIN] = 1;
				tc.c_cc[VTIME] = 0;
				if (tcsetattr(fd, TCSANOW, &tc) < 0) {
					perror("tcsetattr");
				}
			}
			ttydevice=fd;
			
			u8 tmp=usbtck;
			write(fd,&tmp,1);
			
			//drain buffer
			drainfd(fd);
			
			tmp=usbsmp;
			write(fd,&tmp,1);
			assert(read(fd,&tmp,1)==1);
			assert((tmp&usbtck)==0);
			tmp=usbtck|usbsmp;
			write(fd,&tmp,1);
			assert(read(fd,&tmp,1)==1);
			assert((tmp&usbtck)!=0);
			
			break;
		}
		case 'r':
		{
			if(argc<3) goto print_usage;
			bitrepeat=atoi(argv[2]);
			int fd=1;
			ttydevice=fd;
			
			u8 tmp=usbtck;
			write(fd,&tmp,1);
			break;
		}
		default:
			goto print_usage;
	}
	
	
	
	
	svfParser parser;
	svfPlayer player;
	int cmds=0;
	int tclkCycles=0;
	
	parser.reset();
	player.reset();
	while(true) {
		char* line=NULL;
		size_t n;
		if(getline(&line,&n,stdin)<0) break;
		parser.processLine(line,strlen(line));
		//printf("line %d:\n",parser.lineNum);
		svfCommand cmd;
		while(parser.nextCommand(cmd)) {
			player.processCommand(cmd);
			cmds++;
		}
		lineNum=parser.lineNum;
		
		tclkCycles += player.outBuffer.length();
		if(argv[1][1]=='s')
			executeCommands_soc(player.outBuffer);
		else if(argv[1][1]=='o')
			executeCommands_opi(player.outBuffer);
		else executeCommands_serial(player.outBuffer, argv[1][1]=='u');
		
		//assert(write(1,player.outBuffer.data(), player.outBuffer.length())==player.outBuffer.length());
		
		player.outBuffer.clear();
		
		free(line);
	}
	fprintf(stderr,"%d commands executed successfully; %d tclk cycles total\n",cmds,tclkCycles);
	return 0;
}
