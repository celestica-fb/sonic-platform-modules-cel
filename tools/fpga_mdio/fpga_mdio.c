/********************************************************************
Author: Maxwill Ma <mamax@celestica.com>
    fpga_mdio

    user-space appliction to access fpga mdio related registers.

    As a "root" previledge, this program can run well
    while other user group would report system errors under Linux OS.

*********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <fcntl.h>      //for open()
#include <unistd.h>     //for close()
#include <string.h>
#include <stdint.h>

#define DEV_CHAR_FILENAME "/dev/fwupgrade"

struct fpga_reg_data {
    uint32_t phy;// FPGA phy address
    uint32_t reg;//FPGA register address
    uint32_t value;//FPGA register value
};

enum{
    READREG,
    WRITEREG,
} reg_data;

#define MDIO_MAGIC      'k'
#define MDIO0_READREG      _IOR(MDIO_MAGIC, 0x80, reg_data) & 0xffff
#define MDIO0_WRITEREG     _IOW(MDIO_MAGIC, 0x81, reg_data) & 0xffff
#define MDIO0_REGTEST      _IO(MDIO_MAGIC,  0x82) & 0xffff

unsigned int mdio_write(uint32_t phy_addr, uint32_t reg_addr, uint32_t reg_value)
{
    int fd;
    int ret = 0;
    struct fpga_reg_data fpga_reg;

    fd = open(DEV_CHAR_FILENAME, O_RDWR);

    fpga_reg.phy = phy_addr;
    fpga_reg.reg = reg_addr;
    fpga_reg.value = reg_value;

    ret = ioctl(fd, MDIO0_WRITEREG, (void *)&fpga_reg);
    if (ret < 0)
       printf("write 0x%x to reg[0x%x] fail!\n", reg_value, reg_addr);
    close(fd);
    return ret;
}

unsigned int mdio_read(uint32_t phy_addr, uint32_t reg_addr, uint32_t * ret_val)
{
    int fd = 0;
    int ret = 0;

    struct fpga_reg_data fpga_reg;

    fd = open(DEV_CHAR_FILENAME, O_RDWR);

    if (fd < 0) {
       printf("mdio_read: open /dev/fwupgrade failed! fd=%d\n", fd);
       exit(-1);
    }

    fpga_reg.phy = phy_addr;
    fpga_reg.reg = reg_addr;

    ret = ioctl(fd, MDIO0_READREG, (void *)&fpga_reg);
    if (ret < 0) {
       printf("read reg[0x%x] fail! return error code.\n", fpga_reg.reg);
       close(fd);
       return ret;
    }
    close(fd);

    *ret_val = fpga_reg.value;
    return 0;
}

unsigned int mdio_regtest(void)
{
    int fd;
    int ret = 0;

    struct fpga_reg_data fpga_reg;

    fd = open(DEV_CHAR_FILENAME, O_RDWR);

    fpga_reg.reg = 0x580034c0;

    ret = ioctl(fd, MDIO0_REGTEST, (void *)&fpga_reg);
    if (ret < 0) {
       printf("register test fail! return error code.\n");
       close(fd);
       return ret;
    }
    close(fd);
    return fpga_reg.value;
}
void show_usage(void)
{ printf("command usage:\n");
  printf("      fpga_mdio read XXX(reg_address in decimal)\n");
  printf("      fpga_mdio write XXX(reg_address in decimal) XXX(reg_value in decimal)\n");
  printf("      fpga_mdio regtest\n");
} 

int main(int argc, char **argv)
{
    uint32_t reg_val = 0, reg_addr = 0, phy_addr = 0;

    printf(" FPGA MDIO version 0.0.1 \n");
    printf(" build date : %s %s\n",__DATE__,__TIME__);

   /* 
    *analyse option and warn if something inappropriate
    *
   */
    if (argc != 2 && argc != 3 && argc != 4) {
        show_usage();
        return -5;
    }

    if (!strcmp(argv[1], "read")) {
        if (argc != 3) {
            show_usage();
            return -5;
        } else {
                 reg_addr = atoi(argv[2]);
                 printf("read reg[0x%x]\n", reg_addr);
                 mdio_read(phy_addr, reg_addr, &reg_val);
                 printf("reg[0x%x]=0x%x\n", reg_addr, reg_val);
             }
    } else if (!strcmp(argv[1], "write")) {
                if (argc != 4) {
                     show_usage();
                     return -5;
                } else {
                         reg_addr = atoi(argv[2]);
                         reg_val = atoi(argv[3]);
                         printf("write 0x%x to reg[0x%x]\n", reg_val, reg_addr);
                         mdio_write(phy_addr, reg_addr, reg_val);
                     }
           } else if (!strcmp(argv[1], "regtest")) {
                     if (argc != 2) {
                        show_usage();
                        return -5;
                     } else {
                             printf("register test \n");
                             reg_val = mdio_regtest();
                             printf("reg[0x%x]=0x%x\n", 0x580034c0, reg_val);
                          }
                   }
}
