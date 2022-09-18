
/*
 * file encoding utf-8
 * 
 * modbus_rtu.c
 *
 * Copyright (C) 2021 Inker.Dong
 *
 * Changelog:
 *
 *   2021-08-06 by Inker.Dong
 *   First create.
 *
 */

#include <stdbool.h>

#include "main.h"
#include "crc.h"
#include "modbus_reg.h"
#include "modbus_rtu.h"

#if ( MODBUS_RTU_MODULE_EN == 1 )

/************************************************************************/
/*      Print Message                                                   */
/************************************************************************/
#define MODULE_PRINT_INFO_EN // 屏蔽不定义 关闭此模块打印信息
#ifdef MODULE_PRINT_INFO_EN

#include "myprint.h"  /* 其他地方myprint.h不可出现此之前 */
#ifdef PRINTF_INFO_EN
static int prn_level = 0; /* 信息打印等级 */
#endif

#else

#define PRINT(fmt, ...)             ( (void)0 )
#define PRN_ERR(fmt, ...)           ( (void)0 )
#define PRN_HEXS(a,b)               ( (void)0 )
#define PRN_LEVEL(a,b,fmt, ...)     ( (void)0 )

#endif
/************************************************************************/

#define MODBUS_COM_NUM 2

#define TXD_BUF_SIZE 256
#define RXD_BUF_SIZE 261
struct modbus_rtu_cfg_s {
    unsigned int modbus_device_id;
    unsigned int byte_over_time;
    unsigned short (*p_modbus_crc16)(char*,unsigned short);
    int (*p_modbug_reg_0X03_func)(unsigned short,unsigned short,char *);
    int (*p_modbug_reg_0X06_func)(unsigned short,char *);
    int (*p_modbug_reg_0X10_func)(unsigned short,unsigned short,char *);
    void (*p_rtu_txd_func0)(char);
    void (*p_rtu_txd_func1)(char);
};

const struct modbus_rtu_cfg_s modbus_rtu_cfg = {
    0X64,
       5, // byte over time 10ms*n
    modbus_crc16,
    modbus_reg_0X03,
    modbus_reg_0X06,
    modbus_reg_0X10,
    put_txd1_queue,
    put_txd2_queue,
};

enum{
    MODBUS_RTU_ID,
    MODBUS_RTU_FUNCTION,
    MODBUS_RTU_0X03,
    MODBUS_RTU_0X06,
    MODBUS_RTU_0X10,
    MODBUS_RTU_CRC,
};

struct com_rxd_state_s {
    unsigned char  rxd_buf[RXD_BUF_SIZE];// Addr_2 + num_2 + cnt_1 + 256 = 261
    unsigned short rxd_crc16;
    
    unsigned short rxd_buf_cnt;  /*接收数据计数*/
    unsigned short rxd_buf_len;  /*数据域长度*/
    unsigned short link_over_time;
    unsigned char  over_time;
    unsigned char  rxd_state;
    unsigned char  rxd_frame_done;
    unsigned char  link_state;
};

struct modbus_rtu_state_s {
    const struct modbus_rtu_cfg_s *p_cfg;
    unsigned char  debug; //

    char           txd_buf[TXD_BUF_SIZE];
    unsigned short calc_crc16;

    unsigned char  ignore_crc;
    unsigned char  com_num;

    struct com_rxd_state_s com_rxd_state[MODBUS_COM_NUM];
};

struct modbus_rtu_state_s modbus_rtu_state = {
    .p_cfg          = &modbus_rtu_cfg,
};

void modbus_rtu_txd(char buf[],unsigned char len)
{
    struct modbus_rtu_state_s *p_sta = &modbus_rtu_state;
    struct com_rxd_state_s *p_com;
    unsigned short crc16;

    p_com = p_sta->com_rxd_state + p_sta->com_num;
    crc16 = p_sta->p_cfg->p_modbus_crc16(p_sta->txd_buf,len);
    p_sta->txd_buf[len++] =  crc16&0XFF;
    p_sta->txd_buf[len++] = (crc16>>8)&0XFF;

#ifdef MODULE_PRINT_INFO_EN
    if (prn_level >= 1) {
        PRINT("modbus txd buf\r\n");
        PRN_HEXS(p_sta->txd_buf,len);
    }
#endif
    if (p_sta->com_num == 0) {
        for (int i=0;i<len;i++) {
            p_sta->p_cfg->p_rtu_txd_func0(p_sta->txd_buf[i]);
        }
    }
    if (p_sta->com_num == 1){
        for (int i=0;i<len;i++) {
            p_sta->p_cfg->p_rtu_txd_func1(p_sta->txd_buf[i]);
        }
    }
}

void modbus_rtu_read_hold_reg(char rxd_buf[])
{
    struct modbus_rtu_state_s *p_sta = &modbus_rtu_state;
    unsigned short addr,num;

    addr   = rxd_buf[0];
    addr <<= 8;
    addr  |= rxd_buf[1];

    num    = rxd_buf[2];
    num  <<= 8;
    num   |= rxd_buf[3];

    if (num > 128) {
        return ;
    }
    if (p_sta->p_cfg->p_modbug_reg_0X03_func(addr,num,p_sta->txd_buf + 3) == true) {
        p_sta->txd_buf[0] = p_sta->p_cfg->modbus_device_id;
        p_sta->txd_buf[1] = 0X03;
        p_sta->txd_buf[2] = num * 2;
        modbus_rtu_txd(p_sta->txd_buf,num * 2 + 3);
    }
}

void modbus_rtu_write_hold_reg(char rxd_buf[])
{
    struct modbus_rtu_state_s *p_sta = &modbus_rtu_state;
    unsigned short addr,value;

    addr    = rxd_buf[0];
    addr  <<= 8;
    addr   |= rxd_buf[1];

    value   = rxd_buf[2];
    value <<= 8;
    value  |= rxd_buf[3];

    if (p_sta->p_cfg->p_modbug_reg_0X06_func(addr,rxd_buf+2) == true) {
        p_sta->txd_buf[0] = p_sta->p_cfg->modbus_device_id;
        p_sta->txd_buf[1] = 0X06;
        p_sta->txd_buf[2] = (addr>>8)&0XFF;
        p_sta->txd_buf[3] =  addr&0XFF;
        p_sta->txd_buf[4] = (value>>8)&0XFF;
        p_sta->txd_buf[5] =  value&0XFF;
        modbus_rtu_txd(p_sta->txd_buf,6);
    }
}

void modbus_rtu_write_hold_regs(char rxd_buf[])
{
    struct modbus_rtu_state_s *p_sta = &modbus_rtu_state;
    unsigned short addr,num;

    addr   = rxd_buf[0];
    addr <<= 8;
    addr  |= rxd_buf[1];

    num    = rxd_buf[2];
    num  <<= 8;
    num   |= rxd_buf[3];

    //byte_cnt = p_sta->buf[4];

    if (num > 128) {
        return ;
    }

    if (p_sta->p_cfg->p_modbug_reg_0X10_func(addr,num,rxd_buf + 5) == true) {
        p_sta->txd_buf[0] = p_sta->p_cfg->modbus_device_id;
        p_sta->txd_buf[1] = 0X10;
        p_sta->txd_buf[2] = (addr>>8)&0XFF;
        p_sta->txd_buf[3] =  addr&0XFF;
        p_sta->txd_buf[4] = (num>>8)&0XFF;
        p_sta->txd_buf[5] =  num&0XFF;
        modbus_rtu_txd(p_sta->txd_buf,6);
    }
}

void modbus_rtu_analyse()
{
    struct modbus_rtu_state_s *p_sta = &modbus_rtu_state;
    struct com_rxd_state_s *p_com;

    p_com = p_sta->com_rxd_state + p_sta->com_num;
    switch(p_com->rxd_buf[1]){
        case 0X03:
            modbus_rtu_read_hold_reg(p_com->rxd_buf+2); // 剔除0:ID 1:FUNCTION
            break;
        case 0X06:
            modbus_rtu_write_hold_reg(p_com->rxd_buf+2);
            break;
        case 0X10:
            modbus_rtu_write_hold_regs(p_com->rxd_buf+2);
            break;
    }
}
/************************************************************************/
/*     dbg_cmd Interface                                                */
/************************************************************************/
#include "dbg_cmd.h" // 屏蔽关闭此模块命令行调试
#ifdef DBG_CMD_EN
#define CMD_NAME "MbsRtu"
static void module_msg()
{
    struct modbus_rtu_state_s *p_sta = &modbus_rtu_state;

    DBG_CMD_PRN("ID:%02X BOT:%d\r\n",p_sta->p_cfg->modbus_device_id,p_sta->p_cfg->byte_over_time);
    DBG_CMD_PRN("MbsRtuDebug:%d\r\n", p_sta->debug);
    DBG_CMD_PRN("MbsRtu ignore Crc:%d\r\n", p_sta->ignore_crc);

    for(int i = 0; i < MODBUS_COM_NUM; i++) {
        DBG_CMD_PRN("MbsRtu link state:%d\r\n", p_sta->com_rxd_state[i].link_state);
    }
}

static bool dbg_cmd_func()
{
    struct modbus_rtu_state_s *p_sta = &modbus_rtu_state;

    if (dbg_cmd_exec("help", "", "")) {
        DBG_CMD_PRN("."CMD_NAME"\r\n");
        return false;
    }
    if (dbg_cmd_exec("."CMD_NAME, "", "")) {
        dbg_cmd_print_msg_en();
    }
    if (dbg_cmd_exec(CMD_NAME"Msg", "", "")) {
#ifdef MODULE_PRINT_INFO_EN
        DBG_CMD_PRN("PrnLevel  :%d\r\n", prn_level);
#endif
        module_msg();
        return true;
    }
#ifdef MODULE_PRINT_INFO_EN
    if (dbg_cmd_exec(CMD_NAME"Prf", "1", "<0~1> Set Prn Level")) {
        prn_level = get_param_char(0);
        return true;
    }
#endif
    if (dbg_cmd_exec(CMD_NAME"debug", "1", "<0~1> debug mode")) {
        p_sta->debug = get_param_char(0);
        return true;
    }
    if (dbg_cmd_exec(CMD_NAME"crc", "1", "<0~1> ignore crc")) {
        p_sta->ignore_crc = get_param_char(0);
        return true;
    }
    return false;
}
#endif
/************************************************************************/
/*      Application Interface                                           */
/************************************************************************/
void modbus_rtu_rxd_thread_isr(unsigned char com, char rxd)
{
    struct modbus_rtu_state_s *p_sta = &modbus_rtu_state;
    struct com_rxd_state_s *p_com;

    if (p_sta->debug) {
        return ;
    }

    p_com = p_sta->com_rxd_state + com;

    if (p_com->over_time == 0) {
        /* 接受字节超时复位 */
        p_com->rxd_state     = MODBUS_RTU_ID;
    }
    p_com->over_time = p_sta->p_cfg->byte_over_time;

    switch(p_com->rxd_state){
        case MODBUS_RTU_ID:
            if (rxd == p_sta->p_cfg->modbus_device_id) {
                p_com->rxd_buf[0] = rxd;
                p_com->rxd_state  = MODBUS_RTU_FUNCTION;
            }
            break;
        case MODBUS_RTU_FUNCTION:
            p_com->rxd_state   = MODBUS_RTU_ID;
            p_com->rxd_buf[1]  = rxd;
            p_com->rxd_buf_cnt = 2;// 0:id 1:func
            if (rxd == 0X03) {
                p_com->rxd_state = MODBUS_RTU_0X03;
            }
            if (rxd == 0X06) {
                p_com->rxd_state = MODBUS_RTU_0X06;
            }
            if (rxd == 0X10) {
                p_com->rxd_state = MODBUS_RTU_0X10;
            }
            break;
        case MODBUS_RTU_0X03:
        case MODBUS_RTU_0X06:
            p_com->rxd_buf[p_com->rxd_buf_cnt] = rxd;
            p_com->rxd_buf_cnt++;
            if (p_com->rxd_buf_cnt >= (4 + 2)) {
                p_com->rxd_buf_len = p_com->rxd_buf_cnt;
                p_com->rxd_buf_cnt = 0;
                p_com->rxd_state   = MODBUS_RTU_CRC;
            }
            break;
        case MODBUS_RTU_0X10:
            p_com->rxd_buf[p_com->rxd_buf_cnt] = rxd;
            p_com->rxd_buf_cnt++;
            if (p_com->rxd_buf_cnt >= RXD_BUF_SIZE) {
                p_com->rxd_state = MODBUS_RTU_ID; // 接收超缓存 复位
            } else if ((p_com->rxd_buf_cnt >= (5+2)) && (p_com->rxd_buf_cnt >= p_com->rxd_buf[6]+ (5+2))) {
                p_com->rxd_buf_len = p_com->rxd_buf_cnt;
                p_com->rxd_buf_cnt = 0;
                p_com->rxd_state   = MODBUS_RTU_CRC;
            }
            break;
        case MODBUS_RTU_CRC:
            p_com->rxd_crc16 >>= 8;
            p_com->rxd_crc16  |= rxd<<8;
            p_com->rxd_buf_cnt++;
            if (p_com->rxd_buf_cnt >= 2) {
                p_com->rxd_frame_done   = 1;
                p_com->rxd_state   = MODBUS_RTU_ID;
            }
            break;
    }
}

unsigned char modbus_rtu_link_state(unsigned char com)
{
    struct modbus_rtu_state_s *p_sta = &modbus_rtu_state;

    if (com < MODBUS_COM_NUM) {
        return p_sta->com_rxd_state[com].link_state;
    }
    return 0;
}

void modbus_rtu_10ms_thread_isr()
{
    struct modbus_rtu_state_s *p_sta = &modbus_rtu_state;

    for(int i = 0; i < MODBUS_COM_NUM; i++) {
        /* rxd byte and byte over time cnt */
        if(p_sta->com_rxd_state[i].over_time) {
            p_sta->com_rxd_state[i].over_time--;
        }

        /* link over time cnt */
        if(p_sta->com_rxd_state[i].link_over_time) {
            p_sta->com_rxd_state[i].link_over_time--;
            if (p_sta->com_rxd_state[i].link_over_time == 0) {
                p_sta->com_rxd_state[i].link_state = 0;
            }
        }
    }
}

void modbus_rtu_real_time_thread()
{
    struct modbus_rtu_state_s *p_sta = &modbus_rtu_state;
    struct com_rxd_state_s *p_com;

    for(int i = 0; i < MODBUS_COM_NUM; i++) {
        if(p_sta->com_rxd_state[i].rxd_frame_done) {
            p_sta->com_rxd_state[i].rxd_frame_done = 0;
            p_sta->com_num = i;
            p_com = p_sta->com_rxd_state + p_sta->com_num;
            p_com->link_state = 1;
            p_com->link_over_time = 300; /* link over time 3S */
            p_sta->calc_crc16 = p_sta->p_cfg->p_modbus_crc16(p_com->rxd_buf,p_com->rxd_buf_len);
#ifdef MODULE_PRINT_INFO_EN
            PRN_LEVEL(prn_level,1,"\r\ncom:%d crc rxd:%04X==calc:%04X\r\n", p_sta->com_num, p_com->rxd_crc16,p_sta->calc_crc16);
            if (prn_level >= 1) {
                PRN_HEXS(p_com->rxd_buf,p_com->rxd_buf_len);
            }
#endif
            if((p_sta->ignore_crc != 0) || (p_com->rxd_crc16 == p_sta->calc_crc16)) {
                modbus_rtu_analyse();
            }
        }
    }
}

void modbus_rtu_init(unsigned char mode)
{
    struct modbus_rtu_state_s *p_sta = &modbus_rtu_state;

    p_sta->ignore_crc = mode;

    modbus_reg_init();
#ifdef DBG_CMD_EN
    dbg_cmd_add_list((int)dbg_cmd_func);
#endif // DBG_CMD_EN
}
#endif // MODBUS_RTU_MODULE_EN

