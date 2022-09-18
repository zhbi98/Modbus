
/*
 * file encoding utf-8
 * 
 * modbus_reg.c
 *
 * Copyright (C) 2021 Inker.Dong
 *
 * Changelog:
 *
 *   2021-08-09 by Inker.Dong
 *   First create.
 *
 */

#include "main.h"
#include "project.h"

#include "modbus_reg.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdbool.h>

#if ( MODBUS_REG_MODULE_EN == 1 )

#define SHORT_REG_MSG

//寄存器属性
#define REG_P_W  0X01 // 可写 
#define REG_P_E  0X02 // 掉电存储
#define REG_P_P  0X04 // 写保护

#define REG_P_RXXX (0)//只读 X无效属性
#define REG_P_RWXX (REG_P_W)
#define REG_P_RWXP (REG_P_W | REG_P_P)
#define REG_P_RWEX (REG_P_W | REG_P_E)
#define REG_P_RWEP (REG_P_W | REG_P_E | REG_P_P)


//寄存器类型
enum{
REG_U08,
REG_I16,    
REG_U16,
REG_I32,
REG_U32,
REG_F32,
REG_STR,
};

//寄存器框架
struct modbus_reg_tbl_s{
char               RegType;   // 变量类型
char               Property;  // 变量属性
#ifndef SHORT_REG_MSG
char               Load[8];   // 下载默认值
char               Dn[8];     // 变量下限 8所以最大有效位为7 有小数有效位为6
char               Up[8];     // 变量上限
#endif
char               Name[8];   // 寄存器名称
unsigned short     RegAdd;    // Modbus地址
long              *pAdd;      // 变量地址
char              *pUpData;   // 变量更新标记地址
}t_MbsReg;

//寄存器演示
typedef struct{
char            DemoU08;
char            DemoU08Flag;
unsigned short  DemoI16;
short           DemoU16;
long            DemoI32;
long            DemoU32;
float           DemoF32;
}tDemoMbs;

tDemoMbs DemoMbs;

/*
寄存器列表规则:
1 频繁访问的尽量放寄存器列表前面    因为查询寄存器是自上而下的
2 连续读写的寄存器尽量按顺序排一起  因为读写0X03 0X10命令结合预处理下一个寄存器可以提高效率
*/

//用户配置寄存器列表
const unsigned long MbsVers = 20220707;
/************************************************************************/
/*      Global Variables                                                */
/************************************************************************/
const struct modbus_reg_tbl_s modbus_reg_tbl[]={
#ifdef SHORT_REG_MSG
//type     aitt         name      address 
{REG_U16, REG_P_RWXX,"WifiSta", MBSREG_WIFI_STATE,  (long*)&PrjPkg.mbsreg_wifi_state,      NULL},
{REG_U16, REG_P_RWXX,"TstType", MBSREG_TEST_TYPE,   (long*)&PrjPkg.mbsreg_test_type,(char*)&PrjPkg.test_state_updata},
{REG_U16, REG_P_RWXX,"TstMode", MBSREG_TEST_MODE,   (long*)&PrjPkg.mbsreg_test_mode,(char*)&PrjPkg.test_state_updata},
{REG_U16, REG_P_RWXX,"UnitSta", MBSREG_TEST_UNIT,   (long*)&PrjPkg.mbsreg_test_unit,(char*)&PrjPkg.test_state_updata},
{REG_U16, REG_P_RWXX,"SumSta ", MBSREG_SUM_STATE,   (long*)&PrjPkg.mbsreg_sum_state,(char*)&PrjPkg.test_state_updata},
{REG_U16, REG_P_RWXX,"WifiCfg", MBSREG_WIFI_CFG,    (long*)&PrjPkg.mbsreg_wifi_cfg,  NULL},

{REG_F32, REG_P_RXXX,"Bat_Vol", MBSREG_BATTERY_VAL, (long*)&PrjPkg.batter_voltage,   NULL},
{REG_F32, REG_P_RXXX,"Sum_MAX", MBSREG_SUM_MAX_VAL, (long*)&PrjPkg.max_val,          NULL},
{REG_F32, REG_P_RXXX,"Sum_MIN", MBSREG_SUM_MIN_VAL, (long*)&PrjPkg.min_val,          NULL},
{REG_F32, REG_P_RXXX,"Sum_AVG", MBSREG_SUM_AVG_VAL, (long*)&PrjPkg.avg_val,          NULL},
{REG_F32, REG_P_RXXX,"WindVec", MBSREG_WIND_SPEED,  (long*)&PrjPkg.wind_speed_value, NULL},
{REG_F32, REG_P_RXXX,"Temper ", MBSREG_TEMPERATURE, (long*)&PrjPkg.temperature_value_ntc,NULL},
{REG_F32, REG_P_RXXX,"Humi   ", MBSREG_HUMIDITY,    (long*)&PrjPkg.humidity_value,   NULL},
{REG_F32, REG_P_RXXX,"DP     ", MBSREG_DEW_POINT,   (long*)&PrjPkg.dew_point_value,  NULL},
{REG_F32, REG_P_RXXX,"WB     ", MBSREG_WET_BULB,    (long*)&PrjPkg.wet_bulb_value,   NULL},
{REG_F32, REG_P_RXXX,"WCI    ", MBSREG_WIND_CHILL,  (long*)&PrjPkg.wind_chill_value, NULL},
{REG_F32, REG_P_RXXX,"Flow   ", MBSREG_AIR_FLOW,    (long*)&PrjPkg.air_flow_value,   NULL},

{REG_U32, REG_P_RWXX,"RtcDate", MBSREG_SYS_DATE,    (long*)&PrjPkg.rtc_date_modbus,(char*)&PrjPkg.rtc_date_time_updata},
{REG_U32, REG_P_RWXX,"RtcTime", MBSREG_SYS_TIME,    (long*)&PrjPkg.rtc_time_modbus,(char*)&PrjPkg.rtc_date_time_updata},

{REG_U16, REG_P_RWEX,"LogClr ", MBSREG_STORE_CLEAR, (long*)&PrjPkg.mbsreg_log_clear,         (char*)&PrjPkg.pc_set_updata},
{REG_U16, REG_P_RWEX,"LogNum ", MBSREG_STORE_NUM,   (long*)&PrjPkg.set_log_num,              (char*)&PrjPkg.pc_key_refresh_lcd},
{REG_U16, REG_P_RWEX,"LogRate", MBSREG_STORE_RATE,  (long*)&PrjPkg.set_auto_store_data_time, (char*)&PrjPkg.pc_key_refresh_lcd},
{REG_U16, REG_P_RWEX,"LogSta ", MBSREG_STORE_STATE, (long*)&PrjPkg.mbsreg_log_state,         (char*)&PrjPkg.pc_set_updata},
{REG_U16, REG_P_RWEX,"PowerDn", MBSREG_AUTO_POWER,  (long*)&PrjPkg.set_auto_show_down_time,  (char*)&PrjPkg.pc_key_refresh_lcd},
{REG_U16, REG_P_RWEX,"LcdBL  ", MBSREG_LCD_BL,      (long*)&PrjPkg.mbsreg_lcd_bl,            (char*)&PrjPkg.pc_set_updata},
{REG_U16, REG_P_RWEX,"WF link", MBSREG_WIFI_LINK,   (long*)&PrjPkg.mbsreg_wifi_linke,        (char*)&PrjPkg.pc_set_updata},

{REG_F32, REG_P_RWEX,"SetArea", MBSREG_SET_AREA,    (long*)&PrjPkg.air_flow_area,          NULL},

{REG_U16, REG_P_RXXX,"LogMAX ", MBSREG_LOG_MAX,     (long*)&PrjPkg.log_max_store_num,      NULL},
{REG_U16, REG_P_RXXX,"LogNUM ", MBSREG_LOG_NUM,     (long*)&PrjPkg.log_cur_store_num,      NULL},
{REG_U16, REG_P_RWXX,"LogPos ", MBSREG_LOG_POS,     (long*)&PrjPkg.log_read_store_pos,(char*)&PrjPkg.log_read_store_pos_updata},
{REG_U16, REG_P_RXXX,"LogSta ", MBSREG_LOG_STATE,   (long*)&PrjPkg.log_read_store_state,   NULL},

{REG_U32, REG_P_RXXX,"AutoNum", MBSREG_LOG_AUTO_NUM,(long*)&PrjPkg.log_auto_num,           NULL},
{REG_U32, REG_P_RXXX,"LogDate", MBSREG_LOG_DATE,    (long*)&PrjPkg.log_rtc_date,           NULL},
{REG_U32, REG_P_RXXX,"LogTime", MBSREG_LOG_TIME,    (long*)&PrjPkg.log_rtc_time,           NULL},
{REG_F32, REG_P_RXXX,"LogVec ", MBSREG_LOG_WIND_VEC,(long*)&PrjPkg.log_wind_speed_value,   NULL},
{REG_F32, REG_P_RXXX,"LogTemp", MBSREG_LOG_TEMP,    (long*)&PrjPkg.log_temperature_value,  NULL},
{REG_F32, REG_P_RXXX,"LogHumi", MBSREG_LOG_HUMI,    (long*)&PrjPkg.log_humidity_value,     NULL},
{REG_F32, REG_P_RXXX,"LogDP  ", MBSREG_LOG_DP,      (long*)&PrjPkg.log_dew_point_value,    NULL},
{REG_F32, REG_P_RXXX,"LogWD  ", MBSREG_LOG_WB,      (long*)&PrjPkg.log_wet_bulb_value,     NULL},
{REG_F32, REG_P_RXXX,"LogCHI ", MBSREG_LOG_CHI,     (long*)&PrjPkg.log_wind_chill_value,   NULL},
{REG_F32, REG_P_RXXX,"LogFlow", MBSREG_LOG_FLOW,    (long*)&PrjPkg.log_air_flow_value,     NULL},
{REG_F32, REG_P_RXXX,"LogArea", MBSREG_LOG_AREA,    (long*)&PrjPkg.log_air_flow_area,      NULL},

{REG_U16, REG_P_RXXX,"1234567", MBSREG_END_ADDR,       0,                          NULL}, 
#else
//type     aitt        load        dn         up         name      address 
{REG_U16, REG_P_RWXX,"   0   ", "   0   ", "   3   ", "WifiSta", MBSREG_WIFI_STATE,  (long*)&PrjPkg.mbsreg_wifi_state,      NULL},
{REG_U16, REG_P_RWXX,"   0   ", "   0   ", "   4   ", "TstType", MBSREG_TEST_TYPE,   (long*)&PrjPkg.mbsreg_test_type,(char*)&PrjPkg.test_state_updata},
{REG_U16, REG_P_RWXX,"   0   ", "   0   ", "   3   ", "TstMode", MBSREG_TEST_MODE,   (long*)&PrjPkg.mbsreg_test_mode,(char*)&PrjPkg.test_state_updata},
{REG_U16, REG_P_RWXX,"   0   ", "   0   ", "   5   ", "UnitSta", MBSREG_TEST_UNIT,   (long*)&PrjPkg.mbsreg_test_unit,(char*)&PrjPkg.test_state_updata},
{REG_U16, REG_P_RWXX,"   0   ", "   0   ", "   3   ", "SumSta ", MBSREG_SUM_STATE,   (long*)&PrjPkg.mbsreg_sum_state,(char*)&PrjPkg.test_state_updata},

{REG_F32, REG_P_RXXX,"  4.2  ", "  0.0  ", "  4.2  ", "Bat_Vol", MBSREG_BATTERY_VAL, (long*)&PrjPkg.batter_voltage,   NULL},
{REG_F32, REG_P_RXXX,"  0.5  ", "  0.0  ", "  1.0  ", "Sum_MAX", MBSREG_SUM_MAX_VAL, (long*)&PrjPkg.max_val,          NULL},
{REG_F32, REG_P_RXXX,"  0.5  ", "  0.0  ", "  1.0  ", "Sum_MIN", MBSREG_SUM_MIN_VAL, (long*)&PrjPkg.min_val,          NULL},
{REG_F32, REG_P_RXXX,"  0.5  ", "  0.0  ", "  1.0  ", "Sum_AVG", MBSREG_SUM_AVG_VAL, (long*)&PrjPkg.avg_val,          NULL},
{REG_F32, REG_P_RXXX,"  0.0  ", "  0.0  ", "  30.0 ", "WindVec", MBSREG_WIND_SPEED,  (long*)&PrjPkg.wind_speed_value, NULL},
{REG_F32, REG_P_RXXX,"  37.5 ", " -20.0 ", " 80.0  ", "Temper ", MBSREG_TEMPERATURE, (long*)&PrjPkg.temperature_value_ntc,NULL},
{REG_F32, REG_P_RXXX,"   0.0 ", " -10.0 ", "  50.0 ", "Humi   ", MBSREG_HUMIDITY,    (long*)&PrjPkg.humidity_value,   NULL},
{REG_F32, REG_P_RXXX,"   0.0 ", " -10.0 ", "  50.0 ", "DP     ", MBSREG_DEW_POINT,   (long*)&PrjPkg.dew_point_value,  NULL},
{REG_F32, REG_P_RXXX,"   0.0 ", " -10.0 ", "  50.0 ", "WB     ", MBSREG_WET_BULB,    (long*)&PrjPkg.wet_bulb_value,   NULL},
{REG_F32, REG_P_RXXX,"   0.0 ", " -10.0 ", "  50.0 ", "WCI    ", MBSREG_WIND_CHILL,  (long*)&PrjPkg.wind_chill_value, NULL},
{REG_F32, REG_P_RXXX,"   0.0 ", " -10.0 ", "  50.0 ", "Flow   ", MBSREG_AIR_FLOW,    (long*)&PrjPkg.air_flow_value,    NULL},

{REG_U32, REG_P_RWXX,"210811 ", "210811 ", "210811 ", "RtcDate", MBSREG_SYS_DATE,    (long*)&PrjPkg.rtc_date_modbus,(char*)&PrjPkg.rtc_date_time_updata},
{REG_U32, REG_P_RWXX,"105330 ", "105330 ", "105330 ", "RtcTime", MBSREG_SYS_TIME,    (long*)&PrjPkg.rtc_time_modbus,(char*)&PrjPkg.rtc_date_time_updata},

{REG_U16, REG_P_RWEX,"   0   ", "   0   ", "  7200 ", "StoreTS", MBSREG_STORE_RATE,  (long*)&PrjPkg.set_auto_store_data_time, NULL},
{REG_F32, REG_P_RWEX," 1.000 ", " 0.001 ", " 9.999 ", "SetArea", MBSREG_SET_AREA,    (long*)&PrjPkg.air_flow_area,   NULL},

{REG_U16, REG_P_RXXX,"   0   ", "   0   ", "  1024 ", "LogMAX ", MBSREG_LOG_MAX,     (long*)&PrjPkg.log_max_store_num,      NULL},
{REG_U16, REG_P_RXXX,"   0   ", "   0   ", "  1024 ", "LogNUM ", MBSREG_LOG_NUM,     (long*)&PrjPkg.log_cur_store_num,      NULL},
{REG_U16, REG_P_RWXX,"   0   ", "   0   ", "  1024 ", "LogPos ", MBSREG_LOG_POS,     (long*)&PrjPkg.log_read_store_pos,(char*)&PrjPkg.log_read_store_pos_updata},
{REG_U16, REG_P_RXXX,"   0   ", "   0   ", "   3   ", "LogSta ", MBSREG_LOG_STATE,   (long*)&PrjPkg.log_read_store_state,   NULL},

{REG_U32, REG_P_RXXX,"210811 ", "210811 ", "210811 ", "LogDate", MBSREG_LOG_DATE,    (long*)&PrjPkg.log_rtc_date,           NULL},
{REG_U32, REG_P_RXXX,"105330 ", "105330 ", "105330 ", "LogTime", MBSREG_LOG_TIME,    (long*)&PrjPkg.log_rtc_time,           NULL},
{REG_F32, REG_P_RXXX,"  0.0  ", "  0.0  ", "  30.0 ", "LogVec ", MBSREG_LOG_WIND_VEC,(long*)&PrjPkg.log_wind_speed_value,   NULL},
{REG_F32, REG_P_RXXX,"  37.5 ", " -20.0 ", " 80.0  ", "LogTemp", MBSREG_LOG_TEMP,    (long*)&PrjPkg.log_temperature_value,  NULL},
{REG_F32, REG_P_RXXX,"   0.0 ", "  0.0  ", " 100.0 ", "LogHumi", MBSREG_LOG_HUMI,    (long*)&PrjPkg.log_humidity_value,     NULL},
{REG_F32, REG_P_RXXX,"   0.0 ", " -10.0 ", " 100.0 ", "LogDP  ", MBSREG_LOG_DP,      (long*)&PrjPkg.log_dew_point_value,    NULL},
{REG_F32, REG_P_RXXX,"   0.0 ", " -10.0 ", " 100.0 ", "LogWD  ", MBSREG_LOG_WB,      (long*)&PrjPkg.log_wet_bulb_value,     NULL},
{REG_F32, REG_P_RXXX,"   0.0 ", " -10.0 ", " 100.0 ", "LogCHI ", MBSREG_LOG_CHI,     (long*)&PrjPkg.log_wind_chill_value,   NULL},
{REG_F32, REG_P_RXXX,"   0.0 ", "  0.0  ", " 100.0 ", "LogFlow", MBSREG_LOG_FLOW,    (long*)&PrjPkg.log_air_flow_value,     NULL},
{REG_F32, REG_P_RXXX,"   0.0 ", " 0.001 ", " 9.999 ", "LogArea", MBSREG_LOG_AREA,    (long*)&PrjPkg.log_air_flow_area,      NULL},

//{REG_U08, REG_P_RWXP,"     12", "   0   ", "    123", "DemoU8 ", MBSREG_DEMO_U08, (long*)&DemoMbs.DemoU08,      (char*)&DemoMbs.DemoU08Flag},
//{REG_I16, REG_P_RWXP," -32767", " -32767", "  32767", "DemoI16", MBSREG_DEMO_I16, (long*)&DemoMbs.DemoI16,       NULL},   
//{REG_U16, REG_P_RWXP,"  00000", "  00000", "  65535", "DemoU16", MBSREG_DEMO_U16, (long*)&DemoMbs.DemoU16,       NULL},
//{REG_I32, REG_P_RWXP,"-200000", "-200000", "9999999", "DemoI32", MBSREG_DEMO_I32, (long*)&DemoMbs.DemoI32,       NULL},
//{REG_U32, REG_P_RWXP,"1234567", "1234567", "9999999", "DemoU32", MBSREG_DEMO_U32, (long*)&DemoMbs.DemoU32,       NULL},
//{REG_F32, REG_P_RWXP,"   -7.3", "   -7.3", "    8.3", "DemoF32", MBSREG_DEMO_F32, (long*)&DemoMbs.DemoF32,       NULL},
{REG_U16, REG_P_RXXX,"1234567", "1234567", "1234567", "1234567", MBSREG_END_ADDR,       0,                          NULL}, 
#endif
};

/************************************************************************/
/*      static Variables                                                */
/************************************************************************/

/************************************************************************/
/*      Local Functions                                                 */
/************************************************************************/
int reg_type_I16(char rxd[],int index)
{
    char *pflag = modbus_reg_tbl[index].pUpData;
    short *p,tmp;
    
    p = (short*)modbus_reg_tbl[index].pAdd;
    
    tmp   = rxd[0];
    tmp <<= 8;
    tmp  |= rxd[1];

    if((modbus_reg_tbl[index].Property & REG_P_W) == 0)//不可写
        return false;
#if 0 
    if(modbus_reg_tbl[index].Property & REG_P_P) {//写保护
        dn = atoi(modbus_reg_tbl[index].Dn);
        up = atoi(modbus_reg_tbl[index].Up);
        if((tmp < dn) || (tmp > up))
            return false;
        }
#endif

    if (*p != tmp) {//变量已经改变
        if(modbus_reg_tbl[index].Property & REG_P_E)//掉电存储
            system_param_eeprom();
        }
    
    if(pflag)//标记刷新变量
        *pflag = 1;
    
    *p   = tmp;
    return true;
}
int reg_type_U16(char rxd[],int index)
{
    char *pflag = modbus_reg_tbl[index].pUpData;
    unsigned short *p,tmp;
    
    p = (unsigned short*)modbus_reg_tbl[index].pAdd;
    
    tmp   = rxd[0];
    tmp <<= 8;
    tmp  |= rxd[1];

    if((modbus_reg_tbl[index].Property & REG_P_W) == 0)//不可写
        return false;
#if 0 
    if(modbus_reg_tbl[index].Property & REG_P_P) {//写保护
        dn = atoi(modbus_reg_tbl[index].Dn);
        up = atoi(modbus_reg_tbl[index].Up);
        if((tmp < dn) || (tmp > up))
            return false;
        }
#endif

    if (*p != tmp) {//变量已经改变
        if(modbus_reg_tbl[index].Property & REG_P_E)//掉电存储
            system_param_eeprom();
        }
    
    if(pflag)//标记刷新变量
        *pflag = 1;
    
    *p   = tmp;
    return true;
}
union data_un{
    float         f32;
    long          i32;
    unsigned long u32;
    unsigned char array[4];
};
union data_un bit32;
int reg_type_I32(char rxd[],int index)
{
    char *pflag = modbus_reg_tbl[index].pUpData;
    long *p,tmp;
    
    p = (long*)modbus_reg_tbl[index].pAdd;

    bit32.array[0] = rxd[3];
    bit32.array[1] = rxd[2];
    bit32.array[2] = rxd[1];
    bit32.array[3] = rxd[0];
    tmp  = bit32.i32;

    if((modbus_reg_tbl[index].Property & REG_P_W) == 0)//不可写
        return false;
#if 0 
    if(modbus_reg_tbl[index].Property & REG_P_P)//写保护
        {
        dn = atol(modbus_reg_tbl[index].Dn);
        up = atol(modbus_reg_tbl[index].Up);
        if((tmp < dn) || (tmp > up))
            return FALSE;
        }
#endif

    if (*p != tmp) {//变量已经改变
        if(modbus_reg_tbl[index].Property & REG_P_E)//掉电存储
            system_param_eeprom();
        }
    
    if(pflag)//标记刷新变量
        *pflag = 1;
    
    *p   = tmp;
    return true;
}
int reg_type_U32(char rxd[],int index)
{
    char *pflag = modbus_reg_tbl[index].pUpData;
    unsigned long *p,tmp;
    
    p = (unsigned long*)modbus_reg_tbl[index].pAdd;
    
    bit32.array[0] = rxd[3];
    bit32.array[1] = rxd[2];
    bit32.array[2] = rxd[1];
    bit32.array[3] = rxd[0];
    tmp  = bit32.u32;

    if((modbus_reg_tbl[index].Property & REG_P_W) == 0)//不可写
        return false;
#if 0 
    if(modbus_reg_tbl[index].Property & REG_P_P)//写保护
        {
        dn = atol(modbus_reg_tbl[index].Dn);
        up = atol(modbus_reg_tbl[index].Up);
        if((tmp < dn) || (tmp > up))
            return FALSE;
        }
#endif

    if (*p != tmp) {//变量已经改变
        if(modbus_reg_tbl[index].Property & REG_P_E)//掉电存储
            system_param_eeprom();
        }
    
    if(pflag)//标记刷新变量
        *pflag = 1;
    
    *p   = tmp;
    return true;
}
int reg_type_F32(char rxd[],int index)
{
    char *pflag = modbus_reg_tbl[index].pUpData;
    float *p,tmp;
    
    p = (float*)modbus_reg_tbl[index].pAdd;

    bit32.array[0] = rxd[3];
    bit32.array[1] = rxd[2];
    bit32.array[2] = rxd[1];
    bit32.array[3] = rxd[0];
    tmp  = bit32.f32;

    if((modbus_reg_tbl[index].Property & REG_P_W) == 0)//不可写
        return false;
#if 0 
    if(modbus_reg_tbl[index].Property & REG_P_P)//写保护
        {
        dn = atof(modbus_reg_tbl[index].Dn);
        up = atof(modbus_reg_tbl[index].Up);
        if((tmp < dn) || (tmp > up))
            return FALSE;
        }
#endif

    if (*p != tmp) {//变量已经改变
        if(modbus_reg_tbl[index].Property & REG_P_E)//掉电存储
            system_param_eeprom();
        }
    
    if(pflag)//标记刷新变量
        *pflag = 1;
    
    *p   = tmp;
    return true;
}


/************************************************************************/
/*     dbg_cmd Interface                                                */
/************************************************************************/
#include "dbg_cmd.h"
#ifdef DBG_CMD_EN
const char RwepTab[8][5] = {{"RXXX"},{"RWXX"},{"NULL"},{"RWEX"},{"NULL"},{"RWXP"},{"NULL"},{"RWEP"}};
void modbus_reg_print_msg()
{
    int n = 0;
#ifdef SHORT_REG_MSG
    DBG_CMD_PRN("RWEP Name      RegAdd        Val Type\r\n");
    while(modbus_reg_tbl[n].RegAdd != MBSREG_END_ADDR) {
        DBG_CMD_PRN("%s %s % 6d[0X%00004X]=",
            RwepTab[modbus_reg_tbl[n].Property],
            modbus_reg_tbl[n].Name,
            modbus_reg_tbl[n].RegAdd,
            modbus_reg_tbl[n].RegAdd);
#else 
    DBG_CMD_PRN("RWEP Name      DnVal     UpVal        LoadVal       RegAdd        Val Type\r\n");
    while(modbus_reg_tbl[n].RegAdd != MBSREG_END_ADDR) {
        DBG_CMD_PRN("%s %s [%9s,%9s] %9s % 6d[0X%00004X]=",
            RwepTab[modbus_reg_tbl[n].Property],
            modbus_reg_tbl[n].Name,
            modbus_reg_tbl[n].Dn,
            modbus_reg_tbl[n].Up,
            modbus_reg_tbl[n].Load,
            modbus_reg_tbl[n].RegAdd,
            modbus_reg_tbl[n].RegAdd);
#endif

        if(modbus_reg_tbl[n].RegType == REG_U08) {
            char *p;
            p = (char*)modbus_reg_tbl[n].pAdd;
            DBG_CMD_PRN("%9u U08\r\n",*p);
        }
        if(modbus_reg_tbl[n].RegType == REG_I16) {
            short *p;
            p = (short*)modbus_reg_tbl[n].pAdd;
            DBG_CMD_PRN("%9d I16\r\n",*p);
        }
        if(modbus_reg_tbl[n].RegType == REG_U16) {
            unsigned short *p;
            p = (unsigned short*)modbus_reg_tbl[n].pAdd;
            DBG_CMD_PRN("%9d U16\r\n",*p);
        }
        if(modbus_reg_tbl[n].RegType == REG_I32) {
            long *p;
            p = (long*)modbus_reg_tbl[n].pAdd;
            DBG_CMD_PRN("%9ld I32\r\n",*p);
        }
        if(modbus_reg_tbl[n].RegType == REG_U32) {
            unsigned long *p;
            p = (unsigned long*)modbus_reg_tbl[n].pAdd;
            DBG_CMD_PRN("%9lu U32\r\n",*p);
        }
        if(modbus_reg_tbl[n].RegType == REG_F32) {
            float *p;
            p = (float*)modbus_reg_tbl[n].pAdd;
            DBG_CMD_PRN("%9f F32\r\n",*p);
        }
#if 0
        if(modbus_reg_tbl[n].RegType == REG_STR) {
            short *p;
            p = strlen();
            DBG_CMD_PRN("%9d STR\r\n",*p);
        }
#endif
        n++;
    }
}
void dbg_cmd_set_reg(unsigned short addr, char str[])
{
    char *pflag;
    int n = 0,rev = false;

    while (modbus_reg_tbl[n].RegAdd != MBSREG_END_ADDR) { // 搜索对应地址
        if (addr == modbus_reg_tbl[n].RegAdd) {
            switch (modbus_reg_tbl[n].RegType) {
                case REG_I16:
                    {
                        short *p;
                        p= (short*)modbus_reg_tbl[n].pAdd;
                        *p = atoi(str);
                        rev = true;
                    }
                    break;
                case REG_U16:
                    {
                        unsigned short *p;
                        p= (unsigned short*)modbus_reg_tbl[n].pAdd;
                        *p = atoi(str);
                        rev = true;
                    }
                    break;
                case REG_I32:
                    {
                        long *p;
                        p= (long*)modbus_reg_tbl[n].pAdd;
                        *p = atol(str);
                        rev = true;
                    }
                    break;
                case REG_U32:
                    {
                        unsigned long *p;
                        p = (unsigned long*)modbus_reg_tbl[n].pAdd;
                        *p = atol(str);
                        rev = true;
                    }
                    break;
                case REG_F32:
                    {
                        float *p;
                        p = (float*)modbus_reg_tbl[n].pAdd;
                        *p = atof(str);
                        rev = true;
                    }
                    break;
            } // switch
            pflag = modbus_reg_tbl[n].pUpData;
            if(pflag){
                /* updata flag */
                *pflag = 1;
            }
            break;
        } else {
            n++;
        }
    }
    DBG_CMD_PRN("Set %s!\r\n",rev?"OK":"Faile");  
}
static bool dbg_cmd_func()
{
    if (dbg_cmd_exec("help", "","")) {
        DBG_CMD_PRN(".MbsReg\r\n");
        return false;
    }
    if (dbg_cmd_exec(".MbsReg", "","")) {
        dbg_cmd_print_msg_en();
    }
    if (dbg_cmd_exec("MbsRegMsg", "","")) {
        modbus_reg_0X03_renew(MBSREG_TEST_TYPE,1);
        modbus_reg_0X03_renew(MBSREG_SYS_DATE,1);
        modbus_reg_0X03_renew(MBSREG_LOG_MAX,1);
        modbus_reg_print_msg();
        return true;
    }
    if (dbg_cmd_exec("MbsRegW", "2s","<addr> <\"str\">")) {
        dbg_cmd_set_reg(get_param_short(0),get_param_string(0));
        return true;
    }
    return false;
}
#endif

/************************************************************************/
/*      Application Interface                                           */
/************************************************************************/
void auto_read_log_data()
{
    PrjPkg.log_max_store_num = get_store_max_num();
    PrjPkg.log_cur_store_num = get_store_current_num();

    if (PrjPkg.log_cur_store_num > 0) {
        if (PrjPkg.log_read_store_state == 1) {
            PrjPkg.log_read_store_state  = 2;// 数据读取完成
        } else {
            PrjPkg.log_auto_num++;
        }
        if (PrjPkg.log_auto_num > PrjPkg.log_cur_store_num) {
            PrjPkg.log_auto_num = 1;
        }
        read_test_data(PrjPkg.log_auto_num);
        PrjPkg.log_rtc_date          = PrjPkg.rw_rtc_date;
        PrjPkg.log_rtc_time          = PrjPkg.rw_rtc_time;
        PrjPkg.log_wind_speed_value  = PrjPkg.rw_wind_speed_value;
        PrjPkg.log_temperature_value = PrjPkg.rw_temperature_value;
        PrjPkg.log_humidity_value    = PrjPkg.rw_humidity_value;
        PrjPkg.log_dew_point_value   = PrjPkg.rw_dew_point_value;
        PrjPkg.log_wet_bulb_value    = PrjPkg.rw_wet_bulb_value;
        PrjPkg.log_wind_chill_value  = PrjPkg.rw_wind_chill_value;
        PrjPkg.log_air_flow_value    = PrjPkg.rw_air_flow_value;
        PrjPkg.log_air_flow_area     = PrjPkg.rw_air_flow_area;
    } else {
        PrjPkg.log_auto_num = 0;
    }
}

int modbus_reg_0X03(unsigned short addr,unsigned short num, char txd[])
{
    int reg_tbl_pos = 0;
    int rxd_buf_pos = 0;
    int byte_cnt = 0;
    char *p8;

    modbus_reg_0X03_renew(addr,num);

    /* auto read log store */
    if (addr == MBSREG_LOG_AUTO_NUM) {
        auto_read_log_data();
        DBG_CMD_PRN("auto_log_num:%d\r\n",PrjPkg.log_auto_num);
        PrjPkg.log_read_store_pos = PrjPkg.log_auto_num;
    }

    for (int i=0;i<num;) {
        if(addr == modbus_reg_tbl[reg_tbl_pos+1].RegAdd) {//连续读时 预判下一个寄存器地址 为提高效率
            reg_tbl_pos++;
        } else {//重头开始扫描寄存器列表
            reg_tbl_pos = 0;
            while (modbus_reg_tbl[reg_tbl_pos].RegAdd != MBSREG_END_ADDR) {// 搜索对应地址
                if(addr == modbus_reg_tbl[reg_tbl_pos].RegAdd)
                    break;
                else
                    reg_tbl_pos++;
            }
        }

        if(modbus_reg_tbl[reg_tbl_pos].RegAdd == MBSREG_END_ADDR)
            return false;//地址搜索不到 正常情况是不会执行

        switch(modbus_reg_tbl[reg_tbl_pos].RegType) {
            case REG_U08:
                p8 = (char*)modbus_reg_tbl[reg_tbl_pos].pAdd;
                txd[rxd_buf_pos+1] = *p8;
                txd[rxd_buf_pos+0] = 0;
                byte_cnt = 2;
                break;

            case REG_I16:
            case REG_U16:
                p8 = (char*)modbus_reg_tbl[reg_tbl_pos].pAdd;
                txd[rxd_buf_pos+1] = *p8;p8++;
                txd[rxd_buf_pos+0] = *p8;p8++;
                byte_cnt = 2;
                break;

            case REG_I32:
            case REG_U32:
            case REG_F32:
                p8 = (char*)modbus_reg_tbl[reg_tbl_pos].pAdd;
                txd[rxd_buf_pos+3] = *p8;p8++;
                txd[rxd_buf_pos+2] = *p8;p8++;
                txd[rxd_buf_pos+1] = *p8;p8++;
                txd[rxd_buf_pos+0] = *p8;p8++;
                byte_cnt = 4;
                break;
        }
        i += byte_cnt/2;
        addr += byte_cnt/2;
        rxd_buf_pos += byte_cnt;

        if(addr == MBSREG_END_ADDR)// 最后一个寄存器临界特殊处理
            return true;
    }   
    return true;
}

int modbus_reg_0X06(unsigned short addr, char rxd[])
{
    int reg_tbl_pos = 0;

    while(modbus_reg_tbl[reg_tbl_pos].RegAdd != MBSREG_END_ADDR)// 搜索对应地址
        {
        if(addr == modbus_reg_tbl[reg_tbl_pos].RegAdd)
            break;
        else
            reg_tbl_pos++;
        }

    if(modbus_reg_tbl[reg_tbl_pos].RegAdd == MBSREG_END_ADDR)
        return false;//地址搜索不到 正常情况是不会执行

    switch(modbus_reg_tbl[reg_tbl_pos].RegType)
        {
        case REG_I16:
            if(reg_type_I16(rxd,reg_tbl_pos) == false)
                return false;
            break;

        case REG_U16:
            if(reg_type_U16(rxd,reg_tbl_pos) == false)
                return false;
            break;
        }
    return true;
}

int modbus_reg_0X10(unsigned short addr,unsigned short num, char rxd[])
{
    int reg_tbl_pos = 0;
    int rxd_buf_cnt = 0;
    int byte_cnt = 0;

    for (int i=0;i<num;) {
        if(addr == modbus_reg_tbl[reg_tbl_pos+1].RegAdd) {//连续读时 预判下一个寄存器地址 为提高效率
            reg_tbl_pos++;
        } else {//重头开始扫描寄存器列表
            reg_tbl_pos = 0;
            while (modbus_reg_tbl[reg_tbl_pos].RegAdd != MBSREG_END_ADDR) { // 搜索对应地址
                if(addr == modbus_reg_tbl[reg_tbl_pos].RegAdd)
                    break;
                else
                    reg_tbl_pos++;
            }
        }

        if(modbus_reg_tbl[reg_tbl_pos].RegAdd == MBSREG_END_ADDR)
            return false;//地址搜索不到 正常情况是不会执行

        switch (modbus_reg_tbl[reg_tbl_pos].RegType) {
            case REG_I16:
                if(reg_type_I16(rxd+rxd_buf_cnt,reg_tbl_pos) == false)
                    return false;
                byte_cnt = 2;
                break;
            case REG_U16:
                if(reg_type_U16(rxd+rxd_buf_cnt,reg_tbl_pos) == false)
                    return false;
                byte_cnt = 2;
                break;
            case REG_I32:
                if(reg_type_I32(rxd+rxd_buf_cnt,reg_tbl_pos) == false)
                    return false;
                byte_cnt = 4;
                break;
            case REG_U32:
                if(reg_type_U32(rxd+rxd_buf_cnt,reg_tbl_pos) == false)
                    return false;
                byte_cnt = 4;
                break;
            case REG_F32:
                if(reg_type_F32(rxd+rxd_buf_cnt,reg_tbl_pos) == false)
                    return false;
                byte_cnt = 4;
                break;
        }
        i += byte_cnt/2;
        addr += byte_cnt/2;
        rxd_buf_cnt += byte_cnt;

        if(addr == MBSREG_END_ADDR)// 最后一个寄存器临界特殊处理
            return true;
    }
    return true;
}

void modbus_reg_init()
{
#ifdef DBG_CMD_EN
    dbg_cmd_add_list((int)dbg_cmd_func);
#endif // DBG_CMD_EN
}

#endif // MODBUS_REG_MODULE_EN == 1

