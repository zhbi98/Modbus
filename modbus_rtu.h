#ifndef __MODBUS_RTU_H
#define __MODBUS_RTU_H

#define MODBUS_RTU_MODULE_EN  (1)

enum modbus_port_eu {
    MBS_PORT_PC,
    MBS_PORT_WIFI,
};

#if ( MODBUS_RTU_MODULE_EN == 1 )

extern void modbus_rtu_10ms_thread_isr();
extern void modbus_rtu_rxd_thread_isr(unsigned char com, char rxd);
extern unsigned char modbus_rtu_link_state(unsigned char com);
extern void modbus_rtu_real_time_thread();
extern void modbus_rtu_init(unsigned char mode);

#else

#define modbus_rtu_10ms_thread_isr()
#define modbus_rtu_rxd_thread_isr(com, rxd)
#define modbus_rtu_link_state(com)
#define modbus_rtu_real_time_thread()
#define modbus_rtu_init(mode)

#endif

#endif

