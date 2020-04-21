#ifndef __DS_MANAGER__
#define __DS_MANAGER__

#define SIM_SLOT_0		8
#define SIM_SLOT_1		9

extern void (*cdma_slot_switch_handler)(int slot_switch);
extern void (*gsm_slot_switch_handler)(int slot_switch);

#endif

