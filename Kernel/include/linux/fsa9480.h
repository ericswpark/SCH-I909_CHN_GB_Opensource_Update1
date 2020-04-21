/*
 * Copyright (C) 2010 Samsung Electronics
 * Minkyu Kang <mk7.kang@samsung.com>
 * Wonguk Jeong <wonguk.jeong@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#ifndef _FSA9480_H_
#define _FSA9480_H_

enum {
	FSA9480_DETACHED,
	FSA9480_ATTACHED
};

struct fsa9480_platform_data {
	void (*cfg_gpio) (void);
	void (*usb_cb) (bool attached);
	void (*uart_cb) (bool attached);
	void (*charger_cb) (bool attached);
	void (*jig_cb) (bool attached);
	void (*deskdock_cb) (bool attached);
	void (*cardock_cb) (bool attached);
	void (*reset_cb) (void);
	void (*set_init_flag) (void);
};

enum {
	SWITCH_PORT_AUTO = 0,
	SWITCH_PORT_USB,
	SWITCH_PORT_AUDIO,
	SWITCH_PORT_UART,
	SWITCH_PORT_VAUDIO
};

// ADD by kbj .
#define CALL_FROM_USB_SETTING	0
#define CALL_FROM_ASKON_MENU	1

#define CALL_FROM_USB_CABLE	0
#define CALL_FROM_USB_MENU_SEL	1

extern void fsa9480_manual_switching(int path);

#endif /* _FSA9480_H_ */
