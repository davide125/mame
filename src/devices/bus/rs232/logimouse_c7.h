// license:BSD-3-Clause
// copyright-holders:Davide Cavalca
/**************************************************************************

    Logitech C7 Serial Mouse

**************************************************************************/
#ifndef MAME_BUS_RS232_LOGIMOUSE_C7_H
#define MAME_BUS_RS232_LOGIMOUSE_C7_H

#pragma once

#include "rs232.h"
#include "cpu/m6805/m68705.h"


DECLARE_DEVICE_TYPE(LOGITECH_C7_SERIAL_MOUSE, logitech_c7_mouse_device)

class logitech_c7_mouse_device : public device_t, public device_rs232_port_interface
{
public:
	logitech_c7_mouse_device(machine_config const &mconfig, char const *tag, device_t *owner, uint32_t clock);

protected:
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void device_resolve_objects() override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	virtual void input_txd(int state) override;
	virtual void input_rts(int state) override;

private:
	// MCU port callbacks
	uint8_t mcu_port_a_r(offs_t offset, uint8_t mem_mask);
	uint8_t mcu_port_b_r(offs_t offset, uint8_t mem_mask);
	uint8_t mcu_port_c_r(offs_t offset, uint8_t mem_mask);
	void mcu_port_a_w(offs_t offset, uint8_t data, uint8_t mem_mask);
	void mcu_port_b_w(offs_t offset, uint8_t data, uint8_t mem_mask);

	TIMER_CALLBACK_MEMBER(update_quadrature);

	required_device<m146805f2_device> m_mcu;
	required_ioport m_buttons;
	required_ioport m_x_axis;
	required_ioport m_y_axis;
	required_ioport m_jumpers;

	// state
	uint8_t m_port_a_out;
	uint8_t m_port_b_out;
	uint8_t m_rts;       // host RTS as seen by the mouse (0 = asserted)

	// encoder state
	uint16_t m_x_last;
	uint16_t m_y_last;
	int16_t m_x_count;   // accumulated X movement pending
	int16_t m_y_count;   // accumulated Y movement pending
	uint8_t m_x_phase;   // quadrature phase of the X encoder (PC0/PC1)
	uint8_t m_y_phase;   // quadrature phase of the Y encoder (PC2/PC3)
	emu_timer *m_quadrature_timer;
};

#endif // MAME_BUS_RS232_LOGIMOUSE_C7_H
