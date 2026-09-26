// license:BSD-3-Clause
// copyright-holders:Davide Cavalca
/**************************************************************************

    Logitech Series/2 PS/2 Mouse

**************************************************************************/
#ifndef MAME_BUS_PC_KBD_LOGIMOUSE_S2_H
#define MAME_BUS_PC_KBD_LOGIMOUSE_S2_H

#pragma once

#include "pc_kbdc.h"
#include "cpu/m6805/m68705.h"


DECLARE_DEVICE_TYPE(LOGITECH_S2_PS2_MOUSE, logitech_s2_mouse_device)

class logitech_s2_mouse_device : public device_t, public device_pc_kbd_interface
{
public:
	logitech_s2_mouse_device(machine_config const &mconfig, char const *tag, device_t *owner, uint32_t clock);

protected:
	virtual const tiny_rom_entry *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// device_pc_kbd_interface implementation
	virtual void data_write(int state) override;

private:
	// MCU port callbacks
	uint8_t mcu_port_a_r(offs_t offset, uint8_t mem_mask);
	uint8_t mcu_port_b_r(offs_t offset, uint8_t mem_mask);
	uint8_t mcu_port_c_r(offs_t offset, uint8_t mem_mask);
	void mcu_port_a_w(offs_t offset, uint8_t data, uint8_t mem_mask);
	void mcu_port_b_w(offs_t offset, uint8_t data, uint8_t mem_mask);

	TIMER_CALLBACK_MEMBER(sample_axes);
	void step_encoders();

	required_device<m146805f2_device> m_mcu;
	required_ioport m_buttons;
	required_ioport m_x_axis;
	required_ioport m_y_axis;
	required_ioport m_conf;

	// encoder state
	uint16_t m_x_last;
	uint16_t m_y_last;
	int16_t m_x_count;   // accumulated X movement pending
	int16_t m_y_count;   // accumulated Y movement pending
	uint8_t m_x_phase;   // quadrature phase of the X encoder (PC2/PC3)
	uint8_t m_y_phase;   // quadrature phase of the Y encoder (PC0/PC1)
	bool m_step_y;       // step Y next when both axes have movement pending
	emu_timer *m_sample_timer;
};

#endif // MAME_BUS_PC_KBD_LOGIMOUSE_S2_H
