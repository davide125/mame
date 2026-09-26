// license:BSD-3-Clause
// copyright-holders:Davide Cavalca
/**************************************************************************

    Logitech Series/2 PS/2 Mouse

    Hardware notes:
    - MC146805F2 (CMOS 6805, 28-pin DIP) marked "ZC82596P (C) LOGITECH
      1987 ES 2.0", 4 MHz ceramic resonator divided by 4 internally
    - the dump covers the whole 2 KB address space, including the
      Motorola self-check routines at $4B5-$4FD and $780-$7F5
    - two microswitches, opto-mechanical quadrature encoders behind a
      Logitech "CM B111L" sensor interface chip
    - PCB "S2-L REV C", P/N 200029-02, label "MODEL NO. SERIES 2-7S"
    - no serial hardware: the PS/2 clock and data lines are bit-banged

    MCU pin assignments:
      PA0-PA2  in   not read by the firmware
      PA3      out  DATA line, 0 = pull low (never driven high)
      PA4-PA6  out  encoder sensor supply, pulsed around encoder reads
                    (PA4 reading high at reset selects a factory test mode)
      PA7      in   strap, high enables power saving: WAIT when idle, and
                    the Logitech specific $F8 command puts the MCU in STOP
      PB0      in   left button (0 = pressed)
      PB1      in   right button (0 = pressed)
      PB2      i/o  CLOCK line, read as an input and pulled low by turning
                    the port into an output with the latch bit clear
      PB3-PB7       unused
      PC0-PC1  in   Y encoder quadrature
      PC2-PC3  in   X encoder quadrature
      IRQ      in   DATA line, sampled with BIL/BIH (also wakes STOP)

    Firmware notes:
    - timer set for 1 ms ticks (TCR = $02, TDR = $FA); the $AA $00
      power-on sequence is sent about 370 ms after reset
    - standard PS/2 command set (defaults: 4 counts/mm, 100 reports/s,
      stream mode, reporting disabled), device ID $00; unknown commands
      get a $FE resend request, then $FC
    - the encoders are polled from the idle loops and once at the start
      of each byte on the bus; during a byte they are polled again only
      while consecutive polls see movement on a single axis.  Simultaneous
      X and Y transitions go through handlers which get re-executed for
      every remaining bit of the byte, so the emulated encoders never step
      both axes at once, and only advance when the MCU reads them.

    References:
    - bitsavers.org/pdf/logitech/mouse/Series2_7S_PS2/ (ROM dump, raw
      disassembly, PCB and mouse photos)
    - Motorola MC146805F2 Advance Information data sheet

**************************************************************************/

#include "emu.h"
#include "logimouse_s2.h"

#include <algorithm>

#define LOG_RXTX   (1U << 1)

#define VERBOSE (0)
#include "logmacro.h"


DEFINE_DEVICE_TYPE(LOGITECH_S2_PS2_MOUSE, logitech_s2_mouse_device, "ps2_mouse_logi_s2", "Logitech Series/2 PS/2 Mouse")


namespace {

INPUT_PORTS_START(logimouse_s2)
	PORT_START("BTN")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_BUTTON1 ) PORT_CODE(MOUSECODE_BUTTON1) PORT_NAME("Left Button")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_BUTTON2 ) PORT_CODE(MOUSECODE_BUTTON2) PORT_NAME("Right Button")

	PORT_START("X")
	PORT_BIT( 0xfff, 0x000, IPT_MOUSE_X ) PORT_SENSITIVITY(100) PORT_KEYDELTA(0)

	PORT_START("Y")
	PORT_BIT( 0xfff, 0x000, IPT_MOUSE_Y ) PORT_SENSITIVITY(100) PORT_KEYDELTA(0)

	// PA7 strap.  It was not traced on the dumped board and its factory setting
	// is unknown; Off (continuous polling) is an assumption.
	PORT_START("CONF")
	PORT_CONFNAME( 0x01, 0x00, "Power Saving" )
	PORT_CONFSETTING(    0x00, DEF_STR(Off) )
	PORT_CONFSETTING(    0x01, DEF_STR(On) )
INPUT_PORTS_END

} // anonymous namespace


ROM_START(logimouse_s2)
	ROM_REGION(0x800, "mcu", 0)
	ROM_LOAD("zc82596p_series2_2.0.bin", 0x000, 0x800, CRC(36a17d09) SHA1(abe67acb0854749c385130bebeb68eada421e2cc))
ROM_END


logitech_s2_mouse_device::logitech_s2_mouse_device(machine_config const &mconfig, char const *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, LOGITECH_S2_PS2_MOUSE, tag, owner, clock)
	, device_pc_kbd_interface(mconfig, *this)
	, m_mcu(*this, "mcu")
	, m_buttons(*this, "BTN")
	, m_x_axis(*this, "X")
	, m_y_axis(*this, "Y")
	, m_conf(*this, "CONF")
	, m_x_last(0)
	, m_y_last(0)
	, m_x_count(0)
	, m_y_count(0)
	, m_x_phase(0)
	, m_y_phase(0)
	, m_step_y(false)
	, m_sample_timer(nullptr)
{
}

const tiny_rom_entry *logitech_s2_mouse_device::device_rom_region() const
{
	return ROM_NAME(logimouse_s2);
}

void logitech_s2_mouse_device::device_add_mconfig(machine_config &config)
{
	M146805F2(config, m_mcu, 4'000'000); // 4 MHz ceramic resonator
	m_mcu->porta_r().set(FUNC(logitech_s2_mouse_device::mcu_port_a_r));
	m_mcu->portb_r().set(FUNC(logitech_s2_mouse_device::mcu_port_b_r));
	m_mcu->portc_r().set(FUNC(logitech_s2_mouse_device::mcu_port_c_r));
	m_mcu->porta_w().set(FUNC(logitech_s2_mouse_device::mcu_port_a_w));
	m_mcu->portb_w().set(FUNC(logitech_s2_mouse_device::mcu_port_b_w));
}

ioport_constructor logitech_s2_mouse_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(logimouse_s2);
}

void logitech_s2_mouse_device::device_start()
{
	set_pc_kbdc_device();

	save_item(NAME(m_x_last));
	save_item(NAME(m_y_last));
	save_item(NAME(m_x_count));
	save_item(NAME(m_y_count));
	save_item(NAME(m_x_phase));
	save_item(NAME(m_y_phase));
	save_item(NAME(m_step_y));

	m_sample_timer = timer_alloc(FUNC(logitech_s2_mouse_device::sample_axes), this);
}

void logitech_s2_mouse_device::device_reset()
{
	m_x_last = m_x_axis->read();
	m_y_last = m_y_axis->read();
	m_x_count = 0;
	m_y_count = 0;
	m_x_phase = 0;
	m_y_phase = 0;
	m_step_y = false;

	m_sample_timer->adjust(attotime::from_hz(1000), 0, attotime::from_hz(1000));

	// all port pins are inputs after reset, so both lines are released
	m_pc_kbdc->data_write_from_kb(1);
	m_pc_kbdc->clock_write_from_kb(1);
}

void logitech_s2_mouse_device::data_write(int state)
{
	// DATA is wired to the IRQ pin, low = asserted
	m_mcu->set_input_line(M6805_IRQ_LINE, state ? CLEAR_LINE : ASSERT_LINE);
}

//
// MCU port callbacks
//

uint8_t logitech_s2_mouse_device::mcu_port_a_r(offs_t offset, uint8_t mem_mask)
{
	// PA4 must read low at reset or the firmware enters its test mode
	return BIT(m_conf->read(), 0) ? 0x80 : 0x00;
}

uint8_t logitech_s2_mouse_device::mcu_port_b_r(offs_t offset, uint8_t mem_mask)
{
	uint8_t const buttons = m_buttons->read();
	uint8_t result = 0x00;

	if (!BIT(buttons, 0))
		result |= 0x01; // left released
	if (!BIT(buttons, 1))
		result |= 0x02; // right released
	if (clock_signal())
		result |= 0x04; // CLOCK line (can read -1 before the first update)

	return result;
}

uint8_t logitech_s2_mouse_device::mcu_port_c_r(offs_t offset, uint8_t mem_mask)
{
	// the firmware reads the port once per encoder poll, so advance the
	// emulated encoders here rather than on a free-running clock
	if (!machine().side_effects_disabled())
		step_encoders();

	// PC0/PC1 Y and PC2/PC3 X quadrature, Gray code sequence 0, 1, 3, 2
	static constexpr uint8_t gray_code[4] = { 0x00, 0x01, 0x03, 0x02 };
	return gray_code[m_y_phase & 3] | (gray_code[m_x_phase & 3] << 2);
}

void logitech_s2_mouse_device::mcu_port_a_w(offs_t offset, uint8_t data, uint8_t mem_mask)
{
	if (VERBOSE & LOG_RXTX)
	{
		auto const suppressor(machine().disable_side_effects());

		// PA3 is written for the start bit of each transmitted byte (byte
		// in RAM $58) and after the ACK bit of each received byte (byte in A)
		switch (m_mcu->pc())
		{
		case 0x3e2:
			LOGMASKED(LOG_RXTX, "tx 0x%02x\n", m_mcu->space(AS_PROGRAM).read_byte(0x58));
			break;
		case 0x4af:
			LOGMASKED(LOG_RXTX, "rx 0x%02x\n", m_mcu->state_int(4)); // A register
			break;
		}
	}

	// PA3 pulls DATA low when driven low, the sensor supply pulses on
	// PA4-PA6 have no effect on the emulated encoders
	m_pc_kbdc->data_write_from_kb((BIT(data, 3) || !BIT(mem_mask, 3)) ? 1 : 0);
}

void logitech_s2_mouse_device::mcu_port_b_w(offs_t offset, uint8_t data, uint8_t mem_mask)
{
	// CLOCK is pulled low by making PB2 an output with the latch bit clear
	m_pc_kbdc->clock_write_from_kb((BIT(data, 2) || !BIT(mem_mask, 2)) ? 1 : 0);
}

//
// Encoder emulation
//

TIMER_CALLBACK_MEMBER(logitech_s2_mouse_device::sample_axes)
{
	uint16_t const x_cur = m_x_axis->read() & 0xfff;
	uint16_t const y_cur = m_y_axis->read() & 0xfff;

	int16_t x_delta = int16_t(x_cur) - int16_t(m_x_last);
	int16_t y_delta = int16_t(y_cur) - int16_t(m_y_last);

	// 12-bit wraparound
	if (x_delta > 2047) x_delta -= 4096;
	else if (x_delta < -2048) x_delta += 4096;
	if (y_delta > 2047) y_delta -= 4096;
	else if (y_delta < -2048) y_delta += 4096;

	m_x_last = x_cur;
	m_y_last = y_cur;

	// the MCU stops polling for a while in WAIT or STOP when power saving
	// is enabled, so bound the movement queued for it
	m_x_count = std::clamp<int16_t>(m_x_count + x_delta, -255, 255);
	m_y_count = std::clamp<int16_t>(m_y_count + y_delta, -255, 255);
}

void logitech_s2_mouse_device::step_encoders()
{
	// One Gray code step on one axis per poll (see the firmware notes).
	// The X encoder decrements the X counter on its forward sequence and
	// the Y encoder increments the Y counter (up in PS/2 terms) on its
	// forward sequence, so movement in the positive MAME direction (right
	// or down) runs both sequences backwards.
	bool const x_pending = m_x_count != 0;
	bool const y_pending = m_y_count != 0;

	if (x_pending && (!y_pending || !m_step_y))
	{
		if (m_x_count > 0)
		{
			m_x_phase = (m_x_phase - 1) & 3;
			m_x_count--;
		}
		else
		{
			m_x_phase = (m_x_phase + 1) & 3;
			m_x_count++;
		}
		m_step_y = true;
	}
	else if (y_pending)
	{
		if (m_y_count > 0)
		{
			m_y_phase = (m_y_phase - 1) & 3;
			m_y_count--;
		}
		else
		{
			m_y_phase = (m_y_phase + 1) & 3;
			m_y_count++;
		}
		m_step_y = false;
	}
}
