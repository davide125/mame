// license:BSD-3-Clause
// copyright-holders:Davide Cavalca
/**************************************************************************

    Logitech C7 Serial Mouse

    Hardware notes:
    - MC146805F2P (CMOS 6805, 28-pin DIP) marked "LSC82406P (C) LOGITECH
      1986", 4 MHz crystal divided by 4 internally
    - 1079 bytes of mask ROM and 64 bytes of RAM; the dump covers the
      whole 2 KB address space
    - no UART: TX is bit-banged on PB7 from the timer interrupt, RX is
      sampled on the IRQ pin (start bit by interrupt, data bits by
      BIL/BIH)
    - opto-mechanical quadrature encoders on PC0-PC3, three
      microswitches
    - powered from RTS and DTR at either polarity, RxD supplies the
      negative rail; the control lines cannot reset the mouse

    MCU pin assignments:
      PA0-PA2  in   solder straps R2, R1, R0 (all low on the dumped board)
      PA3      out  pulsed high around encoder reads (encoder LED supply?)
      PA4      in   jumper J3 (baud rate)
      PA5      in   jumper J4 (baud rate)
      PA6      in   jumper J2 (data format)
      PA7      in   jumper J1 (data format)
      PB0      in   right button pressed
      PB1-PB2  in   not traced (see mcu_port_b_r)
      PB3      in   RTS asserted and right button released
      PB4      in   middle button released
      PB5      in   left button released
      PB6      in   jumper J0 (data format)
      PB7      out  serial TX data
      PC0-PC1  in   X encoder quadrature
      PC2-PC3  in   Y encoder quadrature
      IRQ      in   serial RX data

    Jumpers are three-pad solder straps, "0" pulls the pin low.  Factory
    default is all jumpers at 0 (Five Byte Packed Binary, 1200 baud).
    Any RTS toggle switches the firmware to Microsoft compatible format
    at 1200 baud regardless of the jumpers.

    References:
    - Logitech Logimouse C7 Firmware Revision 3.0, January 1986
    - bitsavers.org/pdf/logitech/mouse/C7-3F-9F_serial/ (ROM dump, dump
      notes, PCB and jumper photos)
    - Motorola MC146805F2 Advance Information data sheet

**************************************************************************/

#include "emu.h"
#include "logimouse_c7.h"

#define LOG_PORT_A   (1U << 1)
#define LOG_PORT_B   (1U << 2)

#define VERBOSE (0)
#include "logmacro.h"


DEFINE_DEVICE_TYPE(LOGITECH_C7_SERIAL_MOUSE, logitech_c7_mouse_device, "rs232_mouse_logi_c7", "Logitech C7 Serial Mouse")


namespace {

INPUT_PORTS_START(logimouse_c7)
	PORT_START("BTN")
	PORT_BIT( 0x01, IP_ACTIVE_HIGH, IPT_BUTTON1 ) PORT_CODE(MOUSECODE_BUTTON1) PORT_NAME("Left Button")
	PORT_BIT( 0x02, IP_ACTIVE_HIGH, IPT_BUTTON2 ) PORT_CODE(MOUSECODE_BUTTON3) PORT_NAME("Middle Button")
	PORT_BIT( 0x04, IP_ACTIVE_HIGH, IPT_BUTTON3 ) PORT_CODE(MOUSECODE_BUTTON2) PORT_NAME("Right Button")

	PORT_START("X")
	PORT_BIT( 0xfff, 0x000, IPT_MOUSE_X ) PORT_SENSITIVITY(100) PORT_KEYDELTA(0)

	PORT_START("Y")
	PORT_BIT( 0xfff, 0x000, IPT_MOUSE_Y ) PORT_SENSITIVITY(100) PORT_KEYDELTA(0)

	// solder jumpers J0-J4, bit n = Jn, 1 = pin high (manual sections 15.1 and 15.2)
	PORT_START("JUMPERS")
	PORT_CONFNAME( 0x07, 0x00, "Data Format (J2 J1 J0)" )
	PORT_CONFSETTING(    0x00, "000 Five Byte Packed Binary (Mouse Systems), Stream" )
	PORT_CONFSETTING(    0x04, "100 Three Byte Packed Binary, Prompt" )
	PORT_CONFSETTING(    0x02, "010 Hexadecimal, Prompt" )
	PORT_CONFSETTING(    0x06, "110 Relative Bit Pad One, Stream" )
	PORT_CONFSETTING(    0x01, "001 Reserved" )
	PORT_CONFSETTING(    0x05, "101 MM Series, Stream" )
	PORT_CONFSETTING(    0x03, "011 Absolute Bit Pad One, Stream" )
	PORT_CONFSETTING(    0x07, "111 Microsoft Compatible, Stream" )
	// Hexadecimal at 4800 baud (01010) selects MM Series with Auto Baud instead
	PORT_CONFNAME( 0x18, 0x00, "Baud Rate (J4 J3)" )
	PORT_CONFSETTING(    0x00, "00 1200" )
	PORT_CONFSETTING(    0x08, "01 2400" )
	PORT_CONFSETTING(    0x10, "10 4800" )
	PORT_CONFSETTING(    0x18, "11 9600" )
INPUT_PORTS_END

} // anonymous namespace


ROM_START(logimouse_c7)
	ROM_REGION(0x800, "mcu", 0)
	ROM_LOAD("lsc82406p_c7_3.2.bin", 0x000, 0x800, CRC(bdb50812) SHA1(59c26448161cd3d15dc57827362e11c3f237f571))
ROM_END


logitech_c7_mouse_device::logitech_c7_mouse_device(machine_config const &mconfig, char const *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, LOGITECH_C7_SERIAL_MOUSE, tag, owner, clock)
	, device_rs232_port_interface(mconfig, *this)
	, m_mcu(*this, "mcu")
	, m_buttons(*this, "BTN")
	, m_x_axis(*this, "X")
	, m_y_axis(*this, "Y")
	, m_jumpers(*this, "JUMPERS")
	, m_port_a_out(0xff)
	, m_port_b_out(0xff)
	, m_rts(1)
	, m_x_last(0)
	, m_y_last(0)
	, m_x_count(0)
	, m_y_count(0)
	, m_x_phase(0)
	, m_y_phase(0)
	, m_quadrature_timer(nullptr)
{
}

const tiny_rom_entry *logitech_c7_mouse_device::device_rom_region() const
{
	return ROM_NAME(logimouse_c7);
}

void logitech_c7_mouse_device::device_add_mconfig(machine_config &config)
{
	M146805F2(config, m_mcu, 4'000'000); // 4 MHz crystal
	m_mcu->porta_r().set(FUNC(logitech_c7_mouse_device::mcu_port_a_r));
	m_mcu->portb_r().set(FUNC(logitech_c7_mouse_device::mcu_port_b_r));
	m_mcu->portc_r().set(FUNC(logitech_c7_mouse_device::mcu_port_c_r));
	m_mcu->porta_w().set(FUNC(logitech_c7_mouse_device::mcu_port_a_w));
	m_mcu->portb_w().set(FUNC(logitech_c7_mouse_device::mcu_port_b_w));
}

ioport_constructor logitech_c7_mouse_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(logimouse_c7);
}

void logitech_c7_mouse_device::device_resolve_objects()
{
	m_rts = 1; // deasserted until the host drives it
}

void logitech_c7_mouse_device::device_start()
{
	save_item(NAME(m_port_a_out));
	save_item(NAME(m_port_b_out));
	save_item(NAME(m_rts));
	save_item(NAME(m_x_last));
	save_item(NAME(m_y_last));
	save_item(NAME(m_x_count));
	save_item(NAME(m_y_count));
	save_item(NAME(m_x_phase));
	save_item(NAME(m_y_phase));

	m_quadrature_timer = timer_alloc(FUNC(logitech_c7_mouse_device::update_quadrature), this);
}

void logitech_c7_mouse_device::device_reset()
{
	m_port_a_out = 0xff;
	m_port_b_out = 0xff;
	m_x_last = m_x_axis->read();
	m_y_last = m_y_axis->read();
	m_x_count = 0;
	m_y_count = 0;
	m_x_phase = 0;
	m_y_phase = 0;

	m_quadrature_timer->adjust(attotime::from_hz(1000), 0, attotime::from_hz(1000));
}

void logitech_c7_mouse_device::input_txd(int state)
{
	// RX is sampled on the IRQ pin, mark = high
	m_mcu->set_input_line(M6805_IRQ_LINE, state ? CLEAR_LINE : ASSERT_LINE);
}

void logitech_c7_mouse_device::input_rts(int state)
{
	m_rts = state ? 1 : 0;
}

//
// MCU port callbacks
//

uint8_t logitech_c7_mouse_device::mcu_port_a_r(offs_t offset, uint8_t mem_mask)
{
	uint8_t const jumpers = m_jumpers->read();
	uint8_t result = 0x00; // R0-R2 straps low, PA3 is an output

	if (BIT(jumpers, 3)) result |= 0x10; // J3
	if (BIT(jumpers, 4)) result |= 0x20; // J4
	if (BIT(jumpers, 2)) result |= 0x40; // J2
	if (BIT(jumpers, 1)) result |= 0x80; // J1

	return result;
}

uint8_t logitech_c7_mouse_device::mcu_port_b_r(offs_t offset, uint8_t mem_mask)
{
	// The firmware forms the button state as (PB0-2 & previous) | ~(PB3-5 >> 3):
	// PB3-PB5 read high while released, PB0-PB2 add hysteresis.
	// TODO: PB1/PB2 have not been traced and read low here
	uint8_t const buttons = m_buttons->read();
	uint8_t const jumpers = m_jumpers->read();
	uint8_t result = 0x00;

	if (BIT(buttons, 2))
		result |= 0x01; // right pressed
	if (!m_rts && !BIT(buttons, 2))
		result |= 0x08; // RTS asserted and right released
	if (!BIT(buttons, 1))
		result |= 0x10; // middle released
	if (!BIT(buttons, 0))
		result |= 0x20; // left released
	if (BIT(jumpers, 0))
		result |= 0x40; // J0

	return result;
}

uint8_t logitech_c7_mouse_device::mcu_port_c_r(offs_t offset, uint8_t mem_mask)
{
	// PC0/PC1 X and PC2/PC3 Y quadrature, Gray code sequence 0, 1, 3, 2
	static const uint8_t gray_code[4] = { 0x00, 0x01, 0x03, 0x02 };
	return gray_code[m_x_phase & 3] | (gray_code[m_y_phase & 3] << 2);
}

void logitech_c7_mouse_device::mcu_port_a_w(offs_t offset, uint8_t data, uint8_t mem_mask)
{
	// PA3 (encoder LED supply?) has no effect on the emulated encoders
	uint8_t const changed = m_port_a_out ^ data;
	m_port_a_out = data;
	if (changed)
		LOGMASKED(LOG_PORT_A, "Port A write: %02X (mask %02X)\n", data, mem_mask);
}

void logitech_c7_mouse_device::mcu_port_b_w(offs_t offset, uint8_t data, uint8_t mem_mask)
{
	uint8_t const changed = m_port_b_out ^ data;
	m_port_b_out = data;

	// PB7 is the TX data line, mark = 1
	if (changed & 0x80)
	{
		output_rxd(BIT(data, 7));
		LOGMASKED(LOG_PORT_B, "Serial TX: %d\n", BIT(data, 7));
	}
}

TIMER_CALLBACK_MEMBER(logitech_c7_mouse_device::update_quadrature)
{
	// The firmware samples the encoders once per timer tick (about 1 kHz),
	// so movement is queued and released one quadrature step per tick.
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
	m_x_count += x_delta;
	m_y_count += y_delta;

	if (m_x_count > 0)
	{
		m_x_phase = (m_x_phase + 1) & 3;
		m_x_count--;
	}
	else if (m_x_count < 0)
	{
		m_x_phase = (m_x_phase - 1) & 3;
		m_x_count++;
	}

	// Y is inverted: moving down produces the forward sequence
	if (m_y_count > 0)
	{
		m_y_phase = (m_y_phase - 1) & 3;
		m_y_count--;
	}
	else if (m_y_count < 0)
	{
		m_y_phase = (m_y_phase + 1) & 3;
		m_y_count++;
	}
}
