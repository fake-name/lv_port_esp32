#pragma GCC diagnostic warning "-Wall"
#include "RA8876.hpp"
#include <cstring>

#include "util.hpp"
#include "esp_task_wdt.h"
// #define RA8876_DEBUG


SdramInfo defaultSdramInfo =
{
	120, // 120 MHz
	3,   // CAS latency 3
	4,   // 4 banks
	12,  // 12-bit row addresses
	9,   // 9-bit column addresses
	64   // 64 millisecond refresh time
};

DisplayInfo defaultDisplayInfo =
{
	1024,   // Display width
	600,    // Display height
	50000,  // Pixel clock in kHz

	160,    // Horizontal front porch
	160,    // Horizontal back porch
	70,     // HSYNC pulse width

	12,     // Vertical front porch
	23,     // Vertical back porch
	10      // VSYNC pulse width
};



void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length=8;                     //Command is 8 bits
	t.tx_buffer=&cmd;               //The data is the cmd itself
	t.user=(void*)0;                //D/C needs to be set to 0
	ret=spi_device_polling_transmit(spi, &t);  //Transmit!
	assert(ret==ESP_OK);            //Should have had no issues.
}

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
	esp_err_t ret;
	spi_transaction_t t;
	if (len==0) return;             //no need to send anything
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length=len*8;                 //Len is in bytes, transaction length is in bits.
	t.tx_buffer=data;               //Data
	t.user=(void*)1;                //D/C needs to be set to 1
	ret=spi_device_polling_transmit(spi, &t);  //Transmit!
	assert(ret==ESP_OK);            //Should have had no issues.
}

void RA8876::write_cmd(uint8_t x)
{
	// Output/input buffers need to be aligned to 4 byte boundaries
	// and at least 32 bits, or the ESP-IDF will copy and re-allocate them
	// so DMA can operate properly.
	uint8_t out_data[4] __attribute__ ((aligned (4)));
	out_data[0] = RA8876_CMD_WRITE;
	out_data[1] = x;

	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length=16;                    //Command is 8 bits
	t.tx_buffer=&out_data;          //The data is the cmd itself
	ret=spi_device_polling_transmit(spi, &t);  //Transmit!
	assert(ret==ESP_OK);            //Should have had no issues.
}

void RA8876::write_data(uint8_t x)
{

	// Output/input buffers need to be aligned to 4 byte boundaries
	// and at least 32 bits, or the ESP-IDF will copy and re-allocate them
	// so DMA can operate properly.
	uint8_t out_data[4] __attribute__ ((aligned (4)));
	out_data[0] = RA8876_DATA_WRITE;
	out_data[1] = x;

	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length=16;                    //Command is 8 bits
	t.tx_buffer=&out_data;          //The data is the cmd itself
	ret=spi_device_polling_transmit(spi, &t);  //Transmit!
	assert(ret==ESP_OK);            //Should have had no issues.

}

void RA8876::write_data16(uint16_t x)
{

	// Output/input buffers need to be aligned to 4 byte boundaries
	// and at least 32 bits, or the ESP-IDF will copy and re-allocate them
	// so DMA can operate properly.
	uint8_t out_data[4] __attribute__ ((aligned (4)));
	out_data[0] = RA8876_DATA_WRITE;
	out_data[1] = x & 0xFF;
	out_data[2] = (x >> 8) & 0xFF;

	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length=24;                    //Command is 8 bits
	t.tx_buffer=&out_data;          //The data is the cmd itself
	ret=spi_device_polling_transmit(spi, &t);  //Transmit!
	assert(ret==ESP_OK);            //Should have had no issues.

}

uint8_t RA8876::read_data(void)
{

	// Output/input buffers need to be aligned to 4 byte boundaries
	// and at least 32 bits, or the ESP-IDF will copy and re-allocate them
	// so DMA can operate properly.
	uint8_t out_data[4] __attribute__ ((aligned (4)));
	uint8_t in_data[4] __attribute__ ((aligned (4)));
	out_data[0] = RA8876_DATA_READ;
	out_data[1] = 0;
	in_data[0] = 0;
	in_data[1] = 0;

	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length=16;                    //Command is 8 bits
	t.tx_buffer=&out_data;          //The data is the cmd itself
	t.rx_buffer=&in_data;           //The data is the cmd itself
	ret=spi_device_polling_transmit(spi, &t);  //Transmit!
	assert(ret==ESP_OK);            //Should have had no issues.

	// printf("Read - 0: 0x%02x, 1: 0x%02x\n", in_data[0], in_data[1]);
	return in_data[1];

}


// Reads the special status register.
// This register uses a special cycle type instead of having an address like other registers.
// See data sheet section 19.1.
uint8_t RA8876::read_status(void)
{
	// Output/input buffers need to be aligned to 4 byte boundaries
	// and at least 32 bits, or the ESP-IDF will copy and re-allocate them
	// so DMA can operate properly.
	uint8_t out_data[4] __attribute__ ((aligned (4)));
	uint8_t in_data[4] __attribute__ ((aligned (4)));
	out_data[0] = RA8876_STATUS_READ;
	out_data[1] = 0;
	in_data[0] = 0;
	in_data[1] = 0;

	esp_err_t ret;
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.length=16;                    //Command is 8 bits
	t.tx_buffer=&out_data;          //The data is the cmd itself
	t.rx_buffer=&in_data;           //The data is the cmd itself
	ret=spi_device_polling_transmit(spi, &t);  //Transmit!
	assert(ret==ESP_OK);            //Should have had no issues.

	return in_data[1];

}

void RA8876::write_reg(uint8_t reg, uint8_t x)
{
	// printf("Write reg: 0x%02x -> 0x%02x ....", reg, x);
	this->write_cmd(reg);
	this->write_data(x);
	// printf("Done\n");
}

uint8_t RA8876::read_reg(uint8_t reg)
{
	this->write_cmd(reg);
	return this->read_data();
}

// Like write_reg(), but does two successive register writes of a 16-bit value, low byte first.
void RA8876::write_reg16(uint8_t reg, uint16_t x)
{
	this->write_cmd(reg);
	this->write_data(x & 0xFF);
	this->write_cmd(reg + 1);
	this->write_data(x >> 8);
}

// The RA8876 doesn't seem to allow you to do read/writes larger then 1 byte.
// Ugh.
//
// Like write_reg(), but does two successive register writes of a 16-bit value, low byte first.
void RA8876::write_reg16bbp(uint8_t reg, uint16_t x)
{
	this->write_cmd(reg);
	this->write_data16(x);
	// this->write_cmd(reg + 1);
	// this->write_data(x >> 8);
}

uint16_t RA8876::read_reg16(uint8_t reg)
{
	uint16_t v;

	this->write_cmd(reg);
	v = this->read_data();
	this->write_cmd(reg + 1);
	v |= this->read_data() << 8;

	return v;
}



void RA8876::wait_completion()
{
	// Wait for completion
	uint8_t status = this->read_status();
	int iter = 0;
	while (status & 0x08)
	{
		status = this->read_status();
		iter++;
	}

}


RA8876::RA8876(
		gpio_num_t cs_pin,
		gpio_num_t mosi_pin,
		gpio_num_t miso_pin,
		gpio_num_t clk_pin,
		gpio_num_t wait_pin,
		gpio_num_t int_pin,
		gpio_num_t reset_pin,
		gpio_num_t backlight_pin
	)
	: cs_pin(cs_pin)
	, mosi_pin(mosi_pin)
	, miso_pin(miso_pin)
	, clk_pin(clk_pin)
	, wait_pin(wait_pin)
	, int_pin(int_pin)
	, reset_pin(reset_pin)
	, backlight_pin(backlight_pin)
{

	this->width  = 0;
	this->height = 0;
	this->depth  = 0;


}

// Trigger a hardware reset.
void RA8876::hard_reset(void)
{
	delay_ms(5);
	gpio_set_level(this->reset_pin, 0);
	delay_ms(5);
	gpio_set_level(this->reset_pin, 1);
	delay_ms(5);

	return;
}

// Trigger a soft reset. Note that the data sheet section 19.2 says that this only resets the
//  "internal state machine", not any configuration registers.
void RA8876::soft_reset(void)
{
	// SPI.beginTransaction(this->spiSettings);

	// Trigger soft reset
	this->write_reg(RA8876_REG_SRR, 0x01);
	delay_ms(5);

	// Wait for status register to show "normal operation".
	uint8_t status;
	for (int i = 0; i < 250; i++)
	{
		delay_ms(1);

		if (((status = this->read_status()) & 0x02) == 0)
			break;
	}

	// SPI.endTransaction();

	return;
}

// Given a target frequency in kHz, finds PLL parameters k and n to reach as
//  close as possible to the target frequency without exceeding it.
// The k parameter will be constrained to the range 1..kMax.
// Returns true iff PLL params were found, even if not an exact match.
bool RA8876::calc_pll_params(uint32_t targetFreq, int kMax, PllParams *pll)
{
	bool found = false;
	int foundk = -1;
	int foundn = -1;
	uint32_t foundFreq = 0;
	uint32_t foundError = 0;  // Amount lower than requested frequency

	// k of 0 (i.e. 2 ** 0 = 1) is possible, but not sure if it's a good idea.
	for (int testk = 1; testk <= kMax; testk++)
	{
		if (this->oscClock % (1 << testk))
			continue;  // Step size with this k would be fractional

		int testn = (targetFreq / (this->oscClock / (1 << testk))) - 1;
		if ((testn < 1) || (testn > 63))
			continue;  // Param n out of range for this k

		// Fvco constraint found in data sheet section 6.1.2
		uint32_t fvco = this->oscClock * (testn + 1);
		if ((fvco < 100000) && (fvco > 600000))
			continue;  // Fvco out of range

		// Found some usable params, but perhaps at a lower frequency than requested.
		uint32_t freq = (this->oscClock * (testn + 1)) / (1 << testk);
		uint32_t error = targetFreq - freq;
		if ((!found) || (found && (foundError > error)))
		{
			found = true;
			foundk = testk;
			foundn = testn;
			foundFreq = freq;
			foundError = error;

			// No need to keep searching if the frequency match was exact
			if (foundError == 0)
				break;
		}
	}

	if (found)
	{
		pll->freq = foundFreq;
		pll->k    = foundk;
		pll->n    = foundn;
	}

	return found;
}

// Calculates the clock frequencies and their PLL parameters.
bool RA8876::calc_clocks(void)
{
	// Data sheet section 5.2 gives max clocks:
	//  memClock : 166 MHz
	//  coreClock: 120 MHz (133MHz if not using internal font)
	//  scanClock: 100 MHz

	// Mem clock target is the same as SDRAM speed, but capped at 166 MHz
	uint32_t memClock = this->sdramInfo->speed * 1000;
	if (memClock > 166000)
		memClock = 166000;

	if (!this->calc_pll_params(memClock, 3, &this->memPll))
		return false;

	// Core clock target will be the same as the mem clock, but capped to
	//  120 MHz, because that is the max frequency if we want to use the
	//  internal font.
	uint32_t coreClock = this->memPll.freq;
	if (coreClock > 120000)
		coreClock = 120000;

	if (!this->calc_pll_params(coreClock, 3, &this->corePll))
		return false;

	// Scan clock target will be the display's dot clock, but capped at 100 MHz
	uint32_t scanClock = this->displayInfo->dotClock;
	if (scanClock > 100000)
		scanClock = 100000;

	if (!this->calc_pll_params(scanClock, 7, &this->scanPll))
		return false;

	this->dump_clocks();

	// Data sheet section 6.1.1 rules:
	// 1. Core clock must be less than or equal to mem clock
	if (this->corePll.freq > this->memPll.freq)
		return false;

	// 2. Core clock must be greater than half mem clock
	if ((this->corePll.freq * 2) <= this->memPll.freq)
		return false;

	// 3. Core clock must be greater than (scan clock * 1.5)
	if (this->corePll.freq <= (this->scanPll.freq + (this->scanPll.freq >> 1)))
		return false;

	return true;
}

// Dump clock info to serial monitor.
void RA8876::dump_clocks(void)
{
	#if defined(RA8876_DEBUG)
	Serial.println("\nMem\n---");
	Serial.print("Requested kHz: ");
	Serial.println(this->sdramInfo->speed * 1000);
	Serial.print("Actual kHz   : ");
	Serial.println(this->memPll.freq);
	Serial.print("PLL k        : ");
	Serial.println(this->memPll.k);
	Serial.print("PLL n        : ");
	Serial.println(this->memPll.n);

	Serial.println("\nCore\n----");
	Serial.print("kHz          : ");
	Serial.println(this->corePll.freq);
	Serial.print("PLL k        : ");
	Serial.println(this->corePll.k);
	Serial.print("PLL n        : ");
	Serial.println(this->corePll.n);

	Serial.println("\nScan\n----");
	Serial.print("Requested kHz: ");
	Serial.println(this->displayInfo->dotClock);
	Serial.print("Actual kHz   : ");
	Serial.println(this->scanPll.freq);
	Serial.print("PLL k        : ");
	Serial.println(this->scanPll.k);
	Serial.print("PLL n        : ");
	Serial.println(this->scanPll.n);
	#endif // RA8876_DEBUG

	// TODO: Frame rate?

	return;
}

bool RA8876::init_pll(void)
{
	#if defined(RA8876_DEBUG)
	Serial.println("init PLL");
	#endif // RA8876_DEBUG

	// SPI.beginTransaction(this->spiSettings);

	//Serial.print("DRAM_FREQ ");
	//	Serial.println(this->memPll.freq);
	//Serial.print("7: ");
	//	Serial.println(this->memPll.k << 1);
	//Serial.print("8: ");
	//	Serial.println(this->memPll.n);
	this->write_reg(RA8876_REG_MPLLC1, this->memPll.k << 1);
	this->write_reg(RA8876_REG_MPLLC2, this->memPll.n);

	//Serial.print("CORE_FREQ ");
	//	Serial.println(this->corePll.freq);
	//Serial.print("9: ");
	//	Serial.println(this->corePll.k << 1);
	//Serial.print("A: ");
	//	Serial.println(this->corePll.n);
	this->write_reg(RA8876_REG_SPLLC1, this->corePll.k << 1);
	this->write_reg(RA8876_REG_SPLLC2, this->corePll.n);

	// Per the data sheet, there are two divider fields for the scan clock, but the math seems
	//  to work out if we treat k as a single 3-bit number in bits 3..1.
	//Serial.print("SCAN_FREQ ");
	//	Serial.println(this->scanPll.freq);
	//Serial.print("5: ");
	//	Serial.println(this->scanPll.k << 1);
	//Serial.print("6: ");
	//	Serial.println(this->scanPll.n);
	this->write_reg(RA8876_REG_PPLLC1, this->scanPll.k << 1);
	this->write_reg(RA8876_REG_PPLLC2, this->scanPll.n);

	// Toggle bit 7 of the CCR register to trigger a reconfiguration of the PLLs
	this->write_reg(RA8876_REG_CCR, 0x00);
	delay_ms(2);
	this->write_reg(RA8876_REG_CCR, 0x80);
	delay_ms(2);

	uint8_t ccr = this->read_reg(RA8876_REG_CCR);

	// SPI.endTransaction();

	return (ccr & 0x80) ? true : false;
}

// Initialize SDRAM interface.
bool RA8876::init_memory(SdramInfo *info)
{
	#if defined(RA8876_DEBUG)
	Serial.println("init memory");
	#endif // RA8876_DEBUG

	uint32_t sdramRefreshRate;
	uint8_t sdrar = 0x00;
	uint8_t sdrmd = 0x00;

	// Refresh rate
	sdramRefreshRate = ((uint32_t) info->refresh * info->speed * 1000) >> info->rowBits;

	// Number of banks
	if (info->banks == 2)
		;  // NOP
	else if (info->banks == 4)
		sdrar |= 0x20;
	else
		return false;  // Unsupported number of banks

	// Number of row bits
	if ((info->rowBits < 11) || (info->rowBits > 13))
		return false;  // Unsupported number of row bits
	else
		sdrar |= ((info->rowBits - 11) & 0x03) << 3;

	// Number of column bits
	if ((info->colBits < 8) || (info->colBits > 12))
		return false;  // Unsupported number of column bits
	else
		sdrar |= info->colBits & 0x03;

	// CAS latency
	if ((info->casLatency < 2) || (info->casLatency > 3))
		return false;  // Unsupported CAS latency
	else
		sdrmd |= info->casLatency & 0x03;

	// SPI.beginTransaction(this->spiSettings);

	#if defined(RA8876_DEBUG)
	Serial.print("SDRAR: ");
	Serial.println(sdrar);  // Expected: 0x29 (41 decimal)
	#endif // RA8876_DEBUG
	this->write_reg(RA8876_REG_SDRAR, sdrar);

	#if defined(RA8876_DEBUG)
	Serial.print("SDRMD: ");
	Serial.println(sdrmd);
	#endif // RA8876_DEBUG
	this->write_reg(RA8876_REG_SDRMD, sdrmd);

	#if defined(RA8876_DEBUG)
	Serial.print("sdramRefreshRate: ");
	Serial.println(sdramRefreshRate);
	#endif // RA8876_DEBUG
	this->write_reg(RA8876_REG_SDR_REF_ITVL0, sdramRefreshRate & 0xFF);
	this->write_reg(RA8876_REG_SDR_REF_ITVL1, sdramRefreshRate >> 8);

	// Trigger SDRAM initialization
	this->write_reg(RA8876_REG_SDRCR, 0x01);

	// Wait for SDRAM to be ready
	uint8_t status;
	for (int i = 0; i < 250; i++)
	{
		delay_ms(1);

		if ((status = this->read_status()) & 0x40)
			break;
	}

	// SPI.endTransaction();

	#if defined(RA8876_DEBUG)
	Serial.print("Status: ");
	Serial.println(status);
	#endif // RA8876_DEBUG

	return (status & 0x40) ? true : false;
}

bool RA8876::init_display()
{
	// SPI.beginTransaction(this->spiSettings);

	// Set chip config register
	uint8_t ccr = this->read_reg(RA8876_REG_CCR);
	ccr &= 0xE7;  // 24-bit LCD output
	ccr &= 0xFE;  // 8-bit host data bus
	this->write_reg(RA8876_REG_CCR, ccr);

	this->write_reg(RA8876_REG_MACR, 0x00);  // Direct write, left-to-right-top-to-bottom memory

	this->write_reg(RA8876_REG_ICR, 0x00);  // Graphics mode, memory is SDRAM

	uint8_t dpcr = this->read_reg(RA8876_REG_DPCR);
	dpcr &= 0xFB;  // Vertical scan top to bottom
	dpcr &= 0xF8;  // Colour order RGB
	dpcr |= 0x80;  // Panel fetches PDAT at PCLK falling edge
	this->write_reg(RA8876_REG_DPCR, dpcr);

	uint8_t pcsr = this->read_reg(RA8876_REG_PCSR);
	pcsr |= 0x80;  // XHSYNC polarity high
	pcsr |= 0x40;  // XVSYNC polarity high
	pcsr &= 0xDF;  // XDE polarity high
	this->write_reg(RA8876_REG_PCSR, pcsr);

	// Set display width
	this->write_reg(RA8876_REG_HDWR, (this->displayInfo->width / 8) - 1);
	this->write_reg(RA8876_REG_HDWFTR, (this->displayInfo->width % 8));

	// Set display height
	this->write_reg(RA8876_REG_VDHR0, (this->displayInfo->height - 1) & 0xFF);
	this->write_reg(RA8876_REG_VDHR1, (this->displayInfo->height - 1) >> 8);

	// Set horizontal non-display (back porch)
	this->write_reg(RA8876_REG_HNDR, (this->displayInfo->hBackPorch / 8) - 1);
	this->write_reg(RA8876_REG_HNDFTR, (this->displayInfo->hBackPorch % 8));

	// Set horizontal start position (front porch)
	this->write_reg(RA8876_REG_HSTR, ((this->displayInfo->hFrontPorch + 4) / 8) - 1);

	// Set HSYNC pulse width
	this->write_reg(RA8876_REG_HPWR, ((this->displayInfo->hPulseWidth + 4) / 8) - 1);

	// Set vertical non-display (back porch)
	this->write_reg(RA8876_REG_VNDR0, (this->displayInfo->vBackPorch - 1) & 0xFF);
	this->write_reg(RA8876_REG_VNDR1, (this->displayInfo->vBackPorch - 1) >> 8);

	// Set vertical start position (front porch)
	this->write_reg(RA8876_REG_VSTR, this->displayInfo->vFrontPorch - 1);

	// Set VSYNC pulse width
	this->write_reg(RA8876_REG_VPWR, this->displayInfo->vPulseWidth - 1);

	// Set main window to 16 bits per pixel
	this->write_reg(RA8876_REG_MPWCTR, 0x04);  // PIP windows disabled, 16-bpp, enable sync signals

	// Set main window start address to 0
	this->write_reg(RA8876_REG_MISA0, 0);
	this->write_reg(RA8876_REG_MISA1, 0);
	this->write_reg(RA8876_REG_MISA2, 0);
	this->write_reg(RA8876_REG_MISA3, 0);

	// Set main window image width
	this->write_reg(RA8876_REG_MIW0, this->width & 0xFF);
	this->write_reg(RA8876_REG_MIW1, this->width >> 8);

	// Set main window start coordinates
	this->write_reg(RA8876_REG_MWULX0, 0);
	this->write_reg(RA8876_REG_MWULX1, 0);
	this->write_reg(RA8876_REG_MWULY0, 0);
	this->write_reg(RA8876_REG_MWULY1, 0);

	// Set canvas start address
	this->write_reg(RA8876_REG_CVSSA0, 0);
	this->write_reg(RA8876_REG_CVSSA1, 0);
	this->write_reg(RA8876_REG_CVSSA2, 0);
	this->write_reg(RA8876_REG_CVSSA3, 0);

	// Set canvas width
	this->write_reg(RA8876_REG_CVS_IMWTH0, this->width & 0xFF);
	this->write_reg(RA8876_REG_CVS_IMWTH1, this->width >> 8);

	// Set active window start coordinates
	this->write_reg(RA8876_REG_AWUL_X0, 0);
	this->write_reg(RA8876_REG_AWUL_X1, 0);
	this->write_reg(RA8876_REG_AWUL_Y0, 0);
	this->write_reg(RA8876_REG_AWUL_Y1, 0);

	// Set active window dimensions
	this->write_reg(RA8876_REG_AW_WTH0, this->width & 0xFF);
	this->write_reg(RA8876_REG_AW_WTH1, this->width >> 8);
	this->write_reg(RA8876_REG_AW_HT0, this->height & 0xFF);
	this->write_reg(RA8876_REG_AW_HT1, this->height >> 8);

	// Set canvas addressing mode/colour depth
	uint8_t aw_color = 0x00;  // 2d addressing mode
	if (this->depth == 16)
		aw_color |= 0x01;
	else if (this->depth == 24)
		aw_color |= 0x02;
	this->write_reg(RA8876_REG_AW_COLOR, aw_color);

	// Turn on display
	dpcr = this->read_reg(RA8876_REG_DPCR);
	dpcr |= 0x40;  // Display on
	this->write_reg(RA8876_REG_DPCR, dpcr);

	// TODO: Track backlight pin and turn on backlight

	// SPI.endTransaction();

	return true;
}

bool RA8876::init(void)
{

	// // Set up chip select pin
	// pinMode(this->cs_pin, OUTPUT);



	// pinMode(this->backlight_pin, OUTPUT);

	// pinMode(this->reset_pin, OUTPUT);

	// SPI.begin(
	// 		this->clk_pin,       // sck,
	// 		this->miso_pin,      // miso,
	// 		this->mosi_pin,      // mosi,
	// 		this->cs_pin        // ss
	// 	);

	// this->spiSettings = SPISettings(RA8876_SPI_SPEED, MSBFIRST, SPI_MODE3);

	memset(&(this->buscfg), 0, sizeof(spi_bus_config_t));
	memset(&(this->devcfg), 0, sizeof(spi_device_interface_config_t));

	this->oscClock            = 10 * 1000;  // 10000kHz or 10MHz
	this->sdramInfo           = &defaultSdramInfo;
	this->displayInfo         = &defaultDisplayInfo;
	this->textColor           = 0xFFFF; // White
	this->fontRomInfo.present = false;  // No external font ROM chip

	this->buscfg.miso_io_num     = this->cs_pin;
	this->buscfg.mosi_io_num     = this->mosi_pin;
	this->buscfg.sclk_io_num     = this->clk_pin;
	this->buscfg.miso_io_num     = this->miso_pin;
	this->buscfg.quadwp_io_num   = -1;
	this->buscfg.quadhd_io_num   = -1;
	this->buscfg.max_transfer_sz = 32*320*2+8;

	this->devcfg.clock_speed_hz  = 1*1000*1000;           //Clock out at 10 MHz
	this->devcfg.mode            = 3;                                //SPI mode 3
	this->devcfg.spics_io_num    = this->cs_pin;               //CS pin
	this->devcfg.queue_size      = 7;                          //We want to be able to queue 7 transactions at a time

	// I'm pretty sure the peripheral configures this automatically.
	// Do it anyways.
	gpio_set_direction(cs_pin,        GPIO_MODE_OUTPUT);
	gpio_set_direction(mosi_pin,      GPIO_MODE_OUTPUT);
	gpio_set_direction(clk_pin,       GPIO_MODE_OUTPUT);
	gpio_set_direction(miso_pin,      GPIO_MODE_INPUT);

	//Initialize the SPI bus
	ret = spi_bus_initialize(HSPI_HOST, &(this->buscfg), 1);
	ESP_ERROR_CHECK(ret);

	//Attach the LCD to the SPI bus
	ret = spi_bus_add_device(HSPI_HOST, &(this->devcfg), &(this->spi));
	ESP_ERROR_CHECK(ret);

	// Other GPIO components
	gpio_set_direction(reset_pin,     GPIO_MODE_OUTPUT);
	gpio_set_direction(backlight_pin, GPIO_MODE_OUTPUT);
	gpio_set_direction(wait_pin,      GPIO_MODE_INPUT);
	gpio_set_direction(int_pin,       GPIO_MODE_INPUT);

	gpio_set_level(this->cs_pin,        1);
	gpio_set_level(this->backlight_pin, 1);
	gpio_set_level(this->reset_pin,     1);

	this->width  = this->displayInfo->width;
	this->height = this->displayInfo->height;
	this->depth  = 16;

	this->hard_reset();

	if (!this->calc_clocks())
	{
		printf("calc_clocks failed\n");
		return false;
	}

	if (!this->init_pll())
	{
		printf("init_pll failed\n");
		return false;
	}

	if (!this->init_memory(this->sdramInfo))
	{
		printf("init_memory failed\n");
		return false;
	}

	if (!this->init_display())
	{
		printf("init_display failed\n");
		return false;
	}

	// Set default font
	this->selectInternalFont(RA8876_FONT_SIZE_16);
	this->setTextScale(1);

	return true;
}

void RA8876::initExternalFontRom(int spiIf, enum ExternalFontRom chip)
{
	// See data sheet figure 16-10
	// TODO: GT30L24T3Y supports FAST_READ command (0x0B) and runs at 20MHz. Are the other font chips the same?

	// Find a clock divisor. Values are in the range 2..512 in steps of 2.
	int divisor;
	uint32_t speed = 20000;  // 20MHz target speed
	for (divisor = 2; divisor <= 512; divisor += 2)
	{
		if (this->corePll.freq / divisor <= speed)
			break;
	}

	this->fontRomInfo.present = true;
	this->fontRomInfo.spiInterface = spiIf;
	this->fontRomInfo.spiClockDivisor = divisor;
	this->fontRomInfo.chip = chip;

	#if defined(RA8876_DEBUG)
	Serial.print("External font SPI divisor: ");
	Serial.println(divisor);
	#endif // RA8876_DEBUG

	// SPI.beginTransaction(this->spiSettings);

	// Ensure SPI is enabled in chip config register
	uint8_t ccr = this->read_reg(RA8876_REG_CCR);
	if (!(ccr & 0x02))
		this->write_reg(RA8876_REG_CCR, ccr | 0x02);

	#if defined(RA8876_DEBUG)
	Serial.print("SFL_CTRL: ");
	Serial.println(((spiIf & 1) << 7) | 0x14, HEX);
	#endif // RA8876_DEBUG
	this->write_reg(RA8876_REG_SFL_CTRL, ((spiIf & 1) << 7) | 0x14);  // Font mode, 24-bit address, standard timing, supports FAST_READ
	#if defined(RA8876_DEBUG)
	Serial.print("SPI_DIVSOR: ");
	Serial.println((divisor >> 1) - 1, HEX);
	#endif // RA8876_DEBUG
	this->write_reg(RA8876_REG_SPI_DIVSOR, (divisor >> 1) - 1);
	#if defined(RA8876_DEBUG)
	Serial.print("GTFNT_SEL: ");
	Serial.println((chip & 0x07) << 5, HEX);
	#endif // RA8876_DEBUG
	this->write_reg(RA8876_REG_GTFNT_SEL, (chip & 0x07) << 5);

	// SPI.endTransaction();
}

// Show colour bars of 8 colours in repeating horizontal bars.
// This does not alter video memory, but rather instructs the video controller to display
//  the pattern rather than the contents of memory.
void RA8876::colorBarTest(bool enabled)
{
	// SPI.beginTransaction(this->spiSettings);

	uint8_t dpcr = this->read_reg(RA8876_REG_DPCR);

	if (enabled)
		dpcr = dpcr | 0x20;
	else
		dpcr = dpcr & ~0x20;

	this->write_reg(RA8876_REG_DPCR, dpcr);

	// SPI.endTransaction();
}

void RA8876::drawPixel(int x, int y, uint16_t color)
{
	//Serial.println("drawPixel");
	//Serial.println(read_status());

	// SPI.beginTransaction(this->spiSettings);

	this->write_reg(RA8876_REG_CURH0, x & 0xFF);
	this->write_reg(RA8876_REG_CURH1, x >> 8);

	this->write_reg(RA8876_REG_CURV0, y & 0xFF);
	this->write_reg(RA8876_REG_CURV1, y >> 8);

	this->write_reg(RA8876_REG_MRWDP, color & 0xFF);
	this->write_reg(RA8876_REG_MRWDP, color >> 8);

	// SPI.endTransaction();
}

void RA8876::draw_two_point_shape(int x1, int y1, int x2, int y2, uint16_t color, uint8_t reg, uint8_t cmd)
{
	//Serial.println("drawTwoPointShape");

	// SPI.beginTransaction(this->spiSettings);

	// First point
	this->write_reg(RA8876_REG_DLHSR0, x1 & 0xFF);
	this->write_reg(RA8876_REG_DLHSR1, x1 >> 8);
	this->write_reg(RA8876_REG_DLVSR0, y1 & 0xFF);
	this->write_reg(RA8876_REG_DLVSR1, y1 >> 8);

	// Second point
	this->write_reg(RA8876_REG_DLHER0, x2 & 0xFF);
	this->write_reg(RA8876_REG_DLHER1, x2 >> 8);
	this->write_reg(RA8876_REG_DLVER0, y2 & 0xFF);
	this->write_reg(RA8876_REG_DLVER1, y2 >> 8);

	// Colour
	this->write_reg(RA8876_REG_FGCR, color >> 11 << 3);
	this->write_reg(RA8876_REG_FGCG, ((color >> 5) & 0x3F) << 2);
	this->write_reg(RA8876_REG_FGCB, (color & 0x1F) << 3);

	// Draw
	this->write_reg(reg, cmd);  // Start drawing

	// Wait for completion
	this->wait_completion();
	// SPI.endTransaction();
}

void RA8876::draw_three_point_shape(int x1, int y1, int x2, int y2, int x3, int y3, uint16_t color, uint8_t reg, uint8_t cmd)
{
	//Serial.println("drawThreePointShape");

	// SPI.beginTransaction(this->spiSettings);

	// First point
	this->write_reg(RA8876_REG_DLHSR0, x1 & 0xFF);
	this->write_reg(RA8876_REG_DLHSR1, x1 >> 8);
	this->write_reg(RA8876_REG_DLVSR0, y1 & 0xFF);
	this->write_reg(RA8876_REG_DLVSR1, y1 >> 8);

	// Second point
	this->write_reg(RA8876_REG_DLHER0, x2 & 0xFF);
	this->write_reg(RA8876_REG_DLHER1, x2 >> 8);
	this->write_reg(RA8876_REG_DLVER0, y2 & 0xFF);
	this->write_reg(RA8876_REG_DLVER1, y2 >> 8);

	// Third point
	this->write_reg(RA8876_REG_DTPH0, x3 & 0xFF);
	this->write_reg(RA8876_REG_DTPH1, x3 >> 8);
	this->write_reg(RA8876_REG_DTPV0, y3 & 0xFF);
	this->write_reg(RA8876_REG_DTPV1, y3 >> 8);

	// Colour
	this->write_reg(RA8876_REG_FGCR, color >> 11 << 3);
	this->write_reg(RA8876_REG_FGCG, ((color >> 5) & 0x3F) << 2);
	this->write_reg(RA8876_REG_FGCB, (color & 0x1F) << 3);

	// Draw
	this->write_reg(reg, cmd);  // Start drawing

	// Wait for completion
	this->wait_completion();

	//Serial.print(iter);
	//	Serial.println(" iterations");

	// SPI.endTransaction();
}

void RA8876::draw_ellipse_shape(int x, int y, int xrad, int yrad, uint16_t color, uint8_t cmd)
{
	//Serial.println("drawEllipseShape");

	// SPI.beginTransaction(this->spiSettings);

	// First point
	this->write_reg16(RA8876_REG_DEHR0, x);
	this->write_reg16(RA8876_REG_DEVR0, y);

	// Radii
	this->write_reg16(RA8876_REG_ELL_A0, xrad);
	this->write_reg16(RA8876_REG_ELL_B0, yrad);

	// Colour
	this->write_reg(RA8876_REG_FGCR, color >> 11 << 3);
	this->write_reg(RA8876_REG_FGCG, ((color >> 5) & 0x3F) << 2);
	this->write_reg(RA8876_REG_FGCB, (color & 0x1F) << 3);

	// Draw
	this->write_reg(RA8876_REG_DCR1, cmd);  // Start drawing

	// Wait for completion
	this->wait_completion();

}

void RA8876::setCursor(int x, int y)
{
	// SPI.beginTransaction(this->spiSettings);

	this->write_reg16(RA8876_REG_F_CURX0, x);
	this->write_reg16(RA8876_REG_F_CURY0, y);

	// SPI.endTransaction();
}

int RA8876::getCursorX(void)
{
	// SPI.beginTransaction(this->spiSettings);

	int x = this->read_reg16(RA8876_REG_F_CURX0);

	// SPI.endTransaction();

	return x;
}

int RA8876::getCursorY(void)
{
	// SPI.beginTransaction(this->spiSettings);

	int y = this->read_reg16(RA8876_REG_F_CURY0);

	// SPI.endTransaction();

	return y;
}

// Given a font encoding value, returns the corresponding bit pattern for
//  use by internal fonts.
uint8_t RA8876::internal_font_encoding(enum FontEncoding enc)
{
	uint8_t e;
	switch (enc)
	{
	case RA8876_FONT_ENCODING_8859_2:
		e = 0x01;
		break;
	case RA8876_FONT_ENCODING_8859_4:
		e = 0x02;
		break;
	case RA8876_FONT_ENCODING_8859_5:
		e = 0x03;
		break;
	default:
		e = 0x00;  // ISO-8859-1
		break;
	}

	return e;
}

void RA8876::set_text_mode(void)
{
	// Restore text colour
	this->write_reg(RA8876_REG_FGCR, this->textColor >> 11 << 3);
	this->write_reg(RA8876_REG_FGCG, ((this->textColor >> 5) & 0x3F) << 2);
	this->write_reg(RA8876_REG_FGCB, (this->textColor & 0x1F) << 3);

	this->wait_task_busy();

	// Enable text mode
	uint8_t icr = this->read_reg(RA8876_REG_ICR);
	this->write_reg(RA8876_REG_ICR, icr | 0x04);
}

void RA8876::set_graphics_mode(void)
{
	this->wait_task_busy();

	// Disable text mode
	uint8_t icr = this->read_reg(RA8876_REG_ICR);
	this->write_reg(RA8876_REG_ICR, icr & ~0x04);
}

void RA8876::selectInternalFont(enum FontSize size, enum FontEncoding enc)
{
	this->fontSource = RA8876_FONT_SOURCE_INTERNAL;
	this->fontSize   = size;
	this->fontFlags  = 0;

	// SPI.beginTransaction(this->spiSettings);

	this->write_reg(RA8876_REG_CCR0, 0x00 | ((size & 0x03) << 4) | this->internal_font_encoding(enc));

	uint8_t ccr1 = this->read_reg(RA8876_REG_CCR1);
	ccr1 |= 0x40;  // Transparent background
	this->write_reg(RA8876_REG_CCR1, ccr1);

	// SPI.endTransaction();
}

void RA8876::selectExternalFont(enum ExternalFontFamily family, enum FontSize size, enum FontEncoding enc, FontFlags flags)
{
	this->fontSource = RA8876_FONT_SOURCE_EXT_ROM;
	this->fontSize   = size;
	this->fontFlags  = flags;

	// SPI.beginTransaction(this->spiSettings);

	#if defined(RA8876_DEBUG)
	Serial.print("CCR0: ");
	Serial.println(0x40 | ((size & 0x03) << 4), HEX);
	#endif // RA8876_DEBUG
	this->write_reg(RA8876_REG_CCR0, 0x40 | ((size & 0x03) << 4));  // Select external font ROM and size

	uint8_t ccr1 = this->read_reg(RA8876_REG_CCR1);
	ccr1 |= 0x40;  // Transparent background
	#if defined(RA8876_DEBUG)
	Serial.print("CCR1: ");
	Serial.println(ccr1, HEX);
	#endif // RA8876_DEBUG
	this->write_reg(RA8876_REG_CCR1, ccr1);

	#if defined(RA8876_DEBUG)
	Serial.print("GTFNT_CR: ");
	Serial.println((enc << 3) | (family & 0x03), HEX);
	#endif // RA8876_DEBUG
	this->write_reg(RA8876_REG_GTFNT_CR, (enc << 3) | (family & 0x03));  // Character encoding and family

	// SPI.endTransaction();
}

int RA8876::getTextSizeY(void)
{
	return ((this->fontSize + 2) * 8) * this->textScaleY;
}

void RA8876::setTextScale(int xScale, int yScale)
{
	xScale = constrain(xScale, 1, 4);
	yScale = constrain(yScale, 1, 4);

	this->textScaleX = xScale;
	this->textScaleY = yScale;

	// SPI.beginTransaction(this->spiSettings);

	uint8_t ccr1 = this->read_reg(RA8876_REG_CCR1);
	ccr1 = (ccr1 & 0xF0) | ((xScale - 1) << 2) | (yScale - 1);
	#if defined(RA8876_DEBUG)
	Serial.print("ccr1: ");
	Serial.println(ccr1, HEX);
	#endif // RA8876_DEBUG
	this->write_reg(RA8876_REG_CCR1, ccr1);

	// SPI.endTransaction();
}

// Similar to write(), but does no special handling of control characters.
void RA8876::putChars(const char *buffer, size_t size)
{
	// SPI.beginTransaction(this->spiSettings);

	this->set_text_mode();

	// Write characters
	this->write_cmd(RA8876_REG_MRWDP);
	for (unsigned int i = 0; i < size; i++)
	{
		this->wait_write_fifo();
		this->write_data(buffer[i]);
	}

	this->set_graphics_mode();

	// SPI.endTransaction();
}

void RA8876::putChars16(const uint16_t *buffer, unsigned int count)
{
	// SPI.beginTransaction(this->spiSettings);

	this->set_text_mode();

	// Write characters
	this->write_cmd(RA8876_REG_MRWDP);
	for (unsigned int i = 0; i < count; i++)
	{
		this->wait_write_fifo();
		this->write_data(buffer[i] >> 8);

		this->wait_write_fifo();
		this->write_data(buffer[i] & 0xFF);
	}

	this->set_graphics_mode();

	// SPI.endTransaction();
}

size_t RA8876::write(const uint8_t *buffer, size_t size)
{
	// SPI.beginTransaction(this->spiSettings);

	this->set_text_mode();

	this->write_cmd(RA8876_REG_MRWDP);  // Set current register for writing to memory
	for (unsigned int i = 0; i < size; i++)
	{
		char c = buffer[i];

		if (c == '\r')
			;  // Ignored
		else if (c == '\n')
		{
			setCursor(0, getCursorY() + getTextSizeY());
			this->write_cmd(RA8876_REG_MRWDP);  // Reset current register for writing to memory
		}
		else if ((this->fontFlags & RA8876_FONT_FLAG_XLAT_FULLWIDTH) && ((c >= 0x21) || (c <= 0x7F)))
		{
			// Translate ASCII to Unicode fullwidth form (for Chinese fonts that lack ASCII)
			uint16_t fwc = c - 0x21 + 0xFF01;

			this->wait_write_fifo();
			this->write_data(fwc >> 8);

			this->wait_write_fifo();
			this->write_data(fwc & 0xFF);
		}
		else
		{
			this->wait_write_fifo();
			this->write_data(c);
		}
	}

	this->set_graphics_mode();

	// SPI.endTransaction();

	return size;
}


void RA8876::set_backlight_intensity(const uint8_t intensity)
{
	this->backlight_intensity = intensity;

	// TODO: PWM goes here

}


