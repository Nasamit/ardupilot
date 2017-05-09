/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "Scheduler.h"
#include <stdio.h>

#include "I2CDevice.h"
#include "io.h"
#include "i2c.h"

using namespace x86Duino ;

Semaphore I2CDevice::i2c_semaphore;
extern const AP_HAL::HAL& hal ;

I2CDevice::I2CDevice(uint8_t address) :
    _address(address)
{}

I2CDevice::~I2CDevice()
{
    printf("I2C device bus 0 address 0x%02x closed\n",
           (unsigned)_address);
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    if( !i2cmaster_StartN(0, _address, I2C_WRITE, send_len))
        printf("I2C write fail to start \n");
    for( uint32_t i = 0 ; i < send_len ; i++ )
        if( !i2cmaster_WriteN( 0, send[i]) )   printf("I2C write fail at %u data \n", i);

    if( !i2cmaster_StartN(0, _address, I2C_READ, recv_len))
        printf("I2C read fail to start \n");
    for( uint32_t i = 0 ; i < recv_len ; i++ )
        recv[i] = i2cmaster_ReadN(0) ;
}

AP_HAL::Semaphore *I2CDevice::get_semaphore()
{
    return &i2c_semaphore;
}

/*
  register a periodic callback
*/
AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return ((Scheduler*)hal.scheduler)->register_i2c_process( period_usec, cb) ;;
}

AP_HAL::OwnPtr<AP_HAL::I2CDevice>
I2CDeviceManager::get_device(uint8_t bus, uint8_t address)
{
    auto dev = AP_HAL::OwnPtr<AP_HAL::I2CDevice>(new I2CDevice(address));
    return dev;
}

void I2CDeviceManager::init(void)
{
    #define I2C0_IRQ        (0x0A)
    #define I2C_IOADDR      (0xFB00)
    #define CROSSBARBASE    (0x0A00)
    #define TEST_T          (200)

    #if defined (__86DUINO_AI)
    #define I2C_GPIO_DATA    (0xF224)
    #define I2C_GPIO_DIR     (0xF226)
    #define CRSB_BIT_G9      (0x89)
    #define I2CPIN_POSITION_BIT    (0xC0)
    #define I2C_SCL_PIN_BIT        (0x40)
    #define I2C_SDA_PIN_BIT        (0x80)
    #else
    #define I2C_GPIO_DATA    (0xF204)
    #define I2C_GPIO_DIR     (0xF206)
    #define CRSB_BIT_G0      (0x90)
    #define CRSB_BIT_G1      (0x98)
    #define CRSB_BIT_G2      (0xA0)
    #define CRSB_BIT_G3      (0xA8)
    #define I2C_PIN          (0x03) // GP10, GP11
    #define I2CPIN_POSITION_BIT    (0x03)
    #define I2C_SCL_PIN_BIT        (0x01)
    #define I2C_SDA_PIN_BIT        (0x02)
    #endif
    int usedirq = 0, irqnum, i;
    unsigned long nowtime = 0L;
    unsigned char int_routing_table[16] = {0xff, 0x08, 0xff, 0x02, 0x04, 0x05, 0x07, 0x06, 0xff, 0x01, 0x03, 0x09, 0x0b, 0xff, 0x0d, 0x0f};

    // set I2C base address
    i2c_SetBaseAddress(I2C_IOADDR);

    irqnum = usedirq;   // avoid -Werror=unused-but-set-variable
    // set I2C IRQ
    if((sb_Read8(0xD6) & 0x0F) == 0 || (sb_Read8(0xD6) & 0x0F) == 0x0F)
    {
        usedirq = I2C0_IRQ;
        i2c_SetIRQ(int_routing_table[I2C0_IRQ], I2CIRQ_DISABLE);
    }
    else
    {
        irqnum = sb_Read8(0xD6) & 0x0F;
        for(i=0; i<16; i++)
            if(int_routing_table[i] == irqnum) break;
        if(i != 16)
            usedirq = i;
        else
        {
            usedirq = I2C0_IRQ;
            i2c_SetIRQ(int_routing_table[I2C0_IRQ], I2CIRQ_DISABLE);
        }
    }

    io_outpb(I2C_GPIO_DIR, io_inpb(I2C_GPIO_DIR) & ~(I2CPIN_POSITION_BIT)); // change GPIO(SCL & SDA) DIR to INPUT
#if defined (__86DUINO_AI)
    io_outpw(CROSSBARBASE + CRSB_BIT_G9, 0x01);
#else
    io_outpw(CROSSBARBASE + CRSB_BIT_G1 + 0, 0x0101);
#endif

    // check SCL is HIGH or LOW (Normally, it should be HIGH)
    if((io_inpb(I2C_GPIO_DATA) & I2C_SCL_PIN_BIT) == 0x00)
    {
        printf("ERROR: I2C's SCL is LOW during initial process");
        return;
    }

    // check SDA is HIGH or LOW (Normally, it should be HIGH)
    if((io_inpb(I2C_GPIO_DATA) & I2C_SDA_PIN_BIT) == 0x00)
    {
        printf("I2C's SDA is LOW, we try to repair it");
        for(i=0; i<TEST_T; i++)
        {
            set_pins(0, 1, 1);
            timer_DelayMicroseconds(100);
            set_pins(0, 0, 1);
            timer_DelayMicroseconds(100);
            if((io_inpb(I2C_GPIO_DATA) & I2C_SDA_PIN_BIT) == I2C_SDA_PIN_BIT) break;
        }

        if(i != TEST_T)
        {
            printf("Repair completed");
            // use GPIO to send STOP condition
            set_pins(0, 0, 0);
            timer_DelayMicroseconds(100);
            set_pins(0, 1, 0);
            timer_DelayMicroseconds(100);
            set_pins(0, 1, 1);
            timer_DelayMicroseconds(100);

            // use GPIO to send START condition
            set_pins(0, 1, 1);
            timer_DelayMicroseconds(100);
            set_pins(0, 1, 0);
            timer_DelayMicroseconds(100);
            set_pins(0, 0, 0);
            timer_DelayMicroseconds(100);

            // use GPIO to send STOP condition again
            set_pins(0, 0, 0);
            timer_DelayMicroseconds(100);
            set_pins(0, 1, 0);
            timer_DelayMicroseconds(100);
            set_pins(0, 1, 1);
            timer_DelayMicroseconds(100);
        }
        else
        {
            printf("Repair fail: please try to reset 86Duino");
            return;
        }
    }

    if (i2c_Reset(0) == false)  // assume the status of GPIO/I2C pins are GPIO "IN" or "OUT 1"
    {
        //i2c_Close();
        printf("can't reset the I2C modules\n");
        return;
    }

    io_outpb(I2C_IOADDR + 0x07, (io_inpb(I2C_IOADDR + 0x07) & 0xf0) | 0x07); // I2C0 Extra Control Register
    // DIMC:  Enaable  (0)
    // DI196: Disaable (1)
    // DIAR:  Enaable  (1)
    // DIDC:  Disaable (1)

    i2c_ClearSTAT(0, I2CSTAT_ALL);
    i2c_DisableNoiseFilter(0);
    i2c_DisableStandardHSM(0);

    i2c_SetSpeed(0, I2CMODE_AUTO, 400000L);

    i2cslave_SetAddr(0, 0x7f);
    i2cslave_EnableACK(0);

    // switch form GPIO(RICH_IO) to SCL & SDA
#if defined (__86DUINO_AI)
    io_outpb(CROSSBARBASE + CRSB_BIT_G9, 0x08);
#else
    io_outpw(CROSSBARBASE + CRSB_BIT_G1 + 0, 0x0808);
#endif

}

void I2CDeviceManager::set_pins(int dev, int SCL, int SDA) {
    unsigned char mask = ~(I2CPIN_POSITION_BIT << (dev*2));
    unsigned char dir  = 0;
    unsigned char val  = (unsigned char)((SDA<<1) + SCL) << (dev*2);

    if (SCL == 0) dir = dir | (I2C_SCL_PIN_BIT << (dev*2));  // set the SCL to output 0
    if (SDA == 0) dir = dir | (I2C_SDA_PIN_BIT << (dev*2));  // set the SDA to output 0

    io_outpb(I2C_GPIO_DIR,  (io_inpb(I2C_GPIO_DIR)  & mask) | dir); //set GPIO direction = out
    io_outpb(I2C_GPIO_DATA, (io_inpb(I2C_GPIO_DATA) & mask) | val);
}
