#include "rpi_driver.h"

#include <pigpiod_if2.h>
#include <sstream>

void interrupt_callback(int pigpiod_handle, unsigned int gpio_pin, unsigned int level, uint32_t tick, void* user_data)
{
    // Convert the user data to the rpi_driver class.
    rpi_driver* driver_ptr = static_cast<rpi_driver*>(user_data);

    // Determine if level is a valid high/low.
    if(level < 2)
    {
        // Determine if signal A or signal B has undergone a transition.
        if(gpio_pin == driver_ptr->p_gpio_a())
        {
            driver_ptr->tick_a(static_cast<bool>(level));
        }
        else if(gpio_pin == driver_ptr->p_gpio_b())
        {
            driver_ptr->tick_b(static_cast<bool>(level));
        }
    }
}

rpi_driver::rpi_driver()
{
    // Connect to the pigpio daemon.
    rpi_driver::m_pigpiod_handle = pigpio_start(nullptr, nullptr);

    // Set default GPIO pin values.
    rpi_driver::m_gpio_a = 0;
    rpi_driver::m_gpio_b = 1;
}
rpi_driver::~rpi_driver()
{
    // Disconnect callbacks.
    callback_cancel(rpi_driver::m_callback_a);
    callback_cancel(rpi_driver::m_callback_b);

    // Disconnect from the pigpio daemon.
    pigpio_stop(rpi_driver::m_pigpiod_handle);
}

void rpi_driver::initialize(unsigned int gpio_pin_a, unsigned int gpio_pin_b, unsigned int cpr, double spin_ratio)
{
    // Store the parameters.
    rpi_driver::m_cpr = cpr;
    rpi_driver::m_spin_ratio = spin_ratio;

    // Set input pins and store them.
    rpi_driver::initialize_gpio(gpio_pin_a);
    rpi_driver::initialize_gpio(gpio_pin_b);
    rpi_driver::m_gpio_a = gpio_pin_a;
    rpi_driver::m_gpio_b = gpio_pin_b;

    // Read initial state of the encoder.
    bool level_a = rpi_driver::read_gpio(gpio_pin_a);
    bool level_b = rpi_driver::read_gpio(gpio_pin_b);
    // Use initial pin states to determine initial encoder state.
    rpi_driver::initialize_state(level_a, level_b);

    // Attach interrupts.
    rpi_driver::m_callback_a = rpi_driver::attach_callback(gpio_pin_a);
    rpi_driver::m_callback_b = rpi_driver::attach_callback(gpio_pin_b);
}
void rpi_driver::initialize_gpio(unsigned int gpio_pin)
{
    int result = set_mode(rpi_driver::m_pigpiod_handle, gpio_pin, PI_INPUT);
    switch(result)
    {
    case 0:
    {
        // Success.
        break;
    }
    case PI_BAD_GPIO:
    {
        std::stringstream message;
        message << "initialize_gpio: PIGPIO_D invalid GPIO pin (" << gpio_pin << ")";
        throw std::runtime_error(message.str());
    }
    case PI_BAD_MODE:
    {
        std::stringstream message;
        message << "initialize_gpio: PIGPIO_D bad GPIO mode (" << gpio_pin << ")";
        throw std::runtime_error(message.str());
    }
    case PI_NOT_PERMITTED:
    {
        std::stringstream message;
        message << "initialize_gpio: PIGPIO_D not permitted. (" << gpio_pin << ")";
        throw std::runtime_error(message.str());
    }
    default:
    {
        std::stringstream message;
        message << "initialize_gpio: PIGPIO_D unknown error (" << gpio_pin << ")";
        throw std::runtime_error(message.str());
    }
    }
}
bool rpi_driver::read_gpio(unsigned int gpio_pin)
{
    int result = gpio_read(rpi_driver::m_pigpiod_handle, gpio_pin);
    switch(result)
    {
    case 0:
    {
        return false;
    }
    case 1:
    {
        return true;
    }
    case PI_BAD_GPIO:
    {
        std::stringstream message;
        message << "read_gpio: PIGPIO_D invalid GPIO pin (" << gpio_pin << ").";
        throw std::runtime_error(message.str());
    }
    default:
    {
        throw std::runtime_error("initialize_state: PIGPIO_D unknown error.");
    }
    }
}
unsigned int rpi_driver::attach_callback(unsigned int gpio_pin)
{
    int result = callback_ex(rpi_driver::m_pigpiod_handle, gpio_pin, 2, &interrupt_callback, this);
    if(result >= 0)
    {
        return static_cast<unsigned int>(result);
    }
    else
    {
        switch(result)
        {
        case pigif_bad_malloc:
        {
            std::stringstream message;
            message << "attach_callback: PIGPIO_D bad malloc (" << gpio_pin << ")";
            throw std::runtime_error(message.str());
        }
        case pigif_bad_callback:
        {
            std::stringstream message;
            message << "attach_callback: PIGPIO_D bad callback (" << gpio_pin << ")";
            throw std::runtime_error(message.str());
        }
        case pigif_duplicate_callback:
        {
            std::stringstream message;
            message << "attach_callback: PIGPIO_D duplicate callback (" << gpio_pin << ")";
            throw std::runtime_error(message.str());
        }
        default:
        {
            throw std::runtime_error("attach_callback: PIGPIO_D unknown error.");
        }
        }
    }
}

unsigned int rpi_driver::p_gpio_a()
{
    return rpi_driver::m_gpio_a;
}
unsigned int rpi_driver::p_gpio_b()
{
    return rpi_driver::m_gpio_b;
}
