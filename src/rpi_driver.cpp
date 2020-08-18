#include "rpi_driver.h"

#include <pigpiod_if2.h>
#include <sstream>

// CALLBACKS
void interrupt_callback(int32_t pigpiod_handle, uint32_t gpio_pin, uint32_t level, uint32_t tick, void* user_data)
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

// CONSTRUCTORS
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

// METHODS
void rpi_driver::initialize_gpio(uint32_t gpio_pin_a, uint32_t gpio_pin_b)
{

    // Set input pins and store them.
    rpi_driver::initialize_single_gpio(gpio_pin_a);
    rpi_driver::initialize_single_gpio(gpio_pin_b);
    rpi_driver::m_gpio_a = gpio_pin_a;
    rpi_driver::m_gpio_b = gpio_pin_b;

    // Attach interrupts.
    rpi_driver::m_callback_a = rpi_driver::attach_callback(gpio_pin_a);
    rpi_driver::m_callback_b = rpi_driver::attach_callback(gpio_pin_b);
}
void rpi_driver::initialize_single_gpio(uint32_t gpio_pin)
{
    int32_t result = set_mode(rpi_driver::m_pigpiod_handle, gpio_pin, PI_INPUT);
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
bool rpi_driver::read_gpio(uint32_t gpio_pin)
{
    int32_t result = gpio_read(rpi_driver::m_pigpiod_handle, gpio_pin);
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
uint32_t rpi_driver::attach_callback(uint32_t gpio_pin)
{
    int32_t result = callback_ex(rpi_driver::m_pigpiod_handle, gpio_pin, 2, &interrupt_callback, this);
    if(result >= 0)
    {
        return static_cast<uint32_t>(result);
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

// PROPERTIES
uint32_t rpi_driver::p_gpio_a()
{
    return rpi_driver::m_gpio_a;
}
uint32_t rpi_driver::p_gpio_b()
{
    return rpi_driver::m_gpio_b;
}
