#include "rpi_interface.h"

#include <pigpiod_if2.h>
#include <sstream>

void interrupt_callback(int pigpiod_handle, unsigned int gpio_pin, unsigned int level, uint32_t tick, void* user_data)
{
    // Convert the user data to the rpi_interface class.
    rpi_interface* interface_ptr = static_cast<rpi_interface*>(user_data);

    // Determine if level is a valid high/low.
    if(level < 2)
    {
        // Determine if signal A or signal B has undergone a transition.
        if(gpio_pin == interface_ptr->p_gpio_a())
        {
            interface_ptr->tick_a(static_cast<bool>(level));
        }
        else if(gpio_pin == interface_ptr->p_gpio_b())
        {
            interface_ptr->tick_b(static_cast<bool>(level));
        }
    }
}

rpi_interface::rpi_interface()
{
    // Connect to the pigpio daemon.
    rpi_interface::m_pigpiod_handle = pigpio_start(nullptr, nullptr);

    // Set default GPIO pin values.
    rpi_interface::m_gpio_a = 0;
    rpi_interface::m_gpio_b = 1;
}
rpi_interface::~rpi_interface()
{
    // Disconnect callbacks.
    callback_cancel(rpi_interface::m_callback_a);
    callback_cancel(rpi_interface::m_callback_b);

    // Disconnect from the pigpio daemon.
    pigpio_stop(rpi_interface::m_pigpiod_handle);
}

void rpi_interface::initialize(unsigned int gpio_pin_a, unsigned int gpio_pin_b, unsigned int cpr, double spin_ratio)
{
    // Store the parameters.
    rpi_interface::m_cpr = cpr;
    rpi_interface::m_spin_ratio = spin_ratio;

    // Set input pins and store them.
    rpi_interface::initialize_gpio(gpio_pin_a);
    rpi_interface::initialize_gpio(gpio_pin_b);
    rpi_interface::m_gpio_a = gpio_pin_a;
    rpi_interface::m_gpio_b = gpio_pin_b;

    // Read initial state of the encoder.
    bool level_a = rpi_interface::read_gpio(gpio_pin_a);
    bool level_b = rpi_interface::read_gpio(gpio_pin_b);
    // Use initial pin states to determine initial encoder state.
    rpi_interface::initialize_state(level_a, level_b);

    // Attach interrupts.
    rpi_interface::m_callback_a = rpi_interface::attach_callback(gpio_pin_a);
    rpi_interface::m_callback_b = rpi_interface::attach_callback(gpio_pin_b);
}
void rpi_interface::initialize_gpio(unsigned int gpio_pin)
{
    int result = set_mode(rpi_interface::m_pigpiod_handle, gpio_pin, PI_INPUT);
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
bool rpi_interface::read_gpio(unsigned int gpio_pin)
{
    int result = gpio_read(rpi_interface::m_pigpiod_handle, gpio_pin);
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
unsigned int rpi_interface::attach_callback(unsigned int gpio_pin)
{
    int result = callback_ex(rpi_interface::m_pigpiod_handle, gpio_pin, 2, &interrupt_callback, this);
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

unsigned int rpi_interface::p_gpio_a()
{
    return rpi_interface::m_gpio_a;
}
unsigned int rpi_interface::p_gpio_b()
{
    return rpi_interface::m_gpio_b;
}
