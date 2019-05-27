#ifndef RPI_INTERFACE_H
#define RPI_INTERFACE_H

#include "interface.h"

class rpi_interface : public interface
{
public:
    rpi_interface(unsigned int cpr);
    ~rpi_interface() override;

    void initialize(unsigned int gpio_pin_a, unsigned int gpio_pin_b) override;

    unsigned int p_gpio_a();
    unsigned int p_gpio_b();
private:
    ///
    /// \brief m_pigpiod_handle Stores a handle to the pigpio daemon connection.
    ///
    int m_pigpiod_handle;
    unsigned int m_gpio_a;
    unsigned int m_gpio_b;
    unsigned int m_callback_a;
    unsigned int m_callback_b;

    void initialize_gpio(unsigned int gpio_pin);
    bool read_gpio(unsigned int gpio_pin);
    unsigned int attach_callback(unsigned int gpio_pin);
};

void interrupt_callback(int pigpiod_handle, unsigned int gpio_pin, unsigned int level, uint32_t tick, void* user_data);

#endif // RPI_INTERFACE_H
