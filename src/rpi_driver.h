/// \file rpi_driver.h
/// \brief Defines the rpi_driver class.
#ifndef RPI_DRIVER_H
#define RPI_DRIVER_H

#include "driver.h"

///
/// \brief A Raspberry Pi driver for a generic quadrature encoder.
///
class rpi_driver : public driver
{
public:
    // CONSTRUCTORS
    ///
    /// \brief rpi_driver Initializes a new RPi driver.
    ///
    rpi_driver();
    ~rpi_driver() override;

    // PROPERTIES
    ///
    /// \brief p_gpio_a Gets the GPIO pin connected to signal A of the encoder.
    /// \return The GPIO pin connected to signal A of the encoder.
    ///
    unsigned int p_gpio_a();
    ///
    /// \brief p_gpio_a Gets the GPIO pin connected to signal B of the encoder.
    /// \return The GPIO pin connected to signal B of the encoder.
    ///
    unsigned int p_gpio_b();

protected:
    // METHODS
    void initialize_gpio(unsigned int gpio_pin_a, unsigned int gpio_pin_b) override;
    bool read_gpio(unsigned int gpio_pin) override;

private:
    // VARIABLES
    ///
    /// \brief m_pigpiod_handle Stores a handle to the pigpio daemon connection.
    ///
    int m_pigpiod_handle;
    ///
    /// \brief m_gpio_a Stores the GPIO input pin for signal A of the encoder.
    ///
    unsigned int m_gpio_a;
    ///
    /// \brief m_gpio_b Stores the GPIO input pin for signal B of the encoder.
    ///
    unsigned int m_gpio_b;
    ///
    /// \brief m_callback_a Stores the interrupt callback for signal A of the encoder.
    ///
    unsigned int m_callback_a;
    ///
    /// \brief m_callback_b Stores the interrupt callback for signal B of the encoder.
    ///
    unsigned int m_callback_b;

    // METHODS
    ///
    /// \brief initialize_single_gpio Initializes a single GPIO pin as an input.
    /// \param gpio_pin The GPIO pin to initialize.
    ///
    void initialize_single_gpio(unsigned int gpio_pin);
    ///
    /// \brief attach_callback Attaches an interrupt callback to a GPIO pin.
    /// \param gpio_pin The GPIO pin to attach the interrupt to.
    /// \return Returns the PIGPIO_D callback ID for the callback.
    /// \note The interrupt is set for both rising and falling edges.
    ///
    unsigned int attach_callback(unsigned int gpio_pin);
};

///
/// \brief interrupt_callback The callback function for handling input pin interrupts.
/// \param pigpiod_handle The handle to the pigpio daemon connection that initiated the callback.
/// \param gpio_pin The GPIO pin that generated the interrupt.
/// \param level The new level of the GPIO pin.
/// \param tick The timestamp of the interrupt.
/// \param user_data A void pointer to the rpi_driver instance to manage the interrupt.
///
void interrupt_callback(int pigpiod_handle, unsigned int gpio_pin, unsigned int level, uint32_t tick, void* user_data);

#endif // RPI_DRIVER_H
