/// \file rpi_driver.h
/// \brief Defines the rpi_driver class.
#ifndef RPI_DRIVER_H
#define RPI_DRIVER_H

#include "driver.h"

/// \brief A Raspberry Pi driver for a generic quadrature encoder.
class rpi_driver : public driver
{
public:
    // CONSTRUCTORS
    /// \brief Initializes a new RPi driver.
    rpi_driver();
    ~rpi_driver() override;

    // PROPERTIES
    /// \brief Gets the GPIO pin connected to signal A of the encoder.
    /// \return The GPIO pin connected to signal A of the encoder.
    uint32_t p_gpio_a();
    /// \brief Gets the GPIO pin connected to signal B of the encoder.
    /// \return The GPIO pin connected to signal B of the encoder.
    uint32_t p_gpio_b();

protected:
    // METHODS
    void initialize_gpio(uint32_t gpio_pin_a, uint32_t gpio_pin_b) override;
    bool read_gpio(uint32_t gpio_pin) override;

private:
    // VARIABLES
    /// \brief Stores a handle to the pigpio daemon connection.
    int32_t m_pigpiod_handle;
    /// \brief Stores the GPIO input pin for signal A of the encoder.
    uint32_t m_gpio_a;
    /// \brief Stores the GPIO input pin for signal B of the encoder.
    uint32_t m_gpio_b;
    /// \brief Stores the interrupt callback for signal A of the encoder.
    uint32_t m_callback_a;
    /// \brief Stores the interrupt callback for signal B of the encoder.
    uint32_t m_callback_b;

    // METHODS
    /// \brief Initializes a single GPIO pin as an input.
    /// \param gpio_pin The GPIO pin to initialize.
    void initialize_single_gpio(uint32_t gpio_pin);
    /// \brief Attaches an interrupt callback to a GPIO pin.
    /// \param gpio_pin The GPIO pin to attach the interrupt to.
    /// \return Returns the PIGPIO_D callback ID for the callback.
    /// \note The interrupt is set for both rising and falling edges.
    uint32_t attach_callback(uint32_t gpio_pin);
};

/// \brief The callback function for handling input pin interrupts.
/// \param pigpiod_handle The handle to the pigpio daemon connection that initiated the callback.
/// \param gpio_pin The GPIO pin that generated the interrupt.
/// \param level The new level of the GPIO pin.
/// \param tick The timestamp of the interrupt.
/// \param user_data A void pointer to the rpi_driver instance to manage the interrupt.
void interrupt_callback(int32_t pigpiod_handle, uint32_t gpio_pin, uint32_t level, uint32_t tick, void* user_data);

#endif // RPI_DRIVER_H
