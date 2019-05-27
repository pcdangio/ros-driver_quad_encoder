/// \file rpi_interface.h
/// \brief Defines the rpi_interface class.
#ifndef RPI_INTERFACE_H
#define RPI_INTERFACE_H

#include "interface.h"

///
/// \brief A Raspberry Pi interface for a generic quadrature encoder.
///
class rpi_interface : public interface
{
public:
    // CONSTRUCTORS
    ///
    /// \brief rpi_interface Initializes a new RPi interface.
    /// \param cpr The Counts Per Revolution (CPR) of the encoder.
    ///
    rpi_interface(unsigned int cpr);
    ~rpi_interface() override;

    // METHODS
    void initialize(unsigned int gpio_pin_a, unsigned int gpio_pin_b) override;

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

    ///
    /// \brief initialize_gpio Initializes a GPIO pin as an input.
    /// \param gpio_pin The GPIO pin to initialize.
    ///
    void initialize_gpio(unsigned int gpio_pin);
    ///
    /// \brief read_gpio Reads the current state of a GPIO input pin.
    /// \param gpio_pin The GPIO pin to read.
    /// \return TRUE for logic HIGH, otherwise FALSE for logic LOW.
    ///
    bool read_gpio(unsigned int gpio_pin);
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
/// \param user_data A void pointer to the rpi_interface instance to manage the interrupt.
///
void interrupt_callback(int pigpiod_handle, unsigned int gpio_pin, unsigned int level, uint32_t tick, void* user_data);

#endif // RPI_INTERFACE_H
