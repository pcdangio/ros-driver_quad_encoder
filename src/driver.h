/// \file driver.h
/// \brief Defines the driver class.
#ifndef DRIVER_H
#define DRIVER_H

#include <vector>
#include <stdexcept>
#include <chrono>

/// \brief Provides a base driver class for quadrature encoders.
class driver
{
public:
    // CONSTRUCTORS
    /// \brief Creates a new driver instance.
    driver();
    virtual ~driver() = 0;

    // METHODS
    /// \brief Initializes the driver.
    /// \param gpio_pin_a The GPIO pin connected to signal A of the encoder.
    /// \param gpio_pin_b The GPIO pin connected to signal B of the encoder.
    /// \param ppr The Pulses Per Revolution (PPR) of the encoder.
    void initialize(uint32_t gpio_pin_a, uint32_t gpio_pin_b, uint32_t ppr);
    /// \brief Sets the home position of the axis to the current position.
    void set_home();
    /// \brief Gets the relative position of the axis to the home position in radians.
    /// \return The current position in radians.
    double get_position();

    /// \brief Increments the encoder's state with an A signal pulse.
    /// \param level The new logic level of the A signal.
    void tick_a(bool level);
    /// \brief Increments the encoder's state with an B signal pulse.
    /// \param level The new logic level of the B signal.
    void tick_b(bool level);

    // PROPERTIES
    /// \brief Gets the amount of pulses missed by the encoder.
    /// \return The number of pulses missed.
    uint64_t pulses_missed();

protected:
    // PARAMETERS
    /// \brief Stores the Counts Per Revolution (CPR) of the encoder.
    uint32_t m_cpr;

    // METHODS
    /// \brief Initializes the GPIO input pins of the driver.
    /// \param gpio_pin_a The GPIO pin connected to signal A of the encoder.
    /// \param gpio_pin_b The GPIO pin connected to signal B of the encoder.
    virtual void initialize_gpio(uint32_t gpio_pin_a, uint32_t gpio_pin_b) = 0;
    /// \brief Reads the state of a GPIO pin.
    /// \param gpio_pin The GPIO pin to read the state of.
    /// \return TRUE if logic level high; FALSE if logic level low.
    virtual bool read_gpio(uint32_t gpio_pin) = 0;

private:
    // VARIABLES
    /// \brief A matrix mapping encoder state transitions to position changes.
    std::vector<std::vector<int>> m_transition_matrix;
    /// \brief The current home position in pulse counts.
    int64_t m_home_position;
    /// \brief The current encoder position in pulse counts.
    int64_t m_current_position;
    /// \brief The prior state of the encoder.
    uint32_t m_prior_state;
    /// \brief A counter for how many times a pulse skip has been detected since the home position was last set.
    uint64_t m_pulses_missed;

    // METHODS
    /// \brief Initializes the encoder's state.
    /// \param a The state of signal A.
    /// \param b The state of signal B.
    void initialize_state(bool level_a, bool level_b);
    /// \brief Updates the internal state of the encoder.
    /// \param new_state The observed new state of the encoder through signals A and B.
    void update_state(uint32_t new_state);
};

#endif