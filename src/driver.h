/// \file driver.h
/// \brief Defines the driver class.
#ifndef DRIVER_H
#define DRIVER_H

#include <vector>
#include <stdexcept>
#include <chrono>

///
/// \brief Provides a base driver class for quadrature encoders.
///
class driver
{
public:
    // CONSTRUCTORS
    ///
    /// \brief driver Creates a new driver instance.
    ///
    driver();
    virtual ~driver() = 0;

    // METHODS
    ///
    /// \brief initialize Initializes the driver.
    /// \param gpio_pin_a The GPIO pin connected to signal A of the encoder.
    /// \param gpio_pin_b The GPIO pin connected to signal B of the encoder.
    /// \param cpr The Counts Per Revolution (CPR) of the encoder.
    /// \param spin_ratio The spin ratio from the axis of interest to the encoder.
    ///
    virtual void initialize(unsigned int gpio_pin_a, unsigned int gpio_pin_b, unsigned int cpr, double spin_ratio) = 0;
    ///
    /// \brief set_home Sets the home position of the axis to the current position.
    ///
    void set_home();
    ///
    /// \brief get_position Gets the relative position of the axis to the home position in radians.
    /// \return The current position in radians.
    ///
    double get_position();

    ///
    /// \brief tick_a Increments the encoder's state with an A signal pulse.
    /// \param level The new logic level of the A signal.
    ///
    void tick_a(bool level);
    ///
    /// \brief tick_a Increments the encoder's state with an B signal pulse.
    /// \param level The new logic level of the B signal.
    ///
    void tick_b(bool level);

    // PROPERTIES
    ///
    /// \brief p_pulses_missed Gets the amount of pulses missed by the encoder.
    /// \return The number of pulses missed.
    ///
    long long int p_pulses_missed();

protected:
    // PARAMETERS
    ///
    /// \brief m_cpr Stores the Counts Per Revolution (CPR) of the encoder.
    ///
    unsigned int m_cpr;
    ///
    /// \brief m_spin_ratio Stores the spin ratio between the axis and the encoder.
    ///
    double m_spin_ratio;

    // METHODS
    ///
    /// \brief initialize_state Initializes the encoder's state.
    /// \param a The state of signal A.
    /// \param b The state of signal B.
    ///
    void initialize_state(bool level_a, bool level_b);

private:
    // VARIABLES
    ///
    /// \brief m_transition_matrix A matrix mapping encoder state transitions to position changes.
    ///
    std::vector<std::vector<int>> m_transition_matrix;
    ///
    /// \brief m_home_position The current home position in pulse counts.
    ///
    long long int m_home_position;
    ///
    /// \brief m_current_position The current encoder position in pulse counts.
    ///
    long long int m_current_position;
    ///
    /// \brief m_prior_state The prior state of the encoder.
    ///
    unsigned int m_prior_state;
    ///
    /// \brief m_pulses_missed A counter for how many times a pulse skip has been detected since the home position was last set.
    ///
    long long int m_pulses_missed;

    ///
    /// \brief update_state Updates the internal state of the encoder.
    /// \param new_state The observed new state of the encoder through signals A and B.
    ///
    void update_state(unsigned int new_state);
};

#endif // DRIVER_H
