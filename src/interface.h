/// \file interface.h
/// \brief Defines the interface class.
#ifndef INTERFACE_H
#define INTERFACE_H

#include <vector>
#include <stdexcept>
#include <chrono>

///
/// \brief Provides a base interface class for quadrature encoders.
///
class interface
{
public:
    // CONSTRUCTORS
    ///
    /// \brief interface Creates a new interface instance.
    /// \param cpr The Counts Per Revolution (CPR) of the encoder.
    ///
    interface(unsigned int cpr);
    virtual ~interface() = 0;

    // METHODS
    ///
    /// \brief initialize Initializes the interface.
    /// \param gpio_pin_a The GPIO pin connected to signal A of the encoder.
    /// \param gpio_pin_b The GPIO pin connected to signal B of the encoder.
    ///
    virtual void initialize(unsigned int gpio_pin_a, unsigned int gpio_pin_b) = 0;
    ///
    /// \brief set_home Sets the home position of the axis to a new angle.
    /// \param position The new home position in radians.
    ///
    void set_home(double position);
    ///
    /// \brief reset_home Resets the home position of the axis to the current position.
    ///
    void reset_home();
    ///
    /// \brief get_position Gets the current position of the
    /// \return
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
    /// \brief p_cpr Gets the Counts Per Revolution (CPR) of the encoder.
    /// \return The CPR of the encoder.
    ///
    unsigned int p_cpr();
    ///
    /// \brief p_cpr Sets the Counts Per Revolution (CPR) of the encoder.
    /// \param cpr The CPR of the encoder.
    ///
    void p_cpr(unsigned int cpr);
    ///
    /// \brief p_spin_ratio Gets the spin ratio between the axis and the encoder.
    /// \return The spin ratio between the axis and the encoder.
    ///
    double p_spin_ratio();
    ///
    /// \brief p_spin_ratio Sets the spin ratio between the axis and the encoder.
    /// \param spin_ratio The spin ratio between the axis and the encoder.
    ///
    void p_spin_ratio(double spin_ratio);

protected:
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
    /// \brief m_cpr Stores the Counts Per Revolution (CPR) of the encoder.
    ///
    unsigned int m_cpr;
    ///
    /// \brief m_spin_ratio Stores the spin ratio between the axis and the encoder.
    ///
    double m_spin_ratio;

    ///
    /// \brief update_state Updates the internal state of the encoder.
    /// \param new_state The observed new state of the encoder through signals A and B.
    ///
    void update_state(unsigned int new_state);
};

#endif // INTERFACE_H
