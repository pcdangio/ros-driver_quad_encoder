#include "interface.h"

#include <math.h>

// CONSTRUCTORS
interface::interface()
{
    // Set default values
    interface::m_home_position = 0;
    interface::m_current_position = 0;
    interface::m_prior_state = 0;
    interface::m_pulses_missed = 0;

    // Set the parameters.
    interface::m_cpr = 100;
    interface::m_spin_ratio = 1.0;

    // Set up the qmatrix, which is to be indexed by (prior_state, current_state)
    interface::m_transition_matrix = {{0, 1, -1, 2}, {1, 0, 2, -1}, {-1, 2, 0, 1}, {2, 1, -1, 0}};
}
interface::~interface()
{
    // Do nothing, virtual destructor.
}

// METHODS
void interface::set_home()
{
    // Set the current position as the new home.
    interface::m_home_position = interface::m_current_position;
    // Reset the missed pulses flag.
    interface::m_pulses_missed = 0;
}
double interface::get_position()
{
    return static_cast<double>(interface::m_current_position) / static_cast<double>(interface::m_cpr) * interface::m_spin_ratio * 2.0 * M_PIf64;
}
void interface::initialize_state(bool level_a, bool level_b)
{
    // Initialize the prior state with the current state.
    interface::m_prior_state = (static_cast<unsigned int>(level_a) << 1) | static_cast<unsigned int>(level_b);
}
void interface::tick_a(bool level)
{
    // Calculate the new state.
    unsigned int new_state = interface::m_prior_state | (static_cast<unsigned int>(level) << 1);
    // Update the encoder's state with the new state.
    interface::update_state(new_state);
}
void interface::tick_b(bool level)
{
    // Calculate the new state.
    unsigned int new_state = interface::m_prior_state | static_cast<unsigned int>(level);
    // Update the encoder's state with the new state.
    interface::update_state(new_state);
}
void interface::update_state(unsigned int new_state)
{
    // Get the transition value.
    int transition = interface::m_transition_matrix.at(interface::m_prior_state).at(new_state);

    // Check if there was a valid transition.
    if(transition == 2)
    {
        interface::m_pulses_missed = true;
    }
    else
    {
        // Update the position.
        interface::m_current_position += transition;
    }

    // Update the prior state.
    interface::m_prior_state = new_state;
}
