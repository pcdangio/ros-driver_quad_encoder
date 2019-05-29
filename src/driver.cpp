#include "driver.h"

#include <math.h>

// CONSTRUCTORS
driver::driver()
{
    // Set default values
    driver::m_home_position = 0;
    driver::m_current_position = 0;
    driver::m_prior_state = 0;
    driver::m_pulses_missed = 0;

    // Set the parameters.
    driver::m_cpr = 400;
    driver::m_spin_ratio = 1.0;

    // Set up the qmatrix, which is to be indexed by (prior_state, current_state)
    driver::m_transition_matrix = {{0, 1, -1, 2}, {1, 0, 2, -1}, {-1, 2, 0, 1}, {2, 1, -1, 0}};
}
driver::~driver()
{
    // Do nothing, virtual destructor.
}

// METHODS
void driver::initialize(unsigned int gpio_pin_a, unsigned int gpio_pin_b, unsigned int ppr, double spin_ratio)
{
    // Store the cpr and spin_ratios.
    driver::m_cpr = ppr * 2UL;  // Two counts for each pulse (rising/falling edge).
    driver::m_spin_ratio = spin_ratio;

    // Call the extended class's initialize_gpio method.
    initialize_gpio(gpio_pin_a, gpio_pin_b);

    // Initialize the state using the extended class's read methods.
    bool level_a = read_gpio(gpio_pin_a);
    bool level_b = read_gpio(gpio_pin_b);
    driver::initialize_state(level_a, level_b);
}
void driver::set_home()
{
    // Set the current position as the new home.
    driver::m_home_position = driver::m_current_position;
    // Reset the missed pulses flag.
    driver::m_pulses_missed = 0;
}
double driver::get_position()
{
    return static_cast<double>(driver::m_current_position) / static_cast<double>(driver::m_cpr) * driver::m_spin_ratio * 2.0 * M_PIf64;
}

void driver::tick_a(bool level)
{
    // Calculate the new state.
    unsigned int new_state = (driver::m_prior_state & 1) | (static_cast<unsigned int>(level) << 1);
    // Update the encoder's state with the new state.
    driver::update_state(new_state);
}
void driver::tick_b(bool level)
{
    // Calculate the new state.
    unsigned int new_state = (driver::m_prior_state & 2) | static_cast<unsigned int>(level);
    // Update the encoder's state with the new state.
    driver::update_state(new_state);
}
void driver::initialize_state(bool level_a, bool level_b)
{
    // Initialize the prior state with the current state.
    driver::m_prior_state = (static_cast<unsigned int>(level_a) << 1) | static_cast<unsigned int>(level_b);
}
void driver::update_state(unsigned int new_state)
{
    // Get the transition value.
    int transition = driver::m_transition_matrix.at(driver::m_prior_state).at(new_state);

    // Check if there was a valid transition.
    if(transition == 2)
    {
        driver::m_pulses_missed += 1;
    }
    else
    {
        // Update the position.
        driver::m_current_position += transition;
    }

    // Update the prior state.
    driver::m_prior_state = new_state;
}

long long int driver::p_pulses_missed()
{
    return driver::m_pulses_missed;
}
