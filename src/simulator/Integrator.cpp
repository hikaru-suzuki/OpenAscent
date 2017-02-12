/*
 * OpenAscent: An open source framework for ascent trajectory simulation
 *             and optimization of multi-stage launch vehicles
 *
 * Copyright(C) 2016 Hikaru Suzuki (Super High School Level Physics Tutor)
 * 
 * This program is free software : you can redistribute it and / or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.If not, see <http://www.gnu.org/licenses/>.
 */

#include "Function.h"

#include "Integrator.h"

#include "Trigger.h"

Integrator::Integrator(Function *function) :
	_function(function),
	_current_state0(true)
{
}
void Integrator::init()
{
	_function->build_state(_state0);
	_function->build_state(_state1);
}

bool Integrator::attempt(double h)
{
	_last_step_length = h;
	return attempt_impl(h, current_state(), attempted_state());
}

void Integrator::apply_attempt()
{
	// simply swap the states by setting the index
	_current_state0 = !_current_state0;
}

unsigned int Integrator::register_trigger(Trigger *trigger)
{
	// binds the trigger with this integrator
	trigger->bind_integrator(this);

	_triggers.push_back(trigger);
	return _triggers.size() - 1;
}

bool Integrator::trigger_overrun(unsigned int trigger_index)
{
	return _triggers[trigger_index]->overrun();
}


double Integrator::trigger_length(unsigned int trigger_index)
{
	Trigger *trigger = _triggers[trigger_index];
	trigger->reset_guess();

	double guess_length = _last_step_length;

	// iterate until guess_length meets the exit criteria.
	while (!trigger->guess(guess_length))
		attempt(guess_length);
	
	return guess_length;
}
