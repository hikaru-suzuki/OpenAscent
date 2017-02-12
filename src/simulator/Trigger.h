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

#pragma once

class Integrator;

// A trigger can determine the time of an event, by accessing integrator's states,
// including current, attempted and previous states.
// current implementation of integrator stores 2 states only (current and attempted),
// previous states may be added later for max-value triggers.

class Trigger
{
public:
	virtual ~Trigger()
	{
	}

protected:
	Integrator *_integrator;

public:
	// binds the trigger to an integrator,
	// must be done before integrator run.
	void bind_integrator(Integrator *integrator)
	{
		_integrator = integrator;

		bind_int_impl();
	}

	// customizable binding implementations,
	// may be initializing fast state accessing mechanisms etc.
	virtual void bind_int_impl()
	{
	}

	// determine if the trigger should be fired between the period of integrator's states,
	// from the eariest stored state to the attempted state.
	// currently only determine between current and attempted state.
	// return true if the trigger should be fired.
	virtual bool overrun() = 0;

	// the trigger should be able to cooperate with the integrator to perform a guess run,
	// in order to determine the firing time to a certain precision.
	// reset_guess is called by the integrator before each of the guess run,
	// derived classes should reset any temporary variables used by the run here.
	virtual void reset_guess() = 0;

	// perform a step of the guess run.
	// guess_length is an input/output parameter, represents the length from the current state's time.
	// an estimation value of the firing length will be given in the guess_length parameter.
	// the trigger is responsible of determining a more precise length value, and storing it also in the guess_length parameter.
	// if pre-set precision is met, return true, otherwise return false.
	virtual bool guess(double &guess_length) = 0;
};
