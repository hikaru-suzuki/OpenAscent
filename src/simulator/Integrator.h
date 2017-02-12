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

#include "State.h"

class Function;

class Trigger;

class Integrator
{
public:
	Integrator(Function *function);

	virtual ~Integrator()
	{
	}

protected:
	Function *_function;

	// swapping states used as current or attempt state
	// an integration step is completed by an attempt() call followed by an apply_attempt() call
	// after attempt(), value of the next step is stored in attempt state.
	// after apply_attempt(), value of the next step is "moved" (by swapping the state) into thte current state.
	// if attempt is call again without calling apply_attempt, the previous attempt state value is discarded.
	State _state0;
	State _state1;

	// if _current_state0 = true, _state0 is the current state, _state1 is the attempt state
	// if _current_state0 = false, _state1 is the current state, _state0 is the attempt state
	bool _current_state0;

	// implementation of an attempt, implementation of the integration algorithm.
	virtual bool attempt_impl(double h, State &current, State &attempt) = 0;

	// stores the step length of the last attempt, used for seeding trigger length guess.
	double _last_step_length;

	// registered triggers
	vector<Trigger *> _triggers;

public:
	State &current_state()
	{
		return _current_state0 ? _state0 : _state1;
	}

	State &attempted_state()
	{
		return _current_state0 ? _state1 : _state0;
	}

	// initializtion of the integrator, state is built by _function here. 
	virtual void init();

	// Try an integration step. 
	// Current state will not be changed.
	// Results will be stored in attempt state
	bool attempt(double h);

	// Change current state with the results of last attempt.
	void apply_attempt();

	// register a trigger, trigger must be registered to the integrator in order to function.
	// when registering a trigger, 2 trigger objects will be duplicated and bound with current and attempt state each.
	// the duplicated triggers will be destructed along with the integrator.
	// an ID is assigned for each registered trigger and returned. 
	// ID will be required to access the registered triggers later.
	unsigned int register_trigger(Trigger *trigger);

	// check if the indexed trigger should be fired between current and attempt state
	bool trigger_overrun(unsigned int id);

	// determine the exact fire point between current and attempt state.
	// the fire point equals to current state's t + returned value.
	double trigger_length(unsigned int id);
};