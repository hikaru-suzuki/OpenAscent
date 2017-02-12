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

#include "../common.h"

class State;

// base class of all stateblocks
// a stateblock is a set of variables that always evaluate together
// all stateblock is derived from integration variables of the state, with the help of other invariable values (of vehicle, environment etc.)
// stateblocks are evaluated on demand by a lazy-evaluation mechanism.

// after construction, a block must be added to a state with State::add_block() to function properly.
class StateBlock
{
public:
	virtual ~StateBlock()
	{
	}

protected:
	// a stateblock's parent is the state it belongs
	State *_parent;

	// name of the block
	string _name;

	// valid status flag
	// a stateblock is valid after it is evaluated
	// when the state integration variables' values changed, a stateblock will be set as invalid, until it is evaluated again.
	bool _valid;

	// implementation of the evaluation of block variables
	virtual bool evaluate() = 0;

	// implementation of the access of variables by index
	virtual double value_impl(unsigned int id) = 0;

public:
	void set_parent(State *state)
	{
		_parent = state;
	}

	const string &name() const
	{
		return _name;
	}

	void set_name(const char *name)
	{
		_name = name;
	}

	// initialization, call once after block is added to the state
	virtual bool initialize()
	{
		return true;
	}
	
	// renders this block invalidate, next value() call will trigger an evaluation.
	void invalidate()
	{
		_valid = false;
	}

	// lazy-evaluation
	double value(unsigned int id)
	{
		if (!_valid)
		{
			evaluate();
			_valid = true;
		}

		return value_impl(id);
	}
};