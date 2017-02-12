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

#include "IntVars.h"
#include "IntVarsBlock.h"

class VarRef;

class State
{
public:
	State() : _intvars_block(&_intvars)
	{
		_blocks.push_back(&_intvars_block);
	}

	virtual ~State()
	{
		// delete all blocks except for intvars block
		for (unsigned int i = 1; i < _blocks.size(); ++i)
			delete _blocks[i];
	}

protected:
	// The independant variable of the integration, normally: "Time".
	double _t;

	// A state object is based on an IntVar object.
	// All values held by the state are derived from the integration variables and time.
	IntVars _intvars;

	// IntVarsBlock renders the integration variables as a StateBlock for generalized access.
	IntVarsBlock _intvars_block;

	// All variables other than IntVars is held as blocks.
	vector<StateBlock *> _blocks;

public:
	// set the number of integration variables
	void set_intvars_num(unsigned int size)
	{
		_intvars.set_size(size);
	}

	// access of IntVars.
	const IntVars &intvars() const
	{
		return _intvars;
	}

	IntVars &intvars()
	{
		return _intvars;
	}

	void set_intvars(const IntVars &intvars)
	{
		_intvars = intvars;
	}

	// access of IntVarsBlock
	IntVarsBlock *get_intvars_block()
	{
		return &_intvars_block;
	}

	// access of time
	double t() const
	{
		return _t;
	}

	void set_t(double t)
	{
		_t = t;
	}

	// Adds a block to this state, the blocks will be destrcuted along with the state object.
	// return the ID of the block, the ID can be furthur used to access the block faster (than access by block's name).
	unsigned int add_block(StateBlock *block)
	{
		_blocks.push_back(block);
		block->set_parent(this);
		block->initialize();
		return _blocks.size() - 1;
	}

	// Retrieves a block by ID.
	StateBlock *block(unsigned int id)
	{
		return _blocks[id];
	}

	// Retrieves a block b name. Generally slower. Avoid using this repeatedly in computation.
	StateBlock *block(const string &name)
	{
		// TODO: improve implementation
		for (unsigned int i = 0; i < _blocks.size(); ++i)
			if (_blocks[i]->name() == name)
				return _blocks[i];
		return 0;
	}

	// Retrieve block id with block name
	unsigned int find_block_id(const string &name) const
	{
		for (unsigned int i = 0; i < _blocks.size(); ++i)
			if (_blocks[i]->name() == name)
				return i;
		return 9999;
	}

	// Retrieve variable value with a VarRef object.
	double value(const VarRef &ref);

	// Must be called after finished changing the base values (time, IntVars) and/or the Fuction's evaluation method.
	// Renders all StateBlock variables "invalidate".
	// On a new access of to a StateBlock, it will be re-evaluated with the new base values or funtion.
	void invalidate()
	{
		for (unsigned int i = 0; i < _blocks.size(); ++i)
			_blocks[i]->invalidate();
	}
};