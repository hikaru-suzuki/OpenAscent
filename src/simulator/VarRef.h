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

// reference of block variable, can be used to access the variable
// it is also a fast access handle, access efficiency is highe than named access

class VarRef
{
public:
	// default constructor, must call set_ref() later to finish initialization.
	VarRef()
	{
	}

	VarRef(unsigned int block_id, unsigned int var_id) :
		_block_id(block_id),
		_var_id(var_id)
	{
	}

protected:
	// block id and variable id of the variable.
	unsigned int _block_id;
	unsigned int _var_id;

public:
	void set_ref(const State &state, const string &block_name, unsigned int var_id);

	unsigned int block_id() const
	{
		return _block_id;
	}

	unsigned int var_id() const
	{
		return _var_id;
	}
};