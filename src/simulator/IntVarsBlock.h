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

#include "StateBlock.h"

class IntVars;

// wraps integration variables into a stateblock for generalized access.
// normally, only a part of the integration variables are access thru IntVarsBlock.
// some variables (mostly the optional ones, like variable masses), are accessed thru other specific blocks.

class IntVarsBlock : public StateBlock
{
public:
	IntVarsBlock(IntVars *intvars) : _intvars(intvars)
	{
	}

protected:
	IntVars *_intvars;

	// no evaluation required.
	virtual bool evaluate()
	{
		return true;
	}

	virtual double value_impl(unsigned int id);

public:

	// set the number of integration variables accessed thru this block
	// (not the total number of integration varibles, see State::set_intvars_num())
	void set_var_num(unsigned int num)
	{
		// TODO:
	}

	void set_var_name(unsigned int index, const char *name)
	{
		// TODO: named access
	}
};