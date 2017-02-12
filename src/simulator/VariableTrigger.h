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

#include <Common.h>

#include "Trigger.h"

#include "VarRef.h"

// trigger fired when a block variable value equals to a preset value.
// may not fire if its very close to the maximum or minimum value.

class VariableTrigger : public Trigger
{
public:
	VariableTrigger(const string &block_name, unsigned int var_id, bool ascending, double target) :
		_block_name(block_name),
		_var_id(var_id),
		_ascending(ascending),
		_target(target)
	{
	}

protected:
	// block variable access related
	string _block_name;
	unsigned int _var_id;

	// fast access handle, should be initialized with integrator
	VarRef _varref;

	// if true, the trigger will only be fired when the variable's value is ascending.
	// otherwise, the trigger will only be fired when the variable's value is descending.
	bool _ascending;

	// the target variable value to fire the trigger.
	double _target;

	// guess related;
	double _y0;
	double _x0;

public:
	virtual void bind_int_impl();

	virtual bool overrun();

	virtual void reset_guess();
	
	virtual bool guess(double &guess_length);
};