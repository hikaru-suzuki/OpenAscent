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

class IntVars;
class State;

// represents an ordinary differential equation

class Function
{
public:
	virtual ~Function()
	{
	}

public:
	// function is responsible to tell how many integration variables in a state.
	// typical integration variables includes position, velocity, variable masses.
	// generally the number of masses (tanks) is subject to change.
	virtual unsigned int get_intvar_size() const = 0;

	// function is responsible to "build" a state by adding stateblocks to the state.
	virtual void build_state(State &state) = 0;

	//virtual bool check(State *state) = 0;

	// function is responsible to set state's initial value
	virtual void init_state(State &state) const = 0;

	// calculate integration variables' derivatives
	virtual bool derivatives(State &x, IntVars &y) const = 0;
};