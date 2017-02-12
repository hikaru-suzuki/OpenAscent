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
#include "State.h"
#include "Integrator.h"

class Function;

// RK4 integrator implementation

class RungeKutta : public Integrator
{
public:
	RungeKutta(Function *function);

protected:
	// intermediate variables
	IntVars _k1, _k2, _k3, _k4;
	State _temp_state;

	virtual bool attempt_impl(double h, State &current, State &attempt);

public:
	virtual void init();
};