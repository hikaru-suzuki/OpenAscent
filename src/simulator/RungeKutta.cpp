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

#include "State.h"
#include "Function.h"

#include "RungeKutta.h"

RungeKutta::RungeKutta(Function *function) : Integrator(function)
{
	unsigned int intvar_size = function->get_intvar_size();

	// initialize size of intermediate variables.
	_k1.set_size(intvar_size);
	_k2.set_size(intvar_size);
	_k3.set_size(intvar_size);
	_k4.set_size(intvar_size);
}

bool RungeKutta::attempt_impl(double h, State &current, State &attempt)
{
	// k1
	if (!_function->derivatives(current, _k1))
		return false;

	// k2
	_temp_state.set_intvars(current.intvars() + _k1 * (h / 2.0));
	_temp_state.set_t(current.t() + h / 2.0);
	_temp_state.invalidate();

	if (!_function->derivatives(_temp_state, _k2))
		return false;

	// k3
	_temp_state.set_intvars(current.intvars() + _k2 * (h / 2.0));
	_temp_state.invalidate();

	if (!_function->derivatives(_temp_state, _k3))
		return false;

	// k4
	_temp_state.set_intvars(current.intvars() + _k3 * h);
	_temp_state.set_t(current.t() + h);
	_temp_state.invalidate();

	if (!_function->derivatives(_temp_state, _k4))
		return false;

	// final result
	attempt.set_intvars(current.intvars() + (_k1 + _k2 * 2.0 + _k3 * 2.0 + _k4) * (h / 6.0));
	attempt.set_t(current.t() + h);
	attempt.invalidate();

	return true;
}

void RungeKutta::init()
{
	// must call base class's init()
	Integrator::init();

	// build intermediate state with funtion.
	_function->build_state(_temp_state);
}
