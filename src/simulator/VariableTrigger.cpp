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

#include "VariableTrigger.h"

#include "Integrator.h"

#include "State.h"

void VariableTrigger::bind_int_impl()
{
	// initialize fast access handle of the varible.
	_varref.set_ref(_integrator->current_state(), _block_name, _var_id);
}

bool VariableTrigger::overrun()
{
	if (_ascending)
		return _integrator->current_state().value(_varref) < _target && _integrator->attempted_state().value(_varref) >= _target;
	else
		return _integrator->current_state().value(_varref) > _target && _integrator->attempted_state().value(_varref) <= _target;
}

void VariableTrigger::reset_guess()
{
	// initial base value
	_x0 = 0.0;
	_y0 = _integrator->current_state().value(_varref);
}

bool VariableTrigger::guess(double &guess_length)
{
	double attempt_x = guess_length;
	double attempt_y = _integrator->attempted_state().value(_varref);

	if (fabs(attempt_y - _target) > 1E-6)
	{
		// linear estimation of a new guess_length
		guess_length = _x0 + (_target - _y0) * (attempt_x - _x0) / (attempt_y - _y0);

		// if attempt value is closer than base, select it as the new base
		if (fabs(attempt_y - _target) < fabs(_y0 - _target))
		{
			_x0 = attempt_x;
			_y0 = attempt_y;
		}

		return false;
	}

	return true;
}
