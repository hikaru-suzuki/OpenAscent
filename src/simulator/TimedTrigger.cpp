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

#include "TimedTrigger.h"

#include "Integrator.h"

bool TimedTrigger::overrun()
{
	// can fire when _integrator->current_state().t() = _target, since there may be more than 1 timed trigger fired at the same time.
	return _integrator->current_state().t() <= _target && _integrator->attempted_state().t() >= _target;
}

void TimedTrigger::reset_guess()
{
	_y0 = _integrator->current_state().t();
}
