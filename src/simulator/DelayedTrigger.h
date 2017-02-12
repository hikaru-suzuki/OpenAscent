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

#include "Trigger.h"

class Event;

// a trigger fired at a relative time after happening of base event.

class DelayedTrigger : public Trigger
{
public:
	DelayedTrigger(Event *base_event, double target) :
		_base_event(base_event),
		_target(target)
	{
	}

protected:
	const Event *_base_event;

	// relative time between _base_event's happening time and firing time
	double _target;

	// temporary variable used in the guess run
	double _y0;

public:
	virtual bool overrun();

	virtual void reset_guess();

	virtual bool guess(double &guess_length);
};