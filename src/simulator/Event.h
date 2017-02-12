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

#include "Common.h"

class Trigger;
class Action;
class State;

// an event is a series of actions triggered by a state.

class Event
{
public:
	// trigger will be deconstructed along with the event
	Event(const char *name, Trigger *trigger) :
		_name(name),
		_trigger(trigger),
		_happened(false)
	{
	}

	virtual ~Event();

protected:
	string _name;

	Trigger *_trigger;

	vector<Action *> _actions;

	// true if the event has already happened
	bool _happened;

	// if the event has already happened, stores the happening time.
	double _happen_t;

public:
	const char *name() const
	{
		return _name.c_str();
	}
	
	Trigger *trigger()
	{
		return _trigger;
	}

	// actions added to the event will be destructed along with the event.
	void add_action(Action *action)
	{
		_actions.push_back(action);
	}

	void do_actions(State &state);

	// set the event as happened
	void happen(double t)
	{
		_happened = true;
		_happen_t = t;
	}

	// reset the event as unhappened
	void clear_happen()
	{
		_happened = false;
	}
		
	bool happened() const
	{
		return _happened;
	}

	double happen_t() const
	{
		return _happen_t;
	}
};
