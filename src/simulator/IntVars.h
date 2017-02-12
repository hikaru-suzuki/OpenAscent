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

// Integration Variables
class IntVars
{
public:
	IntVars()
	{
	}

	virtual ~IntVars(void)
	{
	}

protected:
	// IMPORTANT: User must ensure MAX_VAR_NUM is large than the number of all integration variables.
	// Typical integration variables include: position, velocity, attitude, variable mass etc.
	static const unsigned int MAX_VAR_NUM = 13;

	// number of variables
	unsigned int _size;

public:
	double _data[MAX_VAR_NUM];

public:
	void set_size(unsigned int size)
	{
		assert(size <= MAX_VAR_NUM);
		_size = size;
	}

	// retrieve variable by index
	double &operator [](unsigned int i)
	{
		assert(i < _size);
		return _data[i];
	}

	double operator [](unsigned int i) const
	{
		assert(i < _size);
		return _data[i];
	}

	// element-wise add
	friend IntVars operator +(const IntVars &l, const IntVars &r)
	{
		assert(l._size == r._size);

		IntVars result = l;

		for (unsigned int i = 0; i < l._size; ++i)
		{
			result._data[i] += r._data[i];
		}

		return result;
	}

	// multiplies a double value for each element.
	friend IntVars operator *(const IntVars &l, double r)
	{
		IntVars result = l;

		for (unsigned int i = 0; i < l._size; ++i)
		{
			result._data[i] *= r;
		}

		return result;
	}
};