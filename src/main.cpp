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

#include "simulator/Function.h"
#include "simulator/RungeKutta.h"

#include <iostream>

class Tsiolkovsky : public Function
{
public:
	static const unsigned int NVAR = 2;
	static const unsigned int M = 0;
	static const unsigned int V = 1;

private:
	double m0;
	double dm;
	double isp;

public:
	Tsiolkovsky()
	{
		m0 = 100.0;
		dm = 0.1;
		isp = 4000.0;
	}

	virtual ~Tsiolkovsky()
	{
	}

public:
	virtual unsigned int get_intvar_size() const
	{
		return NVAR;
	}

	virtual void build_state(State &state)
	{
		state.set_intvars_num(NVAR);

		return;
	}

	virtual void init_state(State &state) const
	{
		state.intvars()[M] = m0;
		state.intvars()[V] = 0.0;

		return;
	}

	virtual bool derivatives(State &x, IntVars &y) const
	{
		y[M] = -dm;
		y[V] = isp * dm / x.intvars()[M];

		return true;
	}
};

int main()
{
	Tsiolkovsky function;

	RungeKutta integrator(&function);

	integrator.init();

	function.init_state(integrator.current_state());

	std::cout << integrator.current_state().t() << '\t' 
		<< integrator.current_state().intvars()[Tsiolkovsky::M] << '\t'
		<< integrator.current_state().intvars()[Tsiolkovsky::V] << std::endl;		

	for (int i = 0; i < 900; ++i)
	{
		integrator.attempt(1.0);
		integrator.apply_attempt();

		std::cout << integrator.current_state().t() << '\t'
			<< integrator.current_state().intvars()[Tsiolkovsky::M] << '\t'
			<< integrator.current_state().intvars()[Tsiolkovsky::V] << std::endl;		
	}

 	return 0;
}
