#include "Fuel_System.h"


Fuelsystem::Fuelsystem
(
	State& state,
	Input& input,
	Engine& engine

) :
	m_state(state),
	m_input(input),
	m_engine(engine)


	//jetzt Tables wenn nötig
	//und: DER LETZTE EINTRAG DARF KEIN KOMMA HABEN!!!

{
	//Sülz und Bla....
}

void Fuelsystem::zeroInit()
{
	
}

void Fuelsystem::coldInit()
{
	zeroInit();
}

void Fuelsystem::hotInit()
{
	zeroInit();
}

void Fuelsystem::airborneInit()
{
	zeroInit();
}

void Fuelsystem::drawFuel(double dm)
{

}

void Fuelsystem::addFuel(double dm)
{
	//If adding fuel add it to the internal tank.
	if (dm > 0.0 )
		dm = addFuelToTank(INTERNAL, dm);

	//If removing fuel remove it from the internal tanks.
	if (dm < 0.0)
		dm = addFuelToTank(INTERNAL, dm);
}

void Fuelsystem::update(double dt)
{
	//The factor from the engine for transfer since all transfer relies on bleed,
	//either directly to pressurise the tank or to spin a turbopump.
	double rateFactor = CON_fexch;

	double dm = m_engine.getFuelFlow() * dt;

	double enginePower = m_engine.getRPMNorm() > 0.40;

	//Draw from the Fuselage Tank to the engine, minimum usable fuel.
	if (m_fuel[INTERNAL] > 15.0)
	{
		if (dm > 0.0)
			m_fuel[INTERNAL] -= dm;

		m_hasFuel = true;
	}
	else
	{
		m_hasFuel = false;
	}
}


	