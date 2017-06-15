//Standard Libraries

//3rd Parties Libraries

//Local Libraries
#include "gpio_wrapper.h"

//MACRO


GPIO_Wrapper::GPIO_Wrapper()	{
	programmedPin = new bool[AVAILABLE_GPIO_PIN];
	wiringPiSetup();
}

GPIO_Wrapper::~GPIO_Wrapper()	{
	//turn off all pin
	for(std::map<std::string, int>::iterator it = pinsTable.begin(); it != pinsTable.end(); it++)	{
		digitalWrite(it->second, LOW);
	}
	pinsTable.clear();
	delete[] programmedPin;
}

bool GPIO_Wrapper::addPin(std::string name, int pin, int mode)	{
	if(programmedPin[pin])	{
		return false;
	}

	pinsTable[name] = pin;
	pinMode(pin, mode);
	//set default pin to low
	digitalWrite(pin, LOW);
	return true;
}

bool GPIO_Wrapper::writePin(int pin, int value)	{
	if(programmedPin[pin])	{
		return false;
	}
	digitalWrite(pin, value);
}

bool GPIO_Wrapper::writePin(std::string name, int value)	{
	return writePin(pinsTable[name], value);
}

int GPIO_Wrapper::readPin(int pin)	{
	if(programmedPin[pin])	{
		return false;
	}

	digitalRead(pin);
}

int GPIO_Wrapper::readPin(std::string name)	{
	return readPin(pinsTable[name]);
}
