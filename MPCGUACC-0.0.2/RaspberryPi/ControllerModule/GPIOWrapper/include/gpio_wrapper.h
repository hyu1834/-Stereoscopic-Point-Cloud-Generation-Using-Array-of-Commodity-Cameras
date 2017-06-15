#ifndef GPIO_H
	#define GPIO_H

//Standard Libraries
#include <map>
#include <string>

//3rd Parties Libraries
#include <wiringPi.h>

//Local Libraries

//MACRO
#define MIN_GPIO_PIN 2
#define MAX_GPIO_PIN 27
#define AVAILABLE_GPIO_PIN 25

class GPIO_Wrapper	{
	private:
		std::map<std::string, int> pinsTable;
		bool *programmedPin;

	protected:

	public:
		GPIO_Wrapper();
		~GPIO_Wrapper();

		bool addPin(std::string name, int pin, int mode);
		bool writePin(int pin, int value);
		bool writePin(std::string name, int value);
		int readPin(int pin);
		int readPin(std::string name);
};

#endif
