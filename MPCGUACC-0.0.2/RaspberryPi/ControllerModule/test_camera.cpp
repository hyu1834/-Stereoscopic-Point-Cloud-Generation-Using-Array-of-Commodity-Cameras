//Standard Libraries
#include <stdio.h>
#include <iostream>

//3rd Parties Libraries
#include <wiringPi.h>

//Local Libraries
#include "gpio_wrapper.h"

//MACRO
#define CAPTURE_PIN 29
#define TERMINATE_PIN 27


int main(int argc, char** argv)	{
	GPIO_Wrapper *gpio = new GPIO_Wrapper();
	gpio->addPin("CAPTURE_PIN", CAPTURE_PIN, OUTPUT);
	gpio->addPin("TERMINATE_PIN", TERMINATE_PIN, OUTPUT);
	std::cout<<"Added new pin\n";

	char c;
	while(1)	{
		std::cin>>c;
		std::cout<<"Got char: "<<c<<"\n";
		if(c == 'c')	{
			gpio->writePin("CAPTURE_PIN", HIGH);
			delay(100);
			gpio->writePin("CAPTURE_PIN", LOW);
			std::cout<<"capture\n";
		}
		if(c == 'q')	{
			gpio->writePin("TERMINATE_PIN", HIGH);
			delay(100);
			gpio->writePin("TERMINATE_PIN", LOW);
			break;
		}
	}

	delete gpio;
	return 0;
}
