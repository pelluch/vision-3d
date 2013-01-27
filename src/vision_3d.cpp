#include <iostream>

void extractSurfFeatures()
{
	std::cout << "Running surflet test" << std::endl;
}

void printMenu()
{
	std::cout << "1. Extract surflet features" << std::endl;
	std::cout << "2. Exit" << std::endl;
}

int main(int argc, char ** argv)
{

	printMenu();
	int option = 0;
	std::cin >> option;

	while(option < 1 || option > 2)
	{
		std::cout << "Invalid option. Please choose one of the given options." << std::endl;
		printMenu();
		std::cin >> option;
	}

	switch(option)
	{
		case 1:
			extractSurfFeatures();
			break;
		case 2:
			std::cout << "Exiting" << std::endl;
	}

	return 0;
}