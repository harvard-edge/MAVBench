// ConsoleApplication2.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>


#define WINVER 0x0500
#include <windows.h>

	int main()
	{
		// This structure will be used to create the keyboard
		// input event.
		INPUT ip;

		// Pause for 5 seconds.
		int time_to_trigger;
		std::cin >> time_to_trigger;
		Sleep(time_to_trigger*1000);

		// Set up a generic keyboard event.
		ip.type = INPUT_KEYBOARD;
		ip.ki.wScan = 0; // hardware scan code for key
		ip.ki.time = 0;
		ip.ki.dwExtraInfo = 0;

		// Press the "r" key
		ip.ki.wVk = 0x52; // virtual-key code for the "a" key
		ip.ki.dwFlags = 0; // 0 for key press
		SendInput(1, &ip, sizeof(INPUT));
		Sleep(100);
		// Release the "r" key
		ip.ki.dwFlags = KEYEVENTF_KEYUP; // KEYEVENTF_KEYUP for key release
		SendInput(1, &ip, sizeof(INPUT));
	
		// Exit normally
		return 0;
	}

