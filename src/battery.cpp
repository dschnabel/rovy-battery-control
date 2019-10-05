// https://gist.github.com/dschnabel/f2ea33913a214fde07cfbcb7c700b794

#include <iostream>
#include <fstream>
#include <bitset>
#include <math.h>
#include <iomanip>
#include <ctime>

#include <wiringpi/wiringPi.h>
#include <wiringpi/mcp23017.h>
#include <wiringpi/mcp3004.h>

using namespace std;

// mcp23017
#define PIN_CHARGING	101
#define PIN_BAT1		102
#define PIN_BAT2		103
#define PIN_BAT3		104
#define PIN_BAT4		105
#define PIN_BAT5		106
#define PIN_BAT6		107
#define PIN_ADC_ENABLE	108
#define PIN_READ_VOLT	109

// mcp3008
#define PIN_ANALOG1		200

#define CS_ENABLE		0
#define CS_DISABLE		1

#define NOT_CHARGING   -1
#define CHARGING		0
#define FULLY_CHARGED	1

//############ battery voltage ################
double numberToVoltage(int num) {
	if (num == 0) return 0;

	static int base = 15;

	double a = -0.004767104;
	double b = 0.007744394;
	double c = 0.000004493799;
	double d = 2.357942e-9;

	// cubic regression
	return base + (a) + (b * num) + (c * pow(num,2)) - (d * pow(num,3));
}

double getBatteryVoltage() {
	double voltNumber = 0;
	int iterations = 10;

	digitalWrite(PIN_READ_VOLT, 1);
	for (int i = 0; i < iterations; i++) {
		digitalWrite(PIN_ADC_ENABLE, CS_ENABLE);
		voltNumber += analogRead(PIN_ANALOG1);
		digitalWrite(PIN_ADC_ENABLE, CS_DISABLE);
	}
	digitalWrite(PIN_READ_VOLT, 0);

	voltNumber /= iterations;
	return numberToVoltage(voltNumber);
}
//#############################################

//############ charging #######################
int getChargingStatus(string &cellMap) {
	if (!digitalRead(PIN_CHARGING)) {
		return NOT_CHARGING;
	}
	bool done = true;
	for (int i = PIN_BAT6; i >= PIN_BAT1; i--) {
		int status = digitalRead(i);
		if (!status) done = false;
		cellMap += to_string(status);
	}
	if (done) return FULLY_CHARGED;
	else return CHARGING;
}
//#############################################

void haltSystem() {
	system("halt");
}

int main(int argc, char *argv[]) {
	wiringPiSetup();

	// setup mcp23017 (16-bit IO expansion)
	mcp23017Setup(100, 0x20) ;
	pinMode(PIN_CHARGING, INPUT);
	pinMode(PIN_BAT1, INPUT);
	pinMode(PIN_BAT2, INPUT);
	pinMode(PIN_BAT3, INPUT);
	pinMode(PIN_BAT4, INPUT);
	pinMode(PIN_BAT5, INPUT);
	pinMode(PIN_BAT6, INPUT);
	pinMode(PIN_ADC_ENABLE, OUTPUT);
	pinMode(PIN_READ_VOLT, OUTPUT);

	// setup mcp3008 (analog to digital converter)
	mcp3004Setup(200, 0);
	pinMode(PIN_ANALOG1, INPUT);

	// discard first reading
	getBatteryVoltage();

	ofstream logFile;
	int lowLevelCount = 0;

	while (1) {
		delay(10000);
		logFile.open("/var/log/battery.log", ofstream::out | ofstream::app);

		double voltage = getBatteryVoltage();
		logFile << time(0) << ", " << setprecision(4) << voltage << "V, ";

		string cellMap;
		switch (getChargingStatus(cellMap)) {
		case NOT_CHARGING:
			logFile << "not-charging"; break;
		case CHARGING:
			logFile << "charging (" << cellMap << ")"; break;
		case FULLY_CHARGED:
			logFile << "fully-charged"; break;
		}

		logFile << endl;
		logFile.close();

		if (voltage < 18) {
			if (lowLevelCount >= 3) {
				haltSystem();
			}
			lowLevelCount++;
		} else {
			lowLevelCount = 0;
		}
	}
}
