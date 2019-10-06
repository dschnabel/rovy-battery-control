#include <iostream>
#include <fstream>
#include <bitset>
#include <math.h>
#include <string.h>
#include <iomanip>
#include <ctime>
#include <vector>
#include <map>

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

#define LOG_FILE "/var/log/battery.log"
#define BIN_FILE "/opt/voltTimes.bin"

#define VOLTAGE_MAX 25.0962
#define VOLTAGE_MIN 18

typedef struct voltage_time_pair {
	double voltage;
	long timeDiff;
} voltage_time_pair_t;

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

double getBatteryVoltagePercentage(double voltage) {
	double percentage = (voltage - VOLTAGE_MIN) * 100 / (VOLTAGE_MAX - VOLTAGE_MIN);
	return max(percentage, 0.0);

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

//############# voltage history ##################
string& trim(string& str, const string& chars = "\t\n\v\f\r ") {
    str.erase(0, str.find_first_not_of(chars));
    str.erase(str.find_last_not_of(chars) + 1);
    return str;
}

vector<string> split(const string& s, char delimiter) {
   vector<string> tokens;
   string token;
   istringstream tokenStream(s);
   while (getline(tokenStream, token, delimiter)) {
      tokens.push_back(trim(token));
   }
   return tokens;
}

time_t stringToTime(string& str) {
	time_t t = 0;
	try {
		t = stol(str);
	} catch (const invalid_argument& ia) {
		cerr << "Invalid argument: " << str << '\n';
	}
	return t;
}

double stringToVolts(string& str) {
	double v = 0;
	try {
		v = stod(str);
	} catch (const invalid_argument& ia) {
		cerr << "Invalid argument: " << str << '\n';
	}
	return v;
}

void parseVoltageTimesFromLog(string file, map<double, long>& voltTimes) {
	ifstream logFile(file);
	if (logFile.is_open()) {
		vector<string> log;
		string line;
		while (getline(logFile, line)) {
			log.push_back(line);
		}
		logFile.close();

		int mode = 0;
		long fullyCharged = 0;

		for (vector<string>::reverse_iterator rit = log.rbegin(); rit!= log.rend(); ++rit) {
			vector<string> tokens = split(*rit, ',');
			if (tokens.size() < 4) continue;
			string status = tokens[3];

			if (mode == 0) {
				if (status.compare("fully-charged") != 0) {
					continue;
				}
				mode = 1;
			}
			if (mode == 1) {
				if (status.compare("fully-charged") == 0) {
					fullyCharged = stringToTime(tokens[0]);
					continue;
				}
				mode = 2;
			}
			if (mode == 2) {
				if (status.rfind("charging", 0) == 0) {
					time_t t = stringToTime(tokens[0]);
					if (t == 0) continue;

					double v = stringToVolts(tokens[1]);
					if (v == 0) continue;

					voltTimes[v] = fullyCharged - t;
				} else {
					mode = 3;
				}
			}
			if (mode == 3) break;
		}
	}
}

void updateVoltageTimesFromHistory(string file, map<double, long>& voltTimes) {
	double minVolts = voltTimes.begin()->first;
	long maxDiff = voltTimes.begin()->second;

	ifstream vtFile(file, ios::binary);
	if (vtFile.is_open()) {
		size_t size;
		vtFile.read((char*)&size, sizeof(size_t));
		for (uint i = 0; i < size; i++) {
			voltage_time_pair_t vtp;
			vtFile.read((char*)&vtp, sizeof(voltage_time_pair_t));

			if (vtp.voltage < minVolts) {
				voltTimes[vtp.voltage] = max(vtp.timeDiff, maxDiff);
			} else {
				break;
			}
		}
		vtFile.close();
	}
}

void writeVoltageTimesToFile(string file, map<double, long>& voltTimes) {
	ofstream vtFile(file, ios::binary);
	size_t size = voltTimes.size();
	vtFile.write((char*)&size, sizeof(size_t));

	for (const auto& kv : voltTimes) {
		voltage_time_pair_t vtp = {kv.first, kv.second};
		vtFile.write((char*)&vtp, sizeof(voltage_time_pair_t));

	}
	vtFile.close();
}

long getVoltageTime(map<double, long> &voltTimes, double key) {
	map<double, long>::iterator it = voltTimes.find(key);
	if (it != voltTimes.end()) {
		return it->second;
	}

	map<double, long>::iterator near = voltTimes.lower_bound(key);
	if (near != voltTimes.end()) {
		if (near == voltTimes.begin()) {
			return near->second;
		}

		map<double, long>::iterator previous = prev(near);
		if ((key - previous->first) < (near->first - key)) {
			return previous->second;
		} else {
			return near->second;
		}
	} else {
		return 0;
	}
}

string getDurationEstimate(map<double, long> &voltTimes, double volts) {
	long estimation = getVoltageTime(voltTimes, volts);
	int seconds = estimation % 60;
	int minutes = (estimation / 60) % 60;
	int hours = (estimation / 60 / 60) % 60;

	stringstream ss;
	ss << setw(2) << setfill('0') << hours << ":";
	ss << setw(2) << setfill('0') << minutes << ":";
	ss << setw(2) << setfill('0') << seconds;
	return ss.str();
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

	// load and build voltage history
	map<double, long> voltTimes;
	parseVoltageTimesFromLog(LOG_FILE, voltTimes);
	updateVoltageTimesFromHistory(BIN_FILE, voltTimes);
	writeVoltageTimesToFile(BIN_FILE, voltTimes);

	// discard first reading
	getBatteryVoltage();

	ofstream logFile;
	int lowLevelCount = 0;
	double voltageLine = 0;

	// main loop
	while (1) {
		delay(10000);
		logFile.open(LOG_FILE, ofstream::out | ofstream::app);

		double voltage = getBatteryVoltage();
		logFile << time(0) << ", "
				<< fixed << setprecision(2) << voltage << "V, "
				<< getBatteryVoltagePercentage(voltage) << "%, ";

		string cellMap;
		switch (getChargingStatus(cellMap)) {
		case NOT_CHARGING:
			logFile << "not-charging"; break;
		case CHARGING:
			logFile << "charging, " << cellMap;
			if (voltage > voltageLine) {
				voltageLine = voltage;
				logFile  << ", " << getDurationEstimate(voltTimes, voltage);
			}
			break;
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
