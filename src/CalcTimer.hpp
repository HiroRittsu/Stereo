#include <stdio.h>
#include <iostream>
#include <chrono>

#ifndef   CalcTimer_HPP
#define   CalcTimer_HPP

using namespace std;

class CalcTimer {
public:
	CalcTimer();
	~CalcTimer();
	double convertUnit(double data, std::string unit);
	void startTime();
	double getTime();

private:
	chrono::system_clock::time_point start;
	chrono::system_clock::time_point end;
};

CalcTimer::CalcTimer() {

}

CalcTimer::~CalcTimer() {

}

double CalcTimer::convertUnit(double data, std::string unit) {


	return  0;
}

void CalcTimer::startTime() {

	this->start = chrono::system_clock::now();

}

double CalcTimer::getTime() {

	this->end = chrono::system_clock::now();

	return static_cast<double>(chrono::duration_cast<chrono::microseconds>(this->end - this->start).count() / 1000.0);
}


#endif