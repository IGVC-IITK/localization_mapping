#include "UTM.h"
#include <iostream>
#include <sstream>
#include <iomanip>
int main(int argc, char *argv[]){
	double latitude,longitude;
	std::cout<<std::setprecision(6)<<std::fixed;
	std::stringstream ss;
	std::string s=argv[1];
	ss << s; ss >> latitude;
	ss.clear();
	s = argv[2];
	ss << s; ss >> longitude;
	ss.clear();

	std::cout<<"latitude: "<<latitude<<"\tlongitude: "<<longitude<<"\n";
	long double x = 0.0000000;
	long double y = 0.0000000;
	int zone = LatLonToUTMXY(latitude,longitude,0,x,y);
	std::cout<<"easting: "<<x<<"\tnorthing: "<<y<<"\tzone: "<<zone<<"\n";

	return 0;

}