// TODO remove unused includes
// TODO add define tags to remove output
// TODO add acknowledgments and license
// TODO add precompiler defines and flags for text or now
// TODO generate make file
// TODO clean up code (formatting & removing unused functions)
// TODO create git repository
// TODO Hack in a mutex lock


#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <thread>
#include <fstream>
#include <string>
#include <iostream>
#include <fstream>
// TODO figure out industry standard for this
#include "../includes/mavlink/common/mavlink.h"
#include "../includes/common/util.h"
#include "../includes/nlohmann/json.hpp"

struct coordinate {
	double lat;
	double lon;
	double alt;
};

void parseInputs(int, char**);
void printInputHelp();
static int sendMavlinkMessage(mavlink_message_t *);
static int setupConnection(const char *, int);
void endConnection();
void sendQGCMessage(const char *, int);
double findDistance(coordinate *, coordinate *);
void printMavlinkMessagesIDs();
void getGPSCoordinates();
void readConfigFile(std::string);

double deg2rad(double deg) { return (deg * M_PI /180); };
double rad2deg(double rad) { return (rad * 180 / M_PI); };

/*
// TODO Add more options and split into multiple lines
void printInputHelp() {
	std::cout << "**************************\nSparrow usage\n -L [Latitude] [Longitude]\n -F [Configuration File]\n -I [IP Address]:[Port]\n -? <Help>\nLOADED DEFAULT VALUES FOR REMAINING OPTIONS\n***************************\n";
}

// TODO add more error checking and make it functional
void parseInputs(int argc, char *argv[]) {
	if(argc > 1) {
		int count = 1;
		while(count < argc) {
			if ((sizeof(argv[count]) / sizeof(*argv[count]) != 2) && (argv[count][0] == '-')) {
				// Add error checking for type and assign values and add -s and -t -?
				switch(argv[count][1]) {
					case 'F':
					case 'f':
						if ((count + 1) < argc) {
							std::cout << "File option selected: " << argv[count + 1] << '\n';
							// Load Config File again
						} else {
							printInputHelp();
							return;
						}
						count += 2;
						break;
					case 'I':
					case 'i':
						if ((count + 1) < argc) {
							std::cout << "IP address option selected: " << argv[count + 1] << '\n';
						} else {
							printInputHelp();
							return;
						}
						count += 2;
						break;
					case 'L':
					case 'l':
						if ((count + 2) < argc) {
							std::cout << "position added\n";
						} else {
							printInputHelp();
							return;
						}
						count += 3;
						break;
					case 'T':
					case 't':
						// target ID
						break;
					case 'P':
					case 'p':
						//port
					case 'R':
					case 'r':
						//rally
					case 'S':
					case 's':
						//Sys ID
					case 'A':
					case 'a':
						// Auth ID
					case 'H':
					case 'h':
					case '?':
					default:
						printInputHelp();
						return;

				}
			} else {
				printInputHelp();
				return;
			}

			//++count;
		}
	}

	// Parse file
}
*/
