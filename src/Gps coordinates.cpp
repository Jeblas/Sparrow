// Add header define protection
#include "Sparrow.h"

int targetID = 1;
std::string configFileName = "config.json";
char *ip = "127.0.0.1";
//std::string ip = "127.0.0.1";
int port = 5760;
int paramIndex = 0;

bool isTriggered = false;

static int sock = -1;
static struct sockaddr_in sockaddr;
uint8_t mavbuffer[256];
uint8_t rx_buffer[256];

std::vector<coordinate> rallyPoints;
coordinate currentCoor;
coordinate homeCoordinate = {0,0,0};

// TODO check if file is missing
void readConfigFile(std::string fileName) {
	// http://github.com/nlohmann/json 
	nlohmann::json configParameter;
	std::ifstream inputConfFile(fileName);
	inputConfFile >> configParameter;
	
	targetID = configParameter[paramIndex]["targetID"];

	// TODO figure out string to char * conversion
	//ip = configParameter[paramIndex]["ip"];

	port = configParameter[paramIndex]["port"];
	for (auto & element : configParameter[paramIndex]["rallyPoints"]) {
			rallyPoints.push_back({element["Latitude"],element["Longitude"], 0});
		}
	return;

}

// TODO parse command line inputs
void setParameters() {
	readConfigFile(configFileName);
	//parseInputs();
}

// from px4-offboard-mode.cpp
// TODO not working
static int sendMavlinkMessage(mavlink_message_t * msg) {
	uint8_t data[MAVLINK_MAX_PACKET_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(data, msg);

    int r = sendto(sock, data, len, 0, (struct sockaddr *)&sockaddr, sizeof(sockaddr));
    if (r < 0) {
        fprintf(stderr, "Could not send to %d: r=%d (%m)\n", sock, r);
    }
    return r;
}

// Works TODO add error checking
static int startConnection(const char *ip, int port) {
	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock == -1) {
		std::cout << "Could not create socket\n";
		return -1;
	}

	sockaddr.sin_family = AF_INET;
	sockaddr.sin_addr.s_addr = inet_addr(ip);
	sockaddr.sin_port = htons(port);

	std::cout << "Connecting to TCP: " << ip << ':' << port << '\n';

	if (connect(sock, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0) {
		std::cout << "Could not connect to socket\n";
		return -1;
	}
	
	if (fcntl(sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0) {
		std::cout << "Error setting socket tcp_fd as non-blocking\n";
		return -1;
	}

	std::cout << "Sparrow connected\n";
	return 0;
}

void endConnection() {
	if (sock > 0) {
		close(sock);
	}
}

void sendDoReposition(coordinate * destination) {
	mavlink_message_t msg;
	mavlink_command_long_t cmd = {};

	double latitude = destination->lat * 1E7;
	double longitude = destination->lon * 1E7;
	double altitude = currentCoor.alt * 1E3;

	cmd.target_system = targetID;
	cmd.target_component = 0;
	cmd.command = MAV_CMD_DO_REPOSITION;
	cmd.param1 = -1.0f;
	cmd.param2 = MAV_DO_REPOSITION_FLAGS_CHANGE_MODE;
	cmd.param3 = 0.0f;

	cmd.param5 = latitude;	// deg * 1E7
	cmd.param6 = longitude;	// deg * 1E7
	cmd.param7 = altitude;	// meters

	mavlink_msg_command_long_encode(255, MAV_COMP_ID_ALL, &msg, &cmd); //255 is systemID
    sendMavlinkMessage(&msg);
}

void setGetCurrentCoordinate(double * lat, double * lon, double * alt, bool set) {
	if (set == true) {
		std::cout << "Think about this" << '\n';
	} else {
		std::cout << "Think this over" << '\n';
	}
	return;
}

void sendLandNow() {
	mavlink_message_t msg;
	mavlink_command_long_t cmd = {};

	// Firmware/src/modules/commander/px4_custom_mode.h Line 49
	// Firmware/src/modules/commander/commander.cpp Line 668
	cmd.target_system = targetID;
	cmd.target_component = 0;
	cmd.command = MAV_CMD_DO_SET_MODE;
	cmd.param1 = 255; // ??? if (base & custom mode) MAV_MODE
	cmd.param2 = 4; //PX4_CUSTOM_MAIN_MODE_AUTO
	cmd.param3 = 6; //PX4_CUSTOM_SUB_MODE_AUTO_LAND

	mavlink_msg_command_long_encode(255, MAV_COMP_ID_ALL, &msg, &cmd);
    sendMavlinkMessage(&msg);
}

// TODO finish reference wing client
void sendQGCMessage(const char * text, int severity) {
	std::cout << text << '\n';
	mavlink_message_t msg;
	mavlink_msg_statustext_pack(0, 0, &msg, severity, text);
	int len = mavlink_msg_to_send_buffer(mavbuffer, &msg);
	if (send(sock, &mavbuffer, len, 0) == -1) {
		std::cout << "Send QGC Message error\n";
	}
}

double findDistance(coordinate * current, coordinate * location) {
	double start_lat_rad, start_lon_rad, end_lat_rad, end_lon_rad, u, v, dist;
	start_lat_rad = deg2rad(current->lat);
	start_lon_rad = deg2rad(current->lon);
	end_lat_rad = deg2rad(location->lat);
	end_lon_rad = deg2rad(location->lon);
	u = sin((end_lat_rad - start_lat_rad)/2);
	v = sin((end_lon_rad - start_lon_rad)/2);

	// 6371.0 = EARTH's Radius in km
	dist = 2.0 * 6371.0 * asin(sqrt(u * u + cos(start_lat_rad) * cos(end_lat_rad) * v * v));
	return dist;
}

coordinate * getClosestRallyPoint() {
	double currentDistance;

	// TODO check launch positon
	//int index = -1;
	//double shortestDistance = findDistance(&currentCoor, &homeCoordinate);

	int index = 0;
	double shortestDistance = findDistance(&currentCoor, &rallyPoints[0]);
	for (int i = 1; i < rallyPoints.size(); ++i) {
		currentDistance = findDistance(&currentCoor, &rallyPoints[i]);
		if (currentDistance < shortestDistance) {
			shortestDistance = currentDistance;
			index = i;
		}
	}
	if (index == -1) {
		return &homeCoordinate;
	} else {
		return &rallyPoints[index];
	}
}

// TODO clean up
void handleTrigger() {
	coordinate  * closestRallyPoint;
	closestRallyPoint = getClosestRallyPoint();

	//sendQGCMessage("navigating to nearest coordinate at Lat: , Lon:", 1);	// add Lat and Lon Values
	sendDoReposition(closestRallyPoint);

	// .00001 degrees = 3 feet
	while (currentCoor.lat < closestRallyPoint->lat - .00001 
		&& currentCoor.lat > closestRallyPoint->lat + .00001
		&& currentCoor.lon < closestRallyPoint->lon - .00001 
		&& currentCoor.lon > closestRallyPoint->lon + .00001) {
		sleep(2);
	}

	//sendQGCMessage("Initiating Land Now", 1);
	sendLandNow();

	// While armed
	// while altitude != 0 or not disarmed or not something {
	//		sleep(2000);
	// }

	// make new mission to old home location
	// change mode back
	isTriggered = false;
}

void scanMavlinkMessages() {
	mavlink_status_t status;
	mavlink_message_t recv_msg;

	while (true) {
		ssize_t size = recv(sock, &rx_buffer, 256, 0);
		if (size > 0) {
			for (int i = 0; i < size; ++i) {
				if (mavlink_parse_char(MAVLINK_COMM_0, rx_buffer[i], &recv_msg, &status)) {
					switch (recv_msg.msgid) {
						case MAVLINK_MSG_ID_HEARTBEAT:
							mavlink_heartbeat_t heartbeat;
        					mavlink_msg_heartbeat_decode(&recv_msg, &heartbeat);
							// 131072 = Altitude Mode Need to figure out this bug should be 2
							//if (!isTriggered && heartbeat.base_mode == 131072) {
							//	isTriggered = true;
							//	std::thread triggered(handleTrigger);
							//	triggered.detach();
							//}	
						//case MAVLINK_MSG_ID_SYS_STATUS:
						//	break;
						case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
							std::cout << "Lat: " << mavlink_msg_global_position_int_get_lat(&recv_msg) << '\n';
							std::cout << "Lon: " << mavlink_msg_global_position_int_get_lon(&recv_msg) << '\n';
							
							currentCoor.lat = mavlink_msg_global_position_int_get_lat(&recv_msg) * 1E-7;
							currentCoor.lon = mavlink_msg_global_position_int_get_lon(&recv_msg) * 1E-7;
							currentCoor.alt = mavlink_msg_global_position_int_get_alt(&recv_msg) * 1E-3;
							break;
						case MAVLINK_MSG_ID_GPS_RAW_INT:
							// Might never get called since decoding satellites is weird
							if (mavlink_msg_gps_raw_int_get_satellites_visible(&recv_msg) > 0b1010) {				
								currentCoor.lat = mavlink_msg_gps_raw_int_get_lat(&recv_msg) * 1E-7;
								currentCoor.lon = mavlink_msg_gps_raw_int_get_lon(&recv_msg) * 1E-7;
								currentCoor.alt = mavlink_msg_gps_raw_int_get_alt(&recv_msg) * 1E-3;
								std::cout << "Lat: " << mavlink_msg_gps_raw_int_get_lat(&recv_msg) * 1E-7 << '\n';
								std::cout << "Lon: " << mavlink_msg_gps_raw_int_get_lon(&recv_msg) * 1E-7 << '\n';
								std::cout << "Alt: " << mavlink_msg_gps_raw_int_get_alt(&recv_msg) * 1E-3 << '\n';
							}
							break;
						default:
							break;
					}
				}
			}
		}
		sleep(.7);
	}
}

// TODO add movidius and intelligent rally point detection
void searchTerrain() {
	// need movidius and openCV
	while (true) {
		// Take picture
		// process
		sleep (30);
	}
	return;
}

int main(int argc, char *argv[]) {
// Setup
	setParameters();
	startConnection(ip, port);
	// TODO get Launch coordinates and store
	//free(ip);

// Main loops 
	std::thread scan(scanMavlinkMessages);
	std::thread search(searchTerrain);

// Clean up
	scan.join();
	search.join();
	endConnection();
	std::cout << std::flush;
	return 0;
}
