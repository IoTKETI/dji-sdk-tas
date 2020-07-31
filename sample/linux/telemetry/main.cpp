
#include "telemetry_sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
using namespace std;

extern float_t cur_lati;
extern float_t cur_longi;

uint8_t ACTION_EVENT = 0;

#define TAKEOFF 0x01
#define LAND	0x02
#define MOVE	0x04
#define ALT		0x08
#define WAYPOINT	0x10
#define RTH		0x20

float g_xoffd = 0.0;
float g_yoffd = 0.0;
float g_zoffd = 0.0;
float g_yawd = 0.0;
float g_pth = 0.01;
float g_yawth = 0.01;

uint8_t g_numWaypoints = 1;
int g_responseTimeout = 0;
float32_t g_lat = 0.0;
float32_t g_lon = 0.0;
float32_t g_alt = 0.0;

int inProcessing = 0;
                     
void commandAction(Vehicle* vehicle) {
	
	//printf("%02x\r\n", ACTION_EVENT);
	
	// if(ACTION_EVENT & TAKEOFF) {
		
	// 	std::cout << "\r\nTAKEOFF\r\n";
		
	// 	inProcessing = 1;
		
	// 	//vehicle->obtainCtrlAuthority(functionTimeout);
		
	// 	monitoredTakeoff(vehicle);
		
	// 	inProcessing = 0;
	// 	ACTION_EVENT &= ~TAKEOFF;
	// }
	//else 
	// if(ACTION_EVENT & LAND) {
		
	// 	std::cout << "\r\nLAND\r\n";
		
	// 	inProcessing = 1;
		
	// 	//vehicle->obtainCtrlAuthority(functionTimeout);

	// 	monitoredLanding(vehicle);
		
	// 	inProcessing = 0;
	// 	ACTION_EVENT &= ~LAND;
	// }
	// else 
	if(ACTION_EVENT & MOVE) {
		
		printf("\r\nMOVE\r\n");
		
		inProcessing = 1;
		
		//vehicle->obtainCtrlAuthority(functionTimeout);
		
		moveByPositionOffset(vehicle, g_xoffd, g_yoffd, g_zoffd, g_yawd, g_pth, g_yawth);
		
		inProcessing = 0;
		ACTION_EVENT &= ~MOVE;
	}
	else if(ACTION_EVENT & ALT) {
		
		printf("\r\nALT\r\n");
		
		inProcessing = 1;
		
		//vehicle->obtainCtrlAuthority(functionTimeout);
		
		moveByPositionOffset(vehicle, g_xoffd, g_yoffd, g_zoffd, g_yawd, g_pth, g_yawth);
		
		inProcessing = 0;
		ACTION_EVENT &= ~ALT;
	}
	// else if(ACTION_EVENT & WAYPOINT) {
		
	// 	printf("\r\nWAYPOINT\r\n");
		
	// 	inProcessing = 1;
		
	// 	//vehicle->obtainCtrlAuthority(functionTimeout);
		
	// 	runWaypointMission(vehicle, g_numWaypoints, g_responseTimeout, g_lat, g_lon, g_alt);
		
	// 	inProcessing = 0;
	// 	ACTION_EVENT &= ~WAYPOINT;
	// }
	// else if(ACTION_EVENT & RTH) {
		
	// 	printf("\r\nRTH\r\n");
		
	// 	inProcessing = 1;
		
	// 	//vehicle->obtainCtrlAuthority(functionTimeout);
		
	// 	monitoredGoHome(vehicle);
		
	// 	inProcessing = 0;
	// 	ACTION_EVENT &= ~RTH;
	// }
}


typedef struct tTHREAD
{
	Vehicle* v;
	int s;
}Data;


void error(const char *msg)
{
	perror(msg);
	exit(0);
}

void *action_broadcast_data(void *arg)
{
	int sockfd;
	Vehicle* vehicle;
	int readn, writen;
	char buf[256];
	Data* d  = (Data*)arg;
	sockfd = d->s;
	vehicle = d->v;

	while (1)
	{
		getBroadcastData(vehicle, 1, sockfd);
	}
}

void *event_handler(void *arg)
{
	int sockfd;
	Vehicle* vehicle;
	int readn, writen;
	char buf[256];
	Data* d  = (Data*)arg;
	sockfd = d->s;
	vehicle = d->v;

	while (1)
	{
		commandAction(vehicle);
	}
}

void *action_takeoff(void *arg)
{
	int sockfd;
	Vehicle* vehicle;
	int readn, writen;
	char buf[256];
	Data* d  = (Data*)arg;
	sockfd = d->s;
	vehicle = d->v;

	while (1)
	{
		if(ACTION_EVENT & TAKEOFF) {
			ACTION_EVENT &= ~TAKEOFF;

			std::cout << "\r\nTAKEOFF\r\n";
			
			inProcessing = 1;
			
			monitoredTakeoff(vehicle);
			
			inProcessing = 0;
		}
	}
}

void *action_land(void *arg)
{
	int sockfd;
	Vehicle* vehicle;
	int readn, writen;
	char buf[256];
	Data* d  = (Data*)arg;
	sockfd = d->s;
	vehicle = d->v;

	while (1)
	{
		if(ACTION_EVENT & LAND) {
			ACTION_EVENT &= ~LAND;
		
			std::cout << "\r\nLAND\r\n";
			
			inProcessing = 1;
			
			monitoredLanding(vehicle);
			
			inProcessing = 0;
		}
	}
}

void *action_rth(void *arg)
{
	int sockfd;
	Vehicle* vehicle;
	int readn, writen;
	char buf[256];
	Data* d  = (Data*)arg;
	sockfd = d->s;
	vehicle = d->v;

	while (1)
	{
		if(ACTION_EVENT & RTH) {
			ACTION_EVENT &= ~RTH;

			printf("\r\nRTH\r\n");
			
			inProcessing = 1;
			
			monitoredGoHome(vehicle);
			
			inProcessing = 0;
		}
	}
}

void *action_waypoint(void *arg)
{
	int sockfd;
	Vehicle* vehicle;
	int readn, writen;
	char buf[256];
	Data* d  = (Data*)arg;
	sockfd = d->s;
	vehicle = d->v;

	while (1)
	{
		if(ACTION_EVENT & WAYPOINT) {
			ACTION_EVENT &= ~WAYPOINT;

			printf("\r\nWAYPOINT\r\n");
			
			inProcessing = 1;
			
			runWaypointMission(vehicle, g_numWaypoints, g_responseTimeout, g_lat, g_lon, g_alt);
			
			inProcessing = 0;
		}
	}
}


int main(int argc, char *argv[])
{
	// Setup OSDK.
	Data d;

	int functionTimeout = 1;

	int     responseTimeout = 1;
	uint8_t num_way = 1;
	LinuxSetup linuxEnvironment(argc, argv);
	Vehicle*   vehicle = linuxEnvironment.getVehicle();

	if (vehicle == NULL)
	{
		std::cout << "Vehicle not initialized, exiting.\n";
		return -1;
	}

	int sockfd, portno, n;
	struct sockaddr_in serv_addr;
	struct hostent *server;
	pthread_t thread_broadcast_data, thread_t2, thread_takeoff, thread_land, thread_rth, thread_waypoint;
	int th_id_broadcast_data, th_id2, th_id_takeoff, th_id_land, th_id_rth, th_id_waypoint;
	char buffer[256];
	int time = 0;

	portno = atoi("3105");
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0)
		error("ERROR opening socket");
	server = gethostbyname("127.0.0.1");

	bzero((char *)&serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	bcopy((char *)server->h_addr, (char *)&serv_addr.sin_addr.s_addr, server->h_length);
	serv_addr.sin_port = htons(portno);

	if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
	{
		error("ERROR connecting");
	}

	d.v = vehicle;
	d.s = sockfd;

	th_id2 = pthread_create(&thread_t2, NULL, event_handler, (void *)&d);
	if (th_id2 != 0)
	{
		perror("Thread Create 2 Error");
		return 1;
	}
	pthread_detach(thread_t2);

	th_id_broadcast_data = pthread_create(&thread_broadcast_data, NULL, action_broadcast_data, (void *)&d);
	if (th_id_broadcast_data != 0)
	{
		perror("Thread of broadcast data Create Error");
		return 1;
	}
	pthread_detach(thread_broadcast_data);
	
	th_id_takeoff = pthread_create(&thread_takeoff, NULL, action_takeoff, (void *)&d);
	if (th_id_takeoff != 0)
	{
		perror("Thread of takeoff Create Error");
		return 1;
	}
	pthread_detach(thread_takeoff);

	th_id_land = pthread_create(&thread_land, NULL, action_land, (void *)&d);
	if (th_id_land != 0)
	{
		perror("Thread of land Create Error");
		return 1;
	}
	pthread_detach(thread_land);

	th_id_rth = pthread_create(&thread_rth, NULL, action_rth, (void *)&d);
	if (th_id_rth != 0)
	{
		perror("Thread of rth Create Error");
		return 1;
	}
	pthread_detach(thread_rth);


	th_id_waypoint = pthread_create(&thread_waypoint, NULL, action_waypoint, (void *)&d);
	if (th_id_waypoint != 0)
	{
		perror("Thread of waypoint Create Error");
		return 1;
	}
	pthread_detach(thread_waypoint);

	vehicle->obtainCtrlAuthority(functionTimeout);

	while (1)
	{
		if (recv(sockfd, buffer, 255, MSG_DONTWAIT) != -1)
		{
			printf(buffer);
			if(inProcessing == 1) {
				printf("\r\nDrone is IN PROCESSING\r\n");
				memset(buffer, 0, sizeof(char)* 256);
			}
			else {
				printf("Data : %s\n", buffer);
				//printf(buffer);

				char* token = NULL;
				
				char split[] = ":";
				string str[50];
				int i=0;

				token = strtok(buffer, split);

				while (token != NULL)
				{
					printf("token = %s\n", token);
					str[i] = string(token);
					cout<<str[i]<<"\n";
					token = strtok(NULL, split);
					i++;
				}

				//vehicle->obtainCtrlAuthority(functionTimeout);
				
				char* sw_d = new char [2];
				strcpy(sw_d, str[0].c_str());


				switch (sw_d[0])
				{
				case 't':
					ACTION_EVENT |= TAKEOFF;
					break;

				case 'l':
					ACTION_EVENT |= LAND;
					break;
									
				case 'm':
					g_xoffd = strtof(str[1].c_str(), 0);
					g_yoffd = strtof(str[2].c_str(), 0);
					g_zoffd = strtof(str[3].c_str(), 0);
					g_yawd = strtof(str[4].c_str(), 0);
					g_pth = 0.01;
					g_yawth = 0.01;
					
					ACTION_EVENT |= MOVE;
					break;
					
				case 'a':
					g_xoffd = 0.0; 
					g_yoffd = 0.0;
					g_zoffd = strtof(str[1].c_str(), 0);
					g_yawd = 0.0;
					g_pth = 0.01;
					g_yawth = 0.01;
					
					ACTION_EVENT |= ALT;
					break;

				case 'g':
					printf("g command = original data : %f, %f, %f\r\n", strtof(str[1].c_str(), 0), strtof(str[2].c_str(), 0), strtof(str[3].c_str(), 0));
						
					g_numWaypoints = num_way;
					g_responseTimeout = responseTimeout;
					g_lat = (strtof(str[1].c_str(), 0)/57.295779513082320876798154814);
					g_lon = (strtof(str[2].c_str(), 0)/57.295779513082320876798154814);
					g_alt = strtof(str[3].c_str(), 0);
					
					printf("g command = g_lat, g_lon, g_alt: %f, %f, %f\r\n", g_lat, g_lon, g_alt);

					ACTION_EVENT |= WAYPOINT;
					break;

				case 'h':
					ACTION_EVENT |= RTH;
					break;

				default:
					break;
				}
					
				memset(buffer, 0, sizeof(char)* 256);
			}
		}
		
		//commandAction(vehicle);
	}

	

	close(sockfd);

	return 0;
}
