#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <curses.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include "Msg_Queue.h"
#include <fcntl.h> 
#include <termios.h>
#include <libxml/parser.h>

#include <sqlite3.h>

#define POSTING_LED 5
#define NETWORK_LED 6
#define BLE_LED 13

#define WITH_FILTER
#define UART_READER
#define MODEMDEVICE				"/dev/ttyUSB0"
#if PC
#define INTERNET_SPEED_FILE_PATH		"/tmp/Internet_Speed.txt"
#define SCANNED_BEACONS_LOG_FILE		"/tmp/Scanned_Beacons.txt"
#define HTTP_Post_Response			"/tmp/http_post_response.txt"
#define Diagnostic_Post_Response		"/tmp/diagnostic_post_response.txt"
#define Configuration_XML_Path			"/tmp/Configuration.xml"
#define Configuration_Text_Path			"/tmp/Configuration.txt"
#define SERVER_RESPONSE_LOG_FILE		"/tmp/Server_Response_Timing.txt"
#define SERVER_RESPONSE_FILE			"/tmp/Server_Response_File.txt"
#define DB_PATH					"/tmp/Student_tag.db"
#elif RPI
#define INTERNET_SPEED_FILE_PATH		"/home/pi/Internet_Speed.txt"
#define SCANNED_BEACONS_LOG_FILE		"/home/pi/Scanned_Beacons.txt"
#define HTTP_Post_Response			"/home/pi/http_post_response.txt"
#define Diagnostic_Post_Response		"/home/pi/diagnostic_post_response.txt"
#define Configuration_XML_Path			"/home/pi/Configuration.xml"
#define Configuration_Text_Path			"/home/pi/Configuration.txt"
#define SERVER_RESPONSE_LOG_FILE		"/home/pi/Server_Response_Timing.txt"
#define SERVER_RESPONSE_FILE			"/home/pi/Server_Response_File.txt"
#define DB_PATH                                 "/home/pi/Student_tag.db"
#endif

#define CUSTOMER_RELEASE_VERSION		"1.1"
#define QA_TESTING_RELEASE_VERSION		"1.4.3"

//#define INBUILT_READER
//#define RAW_DATA_ENABLE
//#define PRINTF_LOG_ENABLE
//#define Enable_Queue_Logs
#define Beacon_Queue_Key 1992
#define Beacon_Data_Length 12
#define RSSI_CHECK 1
int Maximum_Beacons_to_Post = 90;
int Posting_Time_Interval = 10;  // in sec
int RSSI = 99; // Default 99
int diagnostic_flag = 1;
char wifi_ssid[25];
char wifi_password[25];
char wifi_ssid_new[25];
char wifi_password_new[25];
char hostname[100];
char http_post[100];
char request_data[20] = "asdasda123123213";
// Default Value
#define http_packet_count 10
#define diagnostics_timing 3600 // in sec
#define BAUDRATE B115200 
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

int Device_conf_status = 0;
int COM_OK_COUNT = 0;
const char* message;

int openport(char *dev_name);
int fd=0;

//struct termios oldtp, newtp;
typedef struct {
	int iModemFd;
	struct termios options;
	speed_t baudrate ;
	pthread_mutex_t UARTLock;
} UARTDriverInfo_t;

UARTDriverInfo_t UARTDriverInfo;

char *header = "#W";
char footer = '%';
char ble_status = '1';
char mac[13];
uint8_t prefix = 0x10;
uint16_t suffix = 0x0000;
int Beacons_Filter_Count = 0;
int Number_of_Beacons = 0;
int Rough_Beacons_Count = 0;
int Data_Posting_Flag = 0;
char set_of_beacons[5000];
char Final_Packet_To_Post[5000];

static int filling_count = 0;
static int fetching_count = 0;

int callback(void *, int, char **, char **);

struct Beacons_Filter
{
	char beacon[7];
};

struct Beacons_Filter Filter[100];

struct Beacon_Datas
{
	short int status;
	char data[1500];
};

struct Beacon_Datas packets[http_packet_count+1];

struct hci_request ble_hci_request(uint16_t ocf, int clen, void * status, void * cparam)
{
	struct hci_request rq;
	memset(&rq, 0, sizeof(rq));
	rq.ogf = OGF_LE_CTL;
	rq.ocf = ocf;
	rq.cparam = cparam;
	rq.clen = clen;
	rq.rparam = status;
	rq.rlen = 1;
	return rq;
}

int set_interface_attribs(int fd, int speed)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error from tcgetattr: %s\n", strerror(errno));
		return -1;
	}

	cfsetospeed(&tty, (speed_t)speed);
	cfsetispeed(&tty, (speed_t)speed);

	tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;         /* 8-bit characters */
	tty.c_cflag &= ~PARENB;     /* no parity bit */
	tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tty.c_oflag &= ~OPOST;

	/* fetch bytes as they become available */
	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 1;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		printf("Error from tcsetattr: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

void set_mincount(int fd, int mcount)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		printf("Error tcgetattr: %s\n", strerror(errno));
		return;
	}

	tty.c_cc[VMIN] = mcount ? 1 : 0;
	tty.c_cc[VTIME] = 5;        /* half second timer */

	if (tcsetattr(fd, TCSANOW, &tty) < 0)
		printf("Error tcsetattr: %s\n", strerror(errno));
}

int store_it(char *packet_data)
{
	if(filling_count >= http_packet_count)
	{
		filling_count = 0;
	}

	if(packets[filling_count].status == 0 || packets[filling_count].status == 1)
	{
		strcpy(packets[filling_count].data, packet_data);
		//		printf("filling_count = %d\n", filling_count);
		packets[filling_count].status = 1;
		filling_count++;
	}
	else
	{
		printf("filling_count = %d\n",filling_count);
	}
}

int packet_format(){
	int x;
	char gpio_access[100];
	char Student_Count[1];
	char *timestamp = (char *)malloc(sizeof(char) * 16);
	time_t ltime;
	ltime=time(NULL);
	struct tm *tm;
	tm=localtime(&ltime);
	sprintf(timestamp, "%02d%02d%02d%02d%02d%02d",tm->tm_mday, tm->tm_mon + 1, tm->tm_year + 1900 - 2000, tm->tm_hour, tm->tm_min, tm->tm_sec);

#ifdef WITHOUT_FILTER
	Rough_Beacons_Count = Number_of_Beacons;
#else	
	Rough_Beacons_Count = Beacons_Filter_Count;
#endif	

	//	printf("Rough_Beacons_Count = %d\n",Rough_Beacons_Count);

	if(Rough_Beacons_Count < 10)
		sprintf(Student_Count, "%c%d",'0', Rough_Beacons_Count);
	else
		sprintf(Student_Count, "%d", Rough_Beacons_Count);	

	sprintf(Final_Packet_To_Post,"%s%s%s%02X%04X%s%s%c",header, mac, timestamp, prefix, suffix, Student_Count, set_of_beacons, footer);

	//	printf("Final_Packet_To_Post = %s\n",Final_Packet_To_Post);
	store_it(Final_Packet_To_Post);

	Number_of_Beacons = 0;
	Rough_Beacons_Count = 0;
	Beacons_Filter_Count = 0;
	memset(set_of_beacons, 0, 2500);
	return 1;
}

int Dummy_packet_format(){
	char Student_Count[1], curl_buf[1000];
	char *timestamp = (char *)malloc(sizeof(char) * 16);
	time_t ltime;
	ltime=time(NULL);
	struct tm *tm;
	tm=localtime(&ltime);
	sprintf(timestamp, "%02d%02d%02d%02d%02d%02d",tm->tm_mday, tm->tm_mon + 1, tm->tm_year + 1900 - 2000, tm->tm_hour, tm->tm_min, tm->tm_sec);

#ifdef WITHOUT_FILTER
	Rough_Beacons_Count = Number_of_Beacons;
#else	
	Rough_Beacons_Count = Beacons_Filter_Count;
#endif	

	//	printf("Rough_Beacons_Count = %d\n",Rough_Beacons_Count);

	if(Rough_Beacons_Count < 10)
		sprintf(Student_Count, "%c%d",'0', Rough_Beacons_Count);
	else
		sprintf(Student_Count, "%d", Rough_Beacons_Count);	

	sprintf(Final_Packet_To_Post,"%s%s%s%02X%s%s%s%c",header, mac, timestamp, prefix, "__V", CUSTOMER_RELEASE_VERSION, "__", footer);

	sprintf(curl_buf, "%s%s%s%s%s%s%s%s", "curl -m 10 -i -X POST -H 'Content-type: application/x-www-form-urlencoded' -d 'receiver_id=", Final_Packet_To_Post,"' ","http://", hostname, http_post, "?format=xml >", HTTP_Post_Response);
	//			printf("Data - %s\n",curl_buf);
	//			printf("fetching_count = %d\n",fetching_count);
	//			printf("First_Packet - %s\n",curl_buf);
	system(curl_buf);

	Number_of_Beacons = 0;
	Rough_Beacons_Count = 0;
	Beacons_Filter_Count = 0;
	memset(set_of_beacons, 0, 2500);
	return 1;
}

void Beacons(char *buffer)
{

	int i = 0, match = 0;
	char rough_beacon_id[6];
	sprintf(rough_beacon_id,"%c%c%c%c%c%c", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
	//printf("rough_Beacon = %s\n",rough_beacon_id);
#ifdef WITH_FILTER
	if(Number_of_Beacons <= 1)
	{
		//		printf("Clearing\n");
		for(i = 0; i < Beacons_Filter_Count+1; i++)
			strcpy(Filter[i].beacon,"");
		Beacons_Filter_Count = 0;
	}

	for(i = 0; i <= Beacons_Filter_Count; i++)
	{
		//		printf("Structure[%d] - %s\n", i, Filter[i].beacon);
		if(strcmp(Filter[i].beacon,rough_beacon_id) != 0)
		{
			//			printf("Not_Match - %d\n",i);
			match = 0;
		}
		else
		{
			//			printf("Match - %d\n",i);
			match = 1;
			break;
		}
	}
	//	printf("match = %d\n",match);
	if(match == 0)
	{
		strcpy(Filter[Beacons_Filter_Count].beacon,rough_beacon_id);
		printf("Beacons_Filter_Count = %d -------> %s\n",Beacons_Filter_Count, Filter[Beacons_Filter_Count].beacon);
		Beacons_Filter_Count++;
#endif
		if(Number_of_Beacons <= 1)
		{
			Data_Posting_Flag = 0;
			memset(set_of_beacons, 0, 2000);
			sprintf(set_of_beacons, ":%s", buffer);
		}
		else
		{
			strcat(set_of_beacons, ":");
			strcat(set_of_beacons, buffer);
		}
		//		printf("From_Queue - %s                   %d\n", buffer, Number_of_Beacons);
#ifdef WITHOUT_FILTER
		if(Number_of_Beacons >= Maximum_Beacons_to_Post)
#else
			if(Beacons_Filter_Count >= Maximum_Beacons_to_Post)
#endif			
			{
				if(packet_format() == 1)
					printf("Posting Success....!\n");
				else
					printf("Posting Failure...!\n");

				Data_Posting_Flag = 1;
			}
#ifdef WITH_FILTER
	}
#endif	
}

int openport(char *dev_name)
{
	UARTDriverInfo.iModemFd = open(dev_name, O_RDWR | O_NOCTTY | O_NDELAY);
	if (UARTDriverInfo.iModemFd < 0) {
		printf("%s\n", "Open Error");
		return 0;
	}
	fcntl(UARTDriverInfo.iModemFd, F_SETFL, O_RDWR);
	tcgetattr(UARTDriverInfo.iModemFd, &UARTDriverInfo.options);
	UARTDriverInfo.options.c_cflag |= (CLOCAL | CREAD);
	UARTDriverInfo.options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	UARTDriverInfo.options.c_oflag &= ~OPOST;
	cfmakeraw(&UARTDriverInfo.options);
	UARTDriverInfo.options.c_cc[VMIN] = 1;
	UARTDriverInfo.options.c_cc[VTIME] = 0;
	UARTDriverInfo.baudrate = B115200;
	cfsetospeed(&UARTDriverInfo.options, UARTDriverInfo.baudrate);
	cfsetispeed(&UARTDriverInfo.options, UARTDriverInfo.baudrate);
	tcsetattr(UARTDriverInfo.iModemFd, TCSAFLUSH, &UARTDriverInfo.options);
	sleep(2);
	tcflush(UARTDriverInfo.iModemFd, TCIOFLUSH);
	return 1;
}

void *uart_reader()
{
	int port_status = openport(MODEMDEVICE);
	if(port_status == 1)
	{
		ble_status = '1';
		char rough[21];
		char rough_1[12], gpio_access[100];
		int i = 0, rssi_value = 0, tag_false_count = 0;
		unsigned char buf[1];
		int rdlen;
		char beacons_log[100];
		char rssi[2];
		do {
			rdlen = read(UARTDriverInfo.iModemFd, buf, 1);
			if (rdlen > 0) {
				//			printf("%c",buf[0]);
				if(buf[0] == ':' && i == 21)
				{
					rough[i] = '\0';
					rough_1[0] = rough[2];
					rough_1[1] = rough[3];
					rough_1[2] = rough[4];
					rough_1[3] = rough[5];
					rough_1[4] = rough[6];
					rough_1[5] = rough[7];
					rough_1[6] = rough[12];
					rough_1[7] = rough[13];
					rough_1[8] = rough[14];
					rough_1[9] = rough[15];
					rough_1[10] = rough[19];
					rssi[0] = rough[19];
					rough_1[11] = rough[20];
					rssi[1] = rough[20];
					rough_1[12] = '\0';
					rssi[2] = '\0';

					if(rough_1[7] == '5' || rough_1[8] == '5')
					{
						sprintf(beacons_log, "a=`%s` && b=%s  && echo $a,$b >> %s", "date +%T.%N",  rough_1, SCANNED_BEACONS_LOG_FILE);
						//					printf("%s\n",rough_1);
#ifdef RSSI_CHECK
						sscanf(rssi, "%d", &rssi_value);
						if(rssi_value <= RSSI)
						{
							printf("rssi = %d/%d   -   %s\n", rssi_value,RSSI,rough_1);
							Number_of_Beacons++;
							Beacons(rough_1);
						}
						else
						{
							printf("rssi = Not in range.......!\n");
						}
#else
						Number_of_Beacons++;
						Beacons(rough_1);
#endif
						sprintf(gpio_access, "led_manager -o %d 2", BLE_LED);
						system(gpio_access);
						system(beacons_log);
						system("date +%T.%N > /tmp/Current_Time");
						FILE *fp_time = fopen("/tmp/Current_Time", "r");
						if(fp_time != NULL)
						{
							char time_stamp[20];
							fread(time_stamp,18,1,fp_time);
							fclose(fp_time);
							check_db(time_stamp, rough_1, rssi, "write");
						}
						memset(rough, 0, 20);
						memset(rough_1, 0, 12);
					}
					i = 0;
				}
				else if(buf[0] == '<' || buf[0] == '>')
				{
					tag_false_count++;
					//				printf("%c - %d\n",buf[0], tag_false_count);
					if(tag_false_count >= 14)
					{
						printf("<<<<COM_OK>>>>\n");
						COM_OK_COUNT++;
						sprintf(gpio_access, "led_manager -o %d 1", BLE_LED);
						system(gpio_access);
						tag_false_count = 0;
					}
				}
				else if(buf[0] == ':' && i != 21)
				{
					i = 0;
				}
				else{
					rough[i++] = buf[0];
				}
			}
			else
			{
				ble_status = '0';
				printf("PORT OPEN ERROR\n");
				sleep(1);
			}
			//		usleep(100);
		} while (1);
	}
	else
	{
		ble_status = '0';
		printf("COM_PORT Open Fail\n");
		while(1)
		{
			sleep(1);
		}
	}
}

void *beacon_data_posting()
{
	int i = 0, already_posted = 0;

	while(1)
	{
		printf("==========     Beacon_Data_Posting_Thread_Status     ==========\n");
		already_posted = 0;
		Data_Posting_Flag = 0;
		for(i = 0; i < Posting_Time_Interval*1000; i++)
		{
			usleep(1000);
			if(Data_Posting_Flag == 1)
			{
				already_posted = 1;
				break;
			}
		}
		if(already_posted == 0)
		{
			if(packet_format() == 1)
				printf("Posting Success....!\n");
			else
				printf("Posting Failure...!\n");

			already_posted = 1;
		}

	}
}

void OTA_Process()
{
	if( access( "/home/pi/BLE_Student_Project/Student_Project.tar.gz", F_OK ) != -1 ) {
		printf("Firmware file already downloaded...\n");
	}
	else
	{
		printf("Downloading Firmware File....\n");
		char ota_buff[500] = "wget \"http://admin.tturk.in/admin/uploads/firmware/current_wifi/Student_Project.tar.gz\"";
		system(ota_buff);
		sleep(2);
		if( access( "/home/pi/BLE_Student_Project/Student_Project.tar.gz", F_OK ) != -1 )
		{
			system("cp /home/pi/BLE_Student_Project/ota_process.sh /home/pi/ && sudo sh /home/pi/ota_process.sh");
			sleep(2);
			printf("Firmware File Downloaded\n");
		}
	}
}

int is_leaf(xmlNode * node)
{
	xmlNode * child = node->children;
	while(child)
	{
		if(child->type == XML_ELEMENT_NODE) return 0;

		child = child->next;
	}

	return 1;
}

void print_xml(xmlNode * node, int indent_len)
{
	while(node)
	{
		if(node->type == XML_ELEMENT_NODE)
		{
			//          printf("%s %s\n", node->name, is_leaf(node)?xmlNodeGetContent(node):xmlGetProp(node, "id"));
			if(!strcmp(node->name,"rssi_max"))
			{
				sscanf(is_leaf(node)?xmlNodeGetContent(node):xmlGetProp(node, "id"), "%d", &RSSI);
				printf("RSSI - %d\n",RSSI);
			}
			else if(!strcmp(node->name,"http_time"))
			{
				sscanf(is_leaf(node)?xmlNodeGetContent(node):xmlGetProp(node, "id"), "%d", &Posting_Time_Interval);
				printf("Posting_Time_Interval - %d\n",Posting_Time_Interval);
			}
			else if(!strcmp(node->name,"ssid"))
			{
				sscanf(is_leaf(node)?xmlNodeGetContent(node):xmlGetProp(node, "id"), "%s", &wifi_ssid_new);
				printf("wifi_ssid - %s\n",wifi_ssid_new);
			}
			else if(!strcmp(node->name,"password"))
			{
				sscanf(is_leaf(node)?xmlNodeGetContent(node):xmlGetProp(node, "id"), "%s", &wifi_password_new);
				printf("wifi_password - %s\n",wifi_password_new);
			}
			else if(!strcmp(node->name,"status"))
			{
				sscanf(is_leaf(node)?xmlNodeGetContent(node):xmlGetProp(node, "id"), "%d", &Device_conf_status);
				printf("Device_conf_status - %d\n",Device_conf_status);
			}
			else if(!strcmp(node->name,"hostname"))
			{
				sscanf(is_leaf(node)?xmlNodeGetContent(node):xmlGetProp(node, "id"), "%s", &hostname);
				printf("hostname - %s\n", hostname);
			}
			else if(!strcmp(node->name,"http_post"))
			{
				sscanf(is_leaf(node)?xmlNodeGetContent(node):xmlGetProp(node, "id"), "%s", &http_post);
				printf("http_post - %s\n", http_post);
			}
			else if(!strcmp(node->name,"no_beacon"))
			{
				sscanf(is_leaf(node)?xmlNodeGetContent(node):xmlGetProp(node, "id"), "%d", &Maximum_Beacons_to_Post);
				printf("Maximum_Beacons_to_Post - %d\n",Maximum_Beacons_to_Post);
			}
			if(!strcmp(wifi_ssid,wifi_ssid_new) || !strcmp(wifi_password,wifi_password_new))
			{
				char wifi_data[1000];
				sprintf(wifi_data, "%s%s%s%s%s", "sudo sh /home/pi/BLE_Student_Project/wifi_setting.sh ",wifi_ssid_new, " \"", wifi_password_new, "\"");
				strcpy(wifi_ssid, wifi_ssid_new);
				strcpy(wifi_password, wifi_password_new);
				system(wifi_data);
			}

		}
		print_xml(node->children, indent_len + 1);
		node = node->next;
	}
}

void Receiver_Configuration()
{
	xmlDoc *doc = NULL;
	xmlNode *root_element = NULL;

	char curl_buff[1000];
	sprintf(curl_buff, "curl http://admin.tturk.in/admin/api/request/assignsetting/receiverid/%s?format=xml >%s", mac, Configuration_XML_Path);
	system(curl_buff);
	doc = xmlReadFile(Configuration_XML_Path, NULL, 0);

	if (doc == NULL) {
		printf("Could not parse the XML file");
	}

	root_element = xmlDocGetRootElement(doc);

	print_xml(root_element, 1);

	xmlFreeDoc(doc);

	xmlCleanupParser();
}

int server_response()
{
	char *cptr;
	char str[2500];
	FILE *fp_http = NULL;
	fp_http = fopen(SERVER_RESPONSE_FILE, "r");
	if(fp_http != NULL)
	{
		fread(str,1000,1,fp_http);
		fclose(fp_http);
		cptr = strstr(str, request_data);
		if(cptr != NULL)
		{
			printf("Response_Received\n");
			return 0;
		}
		else
		{
			printf("Error - Response_Received\n");
			return 1;
		}
	}
	else
	{
		printf("HTTP_Response Fail to open - Not created....!\n");
		return -1;
	}
}

void http_response_check()
{
	char *cptr;
	char str[2500];
	char substr[] = "[\"";

	FILE *fp_http = NULL;
	fp_http = fopen(HTTP_Post_Response, "r");
	if(fp_http != NULL)
	{
		fread(str,1000,1,fp_http);
		fclose(fp_http);
		cptr = strstr(str, substr);
		if(cptr != NULL)
		{
			if(!strcmp(cptr,"[\"00\"]"))
			{
				printf("Ack 0,0\n");
			}
			else if(!strcmp(cptr,"[\"01\"]"))
			{
				printf("Configuration Changed\n");
				Receiver_Configuration();
				diagnostic_flag = 1;
			}
			else if(!strcmp(cptr,"[\"10\"]"))
			{
				printf("OTA Request Received\n");
				OTA_Process();
			}
			else
			{
				printf("Not_Matched\n");
			}
		}
		else
		{
			printf("String = NULL\n");
		}
	}
	else
	{
		printf("HTTP_Response Fail to open - Not created....!\n");
	}
}

void *diagnostics()
{
	char diagnostics_buf[50], curl_diagnostic_buf[2000];
	char bat_voltage[2] = {'2','5'};
	char wifi_rssi[3];
	char pwr_supply_mode = '0', gps_status = '0', charging_status = '0';
	int diagnostic_timer_count = 0;
	FILE *fp_wifi_rssi;
	while(1)
	{
		if(diagnostic_flag == 1)
		{
			system("iwconfig wlan0 | grep -i --color quality | awk {'print $4'} | cut -c 7,8,9 > /tmp/wifi_strength.txt");
			fp_wifi_rssi = fopen("/tmp/wifi_strength.txt", "r");
			if(fp_wifi_rssi != NULL)
			{
				fread(wifi_rssi, 3, 1, fp_wifi_rssi);
				fclose(fp_wifi_rssi);
				if(wifi_rssi[0] == '-')
					wifi_rssi[0] = '0';
				//			printf("wifi_rssi - %s\n",wifi_rssi);
			}
			sprintf(diagnostics_buf, "%s%s%s%c%c%c%c%d%s%s%c", header, mac, bat_voltage, pwr_supply_mode, gps_status, charging_status, ble_status, Device_conf_status, CUSTOMER_RELEASE_VERSION, wifi_rssi, footer);
			//		printf("\n%s\n", diagnostics_buf);
			sprintf(curl_diagnostic_buf, "%s%s%s%s%s", "curl -m 10 -i -X POST -H 'Content-type: application/x-www-form-urlencoded' -d 'details=", diagnostics_buf,"' ","http://admin.tturk.in/admin/api/diagnostic/data?format=xml >", Diagnostic_Post_Response);
			//		printf("%s\n", curl_diagnostic_buf);
			system(curl_diagnostic_buf);
			diagnostic_flag = 0;
		}
		else
		{
			sleep(1);
			diagnostic_timer_count++;
			if(diagnostic_timer_count >= diagnostics_timing)
			{
				diagnostic_flag = 1;
			}
		}
	}
}

void *server_response_timing()
{
	char server_response_check[500], request_sent_time[50];
	int response_status;

	while(1)
	{
		sprintf(server_response_check,"curl --header \"Content-Type: plain/text\" --request PUT --data %s https://uq4kp3eza5.execute-api.ap-southeast-1.amazonaws.com/production > %s", request_data, SERVER_RESPONSE_FILE);
		system(server_response_check);
		printf("<<<<<<<<<<     Server Response Timing Check     >>>>>>>>>>\n");
		sprintf(request_sent_time, "a=`%s` && b=%s  && echo $a,$b >> %s", "date +%T.%N", "Server_Request_Sent", SERVER_RESPONSE_LOG_FILE);
		system(request_sent_time);
		response_status = server_response(request_data);
		if(response_status == 0)
		{
			sprintf(request_sent_time, "a=`%s` && b=%s  && echo $a,$b >> %s", "date +%T.%N", "Server_Response_Received", SERVER_RESPONSE_LOG_FILE);
			system(request_sent_time);
		}
		sleep(3);
	}
}

void *data_posting()
{
	int i = 0;
	char curl_buf[5000], gpio_access[100];
	while(1)
	{
		if(fetching_count >= http_packet_count)
		{
			fetching_count = 0;
		}


		if(packets[fetching_count].status == 1)
		{
			char rm_buf[50];
			sprintf(rm_buf, "rm %s",HTTP_Post_Response);
			packets[fetching_count].status = 0;
			sprintf(curl_buf, "%s%s%s%s%s%s%s%s", "curl -m 10 -i -X POST -H 'Content-type: application/x-www-form-urlencoded' -d 'receiver_id=", packets[fetching_count].data,"' ","http://", hostname, http_post,"?format=xml >", HTTP_Post_Response);
			//			printf("fetching_count = %d\n",fetching_count);
			if(system("ping -c1 -w1 8.8.8.8 > /dev/null 2>&1") == 0)
			{
				sprintf(gpio_access, "led_manager -o %d 2", POSTING_LED);
				system(gpio_access);

				system(curl_buf);
				http_response_check();
			}
			else
			{
				sprintf(gpio_access, "led_manager -o %d 1", POSTING_LED);
				system(gpio_access);
			}
			fetching_count++;
		}
		usleep(500);
	}
}

int wifi_mac_assigning()
{
	if( access( "/sys/class/net/wlan0/carrier", F_OK ) != -1 ) {
		printf("WIFI Hardware detected.. Yes\n");
		system("cat /sys/class/net/wlan0/address > /home/pi/MAC_ID");
		int i = 0, j = 0;
		char mac_rough[20];
		FILE *fp_mac_id = fopen("/home/pi/MAC_ID", "r");
		if(fp_mac_id != NULL)
		{
			fread(mac_rough,17,1,fp_mac_id);
			fclose(fp_mac_id);

			for(i = 0; i <= 16; i++)
			{
				if(mac_rough[i] != ':')
//					mac[j++] = mac_rough[i];
					mac[j++] = 'A';
			}
			printf("%s\n",mac);
		}
		else
		{
			printf("MAC_ID file not found\n");
		}
		return 0;
	}
	else
	{
		printf("WIFI Hardware detected.. NO\n");
		sleep(30);
		system("sudo reboot");
		return 1;
	}
}

int main()
{
	printf("Welcome to Student tracker.....!\n");
	printf("QA_TESTING_RELEASE_VERSION - %s\n",QA_TESTING_RELEASE_VERSION);
	printf("CUSTOMER_RELEASE_VERSION - %s\n",CUSTOMER_RELEASE_VERSION);

	sleep(3);

	wifi_mac_assigning();

	if( access( "/sys/class/gpio/gpio5/value", F_OK ) != -1 ) {
		printf("GPIO Already exported\n");
	}
	else
	{
		system("led_manager -d 5 out");
		system("led_manager -d 6 out");
		system("led_manager -d 13 out");
		system("led_manager -d 19 in");
	}

	if(check_db("TimeStamp", "BEACON", "RSSI", "init") == 0)
	{
		Receiver_Configuration();
		sleep(2);
		Dummy_packet_format();

		sys_mq_reset(Beacon_Queue_Key);

		pthread_t beacon_data_posting_thread;
		pthread_t data_posting_thread;
		pthread_t uart_reader_thread;
		pthread_t diagnostics_thread;
		pthread_t inbuilt_reader_thread;
		pthread_t server_response_timing_thread;

		int i = 0;
		for(i = 0; i < http_packet_count; i++)
			packets[i].status = 0;

#ifdef UART_READER 
		if(pthread_create(&uart_reader_thread, NULL, uart_reader, NULL)) {
			printf("Error creating thread\n");
			return 1;
		}
#endif

		if(pthread_create(&diagnostics_thread, NULL, diagnostics, NULL)) {
			printf("Error creating diagnostics_thread\n");
			return 1;
		}

		if(pthread_create(&beacon_data_posting_thread, NULL, beacon_data_posting, NULL)) {
			printf("Error creating beacon_data_posting_thread\n");
			return 1;
		}

		if(pthread_create(&data_posting_thread, NULL, data_posting, NULL)) {
			printf("Error creating data_posting_thread\n");
			return 1;
		}
/*
		if(pthread_create(&server_response_timing_thread, NULL, server_response_timing, NULL)) {
			printf("Error creating server_response_timing_thread\n");
			return 1;
		}
*/
		if(pthread_join(uart_reader_thread, NULL)) {
			fprintf(stderr, "Error joining uart_reader_thread\n");
			return 2;
		}
	}
	return 0;
}


int check_db(char *timestamp, char *Data_to_Post, char *RSSI, char *state)
{
	sqlite3 *db;
	char *err_msg = 0;
	sqlite3_stmt *res;

	int rc = sqlite3_open("/tmp/ID.db", &db);

	if (rc != SQLITE_OK) {

		fprintf(stderr, "Cannot open database: %s\n", sqlite3_errmsg(db));
		sqlite3_close(db);

		return 1;
	}

	char sql[1000];

	if(state == "init")
	{
		sprintf(sql, "DROP TABLE IF EXISTS HTTP_Packets; CREATE TABLE HTTP_Packets(%s Text, %s TEXT, %s TEXT);", timestamp, Data_to_Post, RSSI);
	}
	else if(state == "write")
	{
		sprintf(sql, "INSERT INTO HTTP_Packets VALUES('%s', '%s', '%s');", timestamp, Data_to_Post, RSSI);

	}
	rc = sqlite3_exec(db, sql, 0, 0, &err_msg);

	if (rc != SQLITE_OK ) {

		fprintf(stderr, "SQL error: %s\n", err_msg);

		sqlite3_free(err_msg);        
		sqlite3_close(db);

		return 1;
	} 

	sqlite3_close(db);

	return 0;
}

int read_db(int offset)
{
	sqlite3 *db;
	char *err_msg = 0;
	sqlite3_stmt *res;

	int rc = sqlite3_open(DB_PATH, &db);

	if (rc != SQLITE_OK) {
		fprintf(stderr, "Cannot open database: %s\n", sqlite3_errmsg(db));
		sqlite3_close(db);
		return 1;
	}

	char sql[100];

	sprintf(sql, "SELECT Packet FROM HTTP_Packets LIMIT 1 OFFSET %d", offset);
	rc = sqlite3_exec(db, sql, callback, 0, &err_msg);

	if (rc != SQLITE_OK ) {
		fprintf(stderr, "SQL error: %s\n", err_msg);
		sqlite3_free(err_msg);
		sqlite3_close(db);
		return 1;
	}

	sprintf(sql, "DELETE FROM HTTP_Packets LIMIT 1 OFFSET %d",offset);
	rc = sqlite3_exec(db, sql, callback, 0, &err_msg);

	if (rc != SQLITE_OK ) {
		fprintf(stderr, "SQL error: %s\n", err_msg);
		sqlite3_free(err_msg);
		sqlite3_close(db);
		return 1;
	}

	sqlite3_close(db);
	return 0;
}


int callback(void *NotUsed, int argc, char **argv, char **azColName) {
	NotUsed = 0;
	int i =0;
	printf("COUNT = %d\n",argc);
	for(i = 0; i < argc; i++) {
		printf("%s\n", argv[i] ? argv[i] : "NULL");
	}
	return 0;
}

/*
 * speedtest-cli
 *
 * sudo apt-get install python-pip
 *
 * sudo pip install speedtest-cli
 *
 */

// http://admin.tturk.in/admin/uploads/firmware/current_wifi/current_firmware.bin
// http://admin.tturk.in/admin/api/request/assignsetting/receiverid/AAAAAAAAAAAA?format=xml

