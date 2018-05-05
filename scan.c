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

#include <errno.h>
#include <fcntl.h> 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

//#include <sqlite3.h>

#define PACKET_OLD  // PACKET_NEW (with '\n') and PACKET_OLD (without '\n')

#define WITH_FILTER
#define UART_READER
#define PORT_NAME	"/dev/ttyUSB0"
#if PC
#define INTERNET_SPEED_FILE_PATH "/tmp/Internet_Speed.txt"
#define SCANNED_BEACONS_LOF_FILE "/tmp/Scanned_Beacons.txt"
#elif RPI
#define INTERNET_SPEED_FILE_PATH "/home/pi/Internet_Speed.txt"
#define SCANNED_BEACONS_LOF_FILE "/home/pi/Scanned_Beacons.txt"
#endif
//#define INBUILT_READER
//#define RAW_DATA_ENABLE
//#define PRINTF_LOG_ENABLE
//#define Enable_Queue_Logs
#define Beacon_Queue_Key 1992
#define Beacon_Data_Length 12
#define Posting_Time_Interval 5  // in sec
#define RSSI 50 // Default 99

// Default Value
#define Maximum_Beacons_to_Post 99
#define http_packet_count 10


char *header = "#W";
char footer = '%';
//char *mac = "b827eb2f83db";
char *mac = "AAAAAAAAAAAA";
uint8_t prefix = 0x10;
uint16_t suffix = 0x0000;
int Beacons_Filter_Count = 0;
int Number_of_Beacons = 0;
int Rough_Beacons_Count = 0;
int Data_Posting_Flag = 0;
char set_of_beacons[2500];
char Final_Packet_To_Post[2500];

static int filling_count = 0;
static int fetching_count = 0;

int callback(void *, int, char **, char **);

struct Beacons_Filter
{
	char beacon[7];
};

struct Beacons_Filter Filter[Maximum_Beacons_to_Post];

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

void Beacons(char *buffer)
{

	int i = 0, match = 0;
	char rough_beacon_id[6];
	sprintf(rough_beacon_id,"%c%c%c%c%c%c", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
	//printf("rough_Beacon = %s\n",rough_beacon_id);
#ifdef WITH_FILTER
	if(Number_of_Beacons <= 1)
	{
		printf("Clearing\n");
		for(i = 0; i < Beacons_Filter_Count; i++)
			memset(Filter[i].beacon, 0, 7);
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

	if(match != 1)
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
		//	printf("From_Queue - %s                   %d\n", buffer, Number_of_Beacons);
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

void *beacon_data_handling()
{
	sleep(2);
	int i = 0, Beacon_Queue_Id = sys_mq_init(Beacon_Queue_Key);

	uint16_t size;
	uint8_t buffer[12];
	char beacons[13];
	while(1)
	{

		sys_mq_recv(Beacon_Queue_Id, buffer, &size);
		Number_of_Beacons++;
		//printf("Size = %d\n",size);
#ifdef Enable_Queue_Logs
		for(i = 0; i < size; i++)
			printf("%02X,",buffer[i]);
		printf("\nNumber_of_Beacons - %d\n",Number_of_Beacons);
#endif

		buffer[12] = '\0';
		//sprintf(beacons,"%s",buffer);
		//printf("buffer = %s\n",buffer);
		Beacons(buffer);

	}
}

int packet_format(){
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

	printf("Rough_Beacons_Count = %d\n",Rough_Beacons_Count);

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

void *inbuilt_reader()
{
	int ret, status;
	int Beacon_Queue_Id = sys_mq_init(Beacon_Queue_Key);

	// Get HCI device.

	const int device = hci_open_dev(hci_get_route(NULL));
	if ( device < 0 ) { 
		perror("Failed to open HCI device.");
		return 0; 
	}

	// Set BLE scan parameters.

	le_set_scan_parameters_cp scan_params_cp;
	memset(&scan_params_cp, 0, sizeof(scan_params_cp));
	scan_params_cp.type 			= 0x00; 
	scan_params_cp.interval 		= htobs(0x0010);
	scan_params_cp.window 			= htobs(0x0010);
	scan_params_cp.own_bdaddr_type 	= 0x00; // Public Device Address (default).
	scan_params_cp.filter 			= 0x00; // Accept all.

	struct hci_request scan_params_rq = ble_hci_request(OCF_LE_SET_SCAN_PARAMETERS, LE_SET_SCAN_PARAMETERS_CP_SIZE, &status, &scan_params_cp);

	ret = hci_send_req(device, &scan_params_rq, 1000);
	if ( ret < 0 ) {
		hci_close_dev(device);
		perror("Failed to set scan parameters data.");
		return 0;
	}

	// Set BLE events report mask.

	le_set_event_mask_cp event_mask_cp;
	memset(&event_mask_cp, 0, sizeof(le_set_event_mask_cp));
	int i = 0;
	for ( i = 0 ; i < 8 ; i++ ) event_mask_cp.mask[i] = 0xFF;

	struct hci_request set_mask_rq = ble_hci_request(OCF_LE_SET_EVENT_MASK, LE_SET_EVENT_MASK_CP_SIZE, &status, &event_mask_cp);
	ret = hci_send_req(device, &set_mask_rq, 1000);
	if ( ret < 0 ) {
		hci_close_dev(device);
		perror("Failed to set event mask.");
		return 0;
	}

	// Enable scanning.

	le_set_scan_enable_cp scan_cp;
	memset(&scan_cp, 0, sizeof(scan_cp));
	scan_cp.enable 		= 0x01;	// Enable flag.
	scan_cp.filter_dup 	= 0x00; // Filtering disabled.

	struct hci_request enable_adv_rq = ble_hci_request(OCF_LE_SET_SCAN_ENABLE, LE_SET_SCAN_ENABLE_CP_SIZE, &status, &scan_cp);

	ret = hci_send_req(device, &enable_adv_rq, 1000);
	if ( ret < 0 ) {
		hci_close_dev(device);
		perror("Failed to enable scan.");
		return 0;
	}

	// Get Results.

	struct hci_filter nf;
	hci_filter_clear(&nf);
	hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
	hci_filter_set_event(EVT_LE_META_EVENT, &nf);
	if ( setsockopt(device, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) < 0 ) {
		hci_close_dev(device);
		perror("Could not set socket options\n");
		return 0;
	}

	uint8_t buf[45];
	evt_le_meta_event * meta_event;
	le_advertising_info * info;
	int len,ii=0;
	while ( 1 ) {
		len = read(device, buf, sizeof(buf));

#ifdef RAW_DATA_ENABLE
		printf("\n\n");
		for(ii=0;ii<44;ii++)
			printf("%02X",buf[ii]);
		printf("\n\n");
#endif

		if ( len >= HCI_EVENT_HDR_SIZE ) {
			meta_event = (evt_le_meta_event*)(buf+HCI_EVENT_HDR_SIZE+1);
			if ( meta_event->subevent == EVT_LE_ADVERTISING_REPORT ) {
				uint8_t reports_count = meta_event->data[0];
				void * offset = meta_event->data + 1;
				char addr[18];
				info = (le_advertising_info *)offset;
				offset = info->data + info->length + 2;
				ba2str(&(info->bdaddr), addr);

				if(buf[7] == 0x00 && buf[8] == 0x00 && buf[12] == 0x10)
				{
					char Beacon_Data[Beacon_Data_Length+1];
					char RSSI_Rough[2];
					char Battery_Rough[3];
					int rssi = 256 - (info->data[info->length]);
					//					printf("rssi - %d\n",rssi);
					if(rssi > 99)
						rssi = 99;
					else if(rssi < 0)
						rssi = 99;
					//					if(addr[10] == '5')
					//						addr[10] = '8';
					//					if(addr[10] == '6')
					//						addr[10] = '9';

					sprintf(Beacon_Data,"%c%c%c%c%c%c%d%d%d",addr[3],addr[4],addr[6],addr[7],addr[9],addr[10],(int)(buf[43]),0,rssi);

#ifdef PRINTF_LOG_ENABLE
					//					printf("MAC - %s\n",addr);					
					printf("To_Queue - %s\n",Beacon_Data);
#endif

					sys_mq_send(Beacon_Queue_Id, Beacon_Data, Beacon_Data_Length);
				}
				memset(buf,0,45);
			}
		}
	}

	// Disable scanning.

	memset(&scan_cp, 0, sizeof(scan_cp));
	scan_cp.enable = 0x00;	// Disable flag.

	struct hci_request disable_adv_rq = ble_hci_request(OCF_LE_SET_SCAN_ENABLE, LE_SET_SCAN_ENABLE_CP_SIZE, &status, &scan_cp);
	ret = hci_send_req(device, &disable_adv_rq, 1000);
	if ( ret < 0 ) {
		hci_close_dev(device);
		perror("Failed to disable scan.");
		return 0;
	}

	hci_close_dev(device);
}

void *uart_reader()
{
	char *portname = PORT_NAME;
	int fd;
	int wlen;

	int ret, status;
	int Beacon_Queue_Id = sys_mq_init(Beacon_Queue_Key);


	fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0) {
		printf("Error opening %s: %s\n", portname, strerror(errno));
		exit(0);
	}
	/*baudrate 115200, 8 bits, no parity, 1 stop bit */
	set_interface_attribs(fd, B115200);
	//set_mincount(fd, 0);                /* set to pure timed read */

	/* simple output */
	wlen = write(fd, "Hello!\n", 7);
	if (wlen != 7) {
		printf("Error from write: %d, %d\n", wlen, errno);
	}
	tcdrain(fd);    /* delay for output */


	/* simple noncanonical input */
	char rough[21];
	char rough_1[12];
	int i = 0, rssi_value = 0;
	unsigned char buf[1];
	int rdlen;
	char beacons_log[100];
	char rssi[2];
	do {
		rdlen = read(fd, buf, 1);
		if (rdlen > 0) {
			//			printf("%c",buf[0]);
#ifdef PACKET_NEW
			if(buf[0] == '\n' && i == 21)
			{
				rough[i] = '\0';
				rough_1[0] = rough[3];
				rough_1[1] = rough[4];
				rough_1[2] = rough[5];
				rough_1[3] = rough[6];
				rough_1[4] = rough[7];
				rough_1[5] = rough[8];
				rough_1[6] = rough[13];
				rough_1[7] = rough[14];
				rough_1[8] = rough[15];
				rough_1[9] = rough[16];
				rough_1[10] = rough[20];
				rssi[0] = rough[20];
				rough_1[11] = rough[21];
				rssi[1] = rough[21];
				rough_1[12] = '\0';
				rssi[2] = '\0';

				if(rough_1[7] == '5' || rough_1[8] == '5')
				{
					//					sys_mq_send(Beacon_Queue_Id, rough_1, 12);
					sprintf(beacons_log, "%s >> %s && echo %s >> %s", "date +%T.%N", SCANNED_BEACONS_LOF_FILE, rough_1, SCANNED_BEACONS_LOF_FILE);
					//					printf("%s\n",rough_1);
					sscanf(rssi, "%d", &rssi_value);
					if(rssi_value <= RSSI)
					{
						//						printf("rssi = %d\n", rssi_value);
						Number_of_Beacons++;
						Beacons(rough_1);
					}
					system(beacons_log);
					memset(rough, 0, 20);
					memset(rough_1, 0, 12);
				}
				i = 0;
			}
			else if(buf[0] == '\n' && i != 21)
			{
				i = 0;
			}
			else
			{
				rough[i++] = buf[0];
			}
#else
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
					//					sys_mq_send(Beacon_Queue_Id, rough_1, 12);
					sprintf(beacons_log, "%s >> %s && echo %s >> %s", "date +%T.%N", SCANNED_BEACONS_LOF_FILE, rough_1, SCANNED_BEACONS_LOF_FILE);
					//					printf("%s\n",rough_1);
					sscanf(rssi, "%d", &rssi_value);
					if(rssi_value <= RSSI)
					{
						//						printf("rssi = %d\n", rssi_value);
						Number_of_Beacons++;
						Beacons(rough_1);
					}
					system(beacons_log);
					memset(rough, 0, 20);
					memset(rough_1, 0, 12);
				}
				i = 0;
			}
			else if(buf[0] == '\n' && i != 21)
			{
				i = 0;
			}
			else
			{
				rough[i++] = buf[0];
			}
#endif
		}
		//		usleep(100);
	} while (1);
}

void *beacon_data_posting()
{
	int i = 0, already_posted = 0;

	while(1)
	{
		printf("===========================================================       Beacon_data_posting_RUNNING\n");
		already_posted = 0;
		Data_Posting_Flag = 0;
		for(i = 0; i < Posting_Time_Interval*100; i++)
		{
			usleep(10000);
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

void *data_posting()
{
	int i = 0;
	char curl_buf[5000];
	while(1)
	{
		//		printf("===========================================================       DATA_POSTING_RUNNING\n");
		if(fetching_count >= http_packet_count)
		{
			fetching_count = 0;
		}
		if(packets[fetching_count].status == 1)
		{
			//			printf("%s\n", packets[fetching_count].data);
			packets[fetching_count].status = 0;
			sprintf(curl_buf, "%s%s%s%s", "curl -i -X POST -H 'Content-type: application/x-www-form-urlencoded' -d 'receiver_id=", packets[fetching_count].data,"' ","http://comtustec.tturk.in/tturk/api/receiver/wifidatapost?format=xml >> /tmp/http_post_response" );
			//				printf("Date - %s\n",curl_buf);
			system(curl_buf);
			fetching_count++;
			sleep(1);
		}
		else{
			printf("fetching_count = %d\n",fetching_count);
		}
		usleep(500000);
	}
}

int main()
{
	sys_mq_reset(Beacon_Queue_Key);

	//if(check_db("TimeStamp", "Packet", "init") == 0)
	//{

	pthread_t beacon_data_handling_thread;
	pthread_t beacon_data_posting_thread;
	pthread_t data_posting_thread;
	pthread_t uart_reader_thread;
	pthread_t inbuilt_reader_thread;
	pthread_t internet_speed_thread;

	int i = 0;
	for(i = 0; i < http_packet_count; i++)
		packets[i].status = 0;
	/*
	   if(pthread_create(&beacon_data_handling_thread, NULL, beacon_data_handling, NULL)) {
	   printf("Error creating thread\n");
	   return 1;
	   }
	   */
#ifdef UART_READER 
	if(pthread_create(&uart_reader_thread, NULL, uart_reader, NULL)) {
		printf("Error creating thread\n");
		return 1;
	}
#endif

#ifdef INBUILT_READER
	if(pthread_create(&inbuilt_reader_thread, NULL, inbuilt_reader, NULL)) {
		printf("Error creating thread\n");
		return 1;
	}
#endif

	if(pthread_create(&beacon_data_posting_thread, NULL, beacon_data_posting, NULL)) {
		printf("Error creating thread\n");
		return 1;
	}

	if(pthread_create(&data_posting_thread, NULL, data_posting, NULL)) {
		printf("Error creating thread\n");
		return 1;
	}

	if(pthread_join(uart_reader_thread, NULL)) {
		fprintf(stderr, "Error joining thread\n");
		return 2;
	}
	//}
	return 0;
}


int store_it(char *packet_data)
{

	if(filling_count >= http_packet_count)
	{
		filling_count = 0;
	}

	if(packets[filling_count].status == 0)
	{
		strcpy(packets[filling_count].data, packet_data);
		//printf("%s\n",packets[filling_count].data);
		printf("filling_count = %d\n", filling_count);
		packets[filling_count].status = 1;
		filling_count++;
	}
	else
	{
		printf("status = 1 filling_count = %d\n",filling_count);
	}

}
/*
   int check_db(char *timestamp, char *Data_to_Post, char *state)
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
   sprintf(sql, "DROP TABLE IF EXISTS HTTP_Packets; CREATE TABLE HTTP_Packets(%s Text, %s TEXT);", timestamp, Data_to_Post);
   }
   else if(state == "write")
   {
   sprintf(sql, "INSERT INTO HTTP_Packets VALUES('%s', '%s');", timestamp, Data_to_Post);

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

   int rc = sqlite3_open("/tmp/ID.db", &db);

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
*/


/*
 * speedtest-cli
 *
 * sudo apt-get install python-pip
 *
 * sudo pip install speedtest-cli
 *
 */


