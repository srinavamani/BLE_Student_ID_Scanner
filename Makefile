CC=gcc
CFLAGS=-D RPI 
APPNAME = "BLE_Student_ID_Scanner"
LP_1=-lpthread -lbluetooth -lxml2 -lsqlite3
LP_2=-lpthread

all: scan.o server_response_test.o
		$(CC) $(CFLAGS) -o $(APPNAME) scan.o -fno-stack-protector $(LP_1)
		$(CC) $(CFLAGS) -o server_response_test server_response_test.o -fno-stack-protector $(LP_2)
clean: 
		rm $(APPNAME) server_response_time *.o
