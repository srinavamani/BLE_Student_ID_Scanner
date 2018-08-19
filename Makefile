CC=gcc
CFLAGS=-D RPI 
APPNAME = "BLE_Student_ID_Scanner"
LP=-lpthread -lbluetooth -lxml2 -lsqlite3

all: scan.o server_response_test.o
		$(CC) $(CFLAGS) -o $(APPNAME) scan.o -fno-stack-protector $(LP_1)
clean: 
		rm $(APPNAME) *.o
