CC=gcc
CFLAGS=-D RPI 
APPNAME = "BLE_Student_ID_Scanner"
LP=-lpthread -lbluetooth -lxml2 -lsqlite3

all: scan.o
		$(CC) $(CFLAGS) -o $(APPNAME) scan.o -fno-stack-protector $(LP)
clean: 
		rm $(APPNAME) *.o
