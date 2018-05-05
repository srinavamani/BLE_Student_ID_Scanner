CC=gcc
CFLAGS=-D PC 
APPNAME = "BLE_Student_ID_Scanner"
LP=-lpthread -lbluetooth 

all: scan.o 
		$(CC) $(CFLAGS) $(LDFLAGS)-o $(APPNAME) *.o $(LP) 
clean: 
		rm $(APPNAME) *.o
