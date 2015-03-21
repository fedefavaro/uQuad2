CC = gcc
CFLAGS = -Wall
OBJECTS = SerialPort.o FutabaSBUS.o CustomBaud.o

sbus_comm: main.o $(OBJECTS)
	$(CC) -O3 $(CFLAGS) -o sbus_comm main.o $(OBJECTS)

main.o: main.c SerialPort.h FutabaSBUS.h
	$(CC) -O3 $(CFLAGS) -c main.c

SerialPort.o: SerialPort.c
	$(CC) -O3 $(CFLAGS) -c SerialPort.c

FutabaSBUS.o: FutabaSBUS.c
	$(CC) -O3 $(CFLAGS) -c FutabaSBUS.c

CustomBaud.o: CustomBaud.c
	$(CC) -O3 $(CFLAGS) -c CustomBaud.c

clean:
	rm sbus_comm \
	main.o $(OBJECTS)
