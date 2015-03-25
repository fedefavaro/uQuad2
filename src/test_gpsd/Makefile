CC = gcc
CFLAGS = -Wall
LDFLAGS = -lgps

test_leer_gps: main.o
	$(CC) -O3 $(CFLAGS) $(LDFLAGS) -o test_gpsd main.o

main.o: main.c
	$(CC) -O3 $(CFLAGS) $(LDFLAGS) -c main.c

clean:
	rm test_gpsd \
	main.o
