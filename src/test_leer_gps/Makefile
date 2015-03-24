CC = gcc
CFLAGS = -Wall

test_leer_gps: main.o
	$(CC) -O3 $(CFLAGS) -o test_leer_gps main.o

main.o: main.c
	$(CC) -O3 $(CFLAGS) -c main.c

clean:
	rm test_leer_gps \
	main.o
