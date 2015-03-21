CC = gcc
CFLAGS = -Wall

sbus_comm: main.o
	$(CC) -O3 $(CFLAGS) -o test_msgq_client main.o

main.o: main.c
	$(CC) -O3 $(CFLAGS) -c main.c

clean:
	rm test_msgq_client \
	   main.o
