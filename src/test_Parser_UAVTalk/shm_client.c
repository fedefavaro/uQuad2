/*
 * shm-client - client program to demonstrate shared memory.
 */
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#define SHMSZ     27

//
#include <semaphore.h>

sem_t * sem_id;

int main(void)
{
    int shmid, i;
    key_t key;
    float *shm, *s; //char *shm, *s;

   /**
    * Semaphore open
    */
    sem_id=sem_open("/mysem", O_CREAT, S_IRUSR | S_IWUSR, 1);

    /*
     * We need to get the segment named
     * "5678", created by the server.
     */
    key = 5678;

    /*
     * Locate the segment.
     */
    if ((shmid = shmget(key, SHMSZ, 0666)) < 0) {
        perror("shmget");
        exit(1);
    }

    /*
     * Now we attach the segment to our data space.
     */
    if ((shm = shmat(shmid, NULL, 0)) == (void *) -1) {
        perror("shmat");
        exit(1);
    }
    s = shm;

    for (i = 0; i < 3; i++)
    {
       printf("Waiting \n");
       sem_wait(sem_id);
       printf("%lf\n", *s);
       s++;
       sem_post(sem_id);
	sleep(1);
    }


    /**
     * Semaphore Close: Close a named semaphore
     */
    if ( sem_close(sem_id) < 0 )
    {
    	perror("sem_close");
    }

    /**
     * Semaphore unlink: Remove a named semaphore  from the system.
     */
    if ( sem_unlink("/mysem") < 0 )
    {
    	perror("sem_unlink");
    }


    exit(0);
}
