#include <stdlib.h>


void quit(void,...)
{
   exit(1);

}



int main(int argc, char *argv[])
{

 // int ret = system("killall sbusd");

  quit();

  return 0;
}
