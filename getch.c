#include <unistd.h>
#include <sys/select.h>
#include <termios.h>
#include "getch.h"

char _getch(void) {
  char c = 0;
  struct termios old;
  struct termios raw;

  if (tcgetattr(0, &old) != 0)
    return 0;

  raw = old;
  raw.c_lflag &= ~ICANON;
  raw.c_lflag &= ~ECHO;
  raw.c_cc[VMIN]=1;
  raw.c_cc[VTIME]=0;
  if (tcsetattr(0, TCSANOW, &raw) != 0)
    return 0;

  if (read(0, &c, 1) < 0)
    c = 0;

  tcsetattr(0, TCSADRAIN, &old);
  return c;
}

int _kbhit(void) {
  struct timeval tv;
  fd_set readfds;

  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&readfds);
  FD_SET(0, &readfds);

  return select(1, &readfds, NULL, NULL, &tv) > 0;
}
