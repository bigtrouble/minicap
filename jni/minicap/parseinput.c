#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/un.h>
#include <unistd.h>
#include <libevdev.h>

static void parse_input(char* buffer, middlecap_ctx* ctx)
{
  char* cursor;
  long int contact, x, y, pressure, wait, dx, dy;

  cursor = (char*) buffer;
  cursor += 1;

  switch (buffer[0])
  {
    case 'c': // COMMIT
      commit(ctx->state);
      break;
    case 'r': // RESET
      touch_panic_reset_all(ctx->state);
      break;
    case 'd': // TOUCH DOWN
      contact = strtol(cursor, &cursor, 10);
      x = strtol(cursor, &cursor, 10);
      y = strtol(cursor, &cursor, 10);
      pressure = strtol(cursor, &cursor, 10);
      touch_down(ctx->state, contact, calcRealX(ctx->state,x,y), calcRealY(ctx->state,x,y), pressure);
      break;
    case 'm': // TOUCH MOVE
      contact = strtol(cursor, &cursor, 10);
      x = strtol(cursor, &cursor, 10);
      y = strtol(cursor, &cursor, 10);
      pressure = strtol(cursor, &cursor, 10);
      touch_move(ctx->state, contact, calcRealX(ctx->state,x,y), calcRealY(ctx->state,x,y), pressure);
      break;
    case 'u': // TOUCH UP
      contact = strtol(cursor, &cursor, 10);
      touch_up(ctx->state, contact);
      break;
    case 'w':
      wait = strtol(cursor, &cursor, 10);
      usleep(wait * 1000);
      break;
    case 'q': // modify jpg quality ( 0 ~ 100 )
      ctx->quality = atoi(cursor);
      break;
    default:
      break;
  }
}

static void *input_parse_handler(void* state)
{
  middlecap_ctx *ctx = (middlecap_ctx *)state;

  //setvbuf( ((internal_state_t *)state)->input, NULL, _IOLBF, 1024);
  setvbuf(stdin, NULL, _IONBF, 0);
  char read_buffer[80];
  while (fgets(read_buffer, sizeof(read_buffer), ctx->state->input) != NULL)
  { 
    read_buffer[strcspn(read_buffer, "\r\n")] = 0;
    parse_input(read_buffer, ctx);
  }
}