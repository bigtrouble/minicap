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

#define MAX_SUPPORTED_CONTACTS 10
#define VERSION 1

static int g_verbose = 0;

typedef struct
{
  int enabled;
  int tracking_id;
  int x;
  int y;
  int pressure;
} contact_t;

typedef struct
{
  int fd;
  int score;
  char path[100];
  struct libevdev* evdev;
  int has_mtslot;
  int has_tracking_id;
  int has_key_btn_touch;
  int has_touch_major;
  int has_width_major;
  int has_pressure;
  int min_pressure;
  int max_pressure;
  int max_x;
  int max_y;
  int max_contacts;
  int max_tracking_id;
  int tracking_id;
  contact_t contacts[MAX_SUPPORTED_CONTACTS];
  int active_contacts;

  //--------------------------------------------
  FILE* input;
  int realWidth;
  int realHeight;
  int virtualWidth;
  int virtualHeight;
  uint32_t orientation;
} internal_state_t;

static int is_character_device(const char* devpath)
{
  struct stat statbuf;

  if (stat(devpath, &statbuf) == -1) {
    perror("stat");
    return 0;
  }

  if (!S_ISCHR(statbuf.st_mode))
  {
    return 0;
  }

  return 1;
}

static int is_multitouch_device(struct libevdev* evdev)
{
  return libevdev_has_event_code(evdev, EV_ABS, ABS_MT_POSITION_X);
}

static int consider_device(const char* devpath, internal_state_t* state)
{
  int fd = -1;
  int score = 10000;
  const char* name;
  struct libevdev* evdev = NULL;

  if (!is_character_device(devpath))
  {
    goto mismatch;
  }

  if ((fd = open(devpath, O_RDWR)) < 0)
  {
    perror("open");
    fprintf(stderr, "Unable to open device %s for inspection", devpath);
    goto mismatch;
  }

  if (libevdev_new_from_fd(fd, &evdev) < 0)
  {
    fprintf(stderr, "Note: device %s is not supported by libevdev\n", devpath);
    goto mismatch;
  }

  if (!is_multitouch_device(evdev))
  {
    goto mismatch;
  }

  

  if (libevdev_has_event_code(evdev, EV_ABS, ABS_MT_TOOL_TYPE))
  {
    int tool_min = libevdev_get_abs_minimum(evdev, ABS_MT_TOOL_TYPE);
    int tool_max = libevdev_get_abs_maximum(evdev, ABS_MT_TOOL_TYPE);

    if (tool_min > MT_TOOL_FINGER || tool_max < MT_TOOL_FINGER)
    {
      fprintf(stderr, "Note: device %s is a touch device, but doesn't"
        " support fingers\n", devpath);
      goto mismatch;
    }

    score -= tool_max - MT_TOOL_FINGER;
  }

  if (libevdev_has_event_code(evdev, EV_ABS, ABS_MT_SLOT))
  {
    score += 1000;

    // Some devices, e.g. Blackberry PRIV (STV100) have more than one surface
    // you can touch. On the PRIV, the keypad also acts as a touch screen
    // that you can swipe and scroll with. The only differences between the
    // touch devices are that one is named "touch_display" and the other
    // "touch_keypad", the keypad only supports 3 contacts and the display
    // up to 9, and the keypad has a much lower resolution. Therefore
    // increasing the score by the number of contacts should be a relatively
    // safe bet, though we may also want to decrease the score by, say, 1,
    // if the device name contains "key" just in case they decide to start
    // supporting more contacts on both touch surfaces in the future.
    int num_slots = libevdev_get_abs_maximum(evdev, ABS_MT_SLOT);
    score += num_slots;
  }

  // For Blackberry devices, see above.
  name = libevdev_get_name(evdev);
  if (strstr(name, "key") != NULL)
  {
    score -= 1;
  }

  // Alcatel OneTouch Idol 3 has an `input_mt_wrapper` device in addition
  // to direct input. It seems to be related to accessibility, as it shows
  // a touchpoint that you can move around, and then tap to activate whatever
  // is under the point. That wrapper device lacks the direct property.
  if (libevdev_has_property(evdev, INPUT_PROP_DIRECT))
  {
    score += 10000;
  }

  // Some devices may have an additional screen. For example, Meizu Pro7 Plus
  // has a small screen on the back side of the device called sub_touch, while
  // the boring screen in the front is called main_touch. The resolution on
  // the sub_touch device is much much lower. It seems like a safe bet
  // to always prefer the larger device, as long as the score adjustment is
  // likely to be lower than the adjustment we do for INPUT_PROP_DIRECT.
  if (libevdev_has_event_code(evdev, EV_ABS, ABS_MT_POSITION_X))
  {
    int x = libevdev_get_abs_maximum(evdev, ABS_MT_POSITION_X);
    int y = libevdev_get_abs_maximum(evdev, ABS_MT_POSITION_Y);
    score += sqrt(x * y);
  }

  if (state->evdev != NULL)
  {
    if (state->score >= score)
    {
      fprintf(stderr, "Note: device %s was outscored by %s (%d >= %d)\n",
        devpath, state->path, state->score, score);
      goto mismatch;
    }
    else
    {
      fprintf(stderr, "Note: device %s was outscored by %s (%d >= %d)\n",
        state->path, devpath, score, state->score);
    }
  }

  libevdev_free(state->evdev);

  state->fd = fd;
  state->score = score;
  strncpy(state->path, devpath, sizeof(state->path));
  state->evdev = evdev;

  return 1;

mismatch:
  libevdev_free(evdev);

  if (fd >= 0)
  {
    close(fd);
  }

  return 0;
}

static int walk_devices(const char* path, internal_state_t* state)
{
  DIR* dir;
  struct dirent* ent;
  char devpath[FILENAME_MAX];

  if ((dir = opendir(path)) == NULL)
  {
    perror("opendir");
    return -1;
  }

  while ((ent = readdir(dir)) != NULL)
  {
    if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0)
    {
      continue;
    }

    snprintf(devpath, FILENAME_MAX, "%s/%s", path, ent->d_name);

    consider_device(devpath, state);
  }

  closedir(dir);

  return 0;
}

#define WRITE_EVENT(state, type, code, value) _write_event(state, type, #type, code, #code, value)

static int _write_event(internal_state_t* state,
  uint16_t type, const char* type_name,
  uint16_t code, const char* code_name,
  int32_t value)
{
  // It seems that most devices do not require the event timestamps at all.
  // Left here for reference should such a situation arise.
  //
  //   timespec ts;
  //   clock_gettime(CLOCK_MONOTONIC, &ts);
  //   input_event event = {{ts.tv_sec, ts.tv_nsec / 1000}, type, code, value};

  struct input_event event = {{0, 0}, type, code, value};
  ssize_t result;
  ssize_t length = (ssize_t) sizeof(event);

  if (g_verbose)
    fprintf(stderr, "%-12s %-20s %08x\n", type_name, code_name, value);

  result = write(state->fd, &event, length);
  return result - length;
}

static int next_tracking_id(internal_state_t* state)
{
  if (state->tracking_id < INT_MAX)
  {
    state->tracking_id += 1;
  }
  else
  {
    state->tracking_id = 0;
  }

  return state->tracking_id;
}

static int type_a_commit(internal_state_t* state)
{
  int contact;
  int found_any = 0;

  for (contact = 0; contact < state->max_contacts; ++contact)
  {
    switch (state->contacts[contact].enabled)
    {
      case 1: // WENT_DOWN
        found_any = 1;

        state->active_contacts += 1;

        if (state->has_tracking_id)
          WRITE_EVENT(state, EV_ABS, ABS_MT_TRACKING_ID, contact);

        // Send BTN_TOUCH on first contact only.
        if (state->active_contacts == 1 && state->has_key_btn_touch)
          WRITE_EVENT(state, EV_KEY, BTN_TOUCH, 1);

        if (state->has_touch_major)
          WRITE_EVENT(state, EV_ABS, ABS_MT_TOUCH_MAJOR, 0x00000006);

        if (state->has_width_major)
          WRITE_EVENT(state, EV_ABS, ABS_MT_WIDTH_MAJOR, 0x00000004);

        if (state->has_pressure)
          WRITE_EVENT(state, EV_ABS, ABS_MT_PRESSURE, state->contacts[contact].pressure);

        WRITE_EVENT(state, EV_ABS, ABS_MT_POSITION_X, state->contacts[contact].x);
        WRITE_EVENT(state, EV_ABS, ABS_MT_POSITION_Y, state->contacts[contact].y);

        WRITE_EVENT(state, EV_SYN, SYN_MT_REPORT, 0);

        state->contacts[contact].enabled = 2;
        break;
      case 2: // MOVED
        found_any = 1;

        if (state->has_tracking_id)
          WRITE_EVENT(state, EV_ABS, ABS_MT_TRACKING_ID, contact);

        if (state->has_touch_major)
          WRITE_EVENT(state, EV_ABS, ABS_MT_TOUCH_MAJOR, 0x00000006);

        if (state->has_width_major)
          WRITE_EVENT(state, EV_ABS, ABS_MT_WIDTH_MAJOR, 0x00000004);

        if (state->has_pressure)
          WRITE_EVENT(state, EV_ABS, ABS_MT_PRESSURE, state->contacts[contact].pressure);

        WRITE_EVENT(state, EV_ABS, ABS_MT_POSITION_X, state->contacts[contact].x);
        WRITE_EVENT(state, EV_ABS, ABS_MT_POSITION_Y, state->contacts[contact].y);

        WRITE_EVENT(state, EV_SYN, SYN_MT_REPORT, 0);
        break;
      case 3: // WENT_UP
        found_any = 1;

        state->active_contacts -= 1;

        if (state->has_tracking_id)
          WRITE_EVENT(state, EV_ABS, ABS_MT_TRACKING_ID, contact);

        // Send BTN_TOUCH only when no contacts remain.
        if (state->active_contacts == 0 && state->has_key_btn_touch)
          WRITE_EVENT(state, EV_KEY, BTN_TOUCH, 0);

        WRITE_EVENT(state, EV_SYN, SYN_MT_REPORT, 0);

        state->contacts[contact].enabled = 0;
        break;
    }
  }

  if (found_any)
    WRITE_EVENT(state, EV_SYN, SYN_REPORT, 0);

  return 1;
}

static int type_a_touch_panic_reset_all(internal_state_t* state)
{
  int contact;

  for (contact = 0; contact < state->max_contacts; ++contact)
  {
    switch (state->contacts[contact].enabled)
    {
      case 1: // WENT_DOWN
      case 2: // MOVED
        // Force everything to WENT_UP
        state->contacts[contact].enabled = 3;
        break;
    }
  }

  return type_a_commit(state);
}

static int type_a_touch_down(internal_state_t* state, int contact, int x, int y, int pressure)
{
  if (contact >= state->max_contacts)
  {
    return 0;
  }

  if (state->contacts[contact].enabled)
  {
    type_a_touch_panic_reset_all(state);
  }

  state->contacts[contact].enabled = 1;
  state->contacts[contact].x = x;
  state->contacts[contact].y = y;
  state->contacts[contact].pressure = pressure;

  return 1;
}

static int type_a_touch_move(internal_state_t* state, int contact, int x, int y, int pressure)
{
  if (contact >= state->max_contacts || !state->contacts[contact].enabled)
  {
    return 0;
  }

  state->contacts[contact].enabled = 2;
  state->contacts[contact].x = x;
  state->contacts[contact].y = y;
  state->contacts[contact].pressure = pressure;

  return 1;
}

static int type_a_touch_up(internal_state_t* state, int contact)
{
  if (contact >= state->max_contacts || !state->contacts[contact].enabled)
  {
    return 0;
  }

  state->contacts[contact].enabled = 3;

  return 1;
}

static int type_b_commit(internal_state_t* state)
{
  WRITE_EVENT(state, EV_SYN, SYN_REPORT, 0);

  return 1;
}

static int type_b_touch_panic_reset_all(internal_state_t* state)
{
  int contact;
  int found_any = 0;

  for (contact = 0; contact < state->max_contacts; ++contact)
  {
    if (state->contacts[contact].enabled)
    {
      state->contacts[contact].enabled = 0;
      found_any = 1;
    }
  }

  return found_any ? type_b_commit(state) : 1;
}

static int type_b_touch_down(internal_state_t* state, int contact, int x, int y, int pressure)
{
  if (contact >= state->max_contacts)
  {
    return 0;
  }

  if (state->contacts[contact].enabled)
  {
    type_b_touch_panic_reset_all(state);
  }

  state->contacts[contact].enabled = 1;
  state->contacts[contact].tracking_id = next_tracking_id(state);
  state->active_contacts += 1;

  WRITE_EVENT(state, EV_ABS, ABS_MT_SLOT, contact);
  WRITE_EVENT(state, EV_ABS, ABS_MT_TRACKING_ID,
    state->contacts[contact].tracking_id);

  // Send BTN_TOUCH on first contact only.
  if (state->active_contacts == 1 && state->has_key_btn_touch)
    WRITE_EVENT(state, EV_KEY, BTN_TOUCH, 1);

  if (state->has_touch_major)
    WRITE_EVENT(state, EV_ABS, ABS_MT_TOUCH_MAJOR, 0x00000006);

  if (state->has_width_major)
    WRITE_EVENT(state, EV_ABS, ABS_MT_WIDTH_MAJOR, 0x00000004);

  if (state->has_pressure)
    WRITE_EVENT(state, EV_ABS, ABS_MT_PRESSURE, pressure);

  WRITE_EVENT(state, EV_ABS, ABS_MT_POSITION_X, x);
  WRITE_EVENT(state, EV_ABS, ABS_MT_POSITION_Y, y);

  return 1;
}

static int type_b_touch_move(internal_state_t* state, int contact, int x, int y, int pressure)
{
  if (contact >= state->max_contacts || !state->contacts[contact].enabled)
  {
    return 0;
  }

  WRITE_EVENT(state, EV_ABS, ABS_MT_SLOT, contact);

  if (state->has_touch_major)
    WRITE_EVENT(state, EV_ABS, ABS_MT_TOUCH_MAJOR, 0x00000006);

  if (state->has_width_major)
    WRITE_EVENT(state, EV_ABS, ABS_MT_WIDTH_MAJOR, 0x00000004);

  if (state->has_pressure)
    WRITE_EVENT(state, EV_ABS, ABS_MT_PRESSURE, pressure);

  WRITE_EVENT(state, EV_ABS, ABS_MT_POSITION_X, x);
  WRITE_EVENT(state, EV_ABS, ABS_MT_POSITION_Y, y);

  return 1;
}

static int type_b_touch_up(internal_state_t* state, int contact)
{
  if (contact >= state->max_contacts || !state->contacts[contact].enabled)
  {
    return 0;
  }

  state->contacts[contact].enabled = 0;
  state->contacts[contact].enabled = 0;
  state->active_contacts -= 1;

  WRITE_EVENT(state, EV_ABS, ABS_MT_SLOT, contact);
  WRITE_EVENT(state, EV_ABS, ABS_MT_TRACKING_ID, -1);

  // Send BTN_TOUCH only when no contacts remain.
  if (state->active_contacts == 0 && state->has_key_btn_touch)
    WRITE_EVENT(state, EV_KEY, BTN_TOUCH, 0);

  return 1;
}

static int touch_down(internal_state_t* state, int contact, int x, int y, int pressure)
{
  if (state->has_mtslot)
  {
    return type_b_touch_down(state, contact, x, y, pressure);
  }
  else
  {
    return type_a_touch_down(state, contact, x, y, pressure);
  }
}

static int touch_move(internal_state_t* state, int contact, int x, int y, int pressure)
{
  if (state->has_mtslot)
  {
    return type_b_touch_move(state, contact, x, y, pressure);
  }
  else
  {
    return type_a_touch_move(state, contact, x, y, pressure);
  }
}

static int touch_up(internal_state_t* state, int contact)
{
  if (state->has_mtslot)
  {
    return type_b_touch_up(state, contact);
  }
  else
  {
    return type_a_touch_up(state, contact);
  }
}

static int touch_panic_reset_all(internal_state_t* state)
{
  if (state->has_mtslot)
  {
    return type_b_touch_panic_reset_all(state);
  }
  else
  {
    return type_a_touch_panic_reset_all(state);
  }
}

static int commit(internal_state_t* state)
{
  if (state->has_mtslot)
  {
    return type_b_commit(state);
  }
  else
  {
    return type_a_commit(state);
  }
}

static int start_server(char* sockname)
{
  int fd = socket(AF_UNIX, SOCK_STREAM, 0);

  if (fd < 0)
  {
    perror("creating socket");
    return fd;
  }

  struct sockaddr_un addr;
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  strncpy(&addr.sun_path[1], sockname, strlen(sockname));

  if (bind(fd, (struct sockaddr*) &addr,
    sizeof(sa_family_t) + strlen(sockname) + 1) < 0)
  {
    perror("binding socket");
    close(fd);
    return -1;
  }

  listen(fd, 1);

  return fd;
}


static int calcRealX(internal_state_t* state, int x, int y) {
  switch(state->orientation){
    case 0  : return round( (     x * 1.0 / state->virtualWidth) * state->realWidth );
    case 90 : return round( ( 1 - y * 1.0 / state->virtualWidth) * state->realWidth );
    case 180: return round( ( 1 - x * 1.0 / state->virtualWidth) * state->realWidth );
    case 270: return round( (     y * 1.0 / state->virtualWidth) * state->realWidth );
  }  
} 
static int calcRealY(internal_state_t* state, int x, int y) {
  switch(state->orientation){
    case 0  : return round( (    y * 1.0 / state->virtualHeight) * state->realHeight );
    case 90 : return round( (    x * 1.0 / state->virtualHeight) * state->realHeight );
    case 180: return round( (1 - y * 1.0 / state->virtualHeight) * state->realHeight );
    case 270: return round( (1 - x * 1.0 / state->virtualHeight) * state->realHeight );
  }
}

static void parse_input(char* buffer, internal_state_t* state)
{
  char* cursor;
  long int contact, x, y, pressure, wait, dx, dy;

  cursor = (char*) buffer;
  cursor += 1;

  switch (buffer[0])
  {
    case 'c': // COMMIT
      commit(state);
      break;
    case 'r': // RESET
      touch_panic_reset_all(state);
      break;
    case 'd': // TOUCH DOWN
      contact = strtol(cursor, &cursor, 10);
      x = strtol(cursor, &cursor, 10);
      y = strtol(cursor, &cursor, 10);
      pressure = strtol(cursor, &cursor, 10);
      touch_down(state, contact, calcRealX(state,x,y), calcRealY(state,x,y), pressure);
      break;
    case 'm': // TOUCH MOVE
      contact = strtol(cursor, &cursor, 10);
      x = strtol(cursor, &cursor, 10);
      y = strtol(cursor, &cursor, 10);
      pressure = strtol(cursor, &cursor, 10);
      touch_move(state, contact, calcRealX(state,x,y), calcRealY(state,x,y), pressure);
      break;
    case 'u': // TOUCH UP
      contact = strtol(cursor, &cursor, 10);
      touch_up(state, contact);
      break;
    case 'w':
      wait = strtol(cursor, &cursor, 10);
      if (g_verbose)
        fprintf(stderr, "Waiting %ld ms\n", wait);
      usleep(wait * 1000);
      break;
    case 't':
      dx = strtol(cursor, &cursor, 10);
      dy = strtol(cursor, &cursor, 10);
      {
        char buf[128];
        char cmd[100];
        snprintf(cmd, 100, "/system/bin/input trackball roll %ld %ld", dx, dy);
        FILE *fp;
         if( (fp = popen(cmd, "r")) == NULL ){
           fprintf(stderr, "exec input error!");
         }
         pclose(fp);
      }
      break;
      
    default:
      break;
  }
}

static void *io_handler(void* state)
{
  setvbuf( ((internal_state_t *)state)->input, NULL, _IOLBF, 1024);
  char read_buffer[80];
  while (fgets(read_buffer, sizeof(read_buffer), ((internal_state_t *)state)->input) != NULL)
  { 
    read_buffer[strcspn(read_buffer, "\r\n")] = 0;
    parse_input(read_buffer, (internal_state_t*)state);
  }
}
