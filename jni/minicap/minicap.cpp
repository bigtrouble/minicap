#include <fcntl.h>
#include <getopt.h>
#include <linux/fb.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/socket.h>

#include <cmath>
#include <condition_variable>
#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>
#include <time.h>

#include <Minicap.hpp>

#include "util/debug.h"
#include "JpgEncoder.hpp"
#include "Projection.hpp"

#include <sys/time.h>
#include <pthread.h>
#include <context.c>
#include <minitouch.c>
#include <parseinput.c>
#include <termios.h>

#define BANNER_VERSION 1
#define BANNER_SIZE 24

#define DEFAULT_DISPLAY_ID 0

enum {
  QUIRK_DUMB            = 1,
  QUIRK_ALWAYS_UPRIGHT  = 2,
  QUIRK_TEAR            = 4,
};

static void
usage(const char* pname) {
  fprintf(stderr,
    "Usage: %s [-h] [-n <name>]\n"
    "  -d <id>:       Display ID. (%d)\n"
    "  -P <value>:    Display projection (<w>x<h>).\n"
    "  -s:            Take a screenshot and output it to stdout. Needs -P.\n"
    "  -S:            Skip frames when they cannot be consumed quickly enough.\n"
    "  -t:            Attempt to get the capture method running, then exit.\n"
    "  -h:            Show help.\n",
    pname, DEFAULT_DISPLAY_ID
  );
}

class FrameWaiter: public Minicap::FrameAvailableListener {
public:
  FrameWaiter()
    : mPendingFrames(0),
      mTimeout(std::chrono::milliseconds(100)),
      mStopped(false) {
  }

  int
  waitForFrame() {
    std::unique_lock<std::mutex> lock(mMutex);

    while (!mStopped) {
      if (mCondition.wait_for(lock, mTimeout, [this]{return mPendingFrames > 0;})) {
        return mPendingFrames--;
      }
    }

    return 0;
  }

  void
  reportExtraConsumption(int count) {
    std::unique_lock<std::mutex> lock(mMutex);
    mPendingFrames -= count;
  }

  void
  onFrameAvailable() {
    std::unique_lock<std::mutex> lock(mMutex);
    mPendingFrames += 1;
    mCondition.notify_one();
  }

  void
  stop() {
    mStopped = true;
  }

  bool
  isStopped() {
    return mStopped;
  }

private:
  std::mutex mMutex;
  std::condition_variable mCondition;
  std::chrono::milliseconds mTimeout;
  int mPendingFrames;
  bool mStopped;
};



static int
putUInt32LE(unsigned char* data, int value) {
  data[0] = (value & 0x000000FF) >> 0;
  data[1] = (value & 0x0000FF00) >> 8;
  data[2] = (value & 0x00FF0000) >> 16;
  data[3] = (value & 0xFF000000) >> 24;
}

static int
putUInt1LE(unsigned char* data, char value) {
  data[0] = value;
}

static int
try_get_framebuffer_display_info(uint32_t displayId, Minicap::DisplayInfo* info) {
  char path[64];
  sprintf(path, "/dev/graphics/fb%d", displayId);

  int fd = open(path, O_RDONLY);
  if (fd < 0) {
    MCERROR("Cannot open %s", path);
    return -1;
  }

  fb_var_screeninfo vinfo;
  if (ioctl(fd, FBIOGET_VSCREENINFO, &vinfo) < 0) {
    close(fd);
    MCERROR("Cannot get FBIOGET_VSCREENINFO of %s", path);
    return -1;
  }

  info->width = vinfo.xres;
  info->height = vinfo.yres;
  info->orientation = vinfo.rotate;
  info->xdpi = static_cast<float>(vinfo.xres) / static_cast<float>(vinfo.width) * 25.4;
  info->ydpi = static_cast<float>(vinfo.yres) / static_cast<float>(vinfo.height) * 25.4;
  info->size = std::sqrt(
    (static_cast<float>(vinfo.width) * static_cast<float>(vinfo.width)) +
    (static_cast<float>(vinfo.height) * static_cast<float>(vinfo.height))) / 25.4;
  info->density = std::sqrt(
    (static_cast<float>(vinfo.xres) * static_cast<float>(vinfo.xres)) +
    (static_cast<float>(vinfo.yres) * static_cast<float>(vinfo.yres))) / info->size;
  info->secure = false;
  info->fps = 0;

  close(fd);

  return 0;
}

static FrameWaiter gWaiter;



static int pthread_flag = 1;
static int grotation = 0;

static Minicap::DisplayInfo realInfo;
static Minicap::DisplayInfo desiredInfo;
static Minicap* minicap;

static int getRotation(Minicap::DisplayInfo *info) {
  switch (info->orientation) {
        case Minicap::ORIENTATION_0:   return 0;
        case Minicap::ORIENTATION_90:  return 90;
        case Minicap::ORIENTATION_180: return 180;
        case Minicap::ORIENTATION_270: return 270;
  }
  return 0;
}


static Minicap::DisplayInfo *getDisplayInfo() {
  Minicap::DisplayInfo *info = (Minicap::DisplayInfo*) malloc(sizeof(Minicap::DisplayInfo));
  minicap_try_get_display_info(DEFAULT_DISPLAY_ID, info);
  return info;
}

static void *thread_func(void *args) {

    middlecap_ctx *ctx = (middlecap_ctx *) args;
    do{

      timeval delay;
      delay.tv_sec = 0;
      delay.tv_usec = 300 * 1000; // 300 ms
      select(0, NULL, NULL, NULL, &delay);
      free(ctx->preDisplayInfo);
      ctx->preDisplayInfo = ctx->currDisplayInfo;
      ctx->currDisplayInfo = getDisplayInfo();
      ctx->isApplyRotation = 0;
      
    } while(pthread_flag);
}




static middlecap_ctx *ctx = initMiddleCtx();

int
main(int argc, char* argv[]) {

  //
  freopen("/data/local/tmp/middlecap/err.log", "w", stderr);

  struct termios flags;
  tcgetattr(fileno(stdin), &flags);
  flags.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
  flags.c_oflag &= ~OPOST;
  flags.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
  flags.c_cflag &= ~(CSIZE|PARENB);
  flags.c_cflag |= CS8;
  tcsetattr(fileno(stdin), TCSANOW , &flags );

  tcgetattr(fileno(stdout), &flags);
  flags.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
  flags.c_oflag &= ~OPOST;
  flags.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
  flags.c_cflag &= ~(CSIZE|PARENB);
  flags.c_cflag |= CS8;
  tcsetattr(fileno(stdout), TCSANOW , &flags );
  setbuf(stdout, NULL);

  
  uint32_t displayId = DEFAULT_DISPLAY_ID;
  bool takeScreenshot = false;
  bool skipFrames = false;
  Projection proj;

  int opt;
  while ((opt = getopt(argc, argv, "d:n:P:Q:siSth")) != -1) {
    switch (opt) {
    case 'd':
      displayId = atoi(optarg);
      break;
    case 'P': {
      Projection::Parser parser;
      if (!parser.parse(proj, optarg, optarg + strlen(optarg))) {
        std::cerr << "ERROR: invalid format for -P, need <virtual-width>x<virtual-height>" << std::endl;
        return EXIT_FAILURE;
      }
      break;
    }
      break;
    case 's':
      takeScreenshot = true;
      break;
    case 'S':
      skipFrames = true;
      break;
    default:
      return EXIT_FAILURE;
    }
  }


  // Start Android's thread pool so that it will be able to serve our requests.
  minicap_start_thread_pool();



  ctx->preDisplayInfo = getDisplayInfo();
  ctx->currDisplayInfo = getDisplayInfo();

  // Set real display size.
  realInfo.width = ctx->currDisplayInfo->width;
  proj.realWidth = ctx->currDisplayInfo->width;
  realInfo.height = ctx->currDisplayInfo->height;
  proj.realHeight = ctx->currDisplayInfo->height;
  proj.rotation = getRotation(ctx->currDisplayInfo);
  ctx->state->virtualWidth = proj.virtualWidth;
  ctx->state->virtualHeight = proj.virtualHeight;
  ctx->state->orientation = getRotation(ctx->currDisplayInfo);
  proj.forceMaximumSize();
  // proj.forceAspectRatio();
  std::cerr << "INFO: Using projection " << proj << std::endl;
  if (!proj.valid()) {
    std::cerr << "ERROR: missing or invalid -P" << std::endl;
    return EXIT_FAILURE;
  }

  std::cerr << "PID: " << getpid() << std::endl;
  
  // Disable STDOUT buffering.

  char *buf = (char*) malloc(1024*1024*16);
  setvbuf(stdout, buf, _IOFBF, 1024*1024*16);

  // Figure out desired display size.
  desiredInfo.width = proj.virtualWidth;
  desiredInfo.height = proj.virtualHeight;
  desiredInfo.orientation = ctx->currDisplayInfo->orientation;


  // Leave a 5-byte padding to the encoder so that we can inject the flag & size
  // to the same buffer.
  JpgEncoder encoder(5, 0);
  Minicap::Frame frame;
  bool haveFrame = false;

  // Set up minicap.
  minicap = minicap_create(displayId);
  if (minicap == NULL) {
    return EXIT_FAILURE;
  }

  // Figure out the quirks the current capture method has.
  unsigned char quirks = 0;
  switch (minicap->getCaptureMethod()) {
  case Minicap::METHOD_FRAMEBUFFER:
    quirks |= QUIRK_DUMB | QUIRK_TEAR;
    break;
  case Minicap::METHOD_SCREENSHOT:
    quirks |= QUIRK_DUMB;
    break;
  case Minicap::METHOD_VIRTUAL_DISPLAY:
    quirks |= QUIRK_ALWAYS_UPRIGHT;
    break;
  }

  if (minicap->setRealInfo(realInfo) != 0) {
    MCERROR("Minicap did not accept real display info");
    goto disaster;
  }

  if (minicap->setDesiredInfo(desiredInfo) != 0) {
    MCERROR("Minicap did not accept desired display info");
    goto disaster;
  }

  minicap->setFrameAvailableListener(&gWaiter);

  if (minicap->applyConfigChanges() != 0) {
    MCERROR("Unable to start minicap with current config");
    goto disaster;
  }

  if (!encoder.reserveData(realInfo.width, realInfo.height)) {
    MCERROR("Unable to reserve data for JPG encoder");
    goto disaster;
  }

  if (takeScreenshot) {
    if (!gWaiter.waitForFrame()) {
      MCERROR("Unable to wait for frame");
      goto disaster;
    }

    int err;
    if ((err = minicap->consumePendingFrame(&frame)) != 0) {
      MCERROR("Unable to consume pending frame");
      goto disaster;
    }

    if (!encoder.encode(&frame, ctx->quality)) {
      MCERROR("Unable to encode frame");
      goto disaster;
    }

    // if (pumpf(encoder.getEncodedData(), encoder.getEncodedSize()) < 0) {
    //   MCERROR("Unable to output encoded frame data");
    //   goto disaster;
    // }

    return EXIT_SUCCESS;
  }

  if (walk_devices("/dev/input", ctx->state) != 0) {
    fprintf(stderr, "Unable to crawl %s for touch devices\n", "/dev/input");
    goto disaster;
  }

  if (ctx->state->evdev == NULL)
  {
    fprintf(stderr, "Unable to find a suitable touch device\n");
    goto disaster;
  }


  ctx->state->has_mtslot        = libevdev_has_event_code(ctx->state->evdev, EV_ABS, ABS_MT_SLOT);
  ctx->state->has_tracking_id   = libevdev_has_event_code(ctx->state->evdev, EV_ABS, ABS_MT_TRACKING_ID);
  ctx->state->has_key_btn_touch = libevdev_has_event_code(ctx->state->evdev, EV_KEY, BTN_TOUCH);
  ctx->state->has_touch_major   = libevdev_has_event_code(ctx->state->evdev, EV_ABS, ABS_MT_TOUCH_MAJOR);
  ctx->state->has_width_major   = libevdev_has_event_code(ctx->state->evdev, EV_ABS, ABS_MT_WIDTH_MAJOR);
  ctx->state->has_pressure      = libevdev_has_event_code(ctx->state->evdev, EV_ABS, ABS_MT_PRESSURE);
  ctx->state->min_pressure      = ctx->state->has_pressure ? libevdev_get_abs_minimum(ctx->state->evdev, ABS_MT_PRESSURE) : 0;
  ctx->state->max_pressure      = ctx->state->has_pressure ? libevdev_get_abs_maximum(ctx->state->evdev, ABS_MT_PRESSURE) : 0;
  ctx->state->max_x             = libevdev_get_abs_maximum(ctx->state->evdev, ABS_MT_POSITION_X);
  ctx->state->max_y             = libevdev_get_abs_maximum(ctx->state->evdev, ABS_MT_POSITION_Y);

  ctx->state->max_tracking_id = ctx->state->has_tracking_id ? libevdev_get_abs_maximum(ctx->state->evdev, ABS_MT_TRACKING_ID) : INT_MAX;

  if (!ctx->state->has_mtslot && ctx->state->max_tracking_id == 0) {
    // The touch device reports incorrect values. There would be no point
    // in supporting ABS_MT_TRACKING_ID at all if the maximum value was 0
    // (i.e. one contact). This happens on Lenovo Yoga Tablet B6000-F,
    // which actually seems to support ~10 contacts. So, we'll just go with
    // as many as we can and hope that the system will ignore extra contacts.
    ctx->state->max_tracking_id = MAX_SUPPORTED_CONTACTS - 1;
    fprintf(stderr,
      "Note: type A device reports a max value of 0 for ABS_MT_TRACKING_ID. "
      "This means that the device is most likely reporting incorrect "
      "information. Guessing %d.\n",
      ctx->state->max_tracking_id
    );
  }


  ctx->state->max_contacts = ctx->state->has_mtslot
    ? libevdev_get_abs_maximum(ctx->state->evdev, ABS_MT_SLOT) + 1
    : (ctx->state->has_tracking_id ? ctx->state->max_tracking_id + 1 : 2);

  ctx->state->tracking_id = 0;

  int contact;
  for (contact = 0; contact < MAX_SUPPORTED_CONTACTS; ++contact)
  {
    ctx->state->contacts[contact].enabled = 0;
  }

  ctx->state->realWidth = ctx->state->max_x;
  ctx->state->realHeight = ctx->state->max_y;

  if (ctx->state->max_contacts > MAX_SUPPORTED_CONTACTS) {
    fprintf(stderr, "Note: hard-limiting maximum number of contacts to %d\n",
      MAX_SUPPORTED_CONTACTS);
    ctx->state->max_contacts = MAX_SUPPORTED_CONTACTS;
  }

  // Prepare banner for clients.
  unsigned char banner[BANNER_SIZE];
  banner[0] = (unsigned char) BANNER_VERSION;
  banner[1] = (unsigned char) BANNER_SIZE;
  putUInt32LE(banner + 2, getpid());
  putUInt32LE(banner + 6,  realInfo.width);
  putUInt32LE(banner + 10,  realInfo.height);
  putUInt32LE(banner + 14, desiredInfo.width);
  putUInt32LE(banner + 18, desiredInfo.height);
  banner[22] = (unsigned char) desiredInfo.orientation;
  banner[23] = quirks;

  // =======================================================
  pthread_t thread;
  if (pthread_create(&thread, NULL, thread_func, ctx) != 0){
      return EXIT_FAILURE;
  }

  // ======================================================
  pthread_t inputParseThread;
  ctx->state->input = stdin;
  if (pthread_create(&inputParseThread, NULL, input_parse_handler, ctx) != 0){
       return EXIT_FAILURE;
  }
  

  //pumpf((unsigned char*)"MIDDLECAP-FETCH-START\n", strlen("MIDDLECAP-FETCH-START\n"));

    if (pumpf(banner, BANNER_SIZE, 'b') < 0) {
      return EXIT_FAILURE;;
    }
    

    //send init rotation
    {
      unsigned char *rdatainit = (unsigned char*) malloc(4);
      //putUInt1LE(rdatainit, 0x02);
      putUInt32LE(rdatainit,  getRotation(ctx->currDisplayInfo));
      pumpf(rdatainit, 4 , 0x02);
      free(rdatainit);
    }
    

    int pending, err;
    while (!gWaiter.isStopped() && (pending = gWaiter.waitForFrame()) > 0) {
      
      if (skipFrames && pending > 1) {
        // Skip frames if we have too many. Not particularly thread safe,
        // but this loop should be the only consumer anyway (i.e. nothing
        // else decreases the frame count).
        gWaiter.reportExtraConsumption(pending - 1);

        while (--pending >= 1) {
          if ((err = minicap->consumePendingFrame(&frame)) != 0) {
            if (err == -EINTR) {
              //MCINFO("Frame consumption interrupted by EINTR");
              goto close;
            }
            else {
              //MCERROR("Unable to skip pending frame");
              goto disaster;
            }
          }

          minicap->releaseConsumedFrame(&frame);
        }
      }

      if( NULL == (&frame)) {
        //MCINFO(" invalid frame...  ");
        continue;
      }

      if ((err = minicap->consumePendingFrame(&frame)) != 0) {
        if(err == -22){
            //MCINFO(" -22  ");
            minicap->releaseConsumedFrame(&frame);
            continue;
        }
        else if (err == -EINTR) {
          //MCINFO("Frame consumption interrupted by EINTR");
          goto close;
        }
        else {
          //MCERROR("Unable to consume pending frame");
          goto disaster;
        }
      }

      haveFrame = true;

      // Encode the frame.
      if (!encoder.encode(&frame, ctx->quality)) {
        //MCERROR("Unable to encode frame");
        goto disaster;
      }

      // Push it out synchronously because it's fast and we don't care
      // about other clients.
      unsigned char* data = encoder.getEncodedData() - 5;
      size_t size = encoder.getEncodedSize();

      // push frame
      putUInt1LE(data, 0x01);
      putUInt32LE(data + 1, size);
      pumpf(data, size + 5, 0x01);

      // This will call onFrameAvailable() on older devices, so we have
      // to do it here or the loop will stop.
      minicap->releaseConsumedFrame(&frame);
      haveFrame = false;

      if (getRotation(ctx->preDisplayInfo) != getRotation(ctx->currDisplayInfo) && ctx->isApplyRotation == 0 ) {


        unsigned char* rdata = (unsigned char*)malloc(4);
        // putUInt1LE(rdata, 0x02);
        putUInt32LE(rdata, getRotation(ctx->currDisplayInfo));
        pumpf(rdata, 4, 0x02);
        free(rdata);

        desiredInfo.orientation = ctx->currDisplayInfo->orientation;
        ctx->state->orientation = getRotation(ctx->currDisplayInfo);
        minicap->setRealInfo(realInfo);
        minicap->setDesiredInfo(desiredInfo);
        minicap->applyConfigChanges();

        ctx->isApplyRotation = 1;
      }
  }

close:
    MCINFO("Closing client connection");
    // Have we consumed one frame but are still holding it?
    if (haveFrame) {
      minicap->releaseConsumedFrame(&frame);
    }

  minicap_free(minicap);

  return EXIT_SUCCESS;

disaster:
  if (haveFrame) {
    minicap->releaseConsumedFrame(&frame);
  }

  minicap_free(minicap);

  return EXIT_FAILURE;
}
