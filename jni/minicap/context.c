
#define MAX_SUPPORTED_CONTACTS 10
#define DEFAULT_JPG_QUALITY 85

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

typedef struct
{
  unsigned int quality;
  Minicap::DisplayInfo *preDisplayInfo;
  Minicap::DisplayInfo *currDisplayInfo;
  int isApplyRotation;
  internal_state_t *state;

} middlecap_ctx;


static middlecap_ctx* initMiddleCtx() {
  middlecap_ctx* ctx = (middlecap_ctx*) malloc(sizeof(middlecap_ctx));
  ctx->quality = DEFAULT_JPG_QUALITY;
  ctx->preDisplayInfo = NULL;
  ctx->currDisplayInfo = NULL;
  ctx->isApplyRotation = 0; 
  ctx->state = (internal_state_t*) malloc(sizeof(internal_state_t));
  *(ctx->state) = {0};
  return ctx;
}