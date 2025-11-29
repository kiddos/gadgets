#include <string.h>

#include "DEV_Config.h"
#include "GUI_Paint.h"
#include "LCD_1in8.h"
#include "mochi_cute.h"
#include "mochi_laughing.h"
#include "mochi_sitting.h"

#define BAUD_RATE 115200
#define COMMAND_BUFFER_SIZE 64

typedef const unsigned char *const (*frame_array)[10];

int screen_width;
int screen_height;
int row_size;
UWORD image_data[LCD_1IN8_WIDTH * LCD_1IN8_HEIGHT * 2];
int buffer_index = 0;
char command_buffer[COMMAND_BUFFER_SIZE];

frame_array current_frame = &mochi_sitting_frames;

void init() {
  if (DEV_Module_Init() != 0) {
    return;
  }

  LCD_1IN8_Init(HORIZONTAL);
  LCD_1IN8_Clear(0x0000);
  screen_width = LCD_1IN8.WIDTH;
  screen_height = LCD_1IN8.HEIGHT;
  row_size = screen_width * sizeof(UWORD);
  Paint_NewImage((UBYTE *)image_data, screen_width, screen_height, 0, WHITE);

  Paint_SetScale(65);
  Paint_Clear(WHITE);
}

void process_command(char *command) {
  if (strcmp(command, "sit") == 0) {
    current_frame = &mochi_sitting_frames;
    printf("sit\n");
  } else if (strcmp(command, "laugh") == 0) {
    current_frame = &mochi_laughing_frames;
    printf("laugh\n");
  } else if (strcmp(command, "cute") == 0) {
    current_frame = &mochi_cute_frames;
    printf("cute\n");
  } else {
    printf("unknown\n");
  }
}

void read_command() {
  int ch = getchar_timeout_us(1000);
  if (ch != PICO_ERROR_TIMEOUT) {
    char c = (char)ch;
    if (c == '\n' || c == '\r') {
      command_buffer[buffer_index] = '\0';
      process_command(command_buffer);
      buffer_index = 0;
    } else if (buffer_index < (COMMAND_BUFFER_SIZE - 1)) {
      command_buffer[buffer_index] = c;
      buffer_index++;
    } else {
      command_buffer[COMMAND_BUFFER_SIZE - 1] = '\0';
      buffer_index = 0;
    }
  }
}

int main() {
  sleep_ms(100);
  stdio_init_all();

  init();

  int f = 0;
  uint32_t last = to_ms_since_boot(get_absolute_time());
  while (true) {
    read_command();

    uint32_t t = to_ms_since_boot(get_absolute_time());
    Paint_DrawImage((*current_frame)[f], 0, 0, screen_width, screen_height);
    LCD_1IN8_Display(image_data);
    if (t - last >= 33) {
      f = (f + 1) % 10;
      last = t;
    }
  }

  DEV_Module_Exit();
  return 0;
}
