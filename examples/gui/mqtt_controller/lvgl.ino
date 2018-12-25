Ticker tick;
const float togglePeriod = 0.05; //seconds

//Ticker function to keep the GUI running
static void lv_tick_handler(void) {
  lv_tick_inc(togglePeriod * 1000);
}

void disp_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t *color_p) {
  uint16_t c;
  tft.startWrite(); // Start new TFT transaction
  tft.setAddrWindow(x1, y1, (x2 - x1 + 1), (y2 - y1 + 1));
  for (int y = y1; y <= y2; y++) {
    for (int x = x1; x <= x2; x++) {
      c = tft.color565(color_p->red, color_p->green, color_p->blue);
      tft.writePixels(&c, 1);
      color_p++;
    }
  }
  tft.endWrite();
  lv_flush_ready();
}

bool my_input_read(lv_indev_data_t *data)
{
  static uint16_t prev_x, prev_y;
  TSPoint p = ts.getPoint();

  if (p.z > ts.pressureThreshhold) {
    data->point.x = p.x;
    data->point.y = p.y;
    data->state = LV_INDEV_STATE_PR;
    prev_x = data->point.x;
    prev_y = data->point.y;
  } else {
    data->point.x = prev_x;
    data->point.y = prev_y;
    data->state = LV_INDEV_STATE_REL;
  }

  return false;        /*No buffering so no more data read*/
}

void lvgl_task( void * parameter )
{
   delay(50); // wait for littlevgl init
   while(1) {
      lv_task_handler();
      delay(50);
   }
    vTaskDelete( NULL ); // Should never be reached
}

void lvgl_init() {

  lv_init();

  /*Initialize the display*/
  lv_disp_drv_t disp_drv;
  disp_drv.disp_flush = disp_flush;
  lv_disp_drv_register(&disp_drv);

  /* Initialize input device touch */
  lv_indev_drv_t indev_drv;
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read = my_input_read;
  lv_indev_drv_register(&indev_drv);
    
  /*Initialize the graphics library's tick*/
  tick.attach(togglePeriod, lv_tick_handler);

  xTaskCreate(
                    lvgl_task,        /* Task function. */
                    "lvgl",           /* String with name of task. */
                    10000,            /* Stack size in bytes. */
                    NULL,             /* Parameter passed as input of the task */
                    5,                /* Priority of the task. */
                    NULL);            /* Task handle. */

}

