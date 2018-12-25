extern PubSubClient client;

lv_obj_t *sw1, *sw2;

static lv_res_t sw_click_action(lv_obj_t * sw)
{
    char topic[50];
    char msg[50];
    
    uint8_t id = lv_obj_get_free_num(sw);
    uint8_t state = lv_sw_get_state(sw);
    uint16_t frequency = state ? 900 : 700;
    
    EasyBuzzer.singleBeep(
    frequency,  // Frequency in hertz(HZ).  
    20,   // Duration of the beep in milliseconds(ms). 
    NULL    // [Optional] Function to call when done.
    );
  
    printf("Switch %d set to %d\n", id, state);
        
    sprintf(topic,"%s/%d", mqtt_outtopic, id);
    sprintf(msg,"%d", state);
    
    client.publish(topic, msg);

    return LV_RES_OK; 
}

void set_switch_state(int sw, int state) { // Forgive me for doing this :)
    uint16_t frequency = state ? 900 : 700;
      
    if(sw == 1 && state == 1) lv_sw_on(sw1);
    if(sw == 1 && state == 0) lv_sw_off(sw1);
    if(sw == 2 && state == 1) lv_sw_on(sw2);    
    if(sw == 2 && state == 0) lv_sw_off(sw2);  

    EasyBuzzer.singleBeep(
    frequency,  // Frequency in hertz(HZ).  
    20,   // Duration of the beep in milliseconds(ms). 
    NULL    // [Optional] Function to call when done.
    );
}

void create_control_gui(void) {
  
  lv_style_scr.body.main_color = LV_COLOR_WHITE;
  lv_style_scr.body.grad_color = LV_COLOR_GRAY;
    
  /*Create label on the screen. By default it will inherit the style of the screen*/
  lv_obj_t * title = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(title, "Christmas lights controller");
  lv_obj_align(title, NULL, LV_ALIGN_IN_TOP_MID, 0, 20);  /*Align to the top*/

  /*Create styles for the switch*/
  static lv_style_t bg_style;
  static lv_style_t indic_style;
  static lv_style_t knob_on_style;
  static lv_style_t knob_off_style;
  lv_style_copy(&bg_style, &lv_style_pretty);
  bg_style.body.radius = LV_RADIUS_CIRCLE;

  lv_style_copy(&indic_style, &lv_style_pretty_color);
  indic_style.body.radius = LV_RADIUS_CIRCLE;
  indic_style.body.main_color = LV_COLOR_HEX(0x9fc8ef);
  indic_style.body.grad_color = LV_COLOR_HEX(0x9fc8ef);
  indic_style.body.padding.hor = 0;
  indic_style.body.padding.ver = 0;

  lv_style_copy(&knob_off_style, &lv_style_pretty);
  knob_off_style.body.radius = LV_RADIUS_CIRCLE;
  knob_off_style.body.shadow.width = 4;
  knob_off_style.body.shadow.type = LV_SHADOW_BOTTOM;

  lv_style_copy(&knob_on_style, &lv_style_pretty_color);
  knob_on_style.body.radius = LV_RADIUS_CIRCLE;
  knob_on_style.body.shadow.width = 4;
  knob_on_style.body.shadow.type = LV_SHADOW_BOTTOM;

  /*Create a switch and apply the styles*/
  sw1 = lv_sw_create(lv_scr_act(), NULL);
  lv_sw_set_style(sw1, LV_SW_STYLE_BG, &bg_style);
  lv_sw_set_style(sw1, LV_SW_STYLE_INDIC, &indic_style);
  lv_sw_set_style(sw1, LV_SW_STYLE_KNOB_ON, &knob_on_style);
  lv_sw_set_style(sw1, LV_SW_STYLE_KNOB_OFF, &knob_off_style);
  lv_obj_align(sw1, NULL, LV_ALIGN_CENTER, 0, -30);

  /*Create a label right to the bar*/
  lv_obj_t * sw1_label = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(sw1_label, "Kitchen");
  lv_obj_align(sw1_label, sw1, LV_ALIGN_OUT_LEFT_MID, -10, 0);

  lv_obj_set_free_num(sw1, 1);   /*Set a unique number for the switch*/
  lv_sw_set_action(sw1, sw_click_action);

  /*Copy the first switch*/
  sw2 = lv_sw_create(lv_scr_act(), sw1);
  lv_obj_align(sw2, NULL, LV_ALIGN_CENTER, 0, 30);
  lv_obj_t * sw2_label = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(sw2_label, "Garden");
  lv_obj_align(sw2_label, sw2, LV_ALIGN_OUT_LEFT_MID, -10, 0);

  lv_obj_set_free_num(sw2, 2);   /*Set a unique number for the switch*/
  lv_sw_set_action(sw2, sw_click_action);
}

