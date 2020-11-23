void bb_ui_init(UIState *s);
void bb_ui_preinit(UIState *s);
void ui_draw_infobar(UIState *s);
void bb_ui_draw_UI( UIState *s);
void dashcam( UIState *s, int touch_x, int touch_y );
bool bb_ui_poll_update( UIState *s);
bool bb_handle_ui_touch( UIState *s, int touch_x, int touch_y);
int bb_get_button_status(UIState *s, char *btn_name);