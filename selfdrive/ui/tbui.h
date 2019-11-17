
static void ui_draw_infobar(UIState *s) {
  const UIScene *scene = &s->scene;
  int ui_viz_rx = scene->ui_viz_rx;
  bool hasSidebar = !s->scene.uilayout_sidebarcollapsed;
  int rect_w = vwp_w - ui_viz_rx - bdr_s;
  int rect_h = 60;
  int rect_x = 0;
  // rect_y = screen height - board - background height
  int rect_y = vwp_h - bdr_s - (int) (rect_h/2) - 5;
  rect_x = rect_x + ui_viz_rx;

  int text_x = rect_w / 2;
  text_x = text_x + ui_viz_rx;
  int text_y = rect_y + 45;

  // Get local time to display
  char infobar[68];
  time_t t = time(NULL);
  struct tm tm = *localtime(&t);

  char spd[5];
  snprintf(spd, sizeof(spd), "%i ", (int) (s->scene.v_ego * 3.6 + 0.5));

  char ang_steer[9];
  snprintf(ang_steer, sizeof(ang_steer), "%s%03.1f°", s->b.angleSteers < 0? "-" : "+", fabs(s->b.angleSteers));

  char lead_dist[8];
  if (s->scene.lead_status) {
    snprintf(lead_dist, sizeof(lead_dist), "%05.2fm", s->scene.lead_d_rel);
  } else {
    snprintf(lead_dist, sizeof(lead_dist), "%3s", "N/A");
  }

  char maxspeed_str[12];
  float maxspeed = s->scene.v_cruise;
  int maxspeed_calc = maxspeed + 0.5;
  bool is_cruise_set = (maxspeed > 5 && maxspeed != 255);

  if (s->scene.engaged && is_cruise_set) {
    snprintf(maxspeed_str, sizeof(maxspeed_str), "(%d) Kmh", maxspeed_calc);
  } else{
    snprintf(maxspeed_str, sizeof(maxspeed_str), "%s", "(--) Kmh");
  }

  snprintf(
    infobar,
    sizeof(infobar),
    "%04d/%02d/%02d %02d:%02d:%02d | %s %s | DST: %s | ANG: %s",
    tm.tm_year + 1900,
    tm.tm_mon + 1,
    tm.tm_mday,
    tm.tm_hour,
    tm.tm_min,
    tm.tm_sec,
    spd,
    maxspeed_str,
    lead_dist,
    ang_steer
  );

  nvgBeginPath(s->vg);
  nvgRoundedRect(s->vg, rect_x, rect_y, rect_w, rect_h, 5);
  nvgFillColor(s->vg, nvgRGBA(0, 0, 0, 180));
  nvgFill(s->vg);

  nvgFontSize(s->vg, hasSidebar? 40:50);
  nvgFontFace(s->vg, "courbd");
  nvgFillColor(s->vg, nvgRGBA(255, 255, 255, 180));
  nvgTextAlign(s->vg, NVG_ALIGN_CENTER);
  nvgText(s->vg, text_x, text_y, infobar, NULL);
}
