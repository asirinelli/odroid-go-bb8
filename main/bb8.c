#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "input.h"

#if CONFIG_BB8_GUI
#include "odroid_display.h"
#include "../components/ugui/ugui.h"
#endif

#define GATTC_TAG "GATTC_BB8"

#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t * param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t * param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
					  esp_ble_gattc_cb_param_t * param);

static bool Isconnecting = false;

static esp_ble_scan_params_t ble_scan_params = {
  .scan_type = BLE_SCAN_TYPE_ACTIVE,
  .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
  .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
  .scan_interval = 0x50,
  .scan_window = 0x30,
  .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
  esp_gattc_cb_t gattc_cb;
  uint16_t gattc_if;
  uint16_t app_id;
  uint16_t conn_id;
  esp_bd_addr_t remote_bda;
};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
  [PROFILE_A_APP_ID] = {
			.gattc_cb = gattc_profile_event_handler,
			.gattc_if = ESP_GATT_IF_NONE,	/* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
			},
};

enum program_status_enum { CONNECTION, ANTIDOS, TXPOWER, WAKECPU, READY } program_status;

#if CONFIG_BB8_GUI
uint16_t *fb;
UG_GUI gui;

static void pset(UG_S16 x, UG_S16 y, UG_COLOR color)
{
  fb[y * 320 + x] = color;
}

static void ui_update_display()
{
  //ili9341_write_frame(fb);
  ili9341_write_frame_rectangleLE(0, 0, 320, 240, fb);
}

static void ui_draw_title()
{
  const char *TITLE = "BB8";
  const char *AUTHOR = "Antoine Sirinelli";

  UG_FillFrame(0, 0, 319, 239, C_WHITE);

  // Header
  UG_FillFrame(0, 0, 319, 15, C_MIDNIGHT_BLUE);
  UG_FontSelect(&FONT_8X8);
  const short titleLeft = (320 / 2) - (strlen(TITLE) * 9 / 2);
  UG_SetForecolor(C_WHITE);
  UG_SetBackcolor(C_MIDNIGHT_BLUE);
  UG_PutString(titleLeft, 4, TITLE);

  // Footer
  UG_FillFrame(0, 239 - 16, 319, 239, C_MIDNIGHT_BLUE);
  const short footerLeft = (320 / 2) - (strlen(AUTHOR) * 9 / 2);
  UG_SetForecolor(C_DARK_GRAY);
  UG_PutString(footerLeft, 240 - 4 - 8, AUTHOR);
}

void print_message(const char *msg)
{
  const short footerLeft = (320 / 2) - (strlen(msg) * 9 / 2);
  UG_FillFrame(0, 15, 319, 239 - 16, C_WHITE);
  UG_SetForecolor(C_BLACK);
  UG_SetBackcolor(C_WHITE);

  UG_PutString(footerLeft, 240 / 2 - 8, msg);
}

int display_speed = 0;
float display_angle = 0;
UG_COLOR display_color = C_GREEN;

static void DisplaySpeed()
{
  int speed = display_speed;
  float angle = display_angle;

  if (speed > 255)
    speed = 255;

  const int WIDTH = 12;
  const int HEIGHT = 150;
  const int FILL_HEIGHT = HEIGHT * (speed / 255.0f);

  const int RADIUS = 50;

  short left = (2 * 320 / 3);
  short top = (240 / 2) - (HEIGHT / 2) + 16;

  UG_FillFrame(left - 1, top - 1, left + WIDTH + 1, top + HEIGHT + 1, C_WHITE);
  UG_DrawFrame(left - 1, top - 1, left + WIDTH + 1, top + HEIGHT + 1, C_BLACK);

  if (FILL_HEIGHT > 0) {
    UG_FillFrame(left, top + HEIGHT - FILL_HEIGHT, left + WIDTH, top + HEIGHT, display_color);
  }

  UG_SetForecolor(C_BLACK);
  UG_SetBackcolor(C_WHITE);
  char str_speed[4];
  snprintf(str_speed, 4, "%3d", speed);
  UG_PutString(left - 8, top - 16, str_speed);

  UG_FillCircle(320 / 3, 240 / 2 + 16, RADIUS, C_WHITE);
  UG_DrawCircle(320 / 3, 240 / 2 + 16, RADIUS, C_BLACK);
  UG_DrawLine(320 / 3, 240 / 2 + 16,
	      320 / 3 + RADIUS * sinf(M_PI * angle / 180.0), 240 / 2 + 16 - RADIUS * cosf(M_PI * angle / 180.0), C_RED);

  UG_SetForecolor(C_BLACK);
  UG_SetBackcolor(C_WHITE);
  snprintf(str_speed, 4, "%3.0f", angle);
  UG_PutString(320 / 3 - 16, 240 / 2 + 16 - RADIUS - 16, str_speed);

  ui_update_display();
}

#endif

#if CONFIG_STATIC_MAC
static esp_bd_addr_t bb8_mac;

static void init_mac()
{
  int values[6];

  if (6 == sscanf(CONFIG_BB8_MAC, "%x:%x:%x:%x:%x:%x%*c",
		  &values[0], &values[1], &values[2],
		  &values[3], &values[4], &values[5])) {

    for(int i = 0; i < 6; ++i)
      bb8_mac[i] = (uint8_t) values[i];

    ESP_LOGI(GATTC_TAG, "BB8 MAC address configured:");
    esp_log_buffer_hex(GATTC_TAG, bb8_mac, ESP_BD_ADDR_LEN);
  }
  else {
    ESP_LOGE(GATTC_TAG, "Invalid MAC Address!");
  }
}
#endif

uint8_t check_if_bb8(struct ble_scan_result_evt_param scan_rst)
{

#if CONFIG_STATIC_MAC
  if (memcmp(scan_rst.bda, bb8_mac, ESP_BD_ADDR_LEN) == 0)
    return 1;
  else
    return 0;

#else

  uint8_t adv_name_len = 0;
  uint8_t *adv_name;

  adv_name = esp_ble_resolve_adv_data(scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
  ESP_LOGI(GATTC_TAG, "adv_name_len: %d", adv_name_len);
  esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);
  if (adv_name_len > 3)
    if ((adv_name[0] == 'B') && (adv_name[1] == 'B') && (adv_name[2] == '-'))
      return 1;

  return 0;

#endif

}


static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
					  esp_ble_gattc_cb_param_t * param)
{
  esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *) param;

  /* ESP_LOGI(GATTC_TAG, "gattc_profile_event_handler on if %d for event %d", gattc_if, event); */

  switch (event) {
  case ESP_GATTC_REG_EVT:
    ESP_LOGI(GATTC_TAG, "REG_EVT");
    esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
    if (scan_ret) {
      ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
    }
    break;

  case ESP_GATTC_OPEN_EVT:
    ESP_LOGI(GATTC_TAG, "OPEN_EVT with status 0x%x", p_data->open.status);
    if (p_data->open.status != ESP_GATT_OK) {
      //open failed, ignore the first device, connect the second device
      ESP_LOGE(GATTC_TAG, "connect device failed, status %d", p_data->open.status);
      //start_scan();
      break;
    }
    memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->open.remote_bda, 6);
    gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->open.conn_id;
    ESP_LOGI(GATTC_TAG, "ESP_GATTC_OPEN_EVT conn_id %d, if %d, status %d, mtu %d", p_data->open.conn_id, gattc_if,
	     p_data->open.status, p_data->open.mtu);
    ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
    esp_log_buffer_hex(GATTC_TAG, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
    esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req(gattc_if, p_data->open.conn_id);
    if (mtu_ret) {
      ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
    }
    break;
  case ESP_GATTC_CFG_MTU_EVT:
    ESP_LOGI(GATTC_TAG, "CFG_MTU_EVT with status 0x%x", param->cfg_mtu.status);
    if (param->cfg_mtu.status != ESP_GATT_OK) {
      ESP_LOGE(GATTC_TAG, "Config mtu failed");
    }
    ESP_LOGI(GATTC_TAG, "Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu,
	     param->cfg_mtu.conn_id);
    uint16_t handle_antidos = 0x002a;
    uint8_t antidos_code[] = "011i3";
    esp_err_t write1 = esp_ble_gattc_write_char(gattc_if,
						gl_profile_tab[PROFILE_A_APP_ID].conn_id,
						handle_antidos,
						strlen((char *) antidos_code), antidos_code,
						ESP_GATT_WRITE_TYPE_RSP,
						ESP_GATT_AUTH_REQ_NONE);
    if (write1 != ESP_GATT_OK)
      ESP_LOGE(GATTC_TAG, "Error writing antidos");
    break;

  case ESP_GATTC_WRITE_CHAR_EVT:
    if (p_data->write.status != ESP_GATT_OK) {
      ESP_LOGE(GATTC_TAG, "write char failed, error status = %x", p_data->write.status);
    } else {
      /* ESP_LOGI(GATTC_TAG, "write char success"); */
    }
    switch (program_status) {
    case CONNECTION:
      program_status = ANTIDOS;
      ESP_LOGI(GATTC_TAG, "Antidos sent. Sending now txpower");
      uint16_t handle_txpower = 0x0017;
      uint8_t txpower_code = 0x07;
      esp_ble_gattc_write_char(gattc_if,
			       gl_profile_tab[PROFILE_A_APP_ID].conn_id,
			       handle_txpower, 1, &txpower_code, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
      break;
    case ANTIDOS:
      program_status = WAKECPU;
      ESP_LOGI(GATTC_TAG, "TxPower sent. Sending now wakeCPU");
      uint16_t handle_wakecpu = 0x002f;
      uint8_t wakecpu_code = 0x01;
      esp_ble_gattc_write_char(gattc_if,
			       gl_profile_tab[PROFILE_A_APP_ID].conn_id,
			       handle_wakecpu, 1, &wakecpu_code, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
      break;
    case WAKECPU:
      program_status = READY;
      ESP_LOGI(GATTC_TAG, "WakeCPU sent. BB8 ready");
      esp_ble_gattc_register_for_notify(gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, 0x0010);
      break;
    default:
      break;
    }
    break;

  case ESP_GATTC_REG_FOR_NOTIFY_EVT:
    if (p_data->reg_for_notify.status != ESP_GATT_OK)
      ESP_LOGE(GATTC_TAG, "reg notify failed, error status =%x", p_data->reg_for_notify.status);
    break;

  case ESP_GATTC_NOTIFY_EVT:
    ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, Receive notify value:");
    esp_log_buffer_hex(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);
    break;

  case ESP_GATTC_DISCONNECT_EVT:
    ESP_LOGI(GATTC_TAG, "DISCONNECT_EVT");
    break;

  default:
    break;
  }
}


static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t * param)
{
  ESP_LOGI(GATTC_TAG, "esp_gap_cb event: %d", event);

  switch (event) {
  case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
    ESP_LOGI(GATTC_TAG, "SCAN_PARAM_SET_COMPLETE_EVT. starting scanning");
    uint32_t duration = 30;
    esp_ble_gap_start_scanning(duration);
    break;

  case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
    //scan start complete event to indicate scan start successfully or failed
    if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
      ESP_LOGI(GATTC_TAG, "Scan start success");
    } else {
      ESP_LOGE(GATTC_TAG, "Scan start failed");
    }
    break;

  case ESP_GAP_BLE_SCAN_RESULT_EVT:
    ESP_LOGI(GATTC_TAG, "SCAN_RESULTE_EVT");
    esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *) param;

    if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
      esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);

      if (check_if_bb8(scan_result->scan_rst)) {
	ESP_LOGI(GATTC_TAG, "BB8 !!!");
	if (Isconnecting) {
	  break;
	}
	Isconnecting = true;
	esp_ble_gap_stop_scanning();
	esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda,
			   scan_result->scan_rst.ble_addr_type, true);
      }
    }

    break;

  case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
    ESP_LOGI(GATTC_TAG, "SCAN_STOP_COMPLETE_EVT");
    break;


  default:
    break;
  }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t * param)
{
  /* ESP_LOGI(GATTC_TAG, "gattc_cb: EVT %d, gattc if %d, app_id %d", event, gattc_if, param->reg.app_id); */

  /* If event is register event, store the gattc_if for each profile */
  if (event == ESP_GATTC_REG_EVT) {
    if (param->reg.status == ESP_GATT_OK) {
      ESP_LOGI(GATTC_TAG, "App %d registered to interface %d", param->reg.app_id, gattc_if);
      gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
    } else {
      ESP_LOGI(GATTC_TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
      return;
    }
  }

  /* If the gattc_if equal to profile A, call profile A cb handler,
   * so here call each profile's callback */
  do {
    int idx;
    for (idx = 0; idx < PROFILE_NUM; idx++) {
      if (gattc_if == ESP_GATT_IF_NONE ||	/* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
	  gattc_if == gl_profile_tab[idx].gattc_if) {
	if (gl_profile_tab[idx].gattc_cb) {
	  /* ESP_LOGI(GATTC_TAG, "Calling gattc_cb for app %d", idx); */
	  gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
	}
      }
    }
  } while (0);
}

uint8_t seq = 0;

void send_command(uint8_t did, uint8_t cid, uint8_t data_length, uint8_t * data)
{
  uint8_t sop1 = 0xff;
  uint8_t sop2 = 0xff;
  uint16_t handle = 0x000e;
  uint8_t packet_length = data_length + 7;
  uint8_t chk = 0;
  uint8_t *packet;
  packet = malloc(packet_length);
  packet[0] = sop1;
  packet[1] = sop2;
  packet[2] = did;
  packet[3] = cid;
  packet[4] = seq;
  packet[5] = data_length + 1;
  for (uint8_t ii = 0; ii < data_length; ii++)
    packet[6 + ii] = data[ii];

  for (uint8_t ii = 2; ii < 6 + data_length; ii++)
    chk += packet[ii];

  packet[packet_length - 1] = chk ^ 0xff;
  /* ESP_LOGI(GATTC_TAG, "Sent Packet:"); */
  /* esp_log_buffer_hex(GATTC_TAG, packet, packet_length); */

  esp_err_t write = esp_ble_gattc_write_char(gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
					     gl_profile_tab[PROFILE_A_APP_ID].conn_id,
					     handle,
					     packet_length, packet,
					     ESP_GATT_WRITE_TYPE_RSP,
					     ESP_GATT_AUTH_REQ_NONE);
  if (write != ESP_GATT_OK)
    ESP_LOGE(GATTC_TAG, "Error writing command");
  free(packet);
  seq++;
}


void run_bb8()
{
  uint16_t heading = 0;
  uint8_t speed = 0;
  bool show_must_go_on = true;
  uint8_t go = 0x01;
  uint8_t inc_speed = 0x10;
  uint16_t inc_heading = 10;

  odroid_gamepad_state previousState;
  input_read(&previousState);

  while (show_must_go_on) {
    odroid_gamepad_state state;
    input_read(&state);
    uint8_t new_speed;
    uint16_t new_heading;

    new_speed = speed;
    new_heading = heading;

    if (state.values[ODROID_INPUT_RIGHT])
      new_heading = (heading + inc_heading) % 360;

    if (state.values[ODROID_INPUT_LEFT]) {
      if (heading < inc_heading)
	new_heading = 360 + heading - inc_heading;
      else
	new_heading -= inc_heading;
    }

    if (state.values[ODROID_INPUT_A]) {
      if (speed > 0xff - inc_speed)
	new_speed = 0xff;
      else
	new_speed += inc_speed;
    }

    if (state.values[ODROID_INPUT_B]) {
      if (speed < inc_speed)
	new_speed = 0;
      else
	new_speed -= inc_speed;
    }

    if (state.values[ODROID_INPUT_DOWN])
      new_speed = 0;

    /* if(!previousState.values[ODROID_INPUT_UP] && state.values[ODROID_INPUT_UP]) */
    /*        new_heading = (heading + 180) % 360; */

    if (!previousState.values[ODROID_INPUT_SELECT] && state.values[ODROID_INPUT_SELECT]) {
      if (go != 0x02) {
	go = 0x02;
#if CONFIG_BB8_GUI
	display_color = C_RED;
#endif
	// Red
	{
	  uint8_t data[] = { 0xff, 0x00, 0x00, 0x00 };
	  send_command(0x02, 0x20, 4, data);
	}
      } else {
	go = 0x01;
#if CONFIG_BB8_GUI
	display_color = C_GREEN;
#endif
	// Green
	{
	  uint8_t data[] = { 0x00, 0xff, 0x00, 0x00 };
	  send_command(0x02, 0x20, 4, data);
	}

      }
    }


    if (state.values[ODROID_INPUT_MENU]) {
      new_speed = 0;
      show_must_go_on = false;
    }

    if ((new_speed != speed) || (new_heading != heading)) {
      ESP_LOGI(GATTC_TAG, "speed: %hhu->%hhu - heading: %hu->%hu - go:%hhu",
	       speed, new_speed, heading, new_heading, go);

      uint8_t data[] = { new_speed, new_heading >> 8, new_heading & 0xff, 0x00 };

      if (new_speed >= speed)
	data[3] = go;

      send_command(0x02, 0x30, 4, data);
#if CONFIG_BB8_GUI
      display_speed = new_speed;
      display_angle = new_heading;
#endif
    }

    previousState = state;
    vTaskDelay(50 / portTICK_PERIOD_MS);
    speed = new_speed;
    heading = new_heading;
  }
}

void test_input()
{
  odroid_gamepad_state previousState;
  input_read(&previousState);

  while (true) {
    odroid_gamepad_state state;
    input_read(&state);
    if (state.values[ODROID_INPUT_DOWN])
      ESP_LOGI(GATTC_TAG, "DOWN");

    if (!previousState.values[ODROID_INPUT_UP] && state.values[ODROID_INPUT_UP])
      ESP_LOGI(GATTC_TAG, "UP");

    if (!previousState.values[ODROID_INPUT_LEFT] && state.values[ODROID_INPUT_LEFT])
      ESP_LOGI(GATTC_TAG, "LEFT");

    if (!previousState.values[ODROID_INPUT_RIGHT] && state.values[ODROID_INPUT_RIGHT])
      ESP_LOGI(GATTC_TAG, "RIGHT");

    if (!previousState.values[ODROID_INPUT_START] && state.values[ODROID_INPUT_START]) {
      ESP_LOGI(GATTC_TAG, "START");
      break;
    }

    if (!previousState.values[ODROID_INPUT_SELECT] && state.values[ODROID_INPUT_SELECT])
      ESP_LOGI(GATTC_TAG, "SELECT");

    if (!previousState.values[ODROID_INPUT_VOLUME] && state.values[ODROID_INPUT_VOLUME])
      ESP_LOGI(GATTC_TAG, "VOLUME");

    if (!previousState.values[ODROID_INPUT_A] && state.values[ODROID_INPUT_A])
      ESP_LOGI(GATTC_TAG, "A");

    if (!previousState.values[ODROID_INPUT_B] && state.values[ODROID_INPUT_B])
      ESP_LOGI(GATTC_TAG, "B");

    if (!previousState.values[ODROID_INPUT_MENU] && state.values[ODROID_INPUT_MENU]) {
      ESP_LOGI(GATTC_TAG, "MENU");
    }

    previousState = state;
    vTaskDelay(5 / portTICK_PERIOD_MS);

  }
}


void app_main()
{
#if CONFIG_STATIC_MAC
  init_mac();
#endif

  input_init();

  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

#if CONFIG_BB8_GUI
  fb = malloc(320 * 240 * sizeof(uint16_t));
  if (fb == NULL)
    ESP_LOGE(GATTC_TAG, "Memory allocation error, please configure SPIRAM");

  ili9341_init();
  ili9341_clear(0xffff);

  UG_Init(&gui, pset, 320, 240);

  {
    ui_draw_title();

    print_message("Press [start] to connect to BB8");

    ui_update_display();
  }
#endif

  test_input();

#if CONFIG_BB8_GUI
  print_message("Initialisation...");
  ui_update_display();
#endif

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
    ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bluedroid_init();
  if (ret) {
    ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bluedroid_enable();
  if (ret) {
    ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }
  //register the  callback function to the gap module
  ret = esp_ble_gap_register_callback(esp_gap_cb);
  if (ret) {
    ESP_LOGE(GATTC_TAG, "gap register error, error code = %x", ret);
    return;
  }
  //register the callback function to the gattc module
  ret = esp_ble_gattc_register_callback(esp_gattc_cb);
  if (ret) {
    ESP_LOGE(GATTC_TAG, "gattc register error, error code = %x", ret);
    return;
  }

  ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
  if (ret) {
    ESP_LOGE(GATTC_TAG, "gattc app register error, error code = %x", ret);
    return;
  }

  ret = esp_ble_gatt_set_local_mtu(200);
  if (ret) {
    ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", ret);
  }
  program_status = CONNECTION;

#if CONFIG_BB8_GUI
  print_message("Connecting...");
  ui_update_display();
#endif

  while (1) {
    ESP_LOGI(GATTC_TAG, "program_status: %d", program_status);
    if (program_status == READY)
      break;
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

  ESP_LOGI(GATTC_TAG, "BB8 ready to start!");
  for (uint8_t ii = 0; ii < 2; ii++) {
    ESP_LOGI(GATTC_TAG, "Ping.");
    send_command(0x00, 0x01, 0, NULL);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }


  ESP_LOGI(GATTC_TAG, "Stabilisation");
  {
    uint8_t data[] = { 0x01 };
    send_command(0x02, 0x02, 1, data);
  }
  vTaskDelay(100 / portTICK_PERIOD_MS);

  ESP_LOGI(GATTC_TAG, "Collision detection");
  {
    uint8_t data[] = { 0x01, 100, 1, 100, 1, 100 };
    send_command(0x02, 0x12, 6, data);
  }

  vTaskDelay(100 / portTICK_PERIOD_MS);
  // Green
  {
    uint8_t data[] = { 0x00, 0xff, 0x00, 0x00 };
    send_command(0x02, 0x20, 4, data);
  }

  // turn LED on
  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_2, 1);

#if CONFIG_BB8_GUI
  print_message("");
  const esp_timer_create_args_t periodic_timer_args = {
    .callback = &DisplaySpeed,
    .name = "periodic screen update"
  };

  esp_timer_handle_t periodic_timer;
  ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));

  ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 100000));

#endif


  run_bb8();

  // Turn off stabilisation
  {
    uint8_t data[] = { 0x00 };
    send_command(0x02, 0x02, 1, data);
  }
  vTaskDelay(200 / portTICK_PERIOD_MS);

  // Sleep
  {
    uint8_t data[] = { 0x00, 0x05, 0x00, 0x00, 0x00 };
    send_command(0x00, 0x22, 5, data);
  }
  vTaskDelay(200 / portTICK_PERIOD_MS);

  // turn LED off
  // gpio_set_level(GPIO_NUM_2, 0);

  // turn screen off;
#if CONFIG_BB8_GUI
  ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
  backlight_deinit();
#endif

  // Stop Bluetooth
  ESP_ERROR_CHECK(esp_ble_gattc_close(gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
				      gl_profile_tab[PROFILE_A_APP_ID].conn_id));
  
  ESP_LOGI(GATTC_TAG, "stop");
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  ESP_ERROR_CHECK(esp_ble_gap_disconnect(gl_profile_tab[PROFILE_A_APP_ID].remote_bda));

  ESP_LOGI(GATTC_TAG, "stop.");
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  return;

}
