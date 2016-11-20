#include <stdio.h>
#include <tizen.h>
#include <stdlib.h>
#include <unistd.h>
#include <rvc_api.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <service_app.h>
#include <json-glib/json-glib.h>
#include <json-glib/json-gobject.h>

#include <Elementary.h>
#include <efl_extension.h>

#include <string.h>
#include <math.h>
#include <time.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>

#include <camera.h>
#include <wav_player.h>
#include <download.h>
#include <tts.h>
#include <sound_manager.h>

#include "rvc.h"

//json packet size
#define RVC_JSON_SIZE 512

//server port number
#define RVC_SERVER_PORT 5000

#define BUFF_SIZE 512

typedef struct _msg_data {
  long data_type;
  long data_num;
  char data_buff[BUFF_SIZE];
} msg_data;

int mqid;

typedef struct
_camdata
{
    Evas_Object *win;
    Evas_Object *rect;
    Evas *evas;
    camera_h g_camera; /* Camera handle */
} camdata;

static camdata cam_data;
static bool g_enable_shot = true;
static bool g_enable_focus = true;


//json format
static const char *rvc_json_object =
"{"
  "\"mode\":%d,"
  "\"error\":%d,"
  "\"magnet\":%d,"
  "\"suction\":%d,"
  "\"battery\":%d,"
  "\"voice\":%d,"
  "\"reserve\":{"
    "\"once\":{"
      "\"on\":%d,"
      "\"hour\":%d,"
      "\"minute\":%d"
    "},"
    "\"daily\":{"
      "\"on\":%d,"
      "\"hour\":%d,"
      "\"minute\":%d"
    "}"
  "},"
  "\"wheel_vel\":{"
    "\"left\":%d,"
    "\"right\":%d"
  "},"
  "\"pose\":{"
    "\"x\":%f,"
    "\"y\":%f,"
    "\"q\":%f"
  "},"
  "\"bumper\":{"
    "\"left\":%d,"
    "\"right\":%d"
  "},"
    "\"cliff\":{"
    "\"left\":%d,"
    "\"center\":%d,"
    "\"right\":%d"
  "},"
  "\"lift\":{"
    "\"left\":%d,"
    "\"right\":%d"
  "},"
  "\"lin_ang_vel\":{"
    "\"lin\":%f,"
    "\"ang\":%f"
  "}"
"}///";

/**
* This struct has tx information.
*/
typedef struct{
	int mode;
	int error;
	int bumper_left;
	int bumper_right;
	int cliff_left;
	int cliff_center;
	int cliff_right;
	int lift_left;
	int lift_right;
	int magnet;
	int suction;
	int voice;
	int battery;
	int once_on;
	int once_hour;
	int once_minute;
	int daily_on;
	int daily_hour;
	int daily_minute;

	float pose_x;
	float pose_y;
	float pose_q;

	float lin_vel;
	float ang_vel;

	int wheel_vel_left;
	int wheel_vel_right;
}_rvc_tx_s;

/**
* This struct has instance information of application.
*/
typedef struct{
	_rvc_tx_s tx_data;

	int server_socket;
	int client_socket;

	pthread_t rx_thread;
	pthread_t tx_thread;

	int tx_run;
	int rx_run;

#ifdef _DEVICE_TEST_
	player_h player;
	camera_h camera;
#endif
}_rvc_instance_s;

/**
* This function will be called when the mode type of the rvc is changed.
*/
static void
rvc_mode_callback(rvc_mode_type_get_e mode, void* data)
{
	_rvc_instance_s* instance = (_rvc_instance_s*)data;

	if(instance == NULL){
		return;
	}

	instance->tx_data.mode = (unsigned char)mode;

	dlog_print(DLOG_DEBUG, LOG_TAG, "rvc_mode_callback = %d", mode);
}

/**
* This function will be called when error condition of the rvc is changed.
*/
static void
rvc_error_callback(rvc_device_error_type_e error, void* data)
{
	_rvc_instance_s* instance = (_rvc_instance_s*)data;

	if(instance == NULL){
		return;
	}

	instance->tx_data.error = (unsigned char)error;

	dlog_print(DLOG_DEBUG, LOG_TAG, "rvc_error_callback = %d", error);
}

/**
* This function will be called when the wheel velocity of the rvc is changed.
*/
static void
rvc_wheel_callback(signed short wheel_vel_left, signed short wheel_vel_right, void* data)
{
	_rvc_instance_s* instance = (_rvc_instance_s*)data;

	if(instance == NULL){
		return;
	}

	instance->tx_data.wheel_vel_left = wheel_vel_left;
	instance->tx_data.wheel_vel_right = wheel_vel_right;

	dlog_print(DLOG_DEBUG, LOG_TAG, "rvc_wheel_callback = %d, %d", wheel_vel_left, wheel_vel_right);
}

/**
* This function will be called when the pose of the rvc is changed.
*/
static void
rvc_pose_callback(float pose_x, float pose_y, float pose_q, void* data)
{
	_rvc_instance_s* instance = (_rvc_instance_s*)data;

	if(instance == NULL){
		return;
	}

	instance->tx_data.pose_x = pose_x;
	instance->tx_data.pose_y = pose_y;
	instance->tx_data.pose_q = pose_q;

	dlog_print(DLOG_DEBUG, LOG_TAG, "rvc_pose_callback = %f, %f, %f", pose_x, pose_y, pose_q);
}

/**
* This function will be called when the bumper sensor level of the rvc is changed.
*/
static void
rvc_bumper_callback(unsigned char bumper_left, unsigned char bumper_right, void* data)
{
	_rvc_instance_s* instance = (_rvc_instance_s*)data;

	if(instance == NULL){
		return;
	}

	instance->tx_data.bumper_left = bumper_left;
	instance->tx_data.bumper_right = bumper_right;

	dlog_print(DLOG_DEBUG, LOG_TAG, "rvc_bumper_callback = %d, %d", bumper_left, bumper_right);
}

/**
* This function will be called when the cliff value of the rvc is changed.
*/
static void
rvc_cliff_callback(unsigned char cliff_left, unsigned char cliff_center, unsigned char cliff_right, void* data)
{
	_rvc_instance_s* instance = (_rvc_instance_s*)data;

	if(instance == NULL){
		return;
	}

	instance->tx_data.cliff_left = cliff_left;
	instance->tx_data.cliff_center = cliff_center;
	instance->tx_data.cliff_right = cliff_right;

	dlog_print(DLOG_DEBUG, LOG_TAG, "rvc_cliff_callback = %d, %d, %d", cliff_left, cliff_center, cliff_right);
}

/**
* This function will be called when the lift value of the rvc is changed.
*/
static void
rvc_lift_callback(unsigned char lift_left, unsigned char lift_right, void* data)
{
	_rvc_instance_s* instance = (_rvc_instance_s*)data;

	if(instance == NULL){
		return;
	}

	instance->tx_data.lift_left = lift_left;
	instance->tx_data.lift_right = lift_right;

	dlog_print(DLOG_DEBUG, LOG_TAG, "rvc_lift_callback = %d, %d", lift_left, lift_right);
}

/**
* This function will be called when the magnetic sensor level of the rvc is changed.
*/
static void
rvc_magnet_callback(unsigned char magnet, void* data)
{
	_rvc_instance_s* instance = (_rvc_instance_s*)data;

	if(instance == NULL){
		return;
	}

	instance->tx_data.magnet =  magnet;

	dlog_print(DLOG_DEBUG, LOG_TAG, "rvc_magnet_callback = %d", magnet);
}

/**
* This function will be called when the suction level of the rvc is changed.
*/
static void
rvc_suction_callback(rvc_suction_state_e state, void* data)
{
	_rvc_instance_s* instance = (_rvc_instance_s*)data;

	if(instance == NULL){
		return;
	}

	instance->tx_data.suction = (unsigned char)state;

	dlog_print(DLOG_DEBUG, LOG_TAG, "rvc_suction_callback = %d", state);
}

/**
* This function will be called when the battery level of the rvc is changed.
*/
static void
rvc_batt_callback(rvc_batt_level_e level, void* data){
	_rvc_instance_s* instance = (_rvc_instance_s*)data;

	if(instance == NULL){
		return;
	}

	instance->tx_data.battery = (unsigned char)level;

	dlog_print(DLOG_DEBUG, LOG_TAG, "rvc_batt_callback = %d", level);
}

/**
* This function will be called when the voice type of the rvc is changed.
*/
static void
rvc_voice_callback(rvc_voice_type_e type, void* data)
{
	_rvc_instance_s* instance = (_rvc_instance_s*)data;

	if(instance == NULL){
		return;
	}

	instance->tx_data.voice = (unsigned char)type;

	dlog_print(DLOG_DEBUG, LOG_TAG, "rvc_voice_callback = %d", type);
}

/**
* This function will be called when the rvc is the low battery condition.
*/
static void
rvc_batt_low_callback(void* data)
{
	_rvc_instance_s* instance = (_rvc_instance_s*)data;

	if(instance == NULL){
		return;
	}

	dlog_print(DLOG_DEBUG, LOG_TAG, "rvc_batt_low_callback");
}

/**
* This function will be called when linear or angular velocity of the rvc is changed.
*/
static void
rvc_lin_ang_callback(float lin, float ang, void* user_data)
{
	_rvc_instance_s* instance = (_rvc_instance_s*)user_data;

	if(instance == NULL){
		return;
	}

	instance->tx_data.lin_vel = lin;
	instance->tx_data.ang_vel = ang;

	dlog_print(DLOG_DEBUG, LOG_TAG, "rvc_lin_ang_callback = %f, %f", lin, ang);
}

/**
* This function will be called when reservation state of the rvc is changed.
*/
static void
rvc_reservation_callback(rvc_reserve_type_e reserve_type, unsigned char is_on, unsigned char reserve_hh, unsigned char reserve_mm, void* user_data)
{
	_rvc_instance_s* instance = (_rvc_instance_s*)user_data;

	if(instance == NULL){
		return;
	}

	if(reserve_type == RVC_RESERVE_TYPE_ONCE){
		instance->tx_data.once_on = is_on;
		instance->tx_data.once_hour = reserve_hh;
		instance->tx_data.once_minute = reserve_mm;
	}else{
		instance->tx_data.daily_on = is_on;
		instance->tx_data.daily_hour = reserve_hh;
		instance->tx_data.daily_minute = reserve_mm;
	}

	dlog_print(DLOG_DEBUG, LOG_TAG, "rvc_reservation_callback = %d, %d, %d, %d", reserve_type, is_on, reserve_hh, reserve_mm);
}

/**
* This function registers handler which handles the rvc event.
*/
static void
rvc_register_callback(_rvc_instance_s* instance)
{
	if(instance == NULL){
		return;
	}

	rvc_set_mode_evt_cb(rvc_mode_callback, instance);
	rvc_set_error_evt_cb(rvc_error_callback, instance);
	rvc_set_wheel_vel_evt_cb(rvc_wheel_callback, instance);
	rvc_set_pose_evt_cb(rvc_pose_callback, instance);
	rvc_set_bumper_evt_cb(rvc_bumper_callback, instance);
	rvc_set_cliff_evt_cb(rvc_cliff_callback, instance);
	rvc_set_lift_evt_cb(rvc_lift_callback, instance);
	rvc_set_magnet_evt_cb(rvc_magnet_callback, instance);
	rvc_set_suction_evt_cb(rvc_suction_callback, instance);
	rvc_set_batt_evt_cb(rvc_batt_callback, instance);
	rvc_set_voice_evt_cb(rvc_voice_callback, instance);
	rvc_set_batt_low_evt_cb(rvc_batt_low_callback, instance);
	rvc_set_lin_ang_evt_cb(rvc_lin_ang_callback, instance);
	rvc_set_reservation_evt_cb(rvc_reservation_callback, instance);
}

/**
* This function is to get the robot information.
*/
static void
get_rvc_info(_rvc_instance_s* instance)
{
	if(instance == NULL){
		return;
	}

	rvc_get_mode((rvc_mode_type_get_e*)&instance->tx_data.mode);
	rvc_get_error((rvc_device_error_type_e*)&instance->tx_data.error);
	rvc_get_wheel_vel((signed short*)&instance->tx_data.wheel_vel_left, (signed short*)&instance->tx_data.wheel_vel_right);
	rvc_get_bumper((unsigned char*)&instance->tx_data.bumper_left, (unsigned char*)&instance->tx_data.bumper_right);
	rvc_get_pose(&instance->tx_data.pose_x, &instance->tx_data.pose_y, &instance->tx_data.pose_q);
	rvc_get_cliff((unsigned char*)&instance->tx_data.cliff_left, (unsigned char*)&instance->tx_data.cliff_center, (unsigned char*)&instance->tx_data.cliff_right);
	rvc_get_lift((unsigned char*)&instance->tx_data.lift_left, (unsigned char*)&instance->tx_data.lift_right);
	rvc_get_magnet((unsigned char*)&instance->tx_data.magnet);
	rvc_get_suction_state((rvc_suction_state_e *)&instance->tx_data.suction);
	rvc_get_reserve(RVC_RESERVE_TYPE_ONCE, (unsigned char*)&instance->tx_data.once_on, (unsigned char*)&instance->tx_data.once_hour, (unsigned char*)&instance->tx_data.once_minute);
	rvc_get_reserve(RVC_RESERVE_TYPE_DAILY, (unsigned char*)&instance->tx_data.daily_on, (unsigned char*)&instance->tx_data.daily_hour, (unsigned char*)&instance->tx_data.daily_minute);
	rvc_get_lin_ang_vel(&instance->tx_data.lin_vel, &instance->tx_data.ang_vel);
	rvc_get_battery_level((rvc_batt_level_e*)&instance->tx_data.battery);
	rvc_get_voice_type((rvc_voice_type_e*)&instance->tx_data.voice);
}

/**
* This function transmits the robot information to a mobile.
*/
static void*
tx_thread_run(void *data)
{
	_rvc_instance_s* instance = (_rvc_instance_s*)data;
    char msg[RVC_JSON_SIZE+1] = {0,};
    struct timeval tv = {0,};
    _rvc_tx_s* tx = &instance->tx_data;

	if(instance == NULL){
		service_app_exit();
		return NULL;
	}

    memset(&tv, 0, sizeof(struct timeval));

    tv.tv_sec = 5;

    setsockopt(instance->client_socket, SOL_SOCKET, SO_SNDTIMEO, (struct timeval *)&tv, sizeof(struct timeval));

	while(instance->tx_run){
		if(instance->client_socket == 0){
			break;
		}

		memset(msg, 0, RVC_JSON_SIZE+1);
		snprintf(msg, RVC_JSON_SIZE, rvc_json_object \
				,tx->mode \
				,tx->error \
				,tx->magnet \
				,tx->suction \
				,tx->battery \
				,tx->voice \
				,tx->once_on, tx->once_hour, tx->once_minute \
				,tx->daily_on, tx->daily_hour, tx->daily_minute \
				,tx->wheel_vel_left, tx->wheel_vel_right \
				,tx->pose_x, tx->pose_y, tx->pose_q \
				,tx->bumper_left, tx->bumper_right \
				,tx->cliff_left, tx->cliff_center, tx->cliff_right \
				,tx->lift_left, tx->lift_right \
				,tx->lin_vel, tx->ang_vel \
				);

		if(write(instance->client_socket , (char*)msg , RVC_JSON_SIZE) == -1){
		    close(instance->client_socket);
			instance->client_socket = 0;
			break;
		}else{
			dlog_print(DLOG_DEBUG, LOG_TAG, "msg = %s", msg);
			usleep(500 * 1000);
		}
/*
		float x, y, q;

		msg_data mq_msg;
		rvc_get_pose(&x, &y, &q);
		mq_msg.data_type = 1;
		mq_msg.data_num = 200;
		sprintf(mq_msg.data_buff, "pose = (%f, %f, %f)", x, y, q);
		if (-1 == msgsnd(mqid, &mq_msg, sizeof(msg_data) - sizeof(long), 0)) {
			perror("msgsnd() failed.");
		}
		*/
	}

	instance->tx_run = false;
    return NULL;
}

static void wav_play_completed (int id, void *user_data) {
	dlog_print(DLOG_DEBUG, LOG_TAG, "RVCMSG: wav play done");
}

static  void download_cb (int download_id, download_state_e state, void *user_data) {
	dlog_print(DLOG_DEBUG, LOG_TAG, "RVCMSG DOWNLOAD:%d", state);
	if (state != DOWNLOAD_STATE_COMPLETED) {
		return;
	}
	int wav_id;
	int res = wav_player_start("/tmp/voice.wav", SOUND_TYPE_MEDIA, wav_play_completed, NULL, &wav_id);
	dlog_print(DLOG_DEBUG, LOG_TAG, "RVCMSG: wav_id %d, res %d", wav_id, res);
}

static void progress_cb(int download_id, unsigned long long received, void *user_data) {
	dlog_print(DLOG_DEBUG, LOG_TAG, "RVCMSG DOWNLOAD PROG:%f", received/1000.0);
}

typedef struct {
	char *language;
	int voice_type;
} tts_voice_s;

typedef struct {
	char *text;
	int utt_id;
} tts_text_s;

static tts_h g_tts;
static GList *g_tts_voice_list = NULL;
static tts_voice_s *g_current_voice = NULL;
static tts_state_e g_current_state;
static GList *g_tts_text_list = NULL;

static void
__tts_state_changed_cb(tts_h tts, tts_state_e previous, tts_state_e current, void* user_data)
{
	dlog_print(DLOG_DEBUG, LOG_TAG, "== State is changed (%d) to (%d)", previous, current);
	if (TTS_STATE_CREATED == previous && TTS_STATE_READY == current) {

	}
	g_current_state = current;
}

static void
__tts_utterance_completed_cb(tts_h tts, int utt_id, void* user_data)
{

}

static void __supported_voice (tts_h tts, const char *language, int voice_type, void *user_data) {
	dlog_print(DLOG_DEBUG, LOG_TAG, "Lang support: %s, type: %d", language, voice_type);
}

static int
init_tts(void *ad)
{
	if (0 != tts_create(&g_tts)) {
		dlog_print(DLOG_ERROR, LOG_TAG, "Fail to tts create");
		return -1;
	}

	if (0 != tts_set_state_changed_cb(g_tts, __tts_state_changed_cb, ad)) {
		dlog_print(DLOG_ERROR, LOG_TAG, "Fail to set state changed cb");
		tts_destroy(g_tts);
		return -1;
	}

	if (0 != tts_set_utterance_completed_cb(g_tts, __tts_utterance_completed_cb, ad)) {
		dlog_print(DLOG_ERROR, LOG_TAG, "Fail to set utt completed cb");
		tts_destroy(g_tts);
		return -1;
	}

	char *language = NULL;
	int voice_type;


	tts_foreach_supported_voices (g_tts, __supported_voice, NULL);
	if (0 != tts_get_default_voice(g_tts, &language, &voice_type)) {
		dlog_print(DLOG_ERROR, LOG_TAG, "Fail to get default voice");
		tts_destroy(g_tts);
		return -1;
	}

	dlog_print(DLOG_DEBUG, LOG_TAG, "language: %s", language);
	free(language);
	language = "ko_KR";
	if (NULL != language) {
		g_current_voice = (tts_voice_s *)calloc(1, sizeof(tts_voice_s));
		if (NULL == g_current_voice) {
			dlog_print(DLOG_ERROR, LOG_TAG, "Fail to memory allocation");
		} else {
			g_current_voice->language = strdup(language);
			g_current_voice->voice_type = voice_type;
		}
	}

	if (0 != tts_prepare(g_tts)) {
		dlog_print(DLOG_ERROR, LOG_TAG, "Fail to tts prepare");
		tts_destroy(g_tts);
		return -1;
	}

	return 0;
}

static void
deinit_tts(void *ad)
{
   // Destroy TTS handle (service disconnect is included)
   if (0 != tts_destroy(g_tts))
   {
      // Do something
   }
}

static void
__tts_add_text(const char* text, const char* lang)
{
	if (NULL != text) {
		dlog_print(DLOG_DEBUG, LOG_TAG, "Add text (%s)", text);
		int utt_id;
		if (NULL != g_current_voice) {
			if (NULL != g_current_voice->language) {
				if (0 != tts_add_text(g_tts, text, lang, g_current_voice->voice_type, TTS_SPEED_AUTO, &utt_id)) {
					dlog_print(DLOG_DEBUG, LOG_TAG, "Fail to add text");
				} else {
					tts_text_s *tmp = (tts_text_s *)calloc(1, sizeof(tts_text_s));
					if (NULL == tmp) {
						dlog_print(DLOG_ERROR, LOG_TAG, "Fail to memory allocation");
					} else {
						tmp->text = strdup(text);
						tmp->utt_id = utt_id;
						g_tts_text_list = g_list_append(g_tts_text_list, tmp);
					}
				}
			}
		}
	}
}

static void
__tts_play() {
	if (TTS_STATE_READY == g_current_state || TTS_STATE_PAUSED == g_current_state) {
		if (0 != tts_play(g_tts)) {
			dlog_print(DLOG_ERROR, LOG_TAG, "Fail to start");
		}
	}
}

/**
* This function processes a JSON object from the received data.
*/
static void
parse_members(JsonObject* object, const gchar *member_name, JsonNode *member_node, gpointer user_data)
{
	_rvc_instance_s* instance = (_rvc_instance_s*)user_data;

	if(instance == NULL){
		return;
	}
	dlog_print(DLOG_DEBUG, LOG_TAG, "RVCMSG member_name:%s", member_name);
	if(g_strcmp0(member_name, "mode") == 0){
		int mode = (int)json_node_get_int(member_node);

		rvc_set_mode((rvc_mode_type_set_e)mode);
	}else if(g_strcmp0(member_name, "control") == 0){
		int control = (int)json_node_get_int(member_node);

		rvc_set_control((rvc_control_dir_e)control);
	}else if(g_strcmp0(member_name, "time") == 0){
		JsonObject* obj = json_node_get_object(member_node);
		int hour = (int)json_node_get_int(json_object_get_member(obj, "hour"));
		int minute = (int)json_node_get_int(json_object_get_member(obj, "minute"));

		rvc_set_time((unsigned char)hour, (unsigned char)minute);
	}else if(g_strcmp0(member_name, "voice") == 0){
		int voice = (int)json_node_get_int(member_node);

		rvc_set_voice((rvc_voice_type_e)voice);
	}else if(g_strcmp0(member_name, "lin_ang_vel") == 0){
		JsonObject* obj = json_node_get_object(member_node);
		float lin = (float)json_node_get_double(json_object_get_member(obj, "lin"));
		float ang = (float)json_node_get_double(json_object_get_member(obj, "ang"));

		dlog_print(DLOG_DEBUG, LOG_TAG, "RVCMSG: lin %f, ang %f", lin, ang);

		rvc_set_lin_ang(lin, ang);
	}else if(g_strcmp0(member_name, "suction") == 0){
		int suction = (int)json_node_get_int(member_node);

		rvc_set_suction_state((rvc_suction_state_e)suction);
	}else if(g_strcmp0(member_name, "wheel_vel") == 0){
		JsonObject* obj = json_node_get_object(member_node);

		int lin = (int)json_node_get_int(json_object_get_member(obj, "left"));
		int ang = (int)json_node_get_int(json_object_get_member(obj, "right"));

		rvc_set_wheel_vel((unsigned short)lin, (unsigned short)ang);
	}else if(g_strcmp0(member_name, "reserve") == 0){
		JsonObject* obj = json_node_get_object(member_node);
		int type = (int)json_node_get_int(json_object_get_member(obj, "type"));
		int on = (int)json_node_get_int(json_object_get_member(obj, "on"));
		int hour = (int)json_node_get_int(json_object_get_member(obj, "hour"));
		int minute = (int)json_node_get_int(json_object_get_member(obj, "minute"));

		if(on == 0){
			rvc_set_reserve_cancel((unsigned char)type);
		}else{
			rvc_set_reserve((unsigned char)type, (unsigned char)hour, (unsigned char)minute);
		}
	}else if(g_strcmp0(member_name, "wav_play") == 0) {
		JsonObject* obj = json_node_get_object(member_node);
		char* uri = (char *)json_node_get_string(json_object_get_member(obj, "url"));
		dlog_print(DLOG_DEBUG, LOG_TAG, "RVCMSG: URI: %s", uri);
		int down_id;
		download_create(&down_id);
		download_set_url(down_id, uri);
		download_set_destination(down_id, "/tmp");
		download_set_file_name(down_id, "voice.wav");
		download_set_state_changed_cb (down_id, download_cb, NULL);
		download_set_progress_cb(down_id, progress_cb, NULL);
		int res = download_start(down_id);
		dlog_print(DLOG_DEBUG, LOG_TAG, "RVCMSG: DOWN START: %d", res);
	} else if(g_strcmp0(member_name, "tts") == 0) {
		JsonObject* obj = json_node_get_object(member_node);
		char* text = (char *)json_node_get_string(json_object_get_member(obj, "text"));
		char* lang = (char *)json_node_get_string(json_object_get_member(obj, "lang"));
		if (lang == NULL) lang = "ko_KR";
		__tts_add_text(text, lang);
		__tts_play();

	} else if(g_strcmp0(member_name, "alarm_play") == 0) {
		JsonObject* obj = json_node_get_object(member_node);

		int wav_id;
		int res = wav_player_start("/tmp/alarm.wav", SOUND_TYPE_MEDIA, wav_play_completed, NULL, &wav_id);
		dlog_print(DLOG_DEBUG, LOG_TAG, "RVCMSG: wav_id %d, res %d", wav_id, res);
	}
}

/**
* This function processes a JSON object from the received data.
*/
static void
parse_cmd(_rvc_instance_s* instance, char* msg)
{
	JsonParser *jsonParser = NULL;
	GError *error = NULL;

	dlog_print(DLOG_DEBUG, LOG_TAG, "parse_cmd:%s", msg);

	if(instance == NULL){
		return;
	}

	jsonParser = json_parser_new();

	if(jsonParser != NULL){
		if(json_parser_load_from_data(jsonParser, msg, -1, &error)){
			JsonNode *root = NULL;
			JsonObject *object = NULL;

			root = json_parser_get_root(jsonParser);

			if (JSON_NODE_TYPE (root) == JSON_NODE_OBJECT) {
				object = json_node_get_object (root);

				if(object != NULL){
					json_object_foreach_member(object, parse_members, instance);
				}
			}
		}

		if(error != NULL){
			g_error_free (error);
		}

		if(jsonParser != NULL){
			g_object_unref(jsonParser);
		}
	}
}

/***/
static void*
rx_thread_run(void *data)
{
	_rvc_instance_s* instance = (_rvc_instance_s*)data;
	struct sockaddr_in client_addr = {0,};
	int rx_recv_size = 0;
    int status = 0;
    socklen_t client_addr_size = 0;
    char msg[RVC_JSON_SIZE+1] = {0,};

	if(instance == NULL){
		service_app_exit();
		return NULL;
	}

	while(true){
		if(listen(instance->server_socket, 5) == -1){
			service_app_exit();
			break;
		}

		client_addr_size = sizeof(client_addr);
		instance->client_socket = accept(instance->server_socket, (struct sockaddr*)&client_addr, &client_addr_size);

		if(instance->client_socket==-1){
			service_app_exit();
			break;
		}

		instance->tx_run = true;

	    if(pthread_create(&instance->tx_thread, NULL, tx_thread_run, (void*)instance) < 0){
	    	instance->tx_run = false;
	   	    dlog_print(DLOG_DEBUG, LOG_TAG, "tx_thread is failed!");
            service_app_exit();
            break;
        }

	    while(instance->tx_run == true){
	    	memset(msg, 0, RVC_JSON_SIZE);
		    rx_recv_size = read(instance->client_socket, msg, RVC_JSON_SIZE);

		    if(rx_recv_size > 0){
                parse_cmd(instance, msg);
		    }else if(rx_recv_size == -1){
			    instance->tx_run = false;

			    if(instance->client_socket!=0){
			        close(instance->client_socket);
			        instance->client_socket = 0;
			    }

			    if(instance->tx_thread != 0){
			        pthread_join(instance->tx_thread, (void **)&status);
			    }
		    }
		}
	}

    return NULL;
}

/**
* This function make a socket for the communication with the Mobile.
*/
static bool
start_server_socket(_rvc_instance_s* instance)
{
	struct sockaddr_in server_addr = {0,};

	if(instance == NULL){
		return false;
	}

	instance->server_socket = socket( PF_INET, SOCK_STREAM, 0);

	if(instance->server_socket == -1){
        dlog_print(DLOG_DEBUG, LOG_TAG, "create failed!");
	    return false;
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(RVC_SERVER_PORT);
    server_addr.sin_addr.s_addr= htonl(INADDR_ANY);

    if(bind(instance->server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr))){
    	dlog_print(DLOG_DEBUG, LOG_TAG, "bind failed!");
    	return false;
    }

    instance->rx_run = true;

    if(pthread_create(&instance->rx_thread, NULL, rx_thread_run, (void*)instance) < 0){
    	instance->rx_run = false;
   	    dlog_print(DLOG_DEBUG, LOG_TAG, "rx_thread is failed!");
   	    return false;
   	}

    return true;
}

static void
_camera_capturing_cb(camera_image_data_s* image, camera_image_data_s* postview, camera_image_data_s* thumbnail, void *user_data)
{
    dlog_print(DLOG_DEBUG, LOG_TAG, "Writing image to file");
    FILE *file = fopen("image.jpg", "w");

    if (image->data != NULL) {
        fwrite(image->data, 1, image->size, file);
    }
    fclose(file);
}

static void
_camera_completed_cb(void *user_data)
{
    int error_code = 0;

    usleep(25000);  /* Display the captured image for 0.025 seconds */

    /* Restart the camera preview */
    //error_code = camera_start_preview(cam_data.g_camera);

    g_enable_focus = true;
}

bool service_app_create(void *data)
{
	_rvc_instance_s* instance = (_rvc_instance_s*)data;

	if(instance != NULL){
		if(rvc_initialize()!=RVC_USER_ERROR_NONE){
			dlog_print(DLOG_DEBUG, LOG_TAG, "rvc_initialize is failed!");
			return false;
		}

		if(start_server_socket(instance) == false){
			rvc_deinitialize();
			dlog_print(DLOG_DEBUG, LOG_TAG, "start_server_socket is failed!");
			return false;
		}

		if (-1 == (mqid = msgget((key_t)913, IPC_CREAT | 0666))) {
			perror("msgget() failed.");
			exit(1);
		}
/*
		int error_code = 0;

		// Create the camera handle
		error_code = camera_create(CAMERA_DEVICE_CAMERA0, &cam_data.g_camera);
		if (error_code == CAMERA_ERROR_NONE) {
		    dlog_print(DLOG_INFO, LOG_TAG, "Camera: error code = %d", error_code);
		} else {
		    dlog_print(DLOG_ERROR, LOG_TAG, "Camera: error code = %d", error_code);
		}

		camera_state_e state;

		// Check the camera state after creating the camera
		error_code = camera_get_state(cam_data.g_camera, &state);
		if (error_code == CAMERA_ERROR_NONE) {
			dlog_print(DLOG_INFO, LOG_TAG, "Camera: error code = %d", error_code);
		} else {
			dlog_print(DLOG_ERROR, LOG_TAG, "Camera: error code = %d", error_code);
		}
		error_code = camera_attr_set_image_quality(cam_data.g_camera, 100);
		error_code = camera_start_preview(cam_data.g_camera);
*/
		init_tts(data);
		int max_vol;
		sound_manager_get_max_volume (SOUND_TYPE_MEDIA, &max_vol);
		dlog_print(DLOG_DEBUG, LOG_TAG, "MAX_VOL %d", max_vol);
		int res = sound_manager_set_volume (SOUND_TYPE_MEDIA, max_vol);
		dlog_print(DLOG_DEBUG, LOG_TAG, "VOL_CHANGE: %d", res);

		get_rvc_info(instance);
		rvc_register_callback(instance);
	}else{
		dlog_print(DLOG_DEBUG, LOG_TAG, "g_instance is null!");
	}

	return true;
}

void service_app_terminate(void *data)
{
	int status = 0;
	_rvc_instance_s* instance = (_rvc_instance_s*)data;

	if(instance!=NULL){
		if(instance->client_socket!=0){
		    close(instance->client_socket);
		    instance->client_socket = 0;
		}

		if(instance->server_socket!=0){
		    close(instance->server_socket);
		    instance->server_socket = 0;
		}

		if(instance->rx_thread != 0){
			instance->rx_run = false;
		    pthread_join(instance->rx_thread, (void **)&status);
		}

		if(instance->tx_thread != 0){
			instance->tx_run = false;
		    pthread_join(instance->tx_thread, (void **)&status);
		}
/*
		int error_code;
		error_code = camera_cancel_focusing(cam_data.g_camera);
		error_code = camera_stop_preview(cam_data.g_camera);
		// Unregister the camera preview callback
		error_code = camera_unset_preview_cb(cam_data.g_camera);
		// Unregister the auto-focus callback
		error_code = camera_unset_focus_changed_cb(cam_data.g_camera);
		error_code = camera_destroy(cam_data.g_camera);
		*/
	}

	dlog_print(DLOG_DEBUG, LOG_TAG, "service_app_terminate");
}

void service_app_control(app_control_h app_control, void *data)
{
    // Todo: add your code here.
}

static void
service_app_lang_changed(app_event_info_h event_info, void *user_data)
{
	/*APP_EVENT_LANGUAGE_CHANGED*/
}

static void
service_app_region_changed(app_event_info_h event_info, void *user_data)
{
	/*APP_EVENT_REGION_FORMAT_CHANGED*/
}

static void
service_app_low_battery(app_event_info_h event_info, void *user_data)
{
	/*APP_EVENT_LOW_BATTERY*/
}

static void
service_app_low_memory(app_event_info_h event_info, void *user_data)
{
	/*APP_EVENT_LOW_MEMORY*/
}

int main(int argc, char* argv[])
{
	_rvc_instance_s instance;
	memset(&instance, 0, sizeof(_rvc_instance_s));

	service_app_lifecycle_callback_s event_callback;
	app_event_handler_h handlers[5] = {NULL, };

	event_callback.create = service_app_create;
	event_callback.terminate = service_app_terminate;
	event_callback.app_control = service_app_control;

	service_app_add_event_handler(&handlers[APP_EVENT_LOW_BATTERY], APP_EVENT_LOW_BATTERY, service_app_low_battery, &instance);
	service_app_add_event_handler(&handlers[APP_EVENT_LOW_MEMORY], APP_EVENT_LOW_MEMORY, service_app_low_memory, &instance);
	service_app_add_event_handler(&handlers[APP_EVENT_LANGUAGE_CHANGED], APP_EVENT_LANGUAGE_CHANGED, service_app_lang_changed, &instance);
	service_app_add_event_handler(&handlers[APP_EVENT_REGION_FORMAT_CHANGED], APP_EVENT_REGION_FORMAT_CHANGED, service_app_region_changed, &instance);

	return service_app_main(argc, argv, &event_callback, &instance);
}
