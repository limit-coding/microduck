#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "duck_core.h"
#include "duck_display.h"
#include "mosaico_imu.h"
#include "mosaico_power.h"
#include "policy_runtime.h"

#define FEEDBACK_TIMEOUT_US 250000
#define COMMAND_TIMEOUT_US 750000
#define CONTROL_PERIOD_US 20000
#define IP_UART UART_NUM_0
#define ROBOT_UART UART_NUM_1
static const char *TAG="s31_duck";
static SemaphoreHandle_t lock;
static struct {
    bool running, wifi, model_ready;
    uint32_t epoch, frames, steps, overruns;
    int64_t last_frame, last_cmd;
    float twist[3], head[4], infer_ms, max_ms, policy_ms, tx_ms, control_hz;
    policy_runtime_stats_t policy;
    int battery;
    char ip[16], reason[80];
} shared;
static void take(void) { xSemaphoreTake(lock,portMAX_DELAY); }
static void give(void) { xSemaphoreGive(lock); }
static void init_uart(uart_port_t port, int tx, int rx, int rx_buffer_size) {
    uart_config_t cfg={.baud_rate=115200,.data_bits=UART_DATA_8_BITS,.parity=UART_PARITY_DISABLE,
        .stop_bits=UART_STOP_BITS_1,.flow_ctrl=UART_HW_FLOWCTRL_DISABLE,.source_clk=UART_SCLK_DEFAULT};
    ESP_ERROR_CHECK(uart_param_config(port,&cfg));
    ESP_ERROR_CHECK(uart_set_pin(port,tx,rx,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(port,rx_buffer_size,0,0,NULL,0));
}
/* Independent of robot baud negotiation, feedback and inference. */
static void ip_report_task(void *arg) {
    (void)arg;
    init_uart(IP_UART,CONFIG_DUCK_IP_UART_TX,CONFIG_DUCK_IP_UART_RX,256);
    TickType_t next=xTaskGetTickCount();
    for(;;) {
        char ip[16],message[112];
        take();
        bool connected=shared.wifi;
        snprintf(ip,sizeof(ip),"%s",connected?shared.ip:"0.0.0.0");
        give();
        int n;
        if(connected) n=snprintf(message,sizeof(message),"WIFI=CONNECTED DUCK_IP=%s URL=http://%s:%d/\r\n",ip,ip,CONFIG_DUCK_WEB_PORT);
        else n=snprintf(message,sizeof(message),"WIFI=CONNECTING DUCK_IP=0.0.0.0\r\n");
        uart_write_bytes(IP_UART,message,n);
        if(n>2) ESP_LOGI(TAG,"%.*s",n-2,message);
        uart_flush_input(IP_UART);
        xTaskDelayUntil(&next,pdMS_TO_TICKS(1000));
    }
}
/* Caller holds lock. A mode transition invalidates any inference in flight. */
static void home_locked(const char *reason) {
    shared.running=false; shared.epoch++;
    memset(shared.twist,0,sizeof(shared.twist)); memset(shared.head,0,sizeof(shared.head));
    snprintf(shared.reason,sizeof(shared.reason),"%s",reason);
}
static bool send_targets(const float *q) {
    uint8_t frame[83]; size_t n=duck_angles_frame(q,frame);
    return uart_write_bytes(ROBOT_UART,frame,n)==(int)n;
}
static void send_setting(uint8_t major,uint8_t value) {
    uint8_t p[2]={1,value},frame[10];
    size_t n=duck_write(major,p,2,frame); uart_write_bytes(ROBOT_UART,frame,n);
}
static void switch_to_run_baud(void) {
    send_setting(5,5);
    ESP_ERROR_CHECK(uart_wait_tx_done(ROBOT_UART,pdMS_TO_TICKS(100)));
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_ERROR_CHECK(uart_set_baudrate(ROBOT_UART,2000000));
    ESP_ERROR_CHECK(uart_flush_input(ROBOT_UART));
    send_setting(3,1); send_targets(duck_default);
}
static void control_task(void *arg) {
    (void)arg;
    init_uart(ROBOT_UART,CONFIG_DUCK_ROBOT_UART_TX,CONFIG_DUCK_ROBOT_UART_RX,8192);
    static float scratch[1024], obs[61], prev[14], action[14], targets[15];
    static duck_output_filter_t output_filter;
    switch_to_run_baud();
    policy_runtime_stats_t stats;
    bool ready=policy_runtime_init(scratch,&stats);
    ESP_ERROR_CHECK(uart_flush_input(ROBOT_UART));
    take();shared.model_ready=ready;shared.policy=stats;
    if(!ready) snprintf(shared.reason,sizeof(shared.reason),"model self-test failed");
    give();
    static duck_parser_t parser;
    static duck_filter_t filter;
    static duck_state_t state, batch[32];
    uint8_t rx[2048]; uint32_t epoch=0;
    int retry=0; int64_t last_received=0,next_ctrl=0,next_retry=esp_timer_get_time()+1000000;
    int64_t rate_started=esp_timer_get_time();uint32_t rate_steps=0;
    for(;;) {
        int n=uart_read_bytes(ROBOT_UART,rx,sizeof(rx),0),count=0;
        for(int i=0;i<n;i++) if(duck_parse_byte(&parser,rx[i],&batch[count])) count++;
        int64_t now=esp_timer_get_time();
        if(count) {
            float dt=last_received ? (now-last_received)*1e-6f/count : 0.01f;
            for(int i=0;i<count;i++) duck_filter(&filter,&batch[i],dt);
            state=batch[count-1];last_received=now;retry=0;
            take();shared.last_frame=now;shared.frames+=count;shared.battery=state.battery;give();
        }
        float twist[3],head[4]; bool run;
        take();
        if(shared.running && (!shared.wifi || !last_received || now-last_received>FEEDBACK_TIMEOUT_US)) home_locked("feedback or Wi-Fi disconnected");
        if(shared.running && now-shared.last_cmd>COMMAND_TIMEOUT_US) home_locked("web control timed out");
        if(epoch!=shared.epoch) { epoch=shared.epoch;memset(prev,0,sizeof(prev));output_filter.initialized=false;next_ctrl=0;rate_started=now;rate_steps=0;shared.control_hz=0; }
        run=shared.running;memcpy(twist,shared.twist,sizeof(twist));memcpy(head,shared.head,sizeof(head));
        give();
        if(now>=next_ctrl) {
            if(next_ctrl==0) next_ctrl=now;
            int64_t started=esp_timer_get_time();bool ok=true;
            float policy_ms=0;
            if(run) {
                duck_observation(&state,state.gyro_rad,state.gravity,prev,twist,head,obs);
                int64_t policy_started=esp_timer_get_time();
                ok=policy_runtime_run(obs,action,scratch);
                policy_ms=(esp_timer_get_time()-policy_started)*0.001f;
                if(ok) {
                    memcpy(prev,action,sizeof(prev));
                    duck_targets(action,targets);
                    for(int i=0;i<15;i++) if(!isfinite(targets[i])) ok=false;
                    if(ok) duck_filter_targets(&output_filter,targets);
                }
            }
            take();
            int64_t checked=esp_timer_get_time();
            if(run && (!ok || checked-shared.last_frame>FEEDBACK_TIMEOUT_US)) home_locked("invalid policy output or stale feedback");
            if(run && checked-shared.last_cmd>COMMAND_TIMEOUT_US) home_locked("web control timed out");
            bool active=run && shared.running && epoch==shared.epoch;
            int64_t tx_started=esp_timer_get_time();
            if(!send_targets(active?targets:duck_default)) home_locked("UART write failed");
            float tx_ms=(esp_timer_get_time()-tx_started)*0.001f;
            float ms=(esp_timer_get_time()-started)*0.001f;
            if(active) {shared.steps++;rate_steps++;shared.infer_ms=ms;shared.policy_ms=policy_ms;shared.tx_ms=tx_ms;if(ms>shared.max_ms) shared.max_ms=ms;}
            give();
            int64_t finished=esp_timer_get_time();
            next_ctrl+=CONTROL_PERIOD_US;
            if(next_ctrl<=finished) {
                take();if(active) shared.overruns++;give();
                next_ctrl+=((finished-next_ctrl)/CONTROL_PERIOD_US+1)*CONTROL_PERIOD_US;
            }
        }
        if(now-rate_started>=1000000) {
            take();shared.control_hz=rate_steps*1000000.0f/(now-rate_started);give();
            rate_steps=0;rate_started=now;
        }
        if((!last_received || now-last_received>500000) && now>=next_retry) {
            /* Probe 2M first (S31-only restart); then 115200 (peer power cycle). */
            take();home_locked("waiting for robot feedback");give();
            if((retry++%2)==1) {
                ESP_ERROR_CHECK(uart_wait_tx_done(ROBOT_UART,pdMS_TO_TICKS(100)));
                ESP_ERROR_CHECK(uart_set_baudrate(ROBOT_UART,115200));
                switch_to_run_baud();
                memset(&parser,0,sizeof(parser));memset(&filter,0,sizeof(filter));
            } else send_setting(3,1);
            next_retry=esp_timer_get_time()+1000000;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
static esp_err_t json_reply(httpd_req_t *req,const char *status,const char *body) {
    httpd_resp_set_status(req,status);httpd_resp_set_type(req,"application/json");
    httpd_resp_set_hdr(req,"Cache-Control","no-store");return httpd_resp_sendstr(req,body);
}
static esp_err_t page(httpd_req_t *req) {
    extern const char html_start[] asm("_binary_index_html_start");
    extern const char html_end[] asm("_binary_index_html_end");
    httpd_resp_set_type(req,"text/html; charset=utf-8");httpd_resp_set_hdr(req,"Cache-Control","no-store");
    return httpd_resp_send(req,html_start,html_end-html_start-1);
}
static esp_err_t status(httpd_req_t *req) {
    char body[1100];
    mosaico_imu_snapshot_t imu; mosaico_imu_get(&imu);
    take();
    int64_t now=esp_timer_get_time();
    long age=shared.last_frame?(long)((now-shared.last_frame)/1000):-1;
    snprintf(body,sizeof(body),"{\"imu_ok\":%s,\"running\":%s,\"wifi\":%s,\"model_ready\":%s,\"feedback_ok\":%s,\"ip\":\"%s\",\"feedback_age_ms\":%ld,\"battery\":%d,\"frames\":%"PRIu32",\"steps\":%"PRIu32",\"infer_ms\":%.3f,\"max_ms\":%.3f,\"overruns\":%"PRIu32",\"policy_ms\":%.3f,\"tx_ms\":%.3f,\"control_hz\":%.2f,\"weights_psram\":%s,\"boot_flash_ms\":%.3f,\"boot_ram_scalar_ms\":%.3f,\"boot_optimized_ms\":%.3f,\"boot_max_error\":%.8f,\"reason\":\"%s\"}",
        mosaico_imu_fresh(&imu,now)?"true":"false",shared.running?"true":"false",shared.wifi?"true":"false",shared.model_ready?"true":"false",
        age>=0 && age<=250?"true":"false",shared.ip,age,shared.battery,shared.frames,shared.steps,shared.infer_ms,shared.max_ms,shared.overruns,
        shared.policy_ms,shared.tx_ms,shared.control_hz,shared.policy.psram?"true":"false",
        shared.policy.flash_ms,shared.policy.ram_scalar_ms,shared.policy.optimized_ms,shared.policy.max_error,shared.reason);
    give();return json_reply(req,"200 OK",body);
}
static esp_err_t imu_status(httpd_req_t *req) {
    mosaico_imu_snapshot_t s;
    mosaico_imu_get(&s);
    int64_t age=s.samples ? (esp_timer_get_time()-s.sample_us)/1000 : -1;
    bool valid=s.ready && s.error==ESP_OK && age>=0 && age<=100;
    char body[1100];
    snprintf(body,sizeof(body),
        "{\"ready\":%s,\"valid\":%s,\"chip_id\":%u,\"age_ms\":%lld,"
        "\"samples\":%"PRIu32",\"errors\":%"PRIu32",\"error\":\"%s\","
        "\"acc_raw\":[%d,%d,%d],\"gyro_raw\":[%d,%d,%d],\"sensor_time\":%"PRIu32","
        "\"odr_hz\":100,\"acc_range_g\":4,\"gyro_range_dps\":2000,\"estimate_valid\":%s,"
        "\"acc_g\":[%.6f,%.6f,%.6f],\"gyro_rad\":[%.6f,%.6f,%.6f],\"gravity\":[%.6f,%.6f,%.6f]}",
        s.ready?"true":"false",valid?"true":"false",s.chip_id,(long long)age,
        s.samples,s.errors,esp_err_to_name(s.error),s.acc[0],s.acc[1],s.acc[2],
        s.gyro[0],s.gyro[1],s.gyro[2],s.sensor_time,mosaico_imu_fresh(&s,esp_timer_get_time())?"true":"false",
        s.acc_g[0],s.acc_g[1],s.acc_g[2],s.gyro_rad[0],s.gyro_rad[1],s.gyro_rad[2],s.gravity[0],s.gravity[1],s.gravity[2]);
    return json_reply(req,"200 OK",body);
}
static esp_err_t start(httpd_req_t *req) {
    take();
    int64_t now=esp_timer_get_time();
    bool allowed=shared.wifi && shared.model_ready && shared.last_frame && now-shared.last_frame<=FEEDBACK_TIMEOUT_US;
    if(allowed && !shared.running) {
        memset(shared.twist,0,sizeof(shared.twist));memset(shared.head,0,sizeof(shared.head));
        shared.running=true;shared.epoch++;shared.last_cmd=now;shared.reason[0]=0;
        shared.steps=shared.overruns=0;shared.infer_ms=shared.max_ms=shared.policy_ms=shared.tx_ms=shared.control_hz=0;
    }
    give();return json_reply(req,allowed?"200 OK":"409 Conflict",allowed?"{\"ok\":true}":"{\"error\":\"Wi-Fi, model or fresh robot feedback unavailable\"}");
}
static esp_err_t stop(httpd_req_t *req) {
    take();home_locked("default posture selected");give();return json_reply(req,"200 OK","{\"ok\":true}");
}
static bool array_values(cJSON *root,const char *key,int n,const float *limits,float *out) {
    cJSON *a=cJSON_GetObjectItemCaseSensitive(root,key);
    if(!cJSON_IsArray(a) || cJSON_GetArraySize(a)!=n) return false;
    for(int i=0;i<n;i++) {
        cJSON *v=cJSON_GetArrayItem(a,i);
        if(!cJSON_IsNumber(v) || !isfinite(v->valuedouble) || fabs(v->valuedouble)>limits[i]+1e-6) return false;
        out[i]=(float)v->valuedouble;
    }
    return true;
}
static esp_err_t command(httpd_req_t *req) {
    char body[512];int pos=0;
    if(req->content_len<=0 || req->content_len>=sizeof(body)) return json_reply(req,"400 Bad Request","{\"error\":\"invalid body length\"}");
    while(pos<req->content_len) {
        int n=httpd_req_recv(req,body+pos,req->content_len-pos);
        if(n<=0) return ESP_FAIL;
        pos+=n;
    }
    body[pos]=0;const char *end=NULL;
    cJSON *root=cJSON_ParseWithOpts(body,&end,true);
    float t[3],h[4];const float tl[3]={0.4f,0.3f,1},hl[4]={1,1,1,1};
    bool valid=cJSON_IsObject(root) && array_values(root,"twist",3,tl,t) && array_values(root,"head",4,hl,h);
    cJSON_Delete(root);
    if(!valid) return json_reply(req,"400 Bad Request","{\"error\":\"invalid twist/head\"}");
    take();memcpy(shared.twist,t,sizeof(t));memcpy(shared.head,h,sizeof(h));shared.last_cmd=esp_timer_get_time();give();
    return json_reply(req,"200 OK","{\"ok\":true}");
}
static void wifi_event(void *arg,esp_event_base_t base,int32_t id,void *data) {
    (void)arg;
    if(base==WIFI_EVENT && id==WIFI_EVENT_STA_START) esp_wifi_connect();
    else if(base==WIFI_EVENT && id==WIFI_EVENT_STA_DISCONNECTED) {
        take();shared.wifi=false;home_locked("Wi-Fi disconnected");give();esp_wifi_connect();
    } else if(base==IP_EVENT && id==IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev=data;
        take();shared.wifi=true;snprintf(shared.ip,sizeof(shared.ip),IPSTR,IP2STR(&ev->ip_info.ip));give();
    }
}
void app_main(void) {
    lock=xSemaphoreCreateMutex();configASSERT(lock);
    snprintf(shared.reason,sizeof(shared.reason),"boot: default posture");
    esp_err_t power_err=mosaico_power_enable();
    if(power_err==ESP_OK) {
        esp_err_t display_err=duck_display_show_static();
        if(display_err!=ESP_OK) ESP_LOGW(TAG,"static display failed: %s",esp_err_to_name(display_err));
    } else ESP_LOGW(TAG,"board power enable failed before display: %s",esp_err_to_name(power_err));
    mosaico_imu_start();
    configASSERT(xTaskCreate(ip_report_task,"wifi_ip_report",3072,NULL,4,NULL)==pdPASS);
    configASSERT(xTaskCreatePinnedToCore(control_task,"duck_control",8192,NULL,8,NULL,CONFIG_FREERTOS_NUMBER_OF_CORES-1)==pdPASS);
    esp_err_t err=nvs_flash_init();
    if(err==ESP_ERR_NVS_NO_FREE_PAGES || err==ESP_ERR_NVS_NEW_VERSION_FOUND) {ESP_ERROR_CHECK(nvs_flash_erase());err=nvs_flash_init();}
    ESP_ERROR_CHECK(err);ESP_ERROR_CHECK(esp_netif_init());ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg=WIFI_INIT_CONFIG_DEFAULT();ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,wifi_event,NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,IP_EVENT_STA_GOT_IP,wifi_event,NULL));
    wifi_config_t wifi={0};
    _Static_assert(sizeof(CONFIG_DUCK_WIFI_SSID)<=33,"SSID too long");
    _Static_assert(sizeof(CONFIG_DUCK_WIFI_PASSWORD)<=64,"Password too long");
    memcpy(wifi.sta.ssid,CONFIG_DUCK_WIFI_SSID,strlen(CONFIG_DUCK_WIFI_SSID));
    memcpy(wifi.sta.password,CONFIG_DUCK_WIFI_PASSWORD,strlen(CONFIG_DUCK_WIFI_PASSWORD));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA,&wifi));ESP_ERROR_CHECK(esp_wifi_start());ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    httpd_config_t http=HTTPD_DEFAULT_CONFIG();http.server_port=CONFIG_DUCK_WEB_PORT;http.stack_size=8192;http.lru_purge_enable=true;http.recv_wait_timeout=1;
    httpd_handle_t server=NULL;ESP_ERROR_CHECK(httpd_start(&server,&http));
    const httpd_uri_t routes[]={
        {.uri="/",.method=HTTP_GET,.handler=page}, {.uri="/api/imu",.method=HTTP_GET,.handler=imu_status}, {.uri="/api/status",.method=HTTP_GET,.handler=status},
        {.uri="/api/start",.method=HTTP_POST,.handler=start}, {.uri="/api/stop",.method=HTTP_POST,.handler=stop},
        {.uri="/api/quit",.method=HTTP_POST,.handler=stop}, {.uri="/api/cmd",.method=HTTP_POST,.handler=command}};
    for(size_t i=0;i<sizeof(routes)/sizeof(routes[0]);i++) ESP_ERROR_CHECK(httpd_register_uri_handler(server,&routes[i]));
}
