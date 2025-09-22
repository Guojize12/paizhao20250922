#include "sd_async.h"
#include "config.h"
#include <SD.h>
#include <FS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_heap_caps.h>

#if !ASYNC_SD_ENABLE
// 关闭时提供空实现
bool sd_async_init(){ return true; }
bool sd_async_start(){ return true; }
void sd_async_stop(bool){ }
bool sd_async_on_sd_ready(){ return true; }
void sd_async_on_sd_lost(){ }
bool sd_async_submit(const char*, const uint8_t*, size_t, uint32_t){ return false; }
bool sd_async_flush(uint32_t){ return true; }
void sd_async_get_stats(SdAsyncStats& out){ memset(&out,0,sizeof(out)); }
bool sd_async_idle(){ return true; }
#else

struct PoolBlk {
  PoolBlk* next;
  size_t   cap;
  size_t   len;
  uint8_t  data[0];
};

struct Job {
  char     path[ASYNC_SD_MAX_PATH];
  PoolBlk* blk;
  bool     is_first;  // 第一块：会先 remove 旧文件
};

static QueueHandle_t  g_q = nullptr;
static TaskHandle_t   g_task = nullptr;
static SemaphoreHandle_t g_mtx = nullptr;

static PoolBlk** g_pool = nullptr;     // 便于统计
static PoolBlk*  g_free = nullptr;
static uint32_t  g_pool_total = 0;

static volatile bool g_running = false;
static volatile bool g_sd_ready = false;
static volatile uint32_t g_enq_ok = 0;
static volatile uint32_t g_enq_drop = 0;
static volatile uint32_t g_wr_ok = 0;
static volatile uint32_t g_wr_fail = 0;
static volatile uint32_t g_q_max = 0;
static volatile bool g_writer_busy = false;

static void pool_init(){
  g_pool_total = 0;
  g_free = nullptr;
  if(!g_pool){
    g_pool = (PoolBlk**)calloc(ASYNC_SD_POOL_BLOCKS, sizeof(PoolBlk*));
  }
  for(uint32_t i=0;i<ASYNC_SD_POOL_BLOCKS;i++){
    size_t alloc_size = sizeof(PoolBlk) + ASYNC_SD_POOL_BLOCK_SIZE;
    void* mem = heap_caps_malloc(alloc_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if(!mem) mem = heap_caps_malloc(alloc_size, MALLOC_CAP_8BIT);
    if(!mem){
 
      continue;
    }
    PoolBlk* b = (PoolBlk*)mem;
    b->cap = ASYNC_SD_POOL_BLOCK_SIZE;
    b->len = 0;
    b->next = g_free;
    g_free = b;
    g_pool[i] = b;
    g_pool_total++;
  }
}

static PoolBlk* pool_take(){
  PoolBlk* b = nullptr;
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  if(g_free){ b = g_free; g_free = g_free->next; b->next=nullptr; b->len=0; }
  xSemaphoreGive(g_mtx);
  return b;
}

static void pool_give(PoolBlk* b){
  if(!b) return;
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  b->next = g_free; g_free = b;
  xSemaphoreGive(g_mtx);
}

static uint32_t pool_free_count(){
  uint32_t n=0;
  xSemaphoreTake(g_mtx, portMAX_DELAY);
  for(PoolBlk* p=g_free;p;p=p->next) n++;
  xSemaphoreGive(g_mtx);
  return n;
}

static bool q_send(Job& j, uint32_t timeout_ms){
  if(!g_q) return false;
  if(xQueueSend(g_q, &j, pdMS_TO_TICKS(timeout_ms)) == pdTRUE){
    g_enq_ok++;
    uint32_t depth = uxQueueMessagesWaiting(g_q);
    if(depth > g_q_max) g_q_max = depth;
    return true;
  }else{
    g_enq_drop++;
    return false;
  }
}

static bool write_chunk(const char* path, const uint8_t* data, size_t len, bool is_first){
  if(!g_sd_ready) return false;
  if(is_first){
    // 尽可能创建一个干净文件
    SD.remove(path);
  }
  File f = SD.open(path, FILE_WRITE);
  if(!f){ return false; }
  size_t w = f.write(data, len);
  f.flush();
  f.close();
  return (w == len);
}

static void writer_task(void*){
  Job j{};
  while(g_running){
    if(xQueueReceive(g_q, &j, pdMS_TO_TICKS(100)) != pdTRUE){
      continue;
    }
    g_writer_busy = true;
    bool ok = write_chunk(j.path, j.blk->data, j.blk->len, j.is_first);
    if(ok) g_wr_ok++; else g_wr_fail++;
    pool_give(j.blk);
    g_writer_busy = false;
  }
  // drain 阶段由 stop() 控制
  vTaskDelete(nullptr);
}

bool sd_async_init(){
  if(!g_mtx) g_mtx = xSemaphoreCreateMutex();
  pool_init();
  if(!g_q) g_q = xQueueCreate(ASYNC_SD_QUEUE_LENGTH, sizeof(Job));
  return (g_mtx && g_q && g_pool_total>0);
}

bool sd_async_start(){
  if(g_task) return true;
  g_running = true;
  BaseType_t rc = xTaskCreatePinnedToCore(writer_task, "sdw",
                                          ASYNC_SD_TASK_STACK, nullptr,
                                          ASYNC_SD_TASK_PRIO, &g_task,
                                          tskNO_AFFINITY);
  return rc == pdPASS;
}

void sd_async_stop(bool drain){
  if(!g_task) return;
  if(drain){
    // 等待队列清空并且不在写
    uint32_t t0 = millis();
    while((uxQueueMessagesWaiting(g_q) > 0 || g_writer_busy) &&
          (millis() - t0 < ASYNC_SD_FLUSH_TIMEOUT_MS)){
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
  g_running = false;
  // 发送一个空唤醒
  Job j{}; xQueueSend(g_q, &j, 0);
  // 等待任务退出
  vTaskDelay(pdMS_TO_TICKS(50));
  g_task = nullptr;
}

bool sd_async_on_sd_ready(){
  g_sd_ready = true;
  return true;
}

void sd_async_on_sd_lost(){
  g_sd_ready = false;
}

bool sd_async_submit(const char* path, const uint8_t* data, size_t len, uint32_t timeout_ms){
  if(!path || !data || len==0) return false;
  if(!g_q || !g_pool_total) return false;

  // 按块切分
  size_t remain = len;
  size_t offset = 0;
  bool   first  = true;
  while(remain){
    size_t chunk = remain;
    if(chunk > ASYNC_SD_POOL_BLOCK_SIZE) chunk = ASYNC_SD_POOL_BLOCK_SIZE;

    PoolBlk* b = pool_take();
    if(!b){
      // 池空：等待或丢弃（这里等待一个超时周期再尝试）
      if(timeout_ms == 0) return false;
      vTaskDelay(pdMS_TO_TICKS(timeout_ms));
      b = pool_take();
      if(!b) return false;
    }
    memcpy(b->data, data + offset, chunk);
    b->len = chunk;

    Job j{};
    strncpy(j.path, path, ASYNC_SD_MAX_PATH-1);
    j.path[ASYNC_SD_MAX_PATH-1] = '\0';
    j.blk = b;
    j.is_first = first;
    first = false;

    if(!q_send(j, timeout_ms)){
      pool_give(b);
      return false;
    }

    offset += chunk;
    remain -= chunk;
  }
  return true;
}

bool sd_async_flush(uint32_t timeout_ms){
  uint32_t t0 = millis();
  while((uxQueueMessagesWaiting(g_q) > 0 || g_writer_busy) &&
        (millis() - t0 < timeout_ms)){
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return (uxQueueMessagesWaiting(g_q) == 0 && !g_writer_busy);
}

void sd_async_get_stats(SdAsyncStats& out){
  out.enq_ok = g_enq_ok;
  out.enq_drop = g_enq_drop;
  out.write_ok = g_wr_ok;
  out.write_fail = g_wr_fail;
  out.pool_total = g_pool_total;
  out.pool_free = pool_free_count();
  out.q_depth = g_q ? uxQueueMessagesWaiting(g_q) : 0;
  out.q_max = g_q_max;
  out.running = g_running;
  out.sd_ready = g_sd_ready;
  out.task_stack_min = g_task ? uxTaskGetStackHighWaterMark(g_task) : 0;
}

bool sd_async_idle(){
  return (uxQueueMessagesWaiting(g_q) == 0 && !g_writer_busy);
}

#endif // ASYNC_SD_ENABLE