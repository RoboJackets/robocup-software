/**
 * RoboJackets: RoboCup SSL Firmware
 *
 * Copyright (C) 2015 RoboJackets JJ
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * toc.cpp - Dynamic logging system for variables defined through macros.
 */

#include "toc.hpp"

#include "rtos.h"
#include "robot_types.hpp"
#include "fp16.hpp"
#include "CommModule.hpp"

// Maximum log payload length
#define LOG_MAX_LEN 30

/* Log packet parameters storage */
#define LOG_MAX_OPS 64
#define LOG_MAX_BLOCKS 8

#define BLOCK_ID_FREE -1

enum {
  SUBC_TOC      = 0,
  SUBC_LOG_CFG  = 1,
  SUBC_LOG_DATA = 2
};

enum {
  CMD_GET_ITEM  = 0,
  CMD_GET_INFO  = 1
};

enum {
  CTRL_CREATE_BLOCK  = 0,
  CTRL_APPEND_BLOCK  = 1,
  CTRL_DELETE_BLOCK  = 2,
  CTRL_START_BLOCK   = 3,
  CTRL_STOP_BLOCK    = 4,
  CTRL_RESET         = 5
};

static const uint8_t typeLength[] = {1, 2, 4, 1, 2, 4, 4, 2};
/*
  [LOG_UINT8]  = 1,
  [LOG_UINT16] = 2,
  [LOG_UINT32] = 4,
  [LOG_INT8]   = 1,
  [LOG_INT16]  = 2,
  [LOG_INT32]  = 4,
  [LOG_FLOAT]  = 4,
  [LOG_FP16]   = 2
};
*/

struct log_ops {
  struct log_ops* next;
  uint8_t storageType : 4;
  uint8_t logType     : 4;
  void* variable;
};

struct log_block {
  int id;
  // xTimerHandle timer;
  uint32_t timer;
  struct log_ops* ops;
};

static struct log_ops logOps[LOG_MAX_OPS];
static struct log_block logBlocks[LOG_MAX_BLOCKS];
// static xSemaphoreHandle logLock;

struct ops_setting {
  uint8_t logType;
  uint8_t id;
} __attribute__((packed));

//Private functions
static void logTOCProcess(int command);
static void logControlProcess(void);

void logRunBlock(void* arg);
// void logBlockTimed(xTimerHandle timer);

//These are set by the Linker
extern struct log_s _log_start;
extern struct log_s _log_stop;

//Pointer to the logeters list and length of it
static struct log_s* logs;
static int logsLen;
// static uint32_t logsCrc;
static int logsCount = 0;

static bool isInit = false;

// Log management functions
static int logAppendBlock(int id, struct ops_setting* settings, int len);
static int logCreateBlock(unsigned char id, struct ops_setting* settings, int len);
static int logDeleteBlock(int id);
static int logStartBlock(int id, unsigned int period);
static int logStopBlock(int id);
static void logReset();


void TOCInit(void)
{
  int i;

  if (isInit)
    return;

  logs = &_log_start;
  logsLen = &_log_stop - &_log_start;
  // logsCrc = crcSlow(logs, logsLen);

  // Big lock that protects the log datastructures
  // logLock = xSemaphoreCreateMutex();

  for (i = 0; i < logsLen; i++) {
    if (!(logs[i].type & LOG_GROUP))
      logsCount++;
  }

  //Manually free all log blocks
  for (i = 0; i < LOG_MAX_BLOCKS; i++)
    logBlocks[i].id = BLOCK_ID_FREE;

  //Init data structures and set the log subsystem in a known state
  logReset();

  // Thread toc_task(Task_Param, NULL, osPriorityBelowNormal);

  isInit = true;
}


bool TOCTest(void)
{
  return isInit;
}


void Task_TOC(void const* arg)
{
  // Store the thread's ID
  osThreadId threadID = Thread::gettid();

  // Store the passed queue pointer
  osMailQId* queueID = (osMailQId*)arg;

  LOG(OK, "TOC logging module ready! Thread ID: %u; Queue ID: %u", threadID, *queueID);

  RTP_t* pkt;

  while (1) {
    osEvent evt = osMailGet(*queueID, osWaitForever);

    if (evt.status == osEventMail) {

      pkt = (RTP_t*)evt.value.p;

      LOG(OK, "Received packet in %u", threadID);

      if (pkt->subclass == SUBC_TOC)
        logTOCProcess(pkt->payload[0]);
      else if (pkt->subclass == SUBC_LOG_CFG)
        logControlProcess();

      osMailFree(*queueID, pkt);
    }

    // Thread::yield();
  }

  osThreadTerminate(threadID);
}


void logTOCProcess(int command)
{
  int ptr = 0;
  int n = 0;
  char* group = "plop";
  RTP_t pkt;

  pkt.header_link = RTP_HEADER(RTP_PORT_LOG, SUBC_TOC, true, false);

  switch (command) {
    //Get info packet about the log implementation
    case CMD_GET_INFO:
      LOG(INF1, "Packet is TOC_GET_INFO\n");
      group = '\0';
      // pkt.header = RTP_HEADER(RTP_PORT_LOG, SUBC_TOC, true, false);
      pkt.payload_size = 8;
      pkt.payload[0] = CMD_GET_INFO;
      pkt.payload[1] = logsCount;
      pkt.payload[2] = LOG_MAX_BLOCKS;
      pkt.payload[3] = LOG_MAX_OPS;
      // memcpy(&pkt.payload[4], &logsCrc, 4);
      // CommModule::send(pkt)
      break;

    //Get log variable
    case CMD_GET_ITEM:
      LOG(INF1, "Packet is TOC_GET_ITEM Id: %d\n", pkt.payload[1]);

      for (ptr = 0; ptr < logsLen; ptr++) { //Ptr points a group
        if (logs[ptr].type & LOG_GROUP) {
          if (logs[ptr].type & LOG_START)
            group = logs[ptr].name;
          else
            group = '\0';
        } else {                      //Ptr points a variable
          if (n == pkt.payload[1])
            break;

          n++;
        }
      }

      pkt.payload[0] = CMD_GET_ITEM;

      if (ptr < logsLen) {
        LOG(INF1, "Item is \"%s\":\"%s\"\n", group, logs[ptr].name);
        // pkt.header_link = RTP_HEADER(RTP_PORT_LOG, SUBC_TOC, true, false);
        // pkt.payload[0] = CMD_GET_ITEM;
        pkt.payload[1] = n;
        pkt.payload[2] = logs[ptr].type;
        memcpy(pkt.payload + 3, group, strlen(group) + 1);
        memcpy(pkt.payload + 3 + strlen(group) + 1, logs[ptr].name, strlen(logs[ptr].name) + 1);
        pkt.payload_size = 3 + 2 + strlen(group) + strlen(logs[ptr].name);
        // CommModule::send(pkt);
      } else {
        LOG(INF1, "Index out of range!");
        // pkt.header_link = RTP_HEADER(RTP_PORT_LOG, SUBC_TOC, true, false);
        // pkt.payload[0] = CMD_GET_ITEM;
        pkt.payload_size = 1;
        // CommModule::send(pkt);
      }

      break;
  }

  CommModule::send(pkt);
}

void logControlProcess(void)
{
  int ret = 0; //ENOEXEC;
  RTP_t pkt;

  switch (pkt.payload[0]) {
    case CTRL_CREATE_BLOCK:
      ret = logCreateBlock(pkt.payload[1],
                           (struct ops_setting*)&pkt.payload[2],
                           (pkt.payload_size - 2) / sizeof(struct ops_setting)
                          );
      break;

    case CTRL_APPEND_BLOCK:
      ret = logAppendBlock(pkt.payload[1],
                           (struct ops_setting*)&pkt.payload[2],
                           (pkt.payload_size - 2) / sizeof(struct ops_setting)
                          );
      break;

    case CTRL_DELETE_BLOCK:
      ret = logDeleteBlock(pkt.payload[1]);
      break;

    case CTRL_START_BLOCK:
      ret = logStartBlock(pkt.payload[1], pkt.payload[2] * 10);
      break;

    case CTRL_STOP_BLOCK:
      ret = logStopBlock(pkt.payload[1]);
      break;

    case CTRL_RESET:
      logReset();
      ret = 0;
      break;
  }

  //Commands answer
  pkt.payload[2] = ret;
  pkt.payload_size = 3;
  CommModule::send(pkt);
}

static int logCreateBlock(unsigned char id, struct ops_setting* settings, int len)
{
  int i;

  for (i = 0; i < LOG_MAX_BLOCKS; i++)
    if (id == logBlocks[i].id) return 0; // return EEXIST;

  for (i = 0; i < LOG_MAX_BLOCKS; i++)
    if (logBlocks[i].id == BLOCK_ID_FREE) break;

  if (i == LOG_MAX_BLOCKS)
    return 0; // return ENOMEM;

  logBlocks[i].id = id;
  logBlocks[i].ops = NULL;

  // logBlocks[i].timer = xTimerCreate((const signed char*)"logTimer", M2T(1000),
  // pdTRUE, &logBlocks[i], logBlockTimed);

  if (logBlocks[i].timer == NULL) {
    logBlocks[i].id = BLOCK_ID_FREE;
    //return ENOMEM;
    return 0;
  }

  LOG(OK, "Added block ID %d\n", id);

  return logAppendBlock(id, settings, len);
}

static int blockCalcLength(struct log_block* block);
static struct log_ops* opsMalloc();
static void opsFree(struct log_ops* ops);
static void blockAppendOps(struct log_block* block, struct log_ops* ops);
static int variableGetIndex(int id);

static int logAppendBlock(int id, struct ops_setting* settings, int len)
{
  int i;
  struct log_block* block;

  LOG(INF1, "Appending %d variable to block %d\n", len, id);

  for (i = 0; i < LOG_MAX_BLOCKS; i++)
    if (logBlocks[i].id == id) break;

  if (i >= LOG_MAX_BLOCKS) {
    LOG(SEVERE, "Trying to append block id %d that doesn't exist.", id);
    // return ENOENT;
    return 0;
  }

  block = &logBlocks[i];

  for (i = 0; i < len; i++) {
    int varId, currentLength = blockCalcLength(block);
    struct log_ops* ops;

    if ((currentLength + typeLength[settings[i].logType & 0x0F]) > LOG_MAX_LEN) {
      LOG(SEVERE, "Trying to append a full block. Block id %d.\n", id);
      // return E2BIG;
      return 0;
    }

    ops = opsMalloc();

    if (!ops) {
      LOG(SEVERE, "No more ops memory free!\n");
      // return ENOMEM;
      return 0;
    }

    if (settings[i].id != 255) { //TOC variable
      varId = variableGetIndex(settings[i].id);

      if (varId < 0) {
        LOG(SEVERE, "Trying to add variable Id %d that does not exists.", settings[i].id);
        // return ENOENT;
        return 0;
      }

      ops->variable    = logs[varId].address;
      ops->storageType = logs[varId].type;
      ops->logType     = settings[i].logType & 0x0F;

      LOG(INF1, "Appended variable %d to block %d\n", settings[i].id, id);
    } else {                     //Memory variable
      //TODO: Check that the address is in ram
      ops->variable    = (void*)(&settings[i] + 1);
      ops->storageType = (settings[i].logType >> 4) & 0x0F;
      ops->logType     = settings[i].logType & 0x0F;
      i += 2;

      LOG(INF1, "Appended var addr 0x%x to block %d\n", (int)ops->variable, id);
    }

    blockAppendOps(block, ops);

    LOG(INF1, "Now length %d\n", blockCalcLength(block));
  }

  return 0;
}

static int logDeleteBlock(int id)
{
  int i;
  struct log_ops* ops;
  struct log_ops* opsNext;

  for (i = 0; i < LOG_MAX_BLOCKS; i++)
    if (logBlocks[i].id == id) break;

  if (i >= LOG_MAX_BLOCKS) {
    LOG(SEVERE, "trying to delete block id %d that doesn't exist.", id);
    // return ENOENT;
    return 0;
  }

  ops = logBlocks[i].ops;

  while (ops) {
    opsNext = ops->next;
    opsFree(ops);
    ops = opsNext;
  }

  if (logBlocks[i].timer != 0) {
    // xTimerStop(logBlocks[i].timer, portMAX_DELAY);
    // xTimerDelete(logBlocks[i].timer, portMAX_DELAY);
    logBlocks[i].timer = 0;
  }

  logBlocks[i].id = BLOCK_ID_FREE;
  return 0;
}

static int logStartBlock(int id, unsigned int period)
{
  int i;

  for (i = 0; i < LOG_MAX_BLOCKS; i++)
    if (logBlocks[i].id == id) break;

  if (i >= LOG_MAX_BLOCKS) {
    LOG(SEVERE, "Trying to start block id %d that doesn't exist.", id);
    // return ENOENT;
    return 0;
  }

  LOG(INF1, "Starting block %d with period %dms\n", id, period);

  if (period > 0) {
    // xTimerChangePeriod(logBlocks[i].timer, M2T(period), 100);
    // xTimerStart(logBlocks[i].timer, 100);
  } else {
    // single-shoot run
    // workerSchedule(logRunBlock, &logBlocks[i]);
  }

  return 0;
}

static int logStopBlock(int id)
{
  int i;

  for (i = 0; i < LOG_MAX_BLOCKS; i++)
    if (logBlocks[i].id == id) break;

  if (i >= LOG_MAX_BLOCKS) {
    LOG(SEVERE, "Trying to stop block id %d that doesn't exist.\n", id);
    // return ENOENT;
    return 0;
  }

  // xTimerStop(logBlocks[i].timer, portMAX_DELAY);

  return 0;
}

/* This function is called by the timer subsystem */
/*
void logBlockTimed(xTimerHandle timer)
{
  workerSchedule(logRunBlock, pvTimerGetTimerID(timer));
}
*/

/* This function is usually called by the worker subsystem */
void logRunBlock(void* arg)
{
  struct log_block* blk = (log_block*)arg;
  struct log_ops* ops = blk->ops;
//  static CRTPPacket pk;
  static RTP_t pk;
  unsigned int timestamp = 0;

  // xSemaphoreTake(logLock, portMAX_DELAY);

  // timestamp = ((long long)xTaskGetTickCount()) / portTICK_RATE_MS;

//  pk.header = RTP_HEADER(RTP_PORT_LOG, SUBC_LOG_DATA, true, false);
  pk.payload_size = 4;
  pk.payload[0] = blk->id;
  pk.payload[1] = timestamp & 0x0ff;
  pk.payload[2] = (timestamp >> 8) & 0x0ff;
  pk.payload[3] = (timestamp >> 16) & 0x0ff;

  while (ops) {
    float variable, valuef = 0;
    int valuei = 0;

    // FPU instructions must run on aligned data. Make sure it is.
    variable = *(float*)ops->variable;

    switch (ops->storageType) {
      case LOG_UINT8:
        valuei = *(uint8_t*)&variable;
        break;

      case LOG_INT8:
        valuei = *(int8_t*)&variable;
        break;

      case LOG_UINT16:
        valuei = *(uint16_t*)&variable;
        break;

      case LOG_INT16:
        valuei = *(int16_t*)&variable;
        break;

      case LOG_UINT32:
        valuei = *(uint32_t*)&variable;
        break;

      case LOG_INT32:
        valuei = *(int32_t*)&variable;
        break;

      case LOG_FLOAT:
        valuei = *(float*)&variable;
        break;
    }

    if (ops->logType == LOG_FLOAT || ops->logType == LOG_FP16) {
      if (ops->storageType == LOG_FLOAT)
        valuef = *(float*)&variable;
      else
        valuef = valuei;

      if (ops->logType == LOG_FLOAT) {
        memcpy(&pk.payload[pk.payload_size], &valuef, 4);
        pk.payload_size += 4;
      } else {
        valuei = single2half(valuef);
        memcpy(&pk.payload[pk.payload_size], &valuei, 2);
        pk.payload_size += 2;
      }
    } else { //logType is an integer
      memcpy(&pk.payload[pk.payload_size], &valuei, typeLength[ops->logType]);
      pk.payload_size += typeLength[ops->logType];
    }

    ops = ops->next;
  }

  // xSemaphoreGive(logLock);

  // Check if the connection is still up, oherwise disable
//  // all the logging and flush all the CRTP queues.
//  if (!crtpIsConnected()) {
  if (true) {
    logReset();
//    crtpReset();
  } else {
    CommModule::send(pk);
  }
}

static int variableGetIndex(int id)
{
  int i;
  int n = 0;

  for (i = 0; i < logsLen; i++) {
    if (!(logs[i].type & LOG_GROUP)) {
      if (n == id)
        break;

      n++;
    }
  }

  if (i >= logsLen)
    return -1;

  return i;
}

static struct log_ops* opsMalloc()
{
  int i;

  for (i = 0; i < LOG_MAX_OPS; i++)
    if (logOps[i].variable == NULL) break;

  if (i >= LOG_MAX_OPS)
    return NULL;

  return &logOps[i];
}

static void opsFree(struct log_ops* ops)
{
  ops->variable = NULL;
}

static int blockCalcLength(struct log_block* block)
{
  struct log_ops* ops;
  int len = 0;

  for (ops = block->ops; ops; ops = ops->next)
    len += typeLength[ops->logType];

  return len;
}

void blockAppendOps(struct log_block* block, struct log_ops* ops)
{
  struct log_ops* o;

  ops->next = NULL;

  if (block->ops == NULL)
    block->ops = ops;
  else {
    for (o = block->ops; o->next; o = o->next);

    o->next = ops;
  }
}

static void logReset(void)
{
  int i;

  if (isInit) {
    //Stop and delete all started log blocks
    for (i = 0; i < LOG_MAX_BLOCKS; i++)
      if (logBlocks[i].id != -1) {
        logStopBlock(logBlocks[i].id);
        logDeleteBlock(logBlocks[i].id);
      }
  }

  //Force free all the log block objects
  for (i = 0; i < LOG_MAX_BLOCKS; i++)
    logBlocks[i].id = BLOCK_ID_FREE;

  //Force free the log ops
  for (i = 0; i < LOG_MAX_OPS; i++)
    logOps[i].variable = NULL;
}

/* Public API to access log TOC from within the copter */
int logGetVarId(char* group, char* name)
{
  int i;
  char* currgroup = '\0';

  for (i = 0; i < logsLen; i++) {
    if (logs[i].type & LOG_GROUP) {
      if (logs[i].type & LOG_START)
        currgroup = logs[i].name;
    } if ((!strcmp(group, currgroup)) && (!strcmp(name, logs[i].name)))

      return i;
  }

  return -1;
}

int logGetInt(int varid)
{
  int valuei = 0;

  // ASSERT(varid >= 0);

  switch (logs[varid].type) {
    case LOG_UINT8:
      valuei = *(uint8_t*)logs[varid].address;
      break;

    case LOG_INT8:
      valuei = *(int8_t*)logs[varid].address;
      break;

    case LOG_UINT16:
      valuei = *(uint16_t*)logs[varid].address;
      break;

    case LOG_INT16:
      valuei = *(int16_t*)logs[varid].address;
      break;

    case LOG_UINT32:
      valuei = *(uint32_t*)logs[varid].address;
      break;

    case LOG_INT32:
      valuei = *(int32_t*)logs[varid].address;
      break;

    case LOG_FLOAT:
      valuei = *(float*)logs[varid].address;
      break;
  }

  return valuei;
}

float logGetFloat(int varid)
{
  // ASSERT(varid >= 0);

  if (logs[varid].type == LOG_FLOAT)
    return *(float*)logs[varid].address;

  return logGetInt(varid);
}

unsigned int logGetUint(int varid)
{
  return (unsigned int)logGetInt(varid);
}

// #endif
