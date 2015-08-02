#if 1

#include "param.hpp"

// Private functions

void paramTask(void* prm);
void paramTOCProcess(int command);

// The following two function SHALL NOT be called outside paramTask!
static void paramWriteProcess(int id, void*);
static void paramReadProcess(int id);
static int variableGetIndex(int id);
static char paramWriteByNameProcess(char* group, char* name, int type, void* valptr);

// These are set by the linker
extern struct param_s _param_start;
extern struct param_s _param_stop;

// Pointer to the parameters list and length of it
static struct param_s* params;
static int paramsLen;
// uint32_t paramsCrc;
static int paramsCount = 0;

static bool isInit = false;


RTP_t p;

void paramInit(void)
{
	int i;

	if (isInit)
		return;

	params = &_param_start;
	paramsLen = &_param_stop - &_param_start;
	// paramsCrc = crcSlow(params, paramsLen);

	for (i = 0; i < paramsLen; i++)
		if (!(params[i].type & PARAM_GROUP))
			paramsCount++;

	Thread console_task(Task_Param, NULL, osPriorityBelowNormal);

	isInit = true;
}

bool paramTest(void)
{
	return isInit;
}


// Pass a queue ID
void Task_Param(void const* arg)
{
	// Store the thread's ID
	osThreadId threadID = Thread::gettid();

	// Store the passed queue pointer
	osMailQId* queueID = (osMailQId*)arg;

	LOG(OK, "Parameter module ready! Thread ID: %u; Queue ID: %u", threadID, *queueID);

	RTP_t* pkt;

	while (1) {
		osEvent evt = osMailGet(*queueID, osWaitForever);

		if (evt.status == osEventMail) {

			pkt = (RTP_t*)evt.value.p;

			LOG(OK, "Received packet! 0x%02X", pkt->payload[0]);

			if (pkt->subclass == TOC_CH) {
				paramTOCProcess(pkt->payload[0]);
			} else if (pkt->subclass == READ_CH) {
				paramReadProcess(pkt->payload[0]);
			} else if (pkt->subclass == WRITE_CH) {
				paramWriteProcess(pkt->payload[0], &(pkt->payload[1]));
			} else if (pkt->subclass == MISC_CH) {
				if (pkt->payload[0] == MISC_SETBYNAME) {
					int i, error, nzero = 0;
					char* group;
					char* name;
					uint8_t type;
					void* valPtr;

					// If the packet contains at least 2 zeros in the first 28 bytes
					// The packet decoding algorithm will not crash
					for (i = 0; i < RTP_MAX_DATA_SIZE; i++) {
						if (pkt->payload[i] == '\0') nzero++;
					}

					if (nzero < 2) return;

					group = (char*)&pkt->payload[1];
					name = (char*)&pkt->payload[1 + strlen(group) + 1];
					type = pkt->payload[1 + strlen(group) + 1 + strlen(name) + 1];
					valPtr = &pkt->payload[1 + strlen(group) + 1 + strlen(name) + 2];

					error = paramWriteByNameProcess(group, name, type, valPtr);

					pkt->payload[1 + strlen(group) + 1 + strlen(name) + 1] = error;
					pkt->payload_size = 1 + strlen(group) + 1 + strlen(name) + 1 + 1;

					//crtpSendPacket(&p);
				}
			}

			osMailFree(*queueID, pkt);
		}

		Thread::wait(1000);
		Thread::yield();
	}

	osThreadTerminate(threadID);
}

void paramTOCProcess(int command)
{
	int ptr, n = 0;
	char* group = '\0';

	switch (command) {
	case CMD_GET_INFO: // Get info packet about the param implementation
		ptr = 0;
		group = '\0';
		// p.header = CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
		p.payload_size = 2;
		p.payload[0] = CMD_GET_INFO;
		p.payload[1] = paramsCount;
		// memcpy(&p.payload[2], &paramsCrc, 4);
		// crtpSendPacket(&p);
		break;

	case CMD_GET_ITEM:  // Get param variable
		for (ptr = 0; ptr < paramsLen; ptr++) { //Ptr points a group
			if (params[ptr].type & PARAM_GROUP) {
				if (params[ptr].type & PARAM_START)
					group = params[ptr].name;
				else
					group = '\0';
			} else {           // Ptr points a variable
				if (n == p.payload[1])
					break;

				n++;
			}
		}

		if (ptr < paramsLen) {
			// p.header = CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
			p.payload[0] = CMD_GET_ITEM;
			p.payload[1] = n;
			p.payload[2] = params[ptr].type;
			memcpy(p.payload + 3, group, strlen(group) + 1);
			memcpy(p.payload + 3 + strlen(group) + 1, params[ptr].name, strlen(params[ptr].name) + 1);
			p.payload_size = 3 + 2 + strlen(group) + strlen(params[ptr].name);
			// crtpSendPacket(&p);
		} else {
			// p.header = CRTP_HEADER(CRTP_PORT_PARAM, TOC_CH);
			p.payload[0] = CMD_GET_ITEM;
			p.payload_size = 1;
			// crtpSendPacket(&p);
		}

		break;
	}
}

static void paramWriteProcess(int ident, void* valptr)
{
	int id = variableGetIndex(ident);

	if (id < 0) {
		p.payload[0] = -1;
		p.payload[1] = ident;
		// p.payload[2] = ENOENT;
		p.payload_size = 3;

		// crtpSendPacket(&p);
		return;
	}

	if (params[id].type & PARAM_RONLY)
		return;

	switch (params[id].type & PARAM_BYTES_MASK) {
	case PARAM_1BYTE:
		*(uint8_t*)params[id].address = *(uint8_t*)valptr;
		break;

	case PARAM_2BYTES:
		*(uint16_t*)params[id].address = *(uint16_t*)valptr;
		break;

	case PARAM_4BYTES:
		*(uint32_t*)params[id].address = *(uint32_t*)valptr;
		break;

	case PARAM_8BYTES:
		*(uint64_t*)params[id].address = *(uint64_t*)valptr;
		break;
	}

	// crtpSendPacket(&p);
}

static char paramWriteByNameProcess(char* group, char* name, int type, void* valptr)
{
	int ptr;
	char* pgroup = '\0';

	for (ptr = 0; ptr < paramsLen; ptr++) { //Ptr points a group
		if (params[ptr].type & PARAM_GROUP) {
			if (params[ptr].type & PARAM_START)
				pgroup = params[ptr].name;
			else
				pgroup = '\0';
		} else {                      //Ptr points a variable
			if (!strcmp(params[ptr].name, name) && !strcmp(pgroup, group))
				break;
		}
	}

	if (ptr >= paramsLen) {
		// return ENOENT;
	}

	if (type != params[ptr].type) {
		// return EINVAL;
	}

	if (params[ptr].type & PARAM_RONLY) {
		// return EACCES;
	}

	switch (params[ptr].type & PARAM_BYTES_MASK) {
	case PARAM_1BYTE:
		*(uint8_t*)params[ptr].address = *(uint8_t*)valptr;
		break;

	case PARAM_2BYTES:
		*(uint16_t*)params[ptr].address = *(uint16_t*)valptr;
		break;

	case PARAM_4BYTES:
		*(uint32_t*)params[ptr].address = *(uint32_t*)valptr;
		break;

	case PARAM_8BYTES:
		*(uint64_t*)params[ptr].address = *(uint64_t*)valptr;
		break;
	}

	return 0;
}

static void paramReadProcess(int ident)
{
	int id = variableGetIndex(ident);

	if (id < 0) {
		p.payload[0] = -1;
		p.payload[1] = ident;
		// p.payload[2] = ENOENT;
		p.payload_size = 3;

		//crtpSendPacket(&p);
		return;
	}

	switch (params[id].type & PARAM_BYTES_MASK) {
	case PARAM_1BYTE:
		memcpy(&p.payload[1], params[id].address, sizeof(uint8_t));
		p.payload_size = 1 + sizeof(uint8_t);
		break;

	case PARAM_2BYTES:
		memcpy(&p.payload[1], params[id].address, sizeof(uint16_t));
		p.payload_size = 1 + sizeof(uint16_t);
		break;

	case PARAM_4BYTES:
		memcpy(&p.payload[1], params[id].address, sizeof(uint32_t));
		p.payload_size = 1 + sizeof(uint32_t);
		break;

	case PARAM_8BYTES:
		memcpy(&p.payload[1], params[id].address, sizeof(uint64_t));
		p.payload_size = 1 + sizeof(uint64_t);
		break;
	}

	// crtpSendPacket(&p);
}

static int variableGetIndex(int id)
{
	int i, n = 0;

	for (i = 0; i < paramsLen; i++) {
		if (!(params[i].type & PARAM_GROUP)) {
			if (n == id)
				break;

			n++;
		}
	}

	if (i >= paramsLen)
		return -1;

	return i;
}

#endif