#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/ipc.h>
#include <sys/msg.h>

#define EN_DEBUG
#ifdef EN_DEBUG
#define DBG(fmt, args...) fprintf(stdout, "MSGDEF: Dbg %s: " fmt, __func__, ##args)
#else
#define DBG(fmt, args...)
#endif

#define MQBUFSIZE (sizeof(message_buf_t)-sizeof(long))
#define Beacon_Queue_Key 1992

typedef struct buf {
	long mtype;
	uint16_t length;
	uint8_t mtext[15];
} message_buf_t;


/* @func    : sys_mq_init
 * @param[in]   : Key for the message queue
 * @return  : message queue ID upon success; else Standard error code.
 */
int sys_mq_init(int msgKey)
{
	int qid;
	key_t key = msgKey;

	qid = msgget(key, 0);
	if (qid < 0) {
		qid = msgget(key, IPC_CREAT | 0666);
		DBG("Creat queue id:%d\n", qid);
	} else
		DBG("queue id:%d\n", qid);
	return qid;
}

/* @func    : sys_mq_kill
 * @param[in]   : Message queue Id to be removed
 * @return  : status 0
 */
int sys_mq_kill(int qid)
{
	msgctl(qid, IPC_RMID, NULL);
	DBG("Kill queue id:%d\n", qid);
	return 0;
}

int sys_mq_reset(int msgKey)
{
	int qid;
	key_t key = msgKey;

	qid = msgget(key, 0);
	if (qid < 0) {
		msgctl(qid, IPC_RMID, NULL);
	}
}

/* @func    : sys_mq_send
 * @param1[in]  : Target queue ID
 * @param2[in]  : Data buffer
 * @param3[in]  : Size of data in bytes
 * @return  : 0 if Success; else Error code.
 */
int sys_mq_send(int mqId, uint8_t *buf, uint16_t size)
{
	int status = -1;
	message_buf_t message_buff;
	message_buff.mtype = 1;
	message_buff.length = size;
	memcpy(message_buff.mtext, buf, size);
	status = msgsnd(mqId, &message_buff, MQBUFSIZE, 0);
	return status;
}

/* @func    : sys_mq_recv
 * @param1[in]  : Target queue ID
 * @param2[out] : Data buffer
 * @param3[out] : Size of data in bytes
 * @return  : 0 if Success; else Error code.
 */
int sys_mq_recv(int mqId, uint8_t *buf, uint16_t *size)
{
	int ret;
	message_buf_t message_buff;
	message_buff.mtype = 1;
	ret = msgrcv(mqId, &message_buff, MQBUFSIZE, message_buff.mtype,
			0);
	if (ret != -1) {
		memcpy(buf, message_buff.mtext, message_buff.length);
		*size = message_buff.length;
		return 0;
	} else
		printf("Messgae Que Returned Fail: %d\n", ret);
	return ret;
}

