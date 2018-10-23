#include <modules/pubsub/pubsub.h>
#include <modules/worker_thread/worker_thread.h>

WORKER_THREAD_TAKEOVER_MAIN(lpwork_thread, LOWPRIO+1)
WORKER_THREAD_SPAWN(can_thread, LOWPRIO+2, 320)
WORKER_THREAD_SPAWN(spi3_thread, HIGHPRIO, 1152)
WORKER_THREAD_SPAWN(i2c2_thread, HIGHPRIO-1, 1088)

PUBSUB_TOPIC_GROUP_CREATE(default_topic_group, 1024)
