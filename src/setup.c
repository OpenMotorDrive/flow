#include <modules/pubsub/pubsub.h>
#include <modules/worker_thread/worker_thread.h>

WORKER_THREAD_TAKEOVER_MAIN(lpwork_thread, LOWPRIO+1)
WORKER_THREAD_SPAWN(can_thread, LOWPRIO+2, 512)
WORKER_THREAD_SPAWN(spi3_thread, HIGHPRIO, 2048)
WORKER_THREAD_SPAWN(i2c_thread, HIGHPRIO-1, 512)

PUBSUB_TOPIC_GROUP_CREATE(default_topic_group, 1024)
