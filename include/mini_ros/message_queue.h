#ifndef MESSAGE_QUEUE_H_
#define MESSAGE_QUEUE_H_

#include "message.h"
#include "multithread_queue.h"

namespace mini_ros {

using MessagePair = std::pair<std::string, std::shared_ptr<Message>>;

using MessageQueue = MultiThreadQueue<MessagePair>;

}
#endif
