#ifndef MSG_H_
#define MSG_H_

#include <memory>

namespace mini_ros {
  struct Message {
    struct {
      std::uint32_t id;
      std::uint32_t timestamp;
    } header;
    
    virtual ~Message() {}
  };
}
#endif
