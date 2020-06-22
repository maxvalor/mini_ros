#ifndef MSG_H_
#define MSG_H_
namespace mini_ros {
  struct Message {
    struct {
      std::uint32_t id;
      std::uint32_t timestamp;
    } header;
  };
}
#endif
