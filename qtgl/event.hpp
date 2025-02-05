#pragma once

#include <functional>
#include <map>
#include <vector>

namespace qtgl {

enum class GLEvent {

  CHANGE_GLOBJECT,
  CHANGE_LIGHT,
  CHANGE_CAMERA

};

using GLEventCallback = std::function<void(void*)>;

struct GLEventIdetitiedCallback {
  long id;
  GLEventCallback callback;
};

class GLEventBus {
 public:
  std::map<GLEvent, std::vector<GLEventIdetitiedCallback>> callbacks;
  static long cnt;
  GLEventBus() {}
  ~GLEventBus() {}

  int on(GLEvent event, GLEventCallback callback) {
    long c = cnt++;
    GLEventIdetitiedCallback idcallback{c, callback};
    if (!callbacks.count(event)) {
      callbacks[event] = std::vector<GLEventIdetitiedCallback>(0);
    }
    callbacks[event].push_back(idcallback);
    return c;
  }

  void fire(GLEvent event, void* target) {
    if (callbacks.count(event)) {
      for (auto idcallback : callbacks[event]) {
        (idcallback.callback)(target);
      }
    }
  }
  void un(GLEvent event, long id) {
    // TODO
  }
};

}  // namespace qtgl