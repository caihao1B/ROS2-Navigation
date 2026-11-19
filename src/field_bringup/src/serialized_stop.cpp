// Process-local compatibility for the locked Gazebo Sim 8.15.0 ABI.
// Serialize concurrent stop event emissions. Do not suppress signals, alter
// return codes, skip destructors, or intercept any motion/sensor function.
#include <cstdio>
#include <cstdlib>
#include <dlfcn.h>
#include <pthread.h>

static pthread_mutex_t stop_mutex = PTHREAD_MUTEX_INITIALIZER;
using Stop = void (*)(void *);
extern "C" void
serialized_server_stop(void *) asm("_ZN2gz3sim2v816SimulationRunner4StopEv");
extern "C" void serialized_server_stop(void *self) {
  static Stop original = reinterpret_cast<Stop>(
      dlsym(RTLD_NEXT, "_ZN2gz3sim2v816SimulationRunner4StopEv"));
  if (!original) {
    std::fputs("GAZEBO_STOP_SYMBOL_NOT_FOUND\n", stderr);
    std::abort();
  }
  pthread_mutex_lock(&stop_mutex);
  std::fputs("GAZEBO_STOP_SERIALIZED\n", stderr);
  original(self);
  pthread_mutex_unlock(&stop_mutex);
}
