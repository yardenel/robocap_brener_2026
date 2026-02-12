#include <Payload.hpp>

void setup() {}

void loop() {
    static Payload payload;
    payload.tick();
}
