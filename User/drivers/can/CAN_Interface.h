//
// Created by Rick on 24-11-22.
//

#ifndef DM_02_HERO_USER_DRIVERS_CAN_CAN_INTERFACE_H_
#define DM_02_HERO_USER_DRIVERS_CAN_CAN_INTERFACE_H_

#include <cstdint>
#include <cstddef>

class CAN_Interface
{
 public:
	int (*initialize)();
	int (*send)(uint32_t id, const uint8_t *data, size_t size);
	int (*receive)(uint32_t *id, uint8_t *data, size_t size);
	void (*deinitialize)();
 private:
};

#endif //DM_02_HERO_USER_DRIVERS_CAN_CAN_INTERFACE_H_
