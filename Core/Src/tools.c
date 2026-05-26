#include "tools.h"

void GPIO_setup(GPIO_TypeDef *__port, uint8_t __pin, uint8_t __mode)
{
    if (__pin < 8)
    {
        __port->CRL &= ~(0xF << (__pin * 4));
        __port->CRL |= (__mode << (__pin * 4));
    }
    else
    {
        __port->CRH &= ~(0xF << ((__pin - 8) * 4));
        __port->CRH |= (__mode << ((__pin - 8) * 4));
    }
}

void Delay_ms(uint32_t ms) {
	ms *= 1000;
	for (volatile uint32_t i = 0; i < ms; ++i) {
		for (volatile uint32_t j = 0; j < 500; ++j);
	}
}

void Delay_us(uint32_t ms) {
	ms *= 50;
	for (volatile uint32_t i = 0; i < ms; ++i) {
		for (volatile uint32_t j = 0; j < 50; ++j);
	}
}

void Clear_buff(char *buff, uint8_t size) {
	while (size) {
		*buff++ = 0;
		size--;
	}
}
