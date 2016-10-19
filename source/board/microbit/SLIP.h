#include <stdint.h>

#define SLIP_END			'\xc0'
#define SLIP_ESC			'\xdb'
#define SLIP_ESC_END		'\xdc'
#define SLIP_ESC_ESC		'\xdd'

enum SLIPCharacters {
	END = 1,
	ESC,
	ESC_END,
	ESC_ESC
};

static int is_slip_character(char c)
{
	switch (c)
	{
		case SLIP_END:
			return END;
		case SLIP_ESC:
			return ESC;
		case SLIP_ESC_END:
			return ESC_END;
		case SLIP_ESC_ESC:
			return ESC_ESC;
	}

	return 0;
}

static int contains_slip_character(uint8_t* buff, uint16_t from, uint16_t len)
{
	for (int i = from; i < from + len; i++)
		if (is_slip_character(buff[i]))
			return i;

	return -1;
}
