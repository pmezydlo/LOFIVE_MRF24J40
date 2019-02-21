#ifndef _DEBUG_H_
#define _DEBUG_H

#include <stdint.h>
#include <stdio.h>

#define DEBUG_ENABLED

#define DEBUG_ERROR    3
#define DEBUG_WARNING  2
#define DEBUG_VERBOSE  1
#define DEBUG_INFO     0


//#define DEBUG(x, ...) (((x == 1) || (x == 2 ) || (x == 3) || (x == 0)) ? printf (__VA_ARGS__) : )
#ifdef DEBUG_ENABLED
	#define DEBUG(...) printf (__VA_ARGS__)
#else
	#define DEBUG(...)
#endif

/*
		DEBUG ("dsa %s %d %s %d\n\r", __func__, __LINE__, __FILE__ , i );
		DEBUG ("%s \n\r", __BASE_FILE__);
*/

/*
void dump_buffer(uint8_t *buf, uint32_t size)
{
	uint8_t index;

	for (index = 0; index < size ; index++) {
		if (index % 16 == 0) {
			printf ("\n");
		}
		printf ("0x%02x, ", buf[index]);
	}
	printf ("\n");
}

*/

#endif
