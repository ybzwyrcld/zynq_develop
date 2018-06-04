/**
 * @file conversion.h
 * @date Sunday, December 06, 2015 at 03:29:47 AM EST
 * @author Brandon Perez (bmperez)
 * @author Jared Choi (jaewonch)
 *
 * Contains some basic macros for converting between different types.
 *
 * @bug No known bugs.
 **/

#ifndef CONVERSION_H_
#define CONVERSION_H_

#include <sys/time.h>           // Timing functions and definitions

// Converts a tval struct to a double value of the time in seconds
#define TVAL_TO_SEC(tval) \
    (((double)(tval).tv_sec) + (((double)(tval).tv_usec) / 1000000.0))

// Converts a byte (integral) value to megabytes (floating-point)
#define BYTE_TO_MB(size) (((double)(size)) / (1024.0 * 1024.0))

// Converts a megabyte (floating-point) value to bytes (integral)
#define MB_TO_BYTE(size) ((size_t)((size) * 1024.0 * 1024.0))


#endif /* CONVERSION_H_ */
