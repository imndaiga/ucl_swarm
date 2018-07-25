#ifndef UTILS_H
#define UTILS_H

/* constants for a pseudo-random number generator, details see Numerical Recipes in C book */

#define IA 16807
#define IM 2147483647
#define AM (1.0/IM)
#define IQ 127773
#define IR 2836

double ran01( long *idum );

#endif