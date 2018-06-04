#include <stdio.h>
#include <stdlib.h>
#include "data.h"

int main(void)
{
	FILE *fp = NULL;
	fp = fopen("./infile", "wb");
	if (fp == NULL)
		perror("open file fail!!!"), exit(1);

	fwrite(data, sizeof data[0], sizeof data / sizeof data[0], fp);

	fclose(fp);
	return 0;
}

