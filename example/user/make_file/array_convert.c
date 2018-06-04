#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "source.h"

int main(int argc, int **argv)
{
	int i = 0;
	char buf[20] = {0};
	char buf1[16] = "int data[] = {\n";
	char buf2[2] = "};";
	int size = sizeof data/ sizeof data[0];

	FILE *fp = fopen("./data.h", "w");
	fwrite(buf1, strlen(buf1), 1, fp);
	for(i = 0; i < size; ++i){
		memset(buf, 0x0, 20);
		sprintf(buf, "	0x%.08x,\n", data[i]);
		fwrite(buf, strlen(buf), 1, fp);
	}

	fwrite(buf2, strlen(buf2), 1, fp);

	fclose(fp);
	return 0;
}

