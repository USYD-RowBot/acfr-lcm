#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include <bot_param/param_client.h>

#define TEST_CONFIG_FILE "../../../config/iver28.cfg"

int main(int argc, char *argv[])
{
	char* key = "sensors.os-remotehelm.gsd.baud";
	char* str;
	int ok;

    BotParam *param = bot_param_new_from_file(TEST_CONFIG_FILE);

	// { int
	int val;
	ok = bot_param_get_int(param, key, &val);
	if(!ok)	printf(" conf result: %d\n", val);
	else	printf(" conf_get_int() failed.\n");
	// } int

	// { bool
	key = "hotel.easydaq.relay8.state";
	ok = bot_param_get_boolean(param, key, &val);
	if(!ok)	printf(" conf result: %d\n", val);
	else	printf(" conf_get_boolean() failed.\n");
	// } bool
	
	// { string
	key = "sensors.os-remotehelm.gsd.io";
	ok = bot_param_get_str(param, key, &str);
	if(!ok)	printf(" conf result: %s\n", str);
	else	printf(" conf_get_str() failed.\n");
	// } string

	// { int array
	key = "sensors.os-remotehelm.x_vs";
	int* array;
	int len = bot_param_get_array_len(param, key);
	array = malloc(sizeof(int)*len);
	printf("len: %d\n",len);
	ok = bot_param_get_int_array(param, key, array, len);
	printf("ok: %d\n",ok);

	int i;
	printf(" [");
	for(i=0;i<(len-1);i++) printf("%d,",array[i]);
	printf("%d]\n",array[i]);
	
	free(array);

	// } int array

	bot_param_destroy(param);
}
