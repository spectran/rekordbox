/*
 * rekordbox.c
 *
 *  Created on: 28 июля 2017 г.
 *      Author: Spectran
 */

#include "ff.h"
#include "rekordbox.h"
#include <string.h>

char wave_token[5] = "PWAV";
char wv2_token[5] = "PWV2";
char wv3_token[5] = "PWV3";
char cob_token[5] = "PCOB";
char path_token[5] = "PPTH";
char qtz_token[5] = "PQTZ";

char tag[5];
extern uint32_t i;

extern uint8_t *lowp_wavebuffer;
extern uint8_t *wavebuffer;

RekordboxTypeDef rekordbox;

extern FIL MyFile;     /* File object */
extern uint8_t buffer[8192];
extern uint16_t bytesread;

static uint8_t FindToken (char *token);

// finds section token in the file. 0 - token is found, 1 - end of file
static uint8_t FindToken (char *token) {
	f_lseek(&MyFile, 0); // search from the beginning
	do {
		do {
			while(f_read(&MyFile, &tag[0], 1, (void *)&bytesread) != FR_OK);
		}
		while(tag[0] != 'P');
		while(f_read(&MyFile, &tag[1], 3, (void *)&bytesread) != FR_OK);
		tag[4] = '\0';
	}
	while(strcmp(tag, token) != 0);
	if(f_eof(&MyFile) == 1) return 1;
	return 0;
}

uint8_t DecodeRekordboxFiles (char *folder) {
	uint32_t data_size = 0;
	if(f_open(&MyFile, "ANLZ0000.DAT", FA_READ) == FR_OK)
	{
		if(FindToken(path_token) != 0) return 1;
		while(f_read(&MyFile, &buffer, 8, (void *)&bytesread) != FR_OK); // dummy read 8 bytes
		for(i=0; i<4; i++) {
			while(f_read(&MyFile, &tag[i], 1, (void *)&bytesread) != FR_OK);
			data_size |= tag[i];
			data_size <<= 8;
		}
		data_size >>= 8;
		while(f_read(&MyFile, rekordbox.filename, data_size, (void *)&bytesread) != FR_OK);
		
		if(FindToken(qtz_token) != 0) return 1;
		while(f_read(&MyFile, &buffer, 16, (void *)&bytesread) != FR_OK); // dummy read 16 bytes
		for(i=0; i<4; i++) {
			while(f_read(&MyFile, &tag[i], 1, (void *)&bytesread) != FR_OK);
			rekordbox.timezones |= tag[i];
			rekordbox.timezones <<= 8;
		}
		rekordbox.timezones >>= 8;
		//while(f_read(&MyFile, rekordbox.timezones, data_size, (void *)&bytesread) != FR_OK);
		
		if(FindToken(wave_token) != 0) return 1;
		while(f_read(&MyFile, &buffer, 8, (void *)&bytesread) != FR_OK); // dummy read 8 bytes
		for(i=0; i<4; i++) {
			while(f_read(&MyFile, &tag[i], 1, (void *)&bytesread) != FR_OK);
			rekordbox.lowp_spectrum_size |= tag[i];
			rekordbox.lowp_spectrum_size <<= 8;
		}
		rekordbox.lowp_spectrum_size >>= 8;
		while(f_read(&MyFile, &buffer, 4, (void *)&bytesread) != FR_OK); // dummy read 4 bytes
		while(f_read(&MyFile, lowp_wavebuffer, rekordbox.lowp_spectrum_size, (void *)&bytesread) != FR_OK);
		
		f_close(&MyFile);
	}
	if(f_open(&MyFile, "ANLZ0000.EXT", FA_READ) == FR_OK)
	{
		if(FindToken(wv3_token) != 0) return 1;
		while(f_read(&MyFile, &buffer, 12, (void *)&bytesread) != FR_OK);
		for(i=0; i<4; i++) {
			while(f_read(&MyFile, &tag[i], 1, (void *)&bytesread) != FR_OK);
			rekordbox.spectrum_size |= tag[i];
			rekordbox.spectrum_size <<= 8;
		}
		rekordbox.spectrum_size >>= 8;
		while(f_read(&MyFile, &buffer, 4, (void *)&bytesread) != FR_OK); // dummy read 4 bytes
		while(f_read(&MyFile, wavebuffer, rekordbox.spectrum_size, (void *)&bytesread) != FR_OK);
		f_close(&MyFile);
	}
	return 0;
}
