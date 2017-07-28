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

char tag[5];
extern uint32_t i;

extern uint8_t *lowp_wavebuffer;
extern uint8_t *wavebuffer;

RekordboxTypeDef rekordbox;

extern FIL MyFile;     /* File object */
extern uint8_t buffer[8192];
extern uint16_t bytesread;

void DecodeRekordboxFiles () {
	if(f_open(&MyFile, "ANLZ0000.DAT", FA_READ) == FR_OK)
	{
		do {
			do {
				while(f_read(&MyFile, &tag[0], 1, (void *)&bytesread) != FR_OK);
			}
			while(tag[0] != 'P');
			while(f_read(&MyFile, &tag[1], 3, (void *)&bytesread) != FR_OK);
			tag[4] = '\0';
		}
		while((strcmp(tag, wave_token) != 0) && (f_eof(&MyFile) == 0));

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
		do {
			do {
				while(f_read(&MyFile, &tag[0], 1, (void *)&bytesread) != FR_OK);
			}
			while(tag[0] != 'P');
			while(f_read(&MyFile, &tag[1], 3, (void *)&bytesread) != FR_OK);
			tag[4] = '\0';
		}
		while((strcmp(tag, wv3_token) != 0) && (f_eof(&MyFile) == 0));
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
}
