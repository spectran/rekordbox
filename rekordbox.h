/*
 * rekordbox.h
 *
 *  Created on: 28 июля 2017 г.
 *      Author: Spectran
 */

#ifndef REKORDBOX_H_
#define REKORDBOX_H_

typedef struct
{
	uint8_t *filename;
	uint32_t timezones;
	uint16_t bpm;
	uint32_t lowp_spectrum_size;
	uint32_t spectrum_size;
}RekordboxTypeDef;

uint8_t DecodeRekordboxFiles (char *folder);

#endif /* REKORDBOX_H_ */
