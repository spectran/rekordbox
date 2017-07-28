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
	uint32_t lowp_spectrum_size;
	uint32_t spectrum_size;
}RekordboxTypeDef;

void DecodeRekordboxFiles ();

#endif /* REKORDBOX_H_ */
