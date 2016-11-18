/*
 * contactor.h
 *
 *  Created on: Jul 13, 2015
 *      Author: Yosh
 */

#ifndef CONTACTOR_H_
#define CONTACTOR_H_

void contactor_init(void);
void contactor1_on(void);
void contactor2_on(void);
void contactor1_off(void);
void contactor2_off(void);
void ContactorAllTurnOn(void);
void ContactorAllTurnOff(void);
uint8_t is_contactor_1_closed(void);
uint8_t is_contactor_2_closed(void);


#endif /* CONTACTOR_H_ */
