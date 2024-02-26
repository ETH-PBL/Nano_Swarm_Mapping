/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Author: Vlad Niculescu
 */

#include <time.h>       // for clock_t, clock(), CLOCKS_PER_SEC

long int xTaskGetTickCount() {
	clock_t time = clock();
	long int time_us = 1000000 * (long int)time;
	time_us = time_us / CLOCKS_PER_SEC;
}

void vTaskDelay() {
	// do nothing
}