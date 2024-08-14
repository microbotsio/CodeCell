/*
 * Copyright 2015-16 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Simple Utility functions common to several SH2 files.
 */

#ifndef SH2_UTIL_H
#define SH2_UTIL_H

#include <stdint.h>

#ifndef ARRAY_LEN
#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))
#endif

uint8_t  readu8(const uint8_t * buffer);
void writeu8(uint8_t * buffer, uint8_t value);
uint16_t readu16(const uint8_t * buffer);
void writeu16(uint8_t * buffer, uint16_t value);
uint32_t readu32(const uint8_t * buffer);
void writeu32(uint8_t * buffer, uint32_t value);

int8_t read8(const uint8_t * buffer);
void write8(uint8_t * buffer, int8_t value);
int16_t read16(const uint8_t * buffer);
void write16(uint8_t * buffer, int16_t value);
int32_t read32(const uint8_t * buffer);
void write32(uint8_t * buffer, int32_t value);

#endif
