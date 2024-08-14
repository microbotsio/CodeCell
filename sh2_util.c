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

#include "sh2_util.h"

uint8_t readu8(const uint8_t *p)
{
    uint8_t retval = p[0];
    return retval;
}

void writeu8(uint8_t * p, uint8_t value)
{
    *p = (uint8_t)(value & 0xFF);
}

uint16_t readu16(const uint8_t *p)
{
    uint16_t retval = p[0] | (p[1] << 8);
    return retval;
}

void writeu16(uint8_t * p, uint16_t value)
{
    *p++ = (uint8_t)(value & 0xFF);
    value >>= 8;
    *p = (uint8_t)(value & 0xFF);
}

uint32_t readu32(const uint8_t *p)
{
    uint32_t retval = p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
    return retval;
}

void writeu32(uint8_t * p, uint32_t value)
{
    *p++ = (uint8_t)(value & 0xFF);
    value >>= 8;
    *p++ = (uint8_t)(value & 0xFF);
    value >>= 8;
    *p++ = (uint8_t)(value & 0xFF);
    value >>= 8;
    *p = (uint8_t)(value & 0xFF);
}

int8_t read8(const uint8_t *p)
{
    int8_t retval = p[0];
    return retval;
}

void write8(uint8_t * p, int8_t value)
{
    *p = (uint8_t)(value & 0xFF);
}

int16_t read16(const uint8_t *p)
{
    int16_t retval = p[0] | (p[1] << 8);
    return retval;
}

void write16(uint8_t * p, int16_t value)
{
    *p++ = (uint8_t)(value & 0xFF);
    value >>= 8;
    *p = (uint8_t)(value & 0xFF);
}

int32_t read32(const uint8_t *p)
{
    int32_t retval = p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
    return retval;
}

void write32(uint8_t * p, int32_t value)
{
    *p++ = (uint8_t)(value & 0xFF);
    value >>= 8;
    *p++ = (uint8_t)(value & 0xFF);
    value >>= 8;
    *p++ = (uint8_t)(value & 0xFF);
    value >>= 8;
    *p = (uint8_t)(value & 0xFF);
}
