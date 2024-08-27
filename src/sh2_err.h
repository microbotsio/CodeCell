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

/** 
 * @file sh2_err.h
 * @author David Wheeler
 * @date 22 May 2015
 * @brief Type definitions for Hillcrest SH-2 API.
 *
 * Struct and type definitions supporting the Hillcrest SH-2 SensorHub API.
 * 
 */


#ifndef SH2_ERR_H
#define SH2_ERR_H


#define SH2_OK                 (0)  /**< Success */
#define SH2_ERR                (-1) /**< General Error */
#define SH2_ERR_BAD_PARAM      (-2) /**< Bad parameter to an API call */
#define SH2_ERR_OP_IN_PROGRESS (-3) /**< Operation in progress */
#define SH2_ERR_IO             (-4) /**< Error communicating with hub */
#define SH2_ERR_HUB            (-5) /**< Error reported by hub */
#define SH2_ERR_TIMEOUT        (-6) /**< Operation timed out */


#endif
