/* 
  Macintosh specific configuration.

  Copyright (C) 2016  Michael Wallace, Mario Garcia.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (At your option) any later version.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef __MAC_CONFIG_H_
#define __MAC_CONFIG_H_

#include <CommProto/architecture/os/arch.h>

#if (COM_TARGET_OS == COM_OS_APPLE)


#define FUNCTOR_NAME __FUNCTION__

#if (COM_DISABLE_ASSERT > 0)
 #define COM_ASSERT(cond)
#else
 #define COM_ASSERT(cond)        assert(cond)
#endif

#define COM_UNUSEDPARAM(unusedparam)  (void)unusedparam

/* Define NULL value */
#ifndef NULL
 #ifdef __cplusplus
  #define NULL 0
 #else
  #define NULL ((void*)0)
 #endif
#endif

#endif

#endif // __MAC_CONFIG_H_
