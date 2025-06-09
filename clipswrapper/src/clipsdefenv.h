/* ** ***************************************************************
* clipsdefenv.h
*
* Author: Mauricio Matamoros
*
* Contains the default environment for CLIPS 6.4 and above
*
** ** **************************************************************/

#ifndef __CLIPSWRAPPER_DEFENV_H__
#define __CLIPSWRAPPER_DEFENV_H__
#pragma once

extern "C"{
	#include "clips/clips.h"
}


namespace clips{
	// The default environment
	extern Environment* defEnv;
}


#endif // __CLIPSWRAPPER_DEFENV_H__
