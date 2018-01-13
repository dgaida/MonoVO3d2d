// Implementation FINISHED
// Documentation FINISHED
// 11.01.2018, Daniel Gaida

#pragma once

#ifndef DEBUG_H
#define DEBUG_H

// Makro used only in this file to control behaviour of Makros defined below
// can also be used in your files, to only print messages to console if you are not
// running in final version (so FULLVERSION_D is not defined)
#define FULLVERSION_D
//#undef FULLVERSION_D

// if defined, then do everything for speed optimization, so no output to console, ...
// real time
#define SPEED_OPTIM
//#undef SPEED_OPTIM

#include <stdio.h>
#include <stdarg.h>
#include <iostream>
#include <sstream>



/*=============================                  =============================*/

// headers used in many files of projects

#include <assert.h>

#include <string>

/*=============================                  =============================*/



//

#ifdef WIN32
#  ifndef __FUNCTION__
#    define __FUNCTION__ " "
#  endif
#endif

// if we do everything for speed then definitely define FULLVERSION_D, if not defined anyway above
#ifdef SPEED_OPTIM
  #define FULLVERSION_D
#endif


/**
 * Used to display trace messages on the standard stream (C++).
 * Use like this : cvCTRACE << "my message" << std::endl;
 */
#ifndef FULLVERSION_D
  #define cvCTRACE std::cout << __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") : "
#else
  #define cvCTRACE std::cout
#endif

/**
 * Used to display error messages on the error stream (C++).
 * Use like this : cvCERROR << "my message" << std::endl;
 */
#define cvCERROR std::cerr << "!!\t" << __FILE__ << ": " << __FUNCTION__ << "(#" << __LINE__ << ") : "



/**
 * Works like cvCTRACE and should be used at the beginning of a function.
 */
#ifndef FULLVERSION_D
	#define cvIN_FCT std::cout << __FILE__ << " (#" << __LINE__ << ") : begin " << __FUNCTION__ << std::endl
#else
	#define cvIN_FCT 0
#endif

/**
 * Works like cvTRACE and should be used at the end of a function.
 */
#ifndef FULLVERSION_D
	#define cvOUT_FCT std::cout << __FILE__ << " (#" << __LINE__ << ") : end " << __FUNCTION__ << std::endl
#else
	#define cvOUT_FCT 0
#endif



#endif 


