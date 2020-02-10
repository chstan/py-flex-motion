///////////////////////////////////////////////////////////////////////////////
//
//  Title     : nimcBasicTypes.h
//  Project   : 
//  Created   : Wed Nov 14 15:20:48 2001
//  Author    : ksreekan
//  Platforms : All
//  Copyright : National Instruments 2001.  All Rights Reserved.
//  Access    : Company Confidential
//  Purpose   : Definition of local types and #defines
//
///////////////////////////////////////////////////////////////////////////////
#ifndef __NIMCBasicTypes_h__
#define __NIMCBasicTypes_h__

// Signed integer type.
typedef long i32;

// Floating point type.
typedef double f64;

// 32-bit floating point type
typedef float f32;

// Signed short type
typedef short i16;

// Signed char type
typedef char i8;

#ifndef __LINUX__
	// Unsigned char type
	typedef unsigned char u8;

	// Unsigned short type
	typedef unsigned short u16;

	// Unsigned integer type.
	typedef unsigned long u32;
#else
	#ifndef __KERNEL__
		// Unsigned char type
		typedef unsigned char u8;

		// Unsigned short type
		typedef unsigned short u16;

		// Unsigned integer type.
		typedef unsigned long u32;
	#endif
#endif

// tBoolean type
typedef unsigned long tBoolean;

#endif // __NIMCBasicTypes_h__