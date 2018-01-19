/*
Copyright (c) 2010 Takashi Kawasaki <espresso3389 _at_ gmail.com>
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _zoomlogic_h_
#define _zoomlogic_h_

#include <vector>
#include <cstring>

#ifdef _WIN32
#define USE_FAST_FLOOR 1 // actually for Intel X86 based processors
#define BIGENDIAN 0
#else
#define USE_FAST_FLOOR 0
#define BIGENDIAN 0 // ???
#endif

#define PZI_RESTRICT

// implementations in _zoomimpl_core_ is not for normal use; please use
// Zoom::rescale.
namespace _zoomimpl_core_
{
	using FLOAT = double;
	using u8 = uint8_t;
	
	enum {CHECK_SATURATION = false};

	//------------------------------------------------------------------------
	template<typename TYPE, size_t SIZE> struct FASTFLOOR
	{
		static TYPE fast_floor(double val)
		{
			return (TYPE)val;
		}
	};
	
	//------------------------------------------------------------------------
	// Typical double to int cast is so slow on Intel X86 architecture that
	// we had better use alternative algorithm to do so.
	// Works for -32728 to 32727.99999236688
	template<typename TYPE_32> struct FASTFLOOR<TYPE_32,4>
	{
		static TYPE_32 fast_floor(double val)
		{
#if USE_FAST_FLOOR
			val = val + (68719476736.0 * 1.5);
			enum {IMAN = BIGENDIAN ? 1 : 0};
			return (((TYPE_32*)&val)[IMAN] >> 16);
#else
			return (TYPE_32)val;
#endif
		}
	};

	//------------------------------------------------------------------------
	template<typename INTTYPE> INTTYPE fast_floor(double val)
	{
		return FASTFLOOR<INTTYPE, sizeof(INTTYPE)>::fast_floor(val);
	}
	
	//------------------------------------------------------------------------
	inline size_t fast_floor_u(double val) {return fast_floor<size_t>(val);}
	inline long fast_floor_i(double val) {return fast_floor<long>(val);}

	//------------------------------------------------------------------------
	inline size_t fast_floor_u(float val) {return (size_t)val;}
	inline long fast_floor_i(float val) {return (long)val;}

	//------------------------------------------------------------------------
	struct SHRINK_MAP
	{
		size_t delta;
		FLOAT rx;
	};
	
	//------------------------------------------------------------------------
	inline size_t create_shrink_map(
		SHRINK_MAP* PZI_RESTRICT ioMap,
		size_t inLeft,
		size_t inRight,
		size_t inSrcWidth,
		FLOAT dx)
	{
		size_t r = inRight;
		ioMap -= inLeft;
		FLOAT lx = dx * inLeft;
		size_t xx = fast_floor_u(lx);
		FLOAT rx = lx - (FLOAT)xx;
		for(size_t x = inLeft; x <= inRight; x++)
		{
			FLOAT lxNext = dx * (x + 1);
			size_t xxNext = fast_floor_u(lxNext);
			if(r == inRight && xxNext + 1 >= inSrcWidth)
				r = x;

			ioMap[x].delta = xxNext - xx;
			ioMap[x].rx = rx;
			xx = xxNext;
			rx = lxNext - (FLOAT)xxNext;
		}
		return r - inLeft;
	}

	//------------------------------------------------------------------------
	template<size_t CHANNELS, bool FREE_END> void shrink_horz(
		FLOAT* PZI_RESTRICT outDest,
		const u8* PZI_RESTRICT inSrc,
		size_t inDestR,
		size_t inDestWidth,
		const SHRINK_MAP* PZI_RESTRICT inMap,
		FLOAT dx = 0)
	{
		const FLOAT* d = outDest;
		const u8* s = inSrc;
		size_t x;
		for(x = 0; x < inDestR; x++)
		{
			const SHRINK_MAP& m0 = inMap[x];
			const SHRINK_MAP& m1 = inMap[x + 1];
			const size_t delta = m0.delta;

			if(delta == 0)
			{
				const FLOAT d0 = m1.rx - m0.rx;
				const FLOAT d1 = m0.rx + m1.rx;
				const FLOAT r0 = (2 - d1)  * d0 / 2;
				const FLOAT r1 = d1 * d0 / 2;
				
				for(size_t i = 0; i < CHANNELS; i++)
					outDest[i] = r0 * inSrc[i];
				inSrc += CHANNELS;
				for(size_t i = 0; i < CHANNELS; i++)
					outDest[i] += r1 * inSrc[i];
				
				inSrc -= CHANNELS;
				outDest += CHANNELS;
			}
			else if(delta == 1)
			{
				const FLOAT r0 = (1 - m0.rx) * (1 - m0.rx) / 2;
				const FLOAT r1 = (1 - m0.rx * m0.rx + m1.rx * (2 - m1.rx)) / 2;
				const FLOAT r2 = m1.rx * m1.rx / 2;
				
				for(size_t i = 0; i < CHANNELS; i++)
					outDest[i] = r0 * inSrc[i];
				inSrc += CHANNELS;
				for(size_t i = 0; i < CHANNELS; i++)
					outDest[i] += r1 * inSrc[i];
				inSrc += CHANNELS;
				for(size_t i = 0; i < CHANNELS; i++)
					outDest[i] += r2 * inSrc[i];
				
				inSrc -= CHANNELS;
				outDest += CHANNELS;
			}
			else
			{
				const FLOAT r0 = (1 - m0.rx) * (1 - m0.rx) / 2;
				const FLOAT r1 = (2 - m0.rx * m0.rx) / 2;
				const FLOAT r2 = (m1.rx * (2 - m1.rx) + 1) / 2;
				const FLOAT r3 = m1.rx * m1.rx / 2;
				
				for(size_t i = 0; i < CHANNELS; i++)
					outDest[i] = r0 * inSrc[i];
				inSrc += CHANNELS;
				for(size_t i = 0; i < CHANNELS; i++)
					outDest[i] += r1 * inSrc[i];
				inSrc += CHANNELS;
				for(size_t j = 2; j < delta; j++)
				{
					for(size_t i = 0; i < CHANNELS; i++)
						outDest[i] += inSrc[i];
					inSrc += CHANNELS;
				}
				for(size_t i = 0; i < CHANNELS; i++)
					outDest[i] += r2 * inSrc[i];
				inSrc += CHANNELS;
				for(size_t i = 0; i < CHANNELS; i++)
					outDest[i] += r3 * inSrc[i];
				
				inSrc -= CHANNELS;
				outDest += CHANNELS;
			}
		}
		
		if(!FREE_END)
		{
			for(; x < inDestWidth; x++)
			{
				for(size_t i = 0; i < CHANNELS; i++)
					outDest[i] = dx * inSrc[i];
				outDest += CHANNELS;
			}
			return;
		}
	}

	//------------------------------------------------------------------------
	inline void mul_copy(
		FLOAT* PZI_RESTRICT outDest,
		size_t inSize,
		const FLOAT* PZI_RESTRICT inSrc,
		FLOAT m)
	{
		for(size_t x = 0; x < inSize; x++)
			outDest[x] = inSrc[x] * m;
	}
	
	//------------------------------------------------------------------------
	inline void accumulate(
		FLOAT* PZI_RESTRICT outDest,
		size_t inSize,
		const FLOAT* PZI_RESTRICT inSrc,
		FLOAT m)
	{
		for(size_t x = 0; x < inSize; x++)
			outDest[x] += inSrc[x] * m;
	}


	//------------------------------------------------------------------------
	template<size_t CHANNELS> void enlarge(
		u8* PZI_RESTRICT outDest,
		long inDestRowStride,
		size_t inDestLeft,
		size_t inDestTop,
		size_t inDestWidth,
		size_t inDestHeight,
		size_t inDestAllWidth,
		size_t inDestAllHeight,
		const u8* PZI_RESTRICT inSrc,
		size_t inSrcRowStride,
		size_t inSrcWidth,
		size_t inSrcHeight)
	{
		const FLOAT dx = (FLOAT)inSrcWidth / inDestAllWidth; // dx <= 1.0
		const FLOAT dy = (FLOAT)inSrcHeight / inDestAllHeight; // dx <= 1.0
		const size_t inDestRight = inDestLeft + inDestWidth;
		const size_t inDestBottom = inDestTop + inDestHeight;

		outDest -= inDestTop * inDestRowStride;
		for(size_t y = inDestTop; y < inDestBottom; y++)
		{
			FLOAT ly = dy * y;
			if(ly < 0) ly = 0;
			size_t yy = fast_floor_u(ly);
			FLOAT ry = ly - (FLOAT)yy;
			u8* p = outDest + y * inDestRowStride;
			const u8* q0 = inSrc + yy * inSrcRowStride;
			const u8* q1;
			if(yy + 1 == inSrcHeight)
				q1 = q0;
			else
				q1 = q0 + inSrcRowStride;

			for(size_t x = inDestLeft; x < inDestRight; x++)
			{
				FLOAT lx = dx * x;
				if(lx < 0) lx = 0;
				size_t xx = fast_floor_u(lx);
				FLOAT rx = lx - (FLOAT)xx;

				const u8* qx0 = q0 + xx * CHANNELS;
				const u8* qx1 = q1 + xx * CHANNELS;

				if(xx + 1 == inSrcWidth)
				{
					for(size_t i = 0; i < CHANNELS; i++)
					{
						long v = fast_floor_i(
							qx0[i] * (1.0 - ry) + qx1[i]* ry);
						//if(v < 0) v = 0; else if(v > 255) v = 255;
						*p++ = (u8)v;
					}
					continue;
				}

				for(size_t i = 0; i < CHANNELS; i++)
				{
					long v = fast_floor_i(
						((1.0 - rx) * qx0[i] + rx * qx0[i + CHANNELS]) * (1.0 - ry)
						+ ((1.0 - rx) * qx1[i] + rx * qx1[i + CHANNELS]) * ry);
					//if(v < 0) v = 0; else if(v > 255) v = 255;
					*p++ = (u8)v;
				}
			}
		}
	}
	
	//------------------------------------------------------------------------
	template<size_t CHANNELS> void shrink(
		u8* PZI_RESTRICT outDest,
		long inDestRowStride,
		size_t inDestLeft,
		size_t inDestTop,
		size_t inDestWidth,
		size_t inDestHeight,
		size_t inDestAllWidth,
		size_t inDestAllHeight,
		const u8* PZI_RESTRICT inSrc,
		size_t inSrcRowStride,
		size_t inSrcWidth,
		size_t inSrcHeight)
	{
		const FLOAT dx = (FLOAT)inSrcWidth / inDestAllWidth; // dx >= 1.0
		const FLOAT dy = (FLOAT)inSrcHeight / inDestAllHeight; // dy >= 1.0
		const size_t inDestRight = inDestLeft + inDestWidth;
		const size_t inDestBottom = inDestTop + inDestHeight;
		const size_t lineSize = inDestWidth * CHANNELS;
		std::vector<FLOAT> buf(lineSize);
		std::vector<FLOAT> tmpBuf(lineSize * 2);
		FLOAT* line0 = &tmpBuf[0];
		FLOAT* line1 = &line0[lineSize];

		size_t left = fast_floor_u(inDestLeft * dx);
		std::vector<SHRINK_MAP> smap(inDestWidth + 1);
		const size_t inDestR = create_shrink_map(&smap[0], inDestLeft, inDestRight, inSrcWidth, dx);
		inSrc += left * CHANNELS;
		
		FLOAT ly1 = dy * inDestTop;
		size_t yy1 = fast_floor_u(ly1);
		FLOAT ry1 = ly1 - (FLOAT)yy1;
		const u8* src = inSrc + yy1 * inSrcRowStride;
		shrink_horz<CHANNELS, false>(line0, src, inDestR, inDestWidth, &smap[0], dx);
		if(yy1 + 1 < inSrcHeight)
		{
			src += inSrcRowStride;
			shrink_horz<CHANNELS, false>(line1, src, inDestR, inDestWidth, &smap[0], dx);
		}

		for(size_t y = inDestTop; y < inDestBottom; y++)
		{
			FLOAT ly0 = ly1;
			size_t yy0 = yy1;
			FLOAT ry0 = ry1;
			size_t y1 = y + 1;
			ly1 = dy * y1;
			yy1 = fast_floor_u(ly1);
			ry1 = ly1 - (FLOAT)yy1;

			if(yy0 + 1 >= inSrcHeight || yy1 + 1 >= inSrcHeight)
			{
				for(size_t x = 0; x < lineSize; x++)
				{
					long f = fast_floor_i(line0[x] / dx);
					if(CHECK_SATURATION)
					{
						if(f < 0) f = 0; else if(f > 255) f = 255;
					}
					outDest[x] = (u8)f;
				}

				const u8* pCopySrc = outDest;
				for(y++; y < inDestBottom; y++)
				{
					outDest += inDestRowStride;
					std::memcpy(outDest, pCopySrc, lineSize);
				}
				return;
			}

			if(yy0 == yy1)
			{
				FLOAT d0 = ry1 - ry0;
				FLOAT d1 = ry0 + ry1;
				FLOAT r0 = (2 - d1)  * d0 / 2;
				FLOAT r1 = d1 * d0 / 2;
				mul_copy(&buf[0], lineSize, line0, r0);
				accumulate(&buf[0], lineSize,
					yy1 + 1 < inSrcHeight ? line1 : line0, r1);
			}
			else if(yy0 + 1 == yy1)
			{
				FLOAT r0 = (1 - ry0) * (1 - ry0) / 2;
				FLOAT r1 = (1 - ry0 * ry0 + ry1 * (2 - ry1)) / 2;
				FLOAT r2 = ry1 * ry1 / 2;
				mul_copy(&buf[0], lineSize, line0, r0);
				accumulate(&buf[0], lineSize, line1, r1);
				
				src = inSrc + (yy0 + 2) * inSrcRowStride;
				shrink_horz<CHANNELS, false>(
					line0, src, inDestR, inDestWidth, &smap[0], dx);
				accumulate(&buf[0], lineSize, line0, r2);
				
				std::swap(line0, line1);
			}
			else
			{
				FLOAT r0 = (1 - ry0) * (1 - ry0) / 2;
				FLOAT r1 = (2 - ry0 * ry0) / 2;
				FLOAT r2 = (ry1 * (2 - ry1) + 1) / 2;
				FLOAT r3 = ry1 * ry1 / 2;
				mul_copy(&buf[0], lineSize, line0, r0);
				accumulate(&buf[0], lineSize, line1, r1);
				
				src = inSrc + (yy0 + 2) * inSrcRowStride;
				for(size_t i = yy0 + 2; i < yy1; i++)
				{
					shrink_horz<CHANNELS, false>(
						line0, src, inDestR, inDestWidth, &smap[0], dx);
					accumulate(&buf[0], lineSize, line0, 1);
					src += inSrcRowStride;
				}

				shrink_horz<CHANNELS, false>(
					line0, src, inDestR, inDestWidth, &smap[0], dx);
				accumulate(&buf[0], lineSize, line0, r2);
				src += inSrcRowStride;
				shrink_horz<CHANNELS, false>(
					line1, src, inDestR, inDestWidth, &smap[0], dx);
				accumulate(&buf[0], lineSize, line1, r3);
			}

			FLOAT d = dx * dy;
			for(size_t x = 0; x < lineSize; x++)
			{
				long f = fast_floor_i(buf[x] / d);
				if(CHECK_SATURATION)
				{
					if(f < 0) f = 0; else if(f > 255) f = 255;
				}
				outDest[x] = (u8)f;
			}
			outDest += inDestRowStride;
		}
	}

} // _zoomimpl_core_

namespace Zoom
{
	using namespace _zoomimpl_core_;
	
	//------------------------------------------------------------------------
	template<size_t CHANNELS> void rescale(
		u8* PZI_RESTRICT outDest,
		long inDestRowStride,
		size_t inDestLeft,
		size_t inDestTop,
		size_t inDestWidth,
		size_t inDestHeight,
		size_t inDestAllWidth,
		size_t inDestAllHeight,
		const u8* PZI_RESTRICT inSrc,
		size_t inSrcRowStride,
		size_t inSrcWidth,
		size_t inSrcHeight)
	{
		if(inDestAllWidth == inSrcWidth && inDestAllHeight == inSrcHeight)
		{
			size_t bytes2Copy = inDestWidth * CHANNELS;
			size_t h = inDestHeight;
			inSrc += inDestLeft * CHANNELS + inDestTop * inDestRowStride;
			for(size_t y = 0; y < h; y++)
			{
				std::memcpy(outDest, inSrc, bytes2Copy);
				outDest += inDestRowStride;
				inSrc += inSrcRowStride;
			}
			return;
		}

		if(inDestAllWidth >= inSrcWidth || inDestAllHeight >= inSrcHeight)
			enlarge<CHANNELS>(
				outDest, inDestRowStride,
				inDestLeft, inDestTop, inDestWidth, inDestHeight,
				inDestAllWidth, inDestAllHeight,
				inSrc, inSrcRowStride, inSrcWidth, inSrcHeight);
		else
			shrink<CHANNELS>(
				outDest, inDestRowStride,
				inDestLeft, inDestTop, inDestWidth, inDestHeight,
				inDestAllWidth, inDestAllHeight,
				inSrc, inSrcRowStride, inSrcWidth, inSrcHeight);
	}
}

#endif // _zoomlogic_h_
