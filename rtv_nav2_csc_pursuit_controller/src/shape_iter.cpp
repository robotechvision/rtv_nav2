
// Copyright (c) 2021 RoboTech Vision
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include <rtv_nav2_csc_pursuit_controller/shape_iter.hpp>
#include <opencv2/imgproc.hpp>

namespace cv {
namespace iter {

enum { XY_SHIFT = 16, XY_ONE = 1 << XY_SHIFT, DRAWING_STORAGE_BLOCK = (1<<12) - 256 };

static void
Line( Mat& img, Point pt1, Point pt2,
      const std::function<void(uint32_t, uchar *)> &f, int connectivity = 8 )
{
    if( connectivity == 0 )
        connectivity = 8;
    else if( connectivity == 1 )
        connectivity = 4;

    LineIterator iterator(img, pt1, pt2, connectivity, true);
    int i, count = iterator.count;

    // printf("%d %d -> %d %d\n", pt1.x, pt1.y, pt2.x, pt2.y);
    for( i = 0; i < count; i++, ++iterator ) {
        // printf("i: %d\n", i);
        f(1, *iterator);
    }
}

static inline void ICV_HLINE(uchar* ptr, int xl, int xr, const std::function<void(uint32_t, uchar *)> &f, int pix_size)
{
    if (xr >= xl)
        f(xr+1 - xl, ptr + xl*pix_size);
}
//end ICV_HLINE()

void
iterateFillPoly( Mat& img, const Point2l* v, int npts, const std::function<void(uint32_t, uchar *)> &f, int line_type )
{
    struct
    {
        int idx, di;
        int64 x, dx;
        int ye;
    }
    edge[2];

    int delta = 0;//1 << shift >> 1;
    int i, y, imin = 0;
    int edges = npts;
    int64 xmin, xmax, ymin, ymax;
    uchar* ptr = img.ptr();
    Size size = img.size();
    int pix_size = (int)img.elemSize();
    Point2l p0;
    int delta1, delta2;

    if( line_type < LINE_AA )
        delta1 = delta2 = XY_ONE >> 1;
    else
        delta1 = XY_ONE - 1, delta2 = 0;

    p0 = v[npts - 1];
    p0.x <<= XY_SHIFT;// - shift;
    p0.y <<= XY_SHIFT;// - shift;

    // assert( 0 <= shift && shift <= XY_SHIFT );
    xmin = xmax = v[0].x;
    ymin = ymax = v[0].y;

    for( i = 0; i < npts; i++ )
    {
        Point2l p = v[i];
        if( p.y < ymin )
        {
            ymin = p.y;
            imin = i;
        }

        ymax = std::max( ymax, p.y );
        xmax = std::max( xmax, p.x );
        xmin = MIN( xmin, p.x );

        p.x <<= XY_SHIFT;// - shift;
        p.y <<= XY_SHIFT;// - shift;

        if( line_type <= 8 )
        {
            Point pt0, pt1;
            pt0.x = (int)(p0.x >> XY_SHIFT);
            pt0.y = (int)(p0.y >> XY_SHIFT);
            pt1.x = (int)(p.x >> XY_SHIFT);
            pt1.y = (int)(p.y >> XY_SHIFT);
            Line( img, pt0, pt1, f, line_type );
        }
        p0 = p;
    }

    xmin = (xmin + delta);// >> shift;
    xmax = (xmax + delta);// >> shift;
    ymin = (ymin + delta);// >> shift;
    ymax = (ymax + delta);// >> shift;

    if( npts < 3 || (int)xmax < 0 || (int)ymax < 0 || (int)xmin >= size.width || (int)ymin >= size.height )
        return;

    ymax = MIN( ymax, size.height - 1 );
    edge[0].idx = edge[1].idx = imin;

    edge[0].ye = edge[1].ye = y = (int)ymin;
    edge[0].di = 1;
    edge[1].di = npts - 1;

    edge[0].x = edge[1].x = -XY_ONE;
    edge[0].dx = edge[1].dx = 0;

    ptr += img.step*y;

    do
    {
        if( line_type < LINE_AA || y < (int)ymax || y == (int)ymin )
        {
            for( i = 0; i < 2; i++ )
            {
                if( y >= edge[i].ye )
                {
                    int idx0 = edge[i].idx, di = edge[i].di;
                    int idx = idx0 + di;
                    if (idx >= npts) idx -= npts;
                    int ty = 0;

                    for (; edges-- > 0; )
                    {
                        ty = (int)((v[idx].y + delta)/* >> shift*/);
                        if (ty > y)
                        {
                            int64 xs = v[idx0].x;
                            int64 xe = v[idx].x;
                            // if (shift != XY_SHIFT)
                            // {
                                xs <<= XY_SHIFT;// - shift;
                                xe <<= XY_SHIFT;// - shift;
                            // }

                            edge[i].ye = ty;
                            edge[i].dx = ((xe - xs)*2 + (ty - y)) / (2 * (ty - y));
                            edge[i].x = xs;
                            edge[i].idx = idx;
                            break;
                        }
                        idx0 = idx;
                        idx += di;
                        if (idx >= npts) idx -= npts;
                    }
                }
            }
        }

        if (edges < 0)
            break;

        if (y >= 0)
        {
            int left = 0, right = 1;
            if (edge[0].x > edge[1].x)
            {
                left = 1, right = 0;
            }

            int xx1 = (int)((edge[left].x + delta1) >> XY_SHIFT);
            int xx2 = (int)((edge[right].x + delta2) >> XY_SHIFT);

            if( xx2 >= 0 && xx1 < size.width )
            {
                if( xx1 < 0 )
                    xx1 = 0;
                if( xx2 >= size.width )
                    xx2 = size.width - 1;
                // printf("%d (%d -> %d)\n", y, xx1, xx2);
                ICV_HLINE( ptr, xx1, xx2, f, pix_size );
            }
        }
        else
        {
            // TODO optimize scan for negative y
        }

        edge[0].x += edge[0].dx;
        edge[1].x += edge[1].dx;
        ptr += img.step;
    }
    while( ++y <= (int)ymax );
}

}
}