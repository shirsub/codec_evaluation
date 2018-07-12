/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014- Centrum Wiskunde en Informatica
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of copyright holder(s)  nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */
//
//  evaluate_compression.cpp
//
//  Created by Kees Blom on 06/04/16.
//  Copyright (c) 2016- Centrum Wiskunde en Informatica. All rights reserved.
//
#include <evaluate_compression.h>
#include <evaluate_compression_impl.hpp>

struct encoder_params
{
	int num_threads;
	bool do_inter_frame;
	int gop_size;
	double exp_factor;
	int octree_bits;
	int color_bits;
	int jpeg_quality;
	int macroblock_size;
};

int main(int argc, char * argv[]) {
	encoder_params par;
	par.num_threads = 4;
	par.do_inter_frame = false;
	par.gop_size = 1;
	par.exp_factor = 0;
	par.octree_bits = 11;
	par.color_bits = 8;
	par.jpeg_quality = 85;
	par.macroblock_size = 16;
	int return_value;
	evaluate_compression_impl<PointXYZRGB> evaluator;

	//return evaluator.evaluate() == true ? 0 : -1;
	return_value = evaluator.evaluate(argc,argv) == true ? 0 : -1;
	class __declspec(dllimport) cwi_encode;
	std::stringstream comp_frame;
	return_value = cwi_encode.cwi_encoder(par, pc, compframe);
	return return_value;
	//__declspec(dllimport)
}
