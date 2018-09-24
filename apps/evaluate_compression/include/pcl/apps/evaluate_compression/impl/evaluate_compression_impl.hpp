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
 *   * Neither the name of copyright holder (s)  nor the names of its
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
//  evaluate_compression.hpp
//  evaluate_compression
//
//  Created by Kees Blom on 01/06/16.
//
//
#ifndef evaluate_compression_hpp
#define evaluate_compression_hpp
// c++ standard library
#include <fstream>
#include<string>
#include <vector>
#include <chrono>
#include <ctime> // for 'strftime'
#include <exception>
#include <stdio.h>
// boost library
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/program_options/parsers.hpp>
namespace po = boost::program_options;

// point cloud library
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <direct.h>
#include <stdlib.h>
#include <evaluate_compression.h>

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
class __declspec(dllimport) cwi_encode
{
public:
	int cwi_encoder(encoder_params param, void* pc, std::stringstream& comp_frame, std::uint64_t timeStamp);
	int cwi_decoder(encoder_params param, void* pc, std::stringstream& comp_frame, std::uint64_t &timeStamp);
};

template<typename PointT>
class evaluate_compression_impl : evaluate_compression {
  // using boost::exception on errors
  public:
    evaluate_compression_impl (){};
    // options handling
    void initialize_options_description ();
    bool get_options (int argc, char** argv);
    void assign_option_values ();
    po::options_description desc_;
    po::variables_map vm_;
    po::positional_options_description pod_;
	bool evaluate(int argc, char** argv);
    // preprocessing, encoding, decoding, quality assessment  void complete_initialization ();
    void complete_initialization ();
      // V1 (common) settings
};

// aux. functions for file reading
using namespace boost::filesystem;
using namespace pcl;
template<typename PointT>
int
load_pcd_file(std::string path, boost::shared_ptr<pcl::PointCloud<PointT> > pc)
{
	int rv = 1;
	PCDReader pcd_reader;
	if (pcd_reader.read(path, *pc) <= 0)
	{
		rv = 0;
	}
	return rv;
}

template<typename PointT>
int
load_ply_file(std::string path, boost::shared_ptr<pcl::PointCloud<PointT> > pc)
{
	int rv = 1;
	PLYReader ply_reader;
	/* next straighforward code c;rashes, work around via PolygonMesh *
	PCLPointCloud2 pc2;
	if (rv && ply_reader.read (path, pc2) < 0)
	{
	fromPCLPointCloud2 (pc2, *pc);
	rv = 0;
	}
	*/
	PolygonMesh mesh;
	if (rv && ply_reader.read(path, mesh) >= 0) {
		pcl::fromPCLPointCloud2(mesh.cloud, *pc);
	}
	else {
		rv = 0;
	}
	return rv;
}
template<typename PointT>
bool
load_input_cloud(std::string filename, boost::shared_ptr<pcl::PointCloud<PointT> > &cloud)
{
	bool rv = false;
	if (boost::ends_with(filename, ".ply"))
	{
		std::cout << "\n Loading ply " << filename;
		if (load_ply_file(filename, cloud))
		{
			rv = true;
		}
	}
	else
	{
		if (boost::ends_with(filename, ".pcd"))
		{
			if (load_pcd_file(filename, cloud))
			{
				rv = true;
			}
		}
	}
	return rv;
}
template<typename PointT>
bool
evaluate_compression_impl<PointT>::evaluate(int argc, char** argv)
{
	bool return_value = true;
	boost::shared_ptr<pcl::PointCloud<PointT> > orgpc(new PointCloud<PointT>());
	std::string filename;
	filename = argv[1];
	std::uint64_t t;
	t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	//cout << filename;
	
	load_input_cloud(filename, orgpc);
	void * pc;
	pc = reinterpret_cast<void *> (&orgpc);
	
	encoder_params par;
	par.num_threads = 1;
	par.do_inter_frame = false;
	par.gop_size = 1;
	par.exp_factor = 0;
	par.octree_bits = 7;
	par.color_bits = 8;
	par.jpeg_quality = 85;
	par.macroblock_size = 16;
	
	cwi_encode enc;
	std::stringstream compframe;
	return_value = enc.cwi_encoder(par, pc, compframe, t);
	std::cout << "Size of compressed frame after encoding : " << sizeof(compframe);
	std::ofstream compressedframe;
	compressedframe.open("compressedFrame.pcc", std::fstream::binary);
	compressedframe << compframe.rdbuf();
	compressedframe.close();
	std::ifstream file("compressedFrame.pcc", std::fstream::binary);
	std::stringstream cfr;
	cfr << file.rdbuf();
	file.close();
	
	//Write again to check for carriage return
	/*
	std::ofstream compressedframe2;
	compressedframe2.open("compressedFrame2.pcc");
	compressedframe2 << cfr.rdbuf();
	compressedframe2.close();
	*/
	//std::cout << "\n Size of dashed compressed file :" << sizeof(cfr) << "\n";
	//end dash trial
	//*/
	//std::ifstream file(filename, std::fstream::binary);
	//std::stringstream cfr;
	//cfr << file.rdbuf();
	//file.close();
	boost::shared_ptr<pcl::PointCloud<PointT> > decpc(new PointCloud<PointT>());
	decpc->makeShared();
	void * dpc;
	dpc = reinterpret_cast<void *> (&decpc);
	//std::cout << "\n Decoding now";
	std::uint64_t decodeStart;
	decodeStart = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	std::uint64_t t1;
	cwi_encode dec;
	//return_value = dec.cwi_decoder(par, dpc, compframe, t1);
	return_value = dec.cwi_decoder(par, dpc, cfr, t1);
	//std::cout << "\n\nDecoded";
	std::cout << "\nNumber of points in decoded cloud: " << (*decpc).points.size();
	std::uint64_t decodeEnd;
	decodeEnd = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	std::cout << "\nDecode took  :" << decodeEnd - decodeStart << " ms";
	std::cout << "\nStart time was :" << t1;
	pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2(*decpc, *cloud2);
	pcl::PLYWriter writer;
	std::string opfile = "decoded_" + filename;
	writer.write(opfile, cloud2);
	std::uint64_t endTime;
	endTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	//std::cout << "\nDecoded ply written\nWhole process took: " << endTime - t1 << " ms";
	return return_value;
}
#endif /* evaluate_compression_hpp */