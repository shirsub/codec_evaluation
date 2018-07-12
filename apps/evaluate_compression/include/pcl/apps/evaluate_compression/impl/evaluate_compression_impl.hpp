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
#include <vector>
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
	boost::shared_ptr<pcl::PointCloud<PointT> > pc(new PointCloud<PointT>());
	std::string filename;
	filename = argv[1];
	//cout << filename;
	load_input_cloud(filename, pc);
	return return_value;
}
#endif /* evaluate_compression_hpp */