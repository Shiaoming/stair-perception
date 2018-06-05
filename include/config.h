/*
 * config.h
 *
 *  Created on: 2017年11月20日
 *      Author: zxm
 */

#ifndef INCLUDE_STAIRDETECTION_CONFIG_H_
#define INCLUDE_STAIRDETECTION_CONFIG_H_

#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <omp.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/thread/thread.hpp>
#include <boost/format.hpp>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>

class Config
{
	private:
		static boost::shared_ptr<Config> config_;
		cv::FileStorage file_;

		Config () {} // private constructor makes a singleton
	public:
		~Config()  // close the file when deconstructing
		{
		    if ( file_.isOpened() )
		        file_.release();
		}

		// set a new config file
		static void setParameterFile( const std::string& filename )
		{
		    if ( config_ == nullptr )
		        config_ = boost::shared_ptr<Config>(new Config);
		    config_->file_ = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );
		    if (!config_->file_.isOpened())
		    {
		        std::cerr<<"parameter file "<<filename<<" does not exist."<<std::endl;
		        config_->file_.release();
		        exit(0);
		        //return;
		    }
		}


		// access the parameter values
		template< typename T >
		static T get( const std::string& key )
		{
			return T( Config::config_->file_[key] );
		}
};

#endif /* INCLUDE_STAIRDETECTION_CONFIG_H_ */
