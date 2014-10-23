/**
 * @file gnd_ls_coordtf/src/main.cpp
 *
 * @brief Test Program
 **/

#include "gnd/gnd-multi-platform.h"
#include "gnd/gnd_ls_coordtf.hpp"

#include <stdio.h>

#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/ChannelFloat32.h"

#include "gnd/gnd_rosutil.hpp"
#include "gnd/gnd_rosmsg_reader.hpp"

#include "gnd/gnd-matrix-base.hpp"
#include "gnd/gnd-vector-base.hpp"
#include "gnd/gnd-matrix-coordinate.hpp"
#include "gnd/gnd-util.h"

// node config
typedef gnd::ls_coordtf::node_config 							node_config_t;

// objects for ros communication
typedef sensor_msgs::LaserScan									msg_laserscan_t;
typedef gnd::rosutil::rosmsgs_reader_stamped<msg_laserscan_t>	msgreader_laserscan_t;
typedef sensor_msgs::PointCloud									msg_pointcloud_t;
typedef sensor_msgs::PointCloud::_points_type::value_type		point_t;
typedef sensor_msgs::ChannelFloat32								msg_intensity_t;
typedef sensor_msgs::ChannelFloat32::_values_type::value_type	intensity_t;

typedef struct {
	char name[128];
	ros::Publisher	 pub;
	msg_pointcloud_t msg;
} topics_pointcloud_t;

int main(int argc, char **argv) {
	node_config_t			node_config;

	{ // ---> start up, read configuration file
		if( argc > 1 ) {
			if( gnd::ls_coordtf::fread_node_config( argv[1], &node_config ) < 0 ){
				char fname[1024];
				fprintf(stdout, "   ... Error: fail to read config file \"%s\"\n", argv[1]);
				sprintf(fname, "%s.tmp", argv[1]);
				// file out configuration file
				if( gnd::ls_coordtf::fwrite_node_config( fname, &node_config ) >= 0 ){
					fprintf(stdout, "            : output sample configuration file \"%s\"\n", fname);
				}
				return -1;
			}
			else {
				fprintf(stdout, "   ... read config file \"%s\"\n", argv[1]);
			}
		}

		{ // ---> check area
			int i = 0;
			while( i < (signed)node_config.areas.size() ) {
				int j = 0;
				while( j < (signed)node_config.areas[i].range.size() ) {
					if( node_config.areas[i].range[j].upper[0] < node_config.areas[i].range[j].lower[0] ||
						node_config.areas[i].range[j].upper[1] < node_config.areas[i].range[j].lower[1] ||
						node_config.areas[i].range[j].upper[2] < node_config.areas[i].range[j].lower[2] ) {
						node_config.areas[i].range.erase(j);
					}
					else {
						j++;
					}
				}

				while( j < (signed)node_config.areas[i].exception.size() ) {
					if( node_config.areas[i].exception[j].upper[0] < node_config.areas[i].exception[j].lower[0] ||
						node_config.areas[i].exception[j].upper[1] < node_config.areas[i].exception[j].lower[1] ||
						node_config.areas[i].exception[j].upper[2] < node_config.areas[i].exception[j].lower[2] ) {
						node_config.areas[i].exception.erase(j);
					}
					else {
						j++;
					}
				}

				if( node_config.areas[i].range.size() == 0 )	node_config.areas.erase(i);
				else											i++;
			}
		}// <--- check area

	} // <--- start up, read configuration file



	{ // ---> initialize rosÅ@platform
		if( node_config.node_name.value[0] ) {
			ros::init(argc, argv, node_config.node_name.value);
		}
		else {
			fprintf(stdout, "   ... Error: node name is null, you must specify the name of this node via config item \"%s\"\n", node_config.node_name.item);
			return -1;
		}
		fprintf(stdout, " node: \"%s\"\n", node_config.node_name.value);
	} // <--- initialize rosÅ@platform


	ros::NodeHandle				nh_ros;

	ros::Subscriber				subsc_laserscan;
	msg_laserscan_t				msg_laserscan;
	msgreader_laserscan_t		msgreader_laserscan;

	char 						topic_name_pointcloud[128];
	ros::Publisher				pub_pointcloud;
	msg_pointcloud_t			msg_pointcloud;

	std::vector<topics_pointcloud_t> topic_areas;

	gnd::matrix::fixed<4,4> mat_coordtf;

	// debug
	FILE *fp_textlog = 0;


	{ // ---> initialize
		int phase = 0;
		ros::Time time_start;
		fprintf(stdout, "---------- initialize ----------\n");

		// ---> show initialize phase task
		if( ros::ok() ) {
			fprintf(stdout, " initialization task\n");
			fprintf(stdout, "   %d. calculate coordinate transform matrix \n", ++phase);
			fprintf(stdout, "   %d. make laser-scan topic subscriber\n", ++phase);
			fprintf(stdout, "   %d. make point-cloud topic publisher\n", ++phase);
			if( node_config.areas.size() > 0 ) {
				fprintf(stdout, "   %d. make each areas point-cloud topic publishers\n", ++phase);
			}
			if( node_config.text_log.value[0] ) {
				fprintf(stdout, "   %d. create text log file\n", ++phase);
			}
			fprintf(stdout, "\n");
		} // <--- show initialize phase task


		// ---> calculate coordinate transform matrix
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => calculate coordinate transform matrix\n");

			if( !node_config.coordinate_name.value[0] ) {
				fprintf(stderr, "    ... error: coordinate name is null\n");
				ros::shutdown();
			}
			else {
				fprintf(stdout, "    ... coordinate name \"%s\"\n", node_config.coordinate_name.value);
				fprintf(stdout, "        origin: %.03lf, %.03lf, %.03lf\n",
						node_config.coordinate_origin.value[0], node_config.coordinate_origin.value[1], node_config.coordinate_origin.value[2]);
				fprintf(stdout, "         front: %.03lf, %.03lf, %.03lf\n",
						node_config.axis_vector_front.value[0], node_config.axis_vector_front.value[1], node_config.axis_vector_front.value[2]);
				fprintf(stdout, "        upside: %.03lf, %.03lf, %.03lf\n",
						node_config.axis_vector_upside.value[0], node_config.axis_vector_upside.value[1], node_config.axis_vector_upside.value[2]);

				gnd::matrix::coordinate_converter(&mat_coordtf,
						node_config.coordinate_origin.value[0], node_config.coordinate_origin.value[1], node_config.coordinate_origin.value[2],
						node_config.axis_vector_front.value[0], node_config.axis_vector_front.value[1], node_config.axis_vector_front.value[2],
						node_config.axis_vector_upside.value[0], node_config.axis_vector_upside.value[1], node_config.axis_vector_upside.value[2]);

				fprintf(stdout, "    ... coordinate transform matrix:\n");
				gnd::matrix::show(stdout, &mat_coordtf, "%.03lf", "        ");
				fprintf(stdout, "    ... ok, show matrix\n");
			}
		} // <--- calculate coordinate transform matrix


		// ---> make laserscan subscriber
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => make laser-scan topic subscriber\n");

			if( !node_config.topic_name_laserscan.value[0] ) {
				fprintf(stderr, "    ... error: laser scan topic name is null\n");
				fprintf(stderr, "        usage: fill \"%s\" item in configuration file\n", node_config.topic_name_laserscan.item);
				ros::shutdown();
			}
			else {
				fprintf(stdout, "    ... topic name is \"%s\"\n", node_config.topic_name_laserscan.value);

				// allocate buffer
				msgreader_laserscan.allocate(400);

				// subscribe
				subsc_laserscan = nh_ros.subscribe(node_config.topic_name_laserscan.value, 400,
						&msgreader_laserscan_t::rosmsg_read,
						msgreader_laserscan.reader_pointer() );
				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make laserscan subscriber


		// ---> make pointcloud publisher
		if( ros::ok() ) {
			fprintf(stdout, "\n");
			fprintf(stdout, " => make point-cloud topic publisher\n");

			if( !node_config.coordinate_name.value ){
				fprintf(stderr, "    ... error: coordinate name is null\n");
				fprintf(stderr, "        usage: fill \"%s\" item in configuration file\n", node_config.coordinate_name.item);
				ros::shutdown();
			}
			else {
				sprintf(topic_name_pointcloud, "%s/%s", node_config.coordinate_name.value, node_config.topic_name_laserscan.value);

				pub_pointcloud = nh_ros.advertise<msg_pointcloud_t>(topic_name_pointcloud, 400);

				msg_pointcloud.header.seq = 0;
				msg_pointcloud.header.stamp = time_start;
				msg_pointcloud.header.frame_id = node_config.coordinate_name.value;

				msg_pointcloud.points.clear();
				msg_pointcloud.channels.resize(1);
				msg_pointcloud.channels[0].name = "intensity";
				msg_pointcloud.channels[0].values.clear();

				fprintf(stdout, "    ... ok\n");
			}
		} // <--- make pointcloud publisher

		// ---> make each areas pointcloud publisher
		if( ros::ok() && node_config.areas.size() > 0 ) {
			int i;
			topics_pointcloud_t topics_pointcloud;
			fprintf(stdout, "\n");
			fprintf(stdout, " => make each areas point-cloud topic publisher\n");

			topic_areas.resize(node_config.areas.size());
			for( i = 0; i < (signed)node_config.areas.size(); i++ ) {
				sprintf(topic_areas[i].name, "%s/%s", topic_name_pointcloud, node_config.areas[i].name);

				topic_areas[i].pub = nh_ros.advertise<msg_pointcloud_t>(topic_areas[i].name, 400);

				topic_areas[i].msg.header.seq = 0;
				topic_areas[i].msg.header.stamp = time_start;
				topic_areas[i].msg.header.frame_id = node_config.coordinate_name.value;

				topic_areas[i].msg.points.clear();
				topic_areas[i].msg.channels.resize(1);
				topic_areas[i].msg.channels[0].name = "intensity";
				topic_areas[i].msg.channels[0].values.clear();
			}
			fprintf(stdout, "    ... ok\n");
		} // <--- make each areas pointcloud publisher

		// --->
		if( ros::ok() && node_config.text_log.value[0] ) {
			fprintf(stdout, "\n");
			fprintf(stdout, "   => create text log file\n", ++phase);

			if( !(fp_textlog = fopen(node_config.text_log.value, "w")) ){
				fprintf(stderr, "    ... error: fail to open \"%s\"\n", node_config.text_log.value );
				ros::shutdown();
			}
			else {
				fprintf(fp_textlog, "#[1. sequence id] [2. x] [3. y]\n");
				fprintf(stdout, "    ... ok, create file \"%s\"\n", node_config.text_log.value );
			}
		}
		// <---

	} // <--- initialize



	if( ros::ok() ){ // ---> operate

		ros::Rate loop_rate(1000);
		double time_current;
		double time_start;
		double time_display;
		int nline_display = 0;


		{ // ---> initialize time
			time_current = ros::Time::now().toSec();
			time_start = time_current;
			time_display = time_start;
		} // <--- initialize time


		// ---> main loop

		fprintf(stderr, " => %s main loop start\n", node_config.node_name.value);
		while( ros::ok() ) {
			// blocking
			loop_rate.sleep();
			// spin once
			ros::spinOnce();

			// time
			time_current = ros::Time::now().toSec();

			// ---> coordinate transform and publish
			if( msgreader_laserscan.copy_next( &msg_laserscan, msg_laserscan.header.seq) == 0) {
				// ---> except no-data case
				if( msg_laserscan.ranges.size() > 0 ) {
					unsigned int i = 0;
					unsigned int j = 0;
					unsigned int k = 0;
					point_t ws_point;			// work space

					msg_pointcloud.points.clear();
					msg_pointcloud.channels[0].values.clear();
					for( j = 0; j < (signed)topic_areas.size(); j++ ) {
						topic_areas[j].msg.points.clear();
						topic_areas[j].msg.channels[0].values.clear();
					}

					// ---> coordinate transform and set value
					for( i = 0; i < (signed)msg_laserscan.ranges.size(); i++ ) {
						gnd::vector::fixed_column<4> point_src, point_dest;

						// error
						if( msg_laserscan.ranges[i] < msg_laserscan.range_min ||
							msg_laserscan.ranges[i] > msg_laserscan.range_max) {
							continue;
						}

						// source coordinate point
						point_src[0] = msg_laserscan.ranges[i] * cos( msg_laserscan.angle_min + msg_laserscan.angle_increment * i );
						point_src[1] = msg_laserscan.ranges[i] * sin( msg_laserscan.angle_min + msg_laserscan.angle_increment * i );
						point_src[2] = 0;
						point_src[3] = 1;

						// coordinate transform
						gnd::matrix::prod( &mat_coordtf, &point_src, &point_dest );

						// set destination coordinate point
						ws_point.x = point_dest[0];
						ws_point.y = point_dest[1];
						ws_point.z = point_dest[2];
						msg_pointcloud.points.push_back(ws_point);

						// set intensity
						if( msg_laserscan.intensities.size() > i ) {
							msg_pointcloud.channels[0].values.push_back(msg_laserscan.intensities[i]);
						}

						// ---> extract each areas data
						for( j = 0; j < (signed)topic_areas.size(); j++ ) {
							for( k = 0; k < (signed)node_config.areas[j].range.size(); k++ ){
								if( ws_point.x > node_config.areas[j].range[k].upper[0] ) break;
								if( ws_point.y > node_config.areas[j].range[k].upper[1] ) break;
								if( ws_point.z > node_config.areas[j].range[k].upper[2] ) break;
								if( ws_point.x < node_config.areas[j].range[k].lower[0] ) break;
								if( ws_point.y < node_config.areas[j].range[k].lower[1] ) break;
								if( ws_point.z < node_config.areas[j].range[k].lower[2] ) break;
							}
							if( k < (signed)node_config.areas[j].range.size() ) continue;
							for( k = 0; k < (signed)node_config.areas[j].exception.size(); k++ ){
								if( ws_point.x < node_config.areas[j].range[k].upper[0] &&
									ws_point.x > node_config.areas[j].range[k].lower[0]) break;
								if( ws_point.y < node_config.areas[j].range[k].upper[1] &&
									ws_point.y > node_config.areas[j].range[k].lower[1]) break;
								if( ws_point.z < node_config.areas[j].range[k].upper[2] &&
									ws_point.z > node_config.areas[j].range[k].lower[2]) break;
							}
							if( k < (signed)node_config.areas[j].exception.size() ) continue;

							topic_areas[j].msg.points.push_back(ws_point);
							if( msg_laserscan.intensities.size() > i ) {
								topic_areas[j].msg.channels[0].values.push_back( msg_laserscan.intensities[i] );
							}
						} // <--- extract each areas data

						// text log
						if( fp_textlog ) {
							fprintf( fp_textlog, "%d %lf %lf %lf %lf\n", msg_pointcloud.header.seq + 1, point_src[0], msg_pointcloud.points[i].y, point_src[0], point_src[1] );
						}
					} // <--- coordinate transform and set value

					{ // ---> set header
						msg_pointcloud.header.seq++;
						msg_pointcloud.header.stamp = msg_laserscan.header.stamp;

						for( j = 0; j < (signed)topic_areas.size(); j++ ) {
							topic_areas[j].msg.header.seq = msg_pointcloud.header.seq;
							topic_areas[j].msg.header.stamp = msg_pointcloud.header.stamp;
						}
					} // <--- set header

					// publish
					pub_pointcloud.publish(msg_pointcloud);
					for( j = 0; j < (signed)topic_areas.size(); j++ ) {
						topic_areas[j].pub.publish(topic_areas[j].msg);
					}

				} // <--- except no-data case
			} // <--- coordinate transform and publish

			// ---> status display
			if( node_config.period_cui_status_display.value > 0 && node_config.period_cui_status_display.value && time_current > time_display ) {
				// clear
				if( nline_display ) {
					fprintf(stderr, "\x1b[%02dA", nline_display);
					nline_display = 0;
				}

				nline_display++; fprintf(stderr, "\x1b[K-------------------- \x1b[1m\x1b[36m%s\x1b[39m\x1b[0m --------------------\n", node_config.node_name.value);
				nline_display++; fprintf(stderr, "\x1b[K operating time : %6.01lf[sec]\n", time_current - time_start);
				nline_display++; fprintf(stderr, "\x1b[K    point cloud : topic name \"%s\" (publish)\n", topic_name_pointcloud );
				nline_display++; fprintf(stderr, "\x1b[K                :       size %3d [num]\n", msg_pointcloud.points.size() );
				nline_display++; fprintf(stderr, "\x1b[K                :  intensity %s\n", msg_pointcloud.channels[0].values.size() == msg_pointcloud.points.size() ? "on" : "off" );
				nline_display++; fprintf(stderr, "\x1b[K                :        seq %d\n", msg_pointcloud.header.seq );
				if( topic_areas.size() > 0 ) {
					nline_display++; fprintf(stderr, "\x1b[K    point cloud : topic name \"%s\" (publish)\n", topic_areas[0].name );
					nline_display++; fprintf(stderr, "\x1b[K                :       size %d\n", topic_areas[0].msg.points.size() );
				}
				nline_display++; fprintf(stderr, "\x1b[K     laser scan : topic name \"%s\" (subscribe)\n", node_config.topic_name_laserscan.value );
				nline_display++; fprintf(stderr, "\x1b[K                :       size %3d [num]\n", msg_laserscan.ranges.size() );
				nline_display++; fprintf(stderr, "\x1b[K                :        seq %d\n", msg_laserscan.header.seq );

				time_display = gnd_loop_next(time_current, time_start, node_config.period_cui_status_display.value);
			} // <--- status display

		} // <--- main loop
	} // <--- operate



	{ // ---> finalize
		if( fp_textlog ) {
			fclose(fp_textlog);
		}

		fprintf(stderr, " ... fin\n");
	} // <--- finalize

	return 0;
}
