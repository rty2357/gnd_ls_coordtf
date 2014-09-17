/*
 * gnd_ls_coordtf_config.hpp
 *
 *  Created on: 2014/09/05
 *      Author: tyamada
 */

#ifndef GND_LS_COORDTF_CONFIG_HPP_
#define GND_LS_COORDTF_CONFIG_HPP_

#include "gnd/gnd-util.h"
#include "gnd/gnd-config-file.hpp"
#include "gnd/gnd-lib-error.h"


// ---> type declaration
namespace gnd {
	namespace ls_coordtf {
		struct node_config;
		typedef struct node_config node_config;

		typedef gnd::conf::parameter_array<char, 256> param_string_t;
		typedef gnd::conf::parameter_array<double, 3> param_geo3d_t;
		typedef gnd::conf::param_int param_int_t;
		typedef gnd::conf::param_long param_long_t;
		typedef gnd::conf::param_double param_double_t;
		typedef gnd::conf::param_bool param_bool_t;
	}
} // <--- type declaration


// ---> const variables definition
namespace gnd {
	namespace ls_coordtf {
		// ---> ros communicate option
		static const param_string_t Default_node_name = {
				"node-name",
				"gnd_ls_coordtf",
				"ros-node name"
		};

		static const param_string_t Default_topic_name_laserscan = {
				"topic-name-laserscan",
				"scan",
				"laser scan topic name"
		};

		static const param_string_t Default_coordinate_name = {
				"coordinate-name",
				"coord_foo",
				"coordinate name, if you set <coordinate-name>, gnd_ls_coordtf publish PointCloud message named <coordinate-name>/<laser scan topic>"
		};
		// <--- ros communicate option

		// ---> coordinate option
		static const param_geo3d_t Default_coordinate_origin = {
				"coordinate-origin",
				{0, 0, 0},
				"the source coordinate origin position on the destination coordinate"
		};

		static const param_geo3d_t Default_axis_vector_front = {
				"coordinate-axis-vector-front",
				{1.0, 0.0, 0.0},
				"the source front axis on the destination coordinate (right hand system)"
		};

		static const param_geo3d_t Default_axis_vector_upside = {
				"coordinate-axis-vector-upside",
				{0.0, 0.0, 1.0},
				"the source front axis on the destination coordinate (right hand system)"
		};
		// <--- coordinate option



		// ---> debug option
		static const param_bool_t Default_status_display = {
				"cui-status-display",
				false,
				"display the node status in terminal. [note] it need ansi color code"
		};

		static const param_string_t Default_text_log = {
				"text-log",
				"",
				"text log file name"
		};
		// <--- debug option
	}
}
// <--- const variables definition



// ---> function declaration
namespace gnd {
	namespace ls_coordtf {

		/**
		 * @brief initialize configure to default parameter
		 * @param [out] p : node_config
		 */
		int init_node_config(node_config *conf);


		/**
		 * @brief config file read
		 * @param [in] fname : file name
		 * @param [out] dest : configuration parameter
		 */
		int fread_node_config( const char* fname, node_config *dest );
		/**
		 * @brief get config parameter from description
		 * @param [out] dest : configuration parameter
		 * @param [in]  src  : configuration description
		 */
		int get_node_config( node_config *dest, gnd::conf::configuration *src );



		/**
		 * @brief config file write
		 * @param [in] fname : file name
		 * @param [in] src   : configuration parameter
		 */
		int fwrite_node_config( const char* fname, node_config *src );
		/**
		 * @brief set config description
		 * @param [out] dest : description
		 * @param [in]   src : parameter
		 */
		int set_node_config( gnd::conf::configuration *dest, node_config *src );
	}
} // <--- function declaration



// ---> type definition
namespace gnd {
	namespace ls_coordtf {
		/**
		 * @brief configuration parameter for gnd_urg_proxy node
		 */
		struct node_config {
			node_config();

			// ros communication
			param_string_t				node_name;					///< node name
			param_string_t				topic_name_laserscan;		///< topic name of laser scan (publish)
			param_string_t				coordinate_name;			///< coordinate name

			// coordinate option
			param_geo3d_t				coordinate_origin;			///< coordinate origin
			param_geo3d_t				axis_vector_front;			///< front axis
			param_geo3d_t				axis_vector_upside;			///< upside axis

			// debug option
			param_bool_t				status_display;				///< cui status display mode
			param_string_t				text_log;					///< text log
		};

		inline
		node_config::node_config() {
			init_node_config(this);
		}
		// <--- struct node_config
	}
}
// <--- type definition



// ---> function definition
namespace gnd {
	namespace ls_coordtf {
		/*
		 * @brief initialize configuration parameter
		 * @param [out] p : node_config
		 */
		inline
		int init_node_config( node_config *p ){
			gnd_assert(!p, -1, "invalid null pointer argument\n" );

			// ros communication parameter
			memcpy( &p->node_name,				&Default_node_name,				sizeof(Default_node_name) );
			memcpy( &p->topic_name_laserscan,	&Default_topic_name_laserscan,	sizeof(Default_topic_name_laserscan) );
			memcpy( &p->coordinate_name,		&Default_coordinate_name,		sizeof(Default_coordinate_name) );
			// hard-ware parameter
			memcpy( &p->coordinate_origin,		&Default_coordinate_origin,		sizeof(Default_coordinate_origin) );
			memcpy( &p->axis_vector_front,		&Default_axis_vector_front,		sizeof(Default_axis_vector_front) );
			memcpy( &p->axis_vector_upside,		&Default_axis_vector_upside,	sizeof(Default_axis_vector_upside) );
			// debug option
			memcpy( &p->status_display,			&Default_status_display,		sizeof(Default_status_display) );
			memcpy( &p->text_log,				&Default_text_log,				sizeof(Default_text_log) );

			return 0;
		}

		/*
		 * @brief config file read
		 * @param [in] fname : file name
		 * @param [out] dest : configuration parameter
		 */
		inline
		int fread_node_config( const char* fname, node_config *dest ) {
			gnd_assert(!fname, -1, "invalid null pointer argument\n" );
			gnd_assert(!dest, -1, "invalid null pointer argument\n" );

			{ // ---> operation
				int ret;
				gnd::conf::file_stream fs;
				// configuration file read
				if( (ret = fs.read(fname)) < 0 )    return ret;

				return get_node_config(dest, &fs);
			} // <--- operation
		}
		/*
		 * @brief get config parameter from description
		 * @param [out] dest : configuration parameter
		 * @param [in]  src  : configuration description
		 */
		inline
		int get_node_config( node_config *dest, gnd::conf::configuration *src ) {
			gnd_assert(!dest, -1, "invalid null pointer argument\n" );
			gnd_assert(!src, -1, "invalid null pointer argument\n" );

			// ros communication parameter
			gnd::conf::get_parameter( src, &dest->node_name );
			gnd::conf::get_parameter( src, &dest->topic_name_laserscan );
			gnd::conf::get_parameter( src, &dest->coordinate_name );
			// coordinate option
			gnd::conf::get_parameter( src, &dest->coordinate_origin );
			gnd::conf::get_parameter( src, &dest->axis_vector_front );
			gnd::conf::get_parameter( src, &dest->axis_vector_upside );
			// debug option
			gnd::conf::get_parameter( src, &dest->status_display );
			gnd::conf::get_parameter( src, &dest->text_log );

			return 0;
		}



		/*
		 * @brief config file write
		 * @param [in] fname : file name
		 * @param [in] src   : configuration parameter
		 */
		inline
		int fwrite_node_config( const char* fname, node_config *src ) {
			gnd_assert(!fname, -1, "invalid null pointer argument\n" );
			gnd_assert(!src, -1, "invalid null pointer argument\n" );
			{ // ---> operation
				int ret;
				gnd::conf::file_stream fs;
				// convert configuration declaration
				if( (ret = set_node_config(&fs, src)) < 0 ) return ret;

				return fs.write(fname);
			} // <--- operation
		}

		/*
		 * @brief set config description
		 * @param [out] dest : description
		 * @param [in]   src : parameter
		 */
		inline
		int set_node_config( gnd::conf::configuration *dest, node_config *src ) {
			gnd_assert(!dest, -1, "invalid null pointer argument\n" );
			gnd_assert(!src, -1, "invalid null pointer argument\n" );

			// ros communication parameter
			gnd::conf::set_parameter( dest, &src->node_name );
			gnd::conf::set_parameter( dest, &src->topic_name_laserscan );
			gnd::conf::set_parameter( dest, &src->coordinate_name );
			// coordinate option
			gnd::conf::set_parameter( dest, &src->coordinate_origin );
			gnd::conf::set_parameter( dest, &src->axis_vector_front );
			gnd::conf::set_parameter( dest, &src->axis_vector_upside );
			// debug option
			gnd::conf::set_parameter( dest, &src->status_display );
			gnd::conf::set_parameter( dest, &src->text_log );

			return 0;
		}
	}
}
// <--- function definition



#endif /* GND_LS_COORDTF_CONFIG_HPP_ */
