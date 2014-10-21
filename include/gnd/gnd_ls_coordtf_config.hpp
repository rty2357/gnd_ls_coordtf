/*
 * gnd_ls_coordtf_config.hpp
 *
 *  Created on: 2014/09/05
 *      Author: tyamada
 */

#ifndef GND_LS_COORDTF_CONFIG_HPP_
#define GND_LS_COORDTF_CONFIG_HPP_

#include "gnd/gnd-util.h"
#include "gnd/gnd-vector-base.hpp"
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
		static const char Configuration_item_areas_rectangle[] =  "areas-rectangle";
		static const char Configuration_item_areas_sample[] =  "sample";
		static const char Configuration_item_areas_range[] =  "range";
		static const char Configuration_item_areas_exception[] =  "exception";
		static const param_geo3d_t Default_area_range_lower = {
				"lower",
				{0.0, 0.0, 0.0},
		};
		static const param_geo3d_t Default_area_range_upper = {
				"upper",
				{0.0, 0.0, 0.0},
		};


		// ---> debug option
		static const param_double_t Default_period_cui_status_display = {
				"period-cui-status-display",
				0.0,
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
		typedef struct {
			double upper[3];
			double lower[3];
		} _range_rectangle_t;

		typedef struct {
			char name[gnd::conf::ItemBufferSize];
			gnd::queue<_range_rectangle_t> range;
			gnd::queue<_range_rectangle_t> exception;
		} _area_rectangle_t;



		/**
		 * @brief configuration parameter for gnd_urg_proxy node
		 */
		struct node_config {
			node_config();

			// ros communication
			param_string_t					node_name;					///< node name
			param_string_t					topic_name_laserscan;		///< topic name of laser scan (publish)
			param_string_t					coordinate_name;			///< coordinate name

			// coordinate option
			param_geo3d_t					coordinate_origin;			///< coordinate origin
			param_geo3d_t					axis_vector_front;			///< front axis
			param_geo3d_t					axis_vector_upside;			///< upside axis

			// area
			gnd::queue<_area_rectangle_t>	areas;						///< areas

			// debug option
			param_double_t					period_cui_status_display;	///< cui status display mode
			param_string_t					text_log;					///< text log
		};

		inline
		node_config::node_config() {
			init_node_config(this);
		}
		// <--- struct node_config
	}

	template<>
	inline
	int queue<ls_coordtf::_area_rectangle_t>::__move__(ls_coordtf::_area_rectangle_t* dest, const ls_coordtf::_area_rectangle_t* src, uint32_t len)
	{
		uint64_t i = 0;
		uint64_t j = 0;

		for(i = 0; i < len; i++){
			strcpy(dest[i].name, src[i].name );
			dest[i].range.clear();
			for( j = 0; j < src[i].range.size(); j++ ){
				dest[i].range.push_back( src[i].range.const_begin() + j  );
			}
			dest[i].exception.clear();
			for( j = 0; j < src[i].exception.size(); j++ ){
				dest[i].exception.push_back( src[i].exception.const_begin() + j );
			}
		}

		return 0;
	}

	template<>
	inline
	int queue<ls_coordtf::_area_rectangle_t>::__copy__(ls_coordtf::_area_rectangle_t* dest, const ls_coordtf::_area_rectangle_t* src, uint32_t len)
	{
		return __move__(dest, src, len);
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

			{ // ---> areas
				_area_rectangle_t area_;
				_range_rectangle_t range_;

				strcpy(area_.name, Configuration_item_areas_sample);
				range_.upper[0] = range_.lower[0] = 0;
				range_.upper[1] = range_.lower[1] = 0;
				range_.upper[2] = range_.lower[2] = 0;

				area_.range.push_back(&range_);
				area_.exception.push_back(&range_);
				p->areas.push_back(&area_);
			} // <--- areas

			// debug option
			memcpy( &p->period_cui_status_display,			&Default_period_cui_status_display,		sizeof(Default_period_cui_status_display) );
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

			{ // ---> areas
				gnd::conf::configuration *areas_;

				dest->areas.clear();

				// ---> find the areas item
				if( (areas_ = src->child_find(Configuration_item_areas_rectangle, 0)) != 0 ) {
					int i;
					param_geo3d_t upper_, lower_;
					gnd::conf::configuration *range_;
					_range_rectangle_t dest_range_;
					_area_rectangle_t dest_area_;

					// ---> scanning loop (areas)
					for( i = 0; i < areas_->nchild(); i++ ) {
						char name_[128];

						// name
						(*areas_)[i].get(name_, 0, 0);
						if( !name_[0] ) {
							continue;
						}
						strcpy(dest_area_.name, name_);


						// ---> scanning loop (range)
						dest_area_.range.clear();
						while( (range_ = (*areas_)[i].child_find(Configuration_item_areas_range)) != 0 ) {
							memcpy( &upper_,	&Default_area_range_upper,	sizeof(Default_area_range_upper) );
							memcpy( &lower_,	&Default_area_range_lower,	sizeof(Default_area_range_lower) );
							if( gnd::conf::get_parameter( range_,  &upper_) < 3 - 1 ) {
							}
							else if( gnd::conf::get_parameter( range_,  &lower_) < 3 - 1 ) {
							}
							else if( upper_.value[0] < lower_.value[0] || upper_.value[1] < lower_.value[1] || upper_.value[2] < lower_.value[2] ) {
							}
							else {
								memcpy( dest_range_.upper, upper_.value, sizeof(dest_range_.upper) );
								memcpy( dest_range_.lower, lower_.value, sizeof(dest_range_.lower) );
								dest_area_.range.push_back(&dest_range_);
							}
							(*areas_)[i].child_erase( range_ );
						}
						if( dest_area_.range.size() == 0 ) continue;
						// <--- scanning loop (range)


						// ---> scanning loop (exception)
						dest_area_.exception.clear();
						while( (range_ = (*areas_)[i].child_find(Configuration_item_areas_exception, 0)) != 0 ) {
							memcpy( &upper_,	&Default_area_range_upper,	sizeof(Default_area_range_upper) );
							memcpy( &lower_,	&Default_area_range_lower,	sizeof(Default_area_range_lower) );

							range_->show(stdout);
							if( gnd::conf::get_parameter( range_,  &upper_) < 3 - 1 ) {
							}
							else if( gnd::conf::get_parameter( range_,  &lower_) < 3 - 1 ) {
							}
							else if( upper_.value[0] < lower_.value[0] || upper_.value[1] < lower_.value[1] || upper_.value[2] < lower_.value[2] ) {
							}
							else {
								memcpy( dest_range_.upper, upper_.value, sizeof(dest_range_.upper) );
								memcpy( dest_range_.lower, lower_.value, sizeof(dest_range_.lower) );

								dest_area_.exception.push_back(&dest_range_);
							}
							(*areas_)[i].child_erase( range_ );
						} // <--- scanning loop (exception)

						dest->areas.push_back(&dest_area_);
					} // <--- scanning loop (areas)

				} // <--- find the areas item
			} // <--- areas

			// debug option
			gnd::conf::get_parameter( src, &dest->period_cui_status_display );
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

			{ // ---> areas
				gnd::conf::configuration areas_;
				gnd::conf::configuration area_;
				gnd::conf::configuration range_;
				gnd::conf::configuration exception_;
				param_geo3d_t upper_, lower_;

				int i,j;

				memcpy( &upper_,	&Default_area_range_upper,	sizeof(Default_area_range_upper) );
				memcpy( &lower_,	&Default_area_range_lower,	sizeof(Default_area_range_lower) );

				areas_.set( Configuration_item_areas_rectangle, 0, 0 );
				// ---> scanning loop (areas)
				for ( i = 0; i < (signed)src->areas.size(); i++ ) {
					if( !src->areas[i].name[0] ) 			continue;
					if( src->areas[i].range.size() <= 0)	continue;

					area_.set(src->areas[i].name, 0, 0);
					range_.set(Configuration_item_areas_range, 0, 0);
					for ( j = 0; j < (signed)src->areas[i].range.size(); j++ ) {
						range_.child_clear();

						upper_.value[0] = src->areas[i].range[j].upper[0];
						upper_.value[1] = src->areas[i].range[j].upper[1];
						upper_.value[2] = src->areas[i].range[j].upper[2];
						gnd::conf::set_parameter( &range_, &upper_ );

						lower_.value[0] = src->areas[i].range[j].lower[0];
						lower_.value[1] = src->areas[i].range[j].lower[1];
						lower_.value[2] = src->areas[i].range[j].lower[2];
						gnd::conf::set_parameter( &range_, &lower_ );

						area_.child_push_back(&range_);
					}

					exception_.set(Configuration_item_areas_exception, 0, 0);
					for ( j = 0; j < (signed)src->areas[i].range.size(); j++ ) {
						exception_.child_clear();

						upper_.value[0] = src->areas[i].range[j].upper[0];
						upper_.value[1] = src->areas[i].range[j].upper[1];
						upper_.value[2] = src->areas[i].range[j].upper[2];
						gnd::conf::set_parameter( &exception_, &upper_ );

						lower_.value[0] = src->areas[i].range[j].lower[0];
						lower_.value[1] = src->areas[i].range[j].lower[1];
						lower_.value[2] = src->areas[i].range[j].lower[2];
						gnd::conf::set_parameter( &exception_, &lower_ );

						area_.child_push_back(&exception_);
					}

					areas_.child_push_back(&area_);
				} // <--- scanning loop (areas)
				dest->child_push_back(&areas_);
			} // <--- areas

			// debug option
			gnd::conf::set_parameter( dest, &src->period_cui_status_display );
			gnd::conf::set_parameter( dest, &src->text_log );

			return 0;
		}
	}
}
// <--- function definition



#endif /* GND_LS_COORDTF_CONFIG_HPP_ */
