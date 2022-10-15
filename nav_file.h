#pragma once

#include "nav_area.h"
#include "micropather.h"

namespace nav_mesh
{
  class nav_file : public micropather::Graph
  {
  public:
    nav_file ( )
    {
    }
    nav_file ( const std::string &nav_mesh_file, std::error_code &err );

    void load ( const std::string &nav_mesh_file, std::error_code &err );

    std::vector< vec3_t > find_path ( vec3_t from, vec3_t to, std::error_code &err );

    // MicroPather implementation
    // @note: at this point the std::error_code part can be ignored since we're sure we can get an area by id -swoopae
    virtual float LeastCostEstimate ( void *start, void *end )
    {
      std::error_code placeholder { };
      auto &start_area = get_area_by_id ( std::uint32_t ( start ), placeholder );
      auto &end_area = get_area_by_id ( std::uint32_t ( end ), placeholder );
      auto distance = start_area.get_center ( ) - end_area.get_center ( );

      return std::sqrtf ( distance.x * distance.x + distance.y * distance.y + distance.z * distance.z );
    }

    // @note: at this point the std::error_code part can be ignored since we're sure we can get an area by id -swoopae
    virtual void AdjacentCost ( void *state, micropather::MPVector< micropather::StateCost > *adjacent )
    {
      std::error_code placeholder { };
      auto &area = get_area_by_id ( std::uint32_t ( state ), placeholder );
      auto &area_connections = area.get_connections ( );

      for ( auto &connection : area_connections )
      {
        auto &connection_area = get_area_by_id ( connection.id, placeholder );
        auto distance = connection_area.get_center ( ) - area.get_center ( );

        micropather::StateCost cost = { reinterpret_cast< void * > ( connection_area.get_id ( ) ),
                                        std::sqrtf ( distance.x * distance.x + distance.y * distance.y + distance.z * distance.z ) };

        adjacent->push_back ( cost );
      }
    }

    virtual void PrintStateInfo ( void *state )
    {
    }

  private:
    nav_area &get_area_by_id ( std::uint32_t id, std::error_code &err );
    nav_area &get_area_by_position ( vec3_t position, std::error_code &err );

    std::unique_ptr< micropather::MicroPather > m_pather = nullptr;

    std::uint8_t m_is_analyzed = 0,
                 m_has_unnamed_areas = 0;

    std::uint16_t m_place_count = 0;

    std::uint32_t m_magic = 0xFEEDFACE,
                  m_version = 0,
                  m_sub_version = 0,
                  m_source_bsp_size = 0,
                  m_area_count = 0;

    nav_buffer m_buffer = { };
    std::vector< nav_area > m_areas = { };
    std::vector< std::string > m_places = { };
  };
} // namespace nav_mesh