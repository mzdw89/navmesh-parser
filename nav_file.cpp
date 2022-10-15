#include "nav_file.h"

#include <iostream>

#pragma warning( push )
#pragma warning( disable : 4715 ) // warning C4715: 'nav_mesh::nav_file::get_area_by_id': not all control paths return a value

namespace nav_mesh
{
  nav_file::nav_file ( const std::string &nav_mesh_file, std::error_code &err )
  {
    load ( nav_mesh_file, err );
  }

  void nav_file::load ( const std::string &nav_mesh_file, std::error_code &err )
  {
    if ( !m_pather )
      m_pather = std::make_unique< micropather::MicroPather > ( this );

    m_pather->Reset ( );
    m_areas.clear ( );
    m_places.clear ( );

    std::error_code load_file { };
    m_buffer.load_from_file ( nav_mesh_file, load_file );

    if ( load_file.value ( ) != 0 )
    {
      err = std::error_code ( 1, std::system_category ( ) );
      return;
    }

    if ( m_buffer.read< std::uint32_t > ( ) != m_magic )
    {
      err = std::error_code ( 2, std::system_category ( ) );
      return;
    }

    m_version = m_buffer.read< std::uint32_t > ( );

    if ( m_version != 16 )
    {
      err = std::error_code ( 3, std::system_category ( ) );
      return;
    }

    m_sub_version = m_buffer.read< std::uint32_t > ( );
    m_source_bsp_size = m_buffer.read< std::uint32_t > ( );
    m_is_analyzed = m_buffer.read< std::uint8_t > ( );
    m_place_count = m_buffer.read< std::uint16_t > ( );

    for ( std::uint16_t i = 0; i < m_place_count; i++ )
    {
      auto place_name_length = m_buffer.read< std::uint16_t > ( );
      std::string place_name ( place_name_length, 0 );

      m_buffer.read ( place_name.data ( ), place_name_length );
      m_places.push_back ( place_name );
    }

    m_has_unnamed_areas = m_buffer.read< std::uint8_t > ( ) != 0;
    m_area_count = m_buffer.read< std::uint32_t > ( );

    if ( m_area_count == 0 )
    {
      err = std::error_code ( 4, std::system_category ( ) );
      return;
    }

    for ( std::uint32_t i = 0; i < m_area_count; i++ )
    {
      nav_area area ( m_buffer );
      m_areas.push_back ( area );
    }

    m_buffer.clear ( );
  }

  std::vector< vec3_t > nav_file::find_path ( vec3_t from, vec3_t to, std::error_code &err )
  {
    std::error_code start_err, end_err;

    auto start = reinterpret_cast< void * > ( get_area_by_position ( from, start_err ).get_id ( ) );
    auto end = reinterpret_cast< void * > ( get_area_by_position ( to, end_err ).get_id ( ) );

    if ( start_err.value ( ) != 0 && end_err.value ( ) != 0 )
    {
      err = std::error_code ( 1, std::system_category ( ) );
      return std::vector< vec3_t > { };
    }

    float total_cost = 0.f;
    micropather::MPVector< void * > path_area_ids = { };

    if ( m_pather->Solve ( start, end, &path_area_ids, &total_cost ) != 0 )
    {
      err = std::error_code ( 2, std::system_category ( ) );
      return std::vector< vec3_t > { };
    }

    std::vector< vec3_t > path = { };
    for ( std::size_t i = 0; i < path_area_ids.size ( ); i++ )
    {
      for ( auto &area : m_areas )
      {
        if ( area.get_id ( ) == std::uint32_t ( path_area_ids[ i ] ) )
        {
          path.push_back ( area.get_center ( ) );
          break;
        }
      }
    }

    return path;
  }

  // @todo: add retn val, msvc allows no return value.
  nav_area &nav_file::get_area_by_id ( std::uint32_t id, std::error_code &err )
  {
    for ( auto &area : m_areas )
    {
      if ( area.get_id ( ) == id )
        return area;
    }

    err = std::error_code ( 1, std::system_category ( ) );
  }

  // @todo: add retn val, msvc allows no return value.
  nav_area &nav_file::get_area_by_position ( vec3_t position, std::error_code &err )
  {
    for ( auto &area : m_areas )
    {
      if ( area.is_within ( position ) )
        return area;
    }

    err = std::error_code ( 1, std::system_category ( ) );
  }
} // namespace nav_mesh

#pragma warning( pop )