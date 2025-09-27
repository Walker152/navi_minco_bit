#pragma once

#include <map>
#include <mutex>
#include <thread> 
namespace MyUtils
{
namespace DataType
{

template < typename _T1, typename _T2 > class BiMap
{
  private:
    std::mutex m_m1_mutex, m_m2_mutex;
    std::map< _T1, _T2 > m1;
    std::map< _T2, _T1 > m2;
    typedef typename std::map< _T1, _T2 >::iterator _iterator;
    typedef typename std::map< _T1, _T2 >::const_iterator _const_iterator;

  public:
    BiMap( ) : m1( ), m2( ) {}
    ~BiMap( ) = default;
    inline void insert( const std::pair< _T1, _T2 > &p )
    {
        std::scoped_lock< std::mutex, std::mutex > lock( m_m1_mutex,
                                                         m_m2_mutex );
        m1.insert( p );
        m2.insert( std::make_pair( p.second, p.first ) );
    }

    inline _const_iterator find_by_first( const _T1 &t )
    {
        std::scoped_lock< std::mutex, std::mutex > lock( m_m1_mutex,
                                                         m_m2_mutex );
        return m1.find( t );
    }
    inline _const_iterator find_by_second( const _T2 &t )
    {
        std::scoped_lock< std::mutex, std::mutex > lock( m_m1_mutex,
                                                         m_m2_mutex );
        auto it = m2.find( t );
        if ( it == m2.end( ) )
        {
            return m1.end( );
        }
        else
        {
            return m1.find( it->second );
        }
    }
    inline void erase_by_first( const _T1 &t )
    {
        std::scoped_lock< std::mutex, std::mutex > lock( m_m1_mutex,
                                                         m_m2_mutex );
        auto it = m1.find( t );
        if ( it != m1.end( ) )
        {
            m1.erase( it->first );
            m2.erase( it->second );
        }
    }
    inline void erase_by_second( const _T2 &t )
    {
        std::scoped_lock< std::mutex, std::mutex > lock( m_m1_mutex,
                                                         m_m2_mutex );
        auto it = m2.find( t );
        if ( it != m2.end( ) )
        {
            m1.erase( it->first );
            m2.erase( it->second );
        }
    }
    inline _const_iterator end( ) const { return m1.end( ); }
    inline _const_iterator begin( ) const { return m1.begin( ); }
    inline size_t size( ) const { return m1.size( ); }
};

} // namespace DataType

} // namespace MyUtils
