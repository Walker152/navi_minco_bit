#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>

namespace MyUtils
{
namespace DataType
{

class ByteArray
{
  private:
    typedef unsigned char uchar;
    typedef std::shared_ptr< uchar > shared_uchar;
    size_t m_capacity; //容量
    size_t m_size;     //当前占用的空间
    shared_uchar m_data;

  public:
    ByteArray( const size_t &capacity = 512 )
        : m_capacity( capacity ), m_size( 0 )
    {
        m_capacity = __changeValue( m_capacity );
        __resize( );
    }

    ByteArray( ByteArray &&other )
        : m_capacity( other.m_capacity ), m_size( other.m_size ),
          m_data( std::move( other.m_data ) )
    {
    }

    ByteArray &operator=( const ByteArray &other ) = default;
    ByteArray( const ByteArray &other ) = default;

    ByteArray( const void *data, const size_t &len ) : m_size(0), m_capacity(0)
    {
        this->copy( data, len );
    }

    ~ByteArray( ) {}

    inline void copy( const ByteArray &other )
    {
        // 即使是原来 other 与 this原先共用一套数据，此时也会再copy一份
        this->copy( other.m_data.get( ), other.m_size );
    }

    /**
     * @brief 拷贝动作会覆盖掉原有数据
     */
    inline void copy( const void *data, const size_t &len )
    {
        m_size = len;
        if ( m_size > m_capacity )
        {
            m_capacity = __changeValue( len );
            m_data = __malloc( m_capacity );
        }
        memcpy( m_data.get( ), data, len );
        return;
    }

    /**
     * @brief 将数据追加到原有数据之后
     */
    void append( const void *data, const size_t &len )
    {
        if ( len + m_size > m_capacity )
        {
            m_capacity = __changeValue( len + m_size );
            __resize( );
        }
        memcpy( m_data.get( ) + m_size, data, len );
        m_size += len;
    }

    static ByteArray append( const void *data1, const size_t &len1,
                             const void *data2, const size_t &len2 )
    {
        ByteArray arr1( data1, len1 );
        arr1.append( data2, len2 );
        return std::move( arr1 );
    }

    inline void swap( ByteArray &other )
    {
        size_t capacity = other.m_capacity;
        size_t size = other.m_size;
        shared_uchar data = other.m_data;

        other.m_capacity = m_capacity;
        other.m_size = m_size;
        other.m_data = m_data;

        m_capacity = capacity;
        m_size = size;
        m_data = data;
    }

    /**
     * @brief 清除已经原先保存的数据
     */
    inline void reset( ) { m_size = 0; }

    inline const uchar *get( ) const { return m_data.get( ); }

    inline size_t size( ) const { return m_size; }
    inline size_t capacity( ) const { return m_capacity; }
    inline size_t end( ) const { return SIZE_MAX; }

    /**
     * @brief 提取[start_ops,
     * end_ops)的数据。如果出现错误则返回的ByteArray的size为0
     */
    ByteArray sub( const size_t &start_ops, size_t end_ops = SIZE_MAX )
    {
        if ( end_ops > m_size )
        {
            end_ops = m_size;
        }
        if ( start_ops >= end_ops )
        {
            return ByteArray( );
        }
        else
        {
            return ByteArray( this->m_data.get( ) + start_ops,
                              end_ops - start_ops );
        }
    }

  private:
    inline size_t __changeValue( const size_t &size )
    {
        size_t e = size_t( log2( size ) );
        size_t ret = size_t( 1 ) << e;
        if ( ret < size )
            ret <<= 1;
        return ret;
    }
    inline shared_uchar __malloc( const size_t &len )
    {
        return shared_uchar( new uchar[ len ], __deleter );
    }
    inline void __resize( )
    {
        shared_uchar temp( new uchar[ m_capacity ], __deleter );
        memcpy( temp.get( ), m_data.get( ), m_size );
        m_data = temp;
        return;
    }
    static void __deleter( uchar *data )
    {
        delete[] data;
        data = nullptr;
    }
};

} // namespace DataType

} // namespace MyUtils
