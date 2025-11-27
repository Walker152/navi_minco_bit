/**
 * ———————————————— 版权声明：本文为CSDN博主「sunwake999」的原创文章，遵循CC 4.0
 BY - SA版权协议，转载请附上原文出处链接及本声明。 原文链接：https : //
 blog.csdn.net/sunweiliang/article/details/88781404
*/

#include <assert.h>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

namespace MyUtils
{
namespace Thread
{

template < typename Callback > struct SlotImpl;

template < typename Callback > struct SignalImpl
{
    typedef std::vector< std::weak_ptr< SlotImpl< Callback > > > SlotList;

    std::mutex mutex_;
    std::shared_ptr< SlotList > slots_;

    SignalImpl( ) : slots_( new SlotList ) {}

    void copyOnWrite( )
    {
        if ( !slots_.unique( ) )
        {
            slots_.reset( new SlotList( *slots_ ) );
        }
        assert( slots_.unique( ) );
    }

    void clean( )
    {
        std::lock_guard< std::mutex > lock( mutex_ );
        copyOnWrite( );
        SlotList &list( *slots_ );
        typename SlotList::iterator it( list.begin( ) );
        while ( it != list.end( ) )
        {
            if ( it->expired( ) )
            {
                it = list.erase( it );
            }
            else
            {
                ++it;
            }
        }
    }
};

template < typename Callback > struct SlotImpl
{
    typedef SignalImpl< Callback > Data;
    std::weak_ptr< Data > data_;
    Callback cb_;
    std::weak_ptr< void > tie_;
    bool tied_;

    SlotImpl( const std::shared_ptr< Data > &data, Callback &&cb )
        : data_( data ), cb_( cb ), tie_( ), tied_( false )
    {
    }

    SlotImpl( const std::shared_ptr< Data > &data, Callback &&cb,
              const std::shared_ptr< void > &tie )
        : data_( data ), cb_( cb ), tie_( tie ), tied_( true )
    {
    }

    ~SlotImpl( )
    {
        std::shared_ptr< Data > data( data_.lock( ) );
        if ( data )
        {
            data->clean( );
        }
    }
};

typedef std::shared_ptr< void > Slot;

template < typename Signature > class Signal;

template < typename RET, typename... ARGS > class Signal< RET( ARGS... ) >
{
  public:
    typedef std::function< void( ARGS... ) > Callback;
    typedef SignalImpl< Callback > SignalImpl_t;
    typedef SlotImpl< Callback > SlotImpl_t;

  private:
    const std::shared_ptr< SignalImpl_t > impl_;

  public:
    Signal( ) : impl_( new SignalImpl_t ) {}

    ~Signal( ) {}

    Slot connect( Callback &&func )
    {
        std::shared_ptr< SlotImpl_t > slotImpl(
            new SlotImpl_t( impl_, std::forward< Callback >( func ) ) );
        add( slotImpl );
        return slotImpl;
    }

    Slot connect( Callback &&func, const std::shared_ptr< void > &tie )
    {
        std::shared_ptr< SlotImpl_t > slotImpl(
            new SlotImpl_t( impl_, func, tie ) );
        add( slotImpl );
        return slotImpl;
    }

    void call( ARGS &&...args )
    {
        SignalImpl_t &impl( *impl_ );
        std::shared_ptr< typename SignalImpl_t::SlotList > slots;
        {
            std::lock_guard< std::mutex > lock( impl.mutex_ );
            slots = impl.slots_;
        }
        typename SignalImpl_t::SlotList &s( *slots );
        for ( typename SignalImpl_t::SlotList::const_iterator it = s.begin( );
              it != s.end( ); ++it )
        {
            std::shared_ptr< SlotImpl_t > slotImpl = it->lock( );
            if ( slotImpl )
            {
                if ( slotImpl->tied_ )
                {
                    std::shared_ptr< void > guard = slotImpl->tie_.lock( );
                    if ( guard )
                    {
                        slotImpl->cb_( args... );
                    }
                }
                else
                {
                    slotImpl->cb_( args... );
                }
            }
        }
    }

  private:
    void add( const std::shared_ptr< SlotImpl_t > &slot )
    {
        SignalImpl_t &impl( *impl_ );
        {
            std::lock_guard< std::mutex > lock( impl.mutex_ );
            impl.copyOnWrite( );
            impl.slots_->push_back( slot );
        }
    }
};

} // namespace Thread

} // namespace MyUtils

